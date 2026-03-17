/**
 * testRtsCtsArdunio.ino
 *
 * Hardware RTS/CTS flow control test for ESP32-S3.
 * Arduino IDE 2 equivalent of the nanoFramework testRtsCts project.
 *
 * Single UART (UART1) loopback — 2 jumper wires required:
 * ┌──────────────────────────────────────────┐
 * │  GPIO17 (TX)  ──────►  GPIO18 (RX)       │  data
 * │  GPIO21 (RTS) ──────►  GPIO47 (CTS)      │  flow control feedback
 * └──────────────────────────────────────────┘
 *
 * The writer task sends 921600-baud framed packets continuously on core 0.
 * The main loop (core 1) reads with a READ_DELAY_MS pause, allowing the
 * RX buffer to fill and RTS to deassert, pausing the writer via CTS.
 *
 * Packet format  (PACKET_SIZE bytes):
 *   [0]      0xAA  header
 *   [1]      0x55  header
 *   [2..5]   uint32 BE  sequence number
 *   [6..7]   uint16 BE  data length (= DATA_SIZE)
 *   [8..71]  uint8[DATA_SIZE]  payload: (seq+i) & 0xFF
 *   [72..73] uint16 BE  CRC-16/CCITT-FALSE over bytes [0..71]
 *
 * On any error (CRC / sequence / data) the writer halts and a hex
 * comparison of expected vs received bytes is printed to Serial.
 *
 * Board: ESP32S3 Dev Module
 * Board package: esp32 by Espressif Systems v2.0.0 or later
 * Baud (monitor): 115200
 */

#include <Arduino.h>
#include "driver/uart.h"     // uart_get_buffered_data_len(), UART_NUM_1

// ── Pin assignments ────────────────────────────────────────────────────────────
static const int TX_PIN  = 17; //4
static const int RX_PIN  = 8; //5
static const int RTS_PIN = 21; //6
static const int CTS_PIN = 47;  //7

// ── Communication parameters ──────────────────────────────────────────────────
//
// Flow control trigger maths:
//   RTS deasserts when the hardware FIFO (128 bytes) backs up to the threshold
//   (122 bytes).  That only happens when the UART driver ring buffer is full and
//   can no longer drain the FIFO fast enough.
//
//   bytes per delay period = BAUD_RATE / 10 * READ_DELAY_MS / 1000
//     115200 baud, 100 ms → ~1152 bytes/tick  (ring buf 4096: never fills → no FC)
//     921600 baud, 100 ms → ~9216 bytes/tick  (ring buf 4096: fills in ~44 ms → FC ✓)
//
//   921600 is the correct rate to observe RTS/CTS toggling.
//
static const uint32_t BAUD_RATE     = 921600;
static const int      READ_DELAY_MS = 100;    // deliberate reader slowdown → triggers flow control
static const int      WRITE_DELAY_MS = 0;     // >0 slows writer for easier observation
static const int      DATA_SIZE     = 128;    // payload bytes per packet (larger = fewer, more visible bursts)

static const int      HEADER_SIZE   = 2;
static const int      SEQ_SIZE      = 4;
static const int      LEN_SIZE      = 2;
static const int      CRC_SIZE      = 2;
static const int      PACKET_SIZE   = HEADER_SIZE + SEQ_SIZE + LEN_SIZE + DATA_SIZE + CRC_SIZE;  // 138

static const uint8_t  HDR0 = 0xAA;
static const uint8_t  HDR1 = 0x55;

// ── Diagnostics ───────────────────────────────────────────────────────────────
// true  → on first error halt the writer and print a hex comparison dump
// false → count errors and keep running
static const bool ENABLE_ERROR_DUMP = true;

// ── UART instance (UART1) ─────────────────────────────────────────────────────
static HardwareSerial testSerial(1);

// ── Shared state ──────────────────────────────────────────────────────────────
// Writer-side (updated only by writerTask on core 0):
static volatile uint32_t txSeq      = 0;
static volatile int      packetsSent = 0;

// Reader-side (updated only by main loop on core 1):
static uint32_t rxSeq     = 0;
static bool     rxSynced  = false;  // false until first valid packet accepted
static int      pktsRx    = 0;
static int      crcErrors = 0;
static int      seqErrors = 0;
static int      dataErrors = 0;

// Set true by the reader on error; checked by the writer each iteration:
static volatile bool stopWriter = false;

// Semaphore protecting cross-core reads of writer counters in stats:
static SemaphoreHandle_t statsSem;

// ── Receive accumulation buffer (main loop only) ──────────────────────────────
// Must hold several delay-periods of data.  At 115200 baud with READ_DELAY_MS=100
// roughly 1152 bytes arrive per tick; 8 KB gives ~7 ticks of headroom.
static uint8_t rxBuf[8192];
static int     rxBufLen = 0;

// ── Hex nibble helper ─────────────────────────────────────────────────────────
static const char HEX_CHARS[] = "0123456789ABCDEF";

static inline char hexNibble(int n) { return HEX_CHARS[n & 0xF]; }

// ── Forward declarations ───────────────────────────────────────────────────────
static uint16_t crc16(const uint8_t *data, int offset, int length);
static void     buildPacket(uint8_t *buf, uint32_t seq);
static void     parsePackets();
static void     dumpPacketComparison(const char *label, uint32_t expectedSeq);
static void     shiftBuf(int count);
static void     printWiringInstructions();
static void     writerTask(void *param);

// =============================================================================
void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println();
    Serial.println("=================================================");
    Serial.println("  RTS/CTS Hardware Flow Control Test");
    Serial.println("  ESP32-S3  —  single UART1 loopback");
    Serial.println("=================================================");
    printWiringInstructions();

    statsSem = xSemaphoreCreateMutex();

    // ── Open UART1 with a generous RX buffer ─────────────────────────────────
    // setRxBufferSize must be called before begin().
    // begin() configures TX and RX in the GPIO matrix.
    // setPins(-1,-1, CTS, RTS) was tried but it silently unassigns RX/TX on
    // some ESP32 Arduino core versions (hwBuf stayed 0), disconnecting the RX
    // path that begin() just set up.
    // Instead, use the ESP-IDF uart_set_pin() with UART_PIN_NO_CHANGE for the
    // TX and RX slots — this is the guaranteed way to add CTS/RTS without
    // touching the existing pin assignments.
    testSerial.setRxBufferSize(4096);
    testSerial.begin(BAUD_RATE, SERIAL_8N1, RX_PIN, TX_PIN);

    // Add RTS/CTS via ESP-IDF: UART_PIN_NO_CHANGE leaves TX and RX untouched
    uart_set_pin(UART_NUM_1,
                 UART_PIN_NO_CHANGE,   // TX — already set by begin()
                 UART_PIN_NO_CHANGE,   // RX — already set by begin()
                 RTS_PIN,
                 CTS_PIN);
    uart_set_hw_flow_ctrl(UART_NUM_1, UART_HW_FLOWCTRL_CTS_RTS, 122);


    Serial.printf("UART1 open: %lu baud, RTS/CTS hardware flow control\n",
                  (unsigned long)BAUD_RATE);
    Serial.printf("Packet size : %d bytes\n", PACKET_SIZE);
    Serial.printf("Read delay  : %d ms  (triggers flow control)\n", READ_DELAY_MS);
    Serial.printf("~%lu bytes arrive per delay period\n",
                  (unsigned long)READ_DELAY_MS * BAUD_RATE / 10000UL);
    Serial.println();

    // ── Writer task on core 0; main loop runs on core 1 ──────────────────────
    xTaskCreatePinnedToCore(writerTask, "writer", 4096, nullptr, 5, nullptr, 0);
}

// ── Main loop (core 1): read + stats ─────────────────────────────────────────
static unsigned long lastStats = 0;
static int lastSentSnap = 0, lastRxSnap = 0;

void loop() {
    if (stopWriter) {
        // Writer halted after error — drain UART so the ring buffer empties
        // and RTS returns LOW (clear visual indication of halted state on
        // logic analyser), then idle so the dump remains readable.
        while (testSerial.available()) testSerial.read();
        delay(5000);
        return;
    }

    // ── Read with deliberate delay to let the RX buffer fill ─────────────────
    // The delay lets bytes accumulate, filling the 4096-byte UART ring buffer.
    // Once full the UART driver can no longer drain the hardware FIFO; the FIFO
    // backs up to the 122-byte RTS threshold and RTS deasserts, pausing the
    // writer via the RTS→CTS loopback.  Draining the ring buffer after the
    // delay drops the FIFO level below threshold and reasserts RTS.
    //
    // readBytes() is NOT used here because it blocks for up to setTimeout()
    // (30 s) waiting for the exact byte count, which deadlocks: writer is paused
    // by CTS, so no new bytes arrive, readBytes never completes, ring buffer
    // never drains, RTS stays deasserted permanently.
    // Instead, read() in a tight loop takes only what is available right now.
    if (testSerial.available()) {
        delay(READ_DELAY_MS);   // deliberate slowdown — triggers flow control

        if (rxBufLen >= (int)sizeof(rxBuf)) {
            // Accumulation buffer full — drain UART and resync
            Serial.printf("[Reader] OVERFLOW: bufLen=%d — draining UART\n", rxBufLen);
            while (testSerial.available()) testSerial.read();
            rxBufLen = 0;
            crcErrors++;
        } else {
            // Non-blocking drain: read exactly what is available right now
            while (testSerial.available() && rxBufLen < (int)sizeof(rxBuf)) {
                rxBuf[rxBufLen++] = (uint8_t)testSerial.read();
            }
            parsePackets();
        }
    }

    // ── Stats every 5 s ──────────────────────────────────────────────────────
    unsigned long now = millis();
    if (now - lastStats >= 5000UL) {
        // Snapshot writer counters across cores
        int sent;
        if (xSemaphoreTake(statsSem, pdMS_TO_TICKS(10)) == pdTRUE) {
            sent = packetsSent;
            xSemaphoreGive(statsSem);
        } else {
            sent = (int)packetsSent; // best-effort if sem unavailable
        }

        int sentRate = (sent  - lastSentSnap) / 5;
        int rxRate   = (pktsRx - lastRxSnap)  / 5;
        lastSentSnap = sent;
        lastRxSnap   = pktsRx;
        long throughput = (long)rxRate * PACKET_SIZE;

        // Hardware-level RX diagnostic:
        //   hwBuf > 0 but available() == 0  → bytes in UART FIFO/driver ring
        //                                     but Arduino layer not surfacing them
        //   hwBuf == 0 && available() == 0  → bytes not reaching the UART at all
        //                                     (pin/GPIO matrix problem)
        size_t hwBuf = 0;
        uart_get_buffered_data_len(UART_NUM_1, &hwBuf);

        Serial.println("─────────── Stats ───────────");
        Serial.printf(" TX: %8d pkts  (%5d/s)\n", sent,   sentRate);
        Serial.printf(" RX: %8d pkts  (%5d/s)\n", pktsRx, rxRate);
        Serial.printf(" In-flight : %d\n", sent - pktsRx);
        Serial.printf(" Throughput: ~%ld B/s  (~%ld kbit/s)\n",
                      throughput, throughput * 8L / 1000L);
        Serial.printf(" Errors  CRC:%d  Seq:%d  Data:%d\n",
                      crcErrors, seqErrors, dataErrors);
        Serial.printf(" HW UART buf: %u B   Arduino available(): %d B\n",
                      (unsigned)hwBuf, testSerial.available());
        if (pktsRx == 0 && hwBuf == 0)
            Serial.println(" Result: NO DATA — bytes not reaching UART (pin/GPIO matrix issue)");
        else if (pktsRx == 0 && hwBuf > 0)
            Serial.println(" Result: NO DATA — bytes in UART buf but Arduino not reading them");
        else if (crcErrors == 0 && seqErrors == 0 && dataErrors == 0)
            Serial.println(" Result: PASS — all packets intact, flow control working");
        else
            Serial.printf(" Result: FAIL — %d total errors\n",
                          crcErrors + seqErrors + dataErrors);
        Serial.println("─────────────────────────────");

        lastStats = now;
    }
}

// ── Writer task (core 0) ──────────────────────────────────────────────────────
static void writerTask(void *param) {
    uint8_t packet[PACKET_SIZE];
    Serial.println("[Writer] started — sending at full speed");

    while (!stopWriter) {
        buildPacket(packet, (uint32_t)txSeq);

        // write() blocks automatically when CTS is deasserted (flow control active)
        testSerial.write(packet, PACKET_SIZE);

        if (xSemaphoreTake(statsSem, portMAX_DELAY) == pdTRUE) {
            txSeq++;
            packetsSent++;
            xSemaphoreGive(statsSem);
        }

        if (WRITE_DELAY_MS > 0)
            vTaskDelay(pdMS_TO_TICKS(WRITE_DELAY_MS));
    }

    Serial.printf("[Writer] HALTED at seq %lu — error detected, inspect output above\n",
                  (unsigned long)txSeq);
    vTaskDelete(nullptr);
}

// ── Packet builder ────────────────────────────────────────────────────────────
static void buildPacket(uint8_t *buf, uint32_t seq) {
    buf[0] = HDR0;
    buf[1] = HDR1;
    buf[2] = (uint8_t)(seq >> 24);
    buf[3] = (uint8_t)(seq >> 16);
    buf[4] = (uint8_t)(seq >>  8);
    buf[5] = (uint8_t)(seq      );
    buf[6] = (uint8_t)(DATA_SIZE >> 8);
    buf[7] = (uint8_t)(DATA_SIZE     );

    const int dataStart = HEADER_SIZE + SEQ_SIZE + LEN_SIZE;
    for (int i = 0; i < DATA_SIZE; i++)
        buf[dataStart + i] = (uint8_t)((seq + i) & 0xFF);

    uint16_t crc = crc16(buf, 0, PACKET_SIZE - CRC_SIZE);
    buf[PACKET_SIZE - 2] = (uint8_t)(crc >> 8);
    buf[PACKET_SIZE - 1] = (uint8_t)(crc      );
}

// ── Packet parser ─────────────────────────────────────────────────────────────
static void parsePackets() {
    while (rxBufLen >= PACKET_SIZE) {

        // ── Find header ───────────────────────────────────────────────────────
        int hdrPos = -1;
        for (int i = 0; i <= rxBufLen - PACKET_SIZE; i++) {
            if (rxBuf[i] == HDR0 && rxBuf[i + 1] == HDR1) {
                hdrPos = i;
                break;
            }
        }
        if (hdrPos < 0) {
            uint8_t last = rxBuf[rxBufLen - 1];
            rxBuf[0] = last;
            rxBufLen = 1;
            return;
        }
        if (hdrPos > 0) {
            seqErrors++;
            Serial.printf("[Reader] Framing: discarding %d pre-header bytes\n", hdrPos);
            shiftBuf(hdrPos);
            continue;
        }
        if (rxBufLen < PACKET_SIZE) return;

        // ── CRC check ─────────────────────────────────────────────────────────
        uint16_t calcCrc   = crc16(rxBuf, 0, PACKET_SIZE - CRC_SIZE);
        uint16_t packetCrc = ((uint16_t)rxBuf[PACKET_SIZE - 2] << 8)
                           |  (uint16_t)rxBuf[PACKET_SIZE - 1];

        if (calcCrc != packetCrc) {
            crcErrors++;
            Serial.printf("[Reader] CRC error: calc=0x%04X pkt=0x%04X\n",
                          calcCrc, packetCrc);
            if (ENABLE_ERROR_DUMP) {
                uint32_t seqGuess = ((uint32_t)rxBuf[2] << 24)
                                  | ((uint32_t)rxBuf[3] << 16)
                                  | ((uint32_t)rxBuf[4] <<  8)
                                  |  (uint32_t)rxBuf[5];
                dumpPacketComparison("CRC ERROR (seq field may be corrupt)", seqGuess);
                stopWriter = true;
                return;
            }
            shiftBuf(2);
            continue;
        }

        // ── Sequence check ────────────────────────────────────────────────────
        uint32_t seq = ((uint32_t)rxBuf[2] << 24)
                     | ((uint32_t)rxBuf[3] << 16)
                     | ((uint32_t)rxBuf[4] <<  8)
                     |  (uint32_t)rxBuf[5];

        if (seq != rxSeq) {
            if (!rxSynced) {
                // First packet after reset: the writer has been running while
                // the ring buffer was filling so the first decoded seq will
                // almost never be 0.  Accept it silently as the sync point.
                Serial.printf("[Reader] Initial sync to seq %lu\n", (unsigned long)seq);
            } else {
                Serial.printf("[Reader] SeqError: expected %lu got %lu\n",
                              (unsigned long)rxSeq, (unsigned long)seq);
                seqErrors++;
                if (ENABLE_ERROR_DUMP) {
                    uint32_t expectedSeq = rxSeq;
                    rxSeq = seq + 1;
                    pktsRx++;
                    rxSynced = true;
                    char label[56];
                    snprintf(label, sizeof(label),
                             "SEQUENCE ERROR (expected %lu, got %lu)",
                             (unsigned long)expectedSeq, (unsigned long)seq);
                    dumpPacketComparison(label, expectedSeq);
                    stopWriter = true;
                    return;
                }
            }
            rxSeq = seq + 1;
        } else {
            rxSeq++;
        }
        pktsRx++;
        rxSynced = true;

        // ── Payload data check ────────────────────────────────────────────────
        const int dataStart = HEADER_SIZE + SEQ_SIZE + LEN_SIZE;
        bool dataOk = true;
        for (int i = 0; i < DATA_SIZE && dataOk; i++) {
            uint8_t expected = (uint8_t)((seq + i) & 0xFF);
            if (rxBuf[dataStart + i] != expected) {
                dataErrors++;
                Serial.printf("[Reader] DataError seq=%lu off=%d exp=0x%02X got=0x%02X\n",
                              (unsigned long)seq, i, expected, rxBuf[dataStart + i]);
                dataOk = false;
            }
        }
        if (!dataOk && ENABLE_ERROR_DUMP) {
            char label[32];
            snprintf(label, sizeof(label), "DATA ERROR (seq=%lu)", (unsigned long)seq);
            dumpPacketComparison(label, seq);
            stopWriter = true;
            return;
        }

        shiftBuf(PACKET_SIZE);
    }
}

// ── Error hex dump ────────────────────────────────────────────────────────────
static void dumpPacketComparison(const char *label, uint32_t expectedSeq) {
    uint8_t exp[PACKET_SIZE];
    buildPacket(exp, expectedSeq);

    Serial.println();
    Serial.print("╔══ "); Serial.print(label); Serial.println(" ══╗");
    Serial.printf("  Expected seq: %lu   Received bytes in buffer: %d\n",
                  (unsigned long)expectedSeq, rxBufLen);
    Serial.println("  Off  | Expected                                        | Received                                        | Match");
    Serial.println("  ─────┼─────────────────────────────────────────────────┼─────────────────────────────────────────────────┼──────");

    const int ROW = 16;
    // Each column: ROW bytes × 3 chars each ("HH "), plus null terminator
    char expStr[ROW * 3 + 1];
    char gotStr[ROW * 3 + 1];
    char mrkStr[ROW * 3 + 1];

    for (int row = 0; row * ROW < PACKET_SIZE; row++) {
        int offset = row * ROW;
        int count  = min(ROW, PACKET_SIZE - offset);

        for (int col = 0; col < ROW; col++) {
            if (col < count) {
                uint8_t e = exp[offset + col];
                uint8_t g = (offset + col < rxBufLen) ? rxBuf[offset + col] : 0xFF;
                bool    m = (e == g);
                expStr[col * 3]     = hexNibble(e >> 4);
                expStr[col * 3 + 1] = hexNibble(e);
                expStr[col * 3 + 2] = ' ';
                gotStr[col * 3]     = hexNibble(g >> 4);
                gotStr[col * 3 + 1] = hexNibble(g);
                gotStr[col * 3 + 2] = ' ';
                mrkStr[col * 3]     = m ? ' ' : '^';
                mrkStr[col * 3 + 1] = m ? ' ' : '^';
                mrkStr[col * 3 + 2] = ' ';
            } else {
                expStr[col*3] = expStr[col*3+1] = expStr[col*3+2] = ' ';
                gotStr[col*3] = gotStr[col*3+1] = gotStr[col*3+2] = ' ';
                mrkStr[col*3] = mrkStr[col*3+1] = mrkStr[col*3+2] = ' ';
            }
        }
        expStr[ROW * 3] = gotStr[ROW * 3] = mrkStr[ROW * 3] = '\0';

        Serial.printf("  %04X | %s| %s| %s\n", offset, expStr, gotStr, mrkStr);
    }
    Serial.println("╚═══════════════════════════════════════════════════════════════════════════╝");
    Serial.println();
}

// ── CRC-16/CCITT-FALSE (poly=0x1021, init=0xFFFF, no reflection) ─────────────
static uint16_t crc16(const uint8_t *data, int offset, int length) {
    uint16_t crc = 0xFFFF;
    for (int i = offset; i < offset + length; i++) {
        crc ^= (uint16_t)((uint16_t)data[i] << 8);
        for (int bit = 0; bit < 8; bit++) {
            if (crc & 0x8000)
                crc = (uint16_t)((crc << 1) ^ 0x1021);
            else
                crc = (uint16_t)(crc << 1);
        }
    }
    return crc;
}

// ── Helpers ───────────────────────────────────────────────────────────────────
static void shiftBuf(int count) {
    int remaining = rxBufLen - count;
    if (remaining > 0)
        memmove(rxBuf, rxBuf + count, remaining);
    rxBufLen = (remaining > 0) ? remaining : 0;
}

static void printWiringInstructions() {
    Serial.println();
    Serial.println("╔══════════════════════════════════════════════════╗");
    Serial.println("║  LOOPBACK WIRING  (2 jumper wires)               ║");
    Serial.println("╠══════════════════════════════════════════════════╣");
    Serial.printf( "║  GPIO%-2d (TX)  ──►  GPIO%-2d (RX)   data           ║\n", TX_PIN,  RX_PIN);
    Serial.printf( "║  GPIO%-2d (RTS) ──►  GPIO%-2d (CTS)  flow control   ║\n", RTS_PIN, CTS_PIN);
    Serial.println("╚══════════════════════════════════════════════════╝");
    Serial.println("  RX buffer fills → RTS (GPIO21) goes LOW");
    Serial.println("  → CTS (GPIO47) goes LOW → write() blocks");
    Serial.println();
}
