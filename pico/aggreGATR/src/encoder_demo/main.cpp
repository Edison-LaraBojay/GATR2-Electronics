// main.cpp
// Encoder demo firmware: counts two quadrature channels and sends one
// encoder record (encoder-record/encoder_record.h) to the Pi every tick.
// No IMU, no commands, no localization. Runs with or without a receiver;
// counts are never reset after boot.

#include <Arduino.h>
#include <hardware/uart.h>
#include <stdio.h>

#include "board.h"
#include "encoder_demo_config.h"
#include "encoder_record.h"
#include "quadrature.h"

namespace
{

uint16_t g_run_id    = 0;
uint16_t g_seq       = 0;
uint32_t g_next_tick = 0;
uint32_t g_sent      = 0;
uint32_t g_skipped   = 0; // ticks with the TX FIFO still busy

encoder_record::Record g_last;

// Serial1.availableForWrite() only reports 0 or 1 on this core.
bool txFifoEmpty() { return (uart_get_hw(uart0)->fr & UART_UARTFR_TXFE_BITS) != 0; }

uint16_t makeRunId() {
    const uint32_t x  = rp2040.hwrand32() ^ time_us_32();
    const uint16_t id = static_cast<uint16_t>(x ^ (x >> 16));
    return id != 0 ? id : 1;
}

// 16 bytes into the empty 32 byte FIFO: never blocks.
void sendRecord() {
    int32_t counts[encoder::kChannels];
    encoder::snapshot(counts);

    encoder_record::Record r;
    r.run_id   = g_run_id;
    r.seq      = g_seq++;
    r.channel0 = counts[0];
    r.channel1 = counts[1];

    const encoder_record::RecordBytes bytes = encoder_record::encode(r);
    Serial1.write(bytes.data(), bytes.size());
    g_last = r;
    ++g_sent;
}

void logUsb() {
    static uint32_t last_ms = 0;
    const uint32_t  now_ms  = millis();
    if (!demo_cfg::kUsbDebug || !Serial || now_ms - last_ms < demo_cfg::kUsbDebugPeriodMs) {
        return;
    }
    last_ms = now_ms;
    char      line[128];
    const int n = snprintf(line, sizeof(line),
                           "encoder demo run=%04X seq=%u ch0=%ld ch1=%ld sent=%lu skipped=%lu\r\n",
                           static_cast<unsigned>(g_run_id), static_cast<unsigned>(g_last.seq),
                           static_cast<long>(g_last.channel0), static_cast<long>(g_last.channel1),
                           static_cast<unsigned long>(g_sent),
                           static_cast<unsigned long>(g_skipped));
    if (n > 0 && static_cast<size_t>(n) < sizeof(line) && Serial.availableForWrite() >= n) {
        Serial.write(reinterpret_cast<const uint8_t*>(line), static_cast<size_t>(n));
    }
}

} // namespace

void setup() {
    if (demo_cfg::kUsbDebug) {
        Serial.begin(demo_cfg::kUsbBaud);
    }
    Serial1.setTX(demo_cfg::kUartTxPin);
    Serial1.setRX(demo_cfg::kUartRxPin);
    Serial1.begin(demo_cfg::kUartBaud);

    for (uint8_t ch = 0; ch < demo_cfg::kChannelsUsed; ++ch) {
        encoder::begin(ch, board::kEncPinA[ch], board::kEncPinB[ch]);
    }
    g_run_id    = makeRunId();
    g_next_tick = micros();
}

void loop() {
    // Signed compare handles micros rollover. After a long stall, skip the
    // missed ticks instead of sending a burst.
    const uint32_t now = micros();
    if (static_cast<int32_t>(now - g_next_tick) >= 0) {
        g_next_tick += demo_cfg::kTickUs;
        if (static_cast<int32_t>(now - g_next_tick) >= 0) {
            g_next_tick = now + demo_cfg::kTickUs;
        }
        if (txFifoEmpty()) {
            sendRecord();
        } else {
            ++g_skipped;
        }
    }
    logUsb();
}
