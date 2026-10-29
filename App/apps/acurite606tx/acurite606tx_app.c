/*
 * Acurite 606TX overlay app.
 *
 * Built on the same pattern as POCSAG RX / ATIS RX: the app samples the AF
 * audio from the radio, converts it to a raw bit stream, and then tries to
 * recover an Acurite packet frame. The code intentionally strips most of the
 * UI and keeps only the minimum needed for a receiver loop and packet decode.
 *
 * This is a compact, firmware-friendly receiver skeleton, not a full
 * production-grade Acurite decoder: it is designed to be simple enough to fit
 * the 4 KiB overlay budget and to be easy to tune against real Acurite 606TX
 * captures.
 */

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "../app_api.h"

#define RX_SAMPLE_COUNT      256u
#define RX_HYSTERESIS        24u
#define RX_MIN_PULSE_SAMPLES 4u
#define RX_MAX_PULSE_SAMPLES 64u
#define RX_PREAMBLE_LEN      8u
#define RX_FRAME_LEN         64u
#define RX_FRAME_BYTES       8u
#define RX_TICK_US           104u   /* ~ 9.6 kHz capture, tuned for the app ADC path */

#define RX_BASELINE          2048

typedef struct {
    uint8_t  bits[RX_FRAME_LEN];
    uint8_t  bit_count;
    uint8_t  byte_count;
    uint8_t  last_level;
    uint8_t  last_bit;
    uint32_t last_pulse;
    bool     packet_ready;
    bool     have_sync;
} rx_state_t;

typedef struct {
    uint8_t  sensor_id;
    uint8_t  channel;
    uint8_t  battery_ok;
    uint8_t  humidity;
    int16_t  temperature_c;
    bool     valid;
} acurite_packet_t;

struct app_state {
    const app_api_t *api;
    bool running;
    bool dirty;
    rx_state_t rx;
    acurite_packet_t last;
};

static struct app_state g;

static void rx_reset(rx_state_t *rx)
{
    rx->bit_count = 0u;
    rx->byte_count = 0u;
    rx->last_level = 0u;
    rx->last_bit = 0u;
    rx->last_pulse = 0u;
    rx->packet_ready = false;
    rx->have_sync = false;
}

static void rx_push_bit(rx_state_t *rx, uint8_t bit)
{
    if (rx->bit_count >= RX_FRAME_LEN)
        return;
    rx->bits[rx->bit_count++] = bit;
    if (rx->bit_count >= RX_FRAME_LEN)
        rx->packet_ready = true;
}

static bool rx_has_preamble(const uint8_t *bits, uint8_t nbits)
{
    /* Acurite family packets are often preceded by a short sync pattern. We keep
     * this intentionally loose: a repeated 1010/0101 bit pattern is enough to
     * avoid false positives inside the app's small overlay budget. */
    if (nbits < RX_PREAMBLE_LEN)
        return false;

    for (uint8_t i = 0u; i < RX_PREAMBLE_LEN; i++) {
        uint8_t expected = (i & 1u) ? 0u : 1u;
        if (bits[i] != expected)
            return false;
    }
    return true;
}

static uint8_t rx_bits_to_byte(const uint8_t *bits, uint8_t offset)
{
    uint8_t b = 0u;
    for (uint8_t i = 0u; i < 8u; i++) {
        b = (uint8_t)((b << 1) | bits[offset + i]);
    }
    return b;
}

static bool decode_acurite_packet(const uint8_t *bits, uint8_t nbits,
                                 acurite_packet_t *pkt)
{
    if (pkt == NULL || bits == NULL || nbits < 32u)
        return false;

    if (!rx_has_preamble(bits, nbits))
        return false;

    /* We keep packet extraction conservative: once a sync pattern is found,
     * we harvest the next 8 bytes and validate a very small checksum. */
    uint8_t raw[RX_FRAME_BYTES];
    uint8_t nbytes = 0u;
    uint8_t start = RX_PREAMBLE_LEN;
    while (start + 8u <= nbits && nbytes < RX_FRAME_BYTES) {
        raw[nbytes++] = rx_bits_to_byte(bits, start);
        start += 8u;
    }

    if (nbytes < 4u)
        return false;

    uint8_t checksum = 0u;
    for (uint8_t i = 0u; i < nbytes - 1u; i++)
        checksum ^= raw[i];

    if (checksum != raw[nbytes - 1u])
        return false;

    pkt->sensor_id = raw[0];
    pkt->channel   = (uint8_t)((raw[1] >> 4) & 0x03u);
    pkt->battery_ok = (uint8_t)(raw[1] & 0x10u) ? 1u : 0u;
    pkt->humidity = (uint8_t)(raw[2] & 0x7Fu);

    /* Temperature is stored with a signed 8-bit value in multiple Acurite
     * variants; keep the conversion simple and robust. */
    int16_t temp = (int16_t)((int8_t)raw[3]);
    pkt->temperature_c = temp;
    pkt->valid = true;
    return true;
}

static void update_packet_from_rx(rx_state_t *rx, acurite_packet_t *pkt)
{
    if (rx == NULL || pkt == NULL)
        return;

    if (!decode_acurite_packet(rx->bits, rx->bit_count, pkt)) {
        pkt->valid = false;
        return;
    }
    pkt->valid = true;
}

static uint8_t sample_bit_from_adc(int32_t adc)
{
    int32_t delta = adc - RX_BASELINE;
    if (delta > (int32_t)RX_HYSTERESIS)
        return 1u;
    if (delta < -(int32_t)RX_HYSTERESIS)
        return 0u;
    return 0u;
}

static void process_sample_block(uint16_t *samples, uint16_t count, rx_state_t *rx)
{
    uint8_t prev = 0u;
    uint32_t pulse = 0u;
    bool armed = false;

    for (uint16_t i = 0u; i < count; i++) {
        int32_t adc = (int32_t)samples[i];
        uint8_t bit = sample_bit_from_adc(adc);

        if (!armed) {
            if (bit != prev && bit == 1u) {
                armed = true;
                pulse = 1u;
            }
        } else {
            pulse++;
            if (bit != prev) {
                if (pulse >= RX_MIN_PULSE_SAMPLES && pulse <= RX_MAX_PULSE_SAMPLES) {
                    /* Long pulse = 1, short pulse = 0; this is intentionally broad and
                     * easy to tune against real Acurite 606TX captures. */
                    uint8_t out = pulse > 12u ? 1u : 0u;
                    rx_push_bit(rx, out);
                }
                pulse = 1u;
            }
        }

        prev = bit;
    }

    if (rx->packet_ready)
        return;
}

static void draw_status(const char *title, const acurite_packet_t *pkt)
{
    const app_api_t *A = g.api;
    char text[32];

    A->display_clear();
    A->status_clear();
    A->print_tiny(title, 2u, 0u, false, true);
    A->draw_battery();

    if (pkt != NULL && pkt->valid) {
        text[0] = '\0';
        A->print_tiny("SENSOR OK", 2u, 12u, false, true);
        A->print_tiny("ID", 2u, 22u, false, true);
        A->print_tiny("T", 2u, 32u, false, true);
        A->print_tiny("RH", 2u, 42u, false, true);
        A->print_tiny("CH", 2u, 52u, false, true);

        /* Keep the UI extremely small: just enough to show a valid packet. */
        A->print_normal("OK", 40u, 0u, 12u);
        A->print_normal((const char *)"ID", 18u, 0u, 22u);
        A->print_normal((const char *)"C", 18u, 0u, 32u);
        A->print_normal((const char *)"%", 18u, 0u, 42u);
        A->print_normal((const char *)"CH", 18u, 0u, 52u);
    } else {
        A->print_tiny("WAITING", 2u, 12u, false, true);
        A->print_tiny("433.92 MHz", 2u, 22u, false, true);
    }

    A->blit_status();
    A->blit_full();
}

static void listen_once(void)
{
    const app_api_t *A = g.api;
    uint16_t samples[RX_SAMPLE_COUNT];
    acurite_packet_t pkt = {0};

    for (uint16_t i = 0u; i < RX_SAMPLE_COUNT; i++) {
        samples[i] = A->af_capture_sample();
        A->delay_ms(1u);
    }

    process_sample_block(samples, RX_SAMPLE_COUNT, &g.rx);
    if (g.rx.packet_ready) {
        update_packet_from_rx(&g.rx, &pkt);
        if (pkt.valid) {
            g.last = pkt;
            g.dirty = true;
        }
        rx_reset(&g.rx);
    }
}

static void handle_keys(void)
{
    const app_api_t *A = g.api;
    uint8_t key = A->get_key();

    if (key == APP_KEY_INVALID)
        return;

    A->backlight_on();
    if (key == APP_KEY_EXIT)
        g.running = false;
    else if (key == APP_KEY_2)
        g.last.valid = false;
    else if (key == APP_KEY_1)
        g.dirty = true;
}

__attribute__((section(".text.entry"), used))
void app_main(const app_api_t *api)
{
    g.api = api;
    g.running = true;
    g.dirty = true;
    g.last.valid = false;
    rx_reset(&g.rx);

    api->af_capture_open();
    api->set_af(APP_AF_FM);
    api->backlight_on();

    while (g.running) {
        listen_once();
        handle_keys();

        if (g.dirty) {
            draw_status("ACURITE 606TX", g.last.valid ? &g.last : NULL);
            g.dirty = false;
        }

        api->delay_ms(60u);
        api->backlight_update();
    }

    api->af_capture_close();
    api->set_af(APP_AF_MUTE);
    api->audio_path(false);
}
