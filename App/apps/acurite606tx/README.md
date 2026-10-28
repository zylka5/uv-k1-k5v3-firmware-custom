/* Copyright 2026
 * Ported from the rtl_433 Acurite 606TX decoder and adapted to the UV-K1
 * overlay-app API v3. This is a compact proof-of-concept receiver: it listens
 * to the demodulated 433.92 MHz RF audio, detects OOK pulse trains, and tries
 * to decode Acurite 606TX temperature/humidity packets in the same style as
 * the upstream rtl_433 implementation.
 */

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "../app_api.h"

#define ACURITE_SAMPLE_RATE      9600u
#define ACURITE_MIN_PULSE_US    200u
#define ACURITE_MAX_PULSE_US    1200u
#define ACURITE_SCAN_COUNT      256u
#define ACURITE_PREAMBLE_BYTES  4u
#define ACURITE_PACKET_BITS     64u
#define ACURITE_BASELINE        2048u
#define ACURITE_FRAME_GAP_MS    1500u

typedef struct {
    uint8_t sensor_id;
    uint8_t channel;
    uint8_t battery;
    uint8_t humidity;
    int16_t temperature_c;
    bool valid;
} acurite606tx_packet_t;

struct app_state {
    const app_api_t *api;
    bool running;
    bool dirty;
    acurite606tx_packet_t last;
    uint32_t last_seen_ms;
    uint32_t sample_count;
};

static struct app_state g;

static uint32_t acurite_clamp_u32(uint32_t value, uint32_t minv, uint32_t maxv)
{
    if (value < minv)
        return minv;
    if (value > maxv)
        return maxv;
    return value;
}

static uint16_t acurite_abs16(int16_t value)
{
    return (uint16_t)(value < 0 ? (int16_t)(-value) : value);
}

static uint8_t acurite_nibble_xor(uint8_t a, uint8_t b)
{
    return (uint8_t)(a ^ b);
}

/* Acurite 606TX is a short OOK / ASK frame. rtl_433 recognises the same family
 * with short repeats and a fixed pattern at the beginning. We keep the port
 * intentionally conservative: accept only packets whose start bytes are within
 * the expected Acurite family and whose CRC/checksum survives a trivial XOR
 * validation. This is enough to reject noise while staying small.
 */
static bool acurite_decode_packet(const uint8_t *bytes, uint8_t len,
                                 acurite606tx_packet_t *packet)
{
    if (len < 8u || bytes == NULL || packet == NULL)
        return false;

    /* Start marker: very common in Acurite family. We do not require a precise
     * bit-perfect decode here; the app is deliberately tolerant and tries to
     * preserve a small footprint. */
    if ((bytes[0] & 0xF0u) != 0x20u && (bytes[0] & 0xF0u) != 0x40u)
        return false;

    uint8_t id = bytes[1];
    uint8_t byte2 = bytes[2];
    uint8_t byte3 = bytes[3];
    uint8_t temp_raw = (uint8_t)((byte2 << 1) | (byte3 >> 7));
    int16_t temp_c = (int16_t)((int8_t)byte2) * 2 + ((int8_t)(byte3 >> 7));
    uint8_t humidity = byte3 & 0x7Fu;
    uint8_t checksum = 0u;
    for (uint8_t i = 0u; i < len - 1u; i++)
        checksum ^= bytes[i];

    if (checksum != bytes[len - 1u])
        return false;

    packet->sensor_id = id;
    packet->channel = (uint8_t)((byte2 >> 4) & 0x03u);
    packet->battery = (uint8_t)(byte2 & 0x10u) ? 1u : 0u;
    packet->humidity = humidity;
    packet->temperature_c = temp_c;
    packet->valid = true;
    return true;
}

static bool acurite_scan_for_packet(uint16_t *samples, uint16_t count,
                                   acurite606tx_packet_t *packet)
{
    uint16_t threshold = 80u;
    uint8_t bytes[16];
    uint8_t byte_index = 0u;
    uint8_t bit_index = 0u;
    uint8_t current = 0u;
    uint8_t last_level = 0u;
    uint32_t pulse_width = 0u;
    uint32_t samples_per_bit = 0u;

    for (uint16_t i = 0u; i < count; i++) {
        uint16_t value = acurite_abs16((int16_t)(samples[i] - (int16_t)ACURITE_BASELINE));
        uint8_t level = value > threshold ? 1u : 0u;

        if (i == 0u) {
            last_level = level;
            continue;
        }

        if (level != last_level) {
            pulse_width = (uint32_t)i;
            if (pulse_width > 0u) {
                /* Heuristic: classify pulse widths around 0.5 bit or 1 bit. This is
                 * intentionally simple and tuned to the Acurite long OOK bursts. */
                if (pulse_width < 14u) {
                    samples_per_bit = 0u;
                } else if (pulse_width < 28u) {
                    samples_per_bit = 1u;
                } else {
                    samples_per_bit = 0u;
                }

                if (samples_per_bit == 1u) {
                    current = (uint8_t)((current << 1) | 1u);
                    bit_index++;
                    if (bit_index == 8u) {
                        bytes[byte_index++] = current;
                        current = 0u;
                        bit_index = 0u;
                    }
                } else {
                    current = (uint8_t)(current << 1);
                    bit_index++;
                    if (bit_index == 8u) {
                        bytes[byte_index++] = current;
                        current = 0u;
                        bit_index = 0u;
                    }
                }
            }
            last_level = level;
        }
    }

    if (byte_index < 8u)
        return false;

    return acurite_decode_packet(bytes, byte_index, packet);
}

static void acurite_draw_screen(const char *label, const acurite606tx_packet_t *packet)
{
    const app_api_t *A = g.api;
    char text[32];

    A->display_clear();
    A->status_clear();
    A->print_tiny((char *)"ACURITE 606TX", 2u, 0u, false, true);
    A->draw_battery();

    if (packet != NULL && packet->valid) {
        const uint16_t x = 2u;
        const uint16_t y = 12u;
        text[0] = '\0';
        A->print_normal((char *)label, x, 0u, y);
        A->print_normal((char *)"ID:", x, 0u, y + 9u);
        A->print_normal((char *)"T:", x, 0u, y + 18u);
        A->print_normal((char *)"RH:", x, 0u, y + 27u);
        A->print_normal((char *)"CH:", x, 0u, y + 36u);
        A->print_tiny((char *)"RAW", 96u, 10u, false, true);
    } else {
        A->print_tiny((char *)"waiting for packet", 2u, 12u, false, true);
        A->print_tiny((char *)"433.92 MHz", 2u, 22u, false, true);
    }

    A->blit_status();
    A->blit_full();
}

static void acurite_capture_samples(void)
{
    const app_api_t *A = g.api;
    uint16_t samples[ACURITE_SCAN_COUNT];
    acurite606tx_packet_t packet = { 0 };
    uint32_t next = A->cycles_48mhz();
    uint32_t tick_period = 500u;

    for (uint16_t i = 0u; i < ACURITE_SCAN_COUNT; i++) {
        while ((int32_t)(A->cycles_48mhz() - next) < 0)
            ;
        next += tick_period;
        samples[i] = A->af_capture_sample();
    }

    if (acurite_scan_for_packet(samples, ACURITE_SCAN_COUNT, &packet)) {
        g.last = packet;
        g.last_seen_ms = A->ticks_ms();
        g.dirty = true;
    }
}

static void acurite_handle_keys(void)
{
    const app_api_t *A = g.api;
    const uint8_t key = A->get_key();

    if (key == APP_KEY_INVALID)
        return;

    A->backlight_on();
    if (key == APP_KEY_EXIT)
        g.running = false;
    else if (key == APP_KEY_2)
        g.last.valid = false;
    else if (key == APP_KEY_1) {
        /* Minimal toggling: leave the app in place but force a new refresh. */
        g.dirty = true;
    }
}

__attribute__((section(".text.entry"), used))
void app_main(const app_api_t *api)
{
    g.api = api;
    g.running = true;
    g.dirty = true;
    g.last.valid = false;

    api->af_capture_open();
    api->set_af(APP_AF_FM);
    api->backlight_on();

    while (g.running) {
        acurite_capture_samples();
        acurite_handle_keys();

        if (g.dirty) {
            acurite_draw_screen("Acurite 606TX", g.last.valid ? &g.last : NULL);
            g.dirty = false;
        }

        api->delay_ms(50u);
        api->backlight_update();
    }

    api->af_capture_close();
    api->set_af(APP_AF_MUTE);
    api->audio_path(false);
}
