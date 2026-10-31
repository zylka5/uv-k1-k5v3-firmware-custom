/*
 * Thermometer app - final v6
 * - odwrócony plus/minus dla intuicji (UP = cieplej)
 * - poprawione wyświetlanie dla wartości >=10 (2 cyfry)
 * - cfg_load / cfg_save przez api
 * - tryb pomiaru RF / OOK-ASK dla czujek meteo Acurite 606TX
 */

#include <stdint.h>
#include <stdbool.h>
#include "../app_api.h"
#include "thermometer_assets.h"

#define ADC1_ADDRESS            0x40012400u
#define ADC_SR_EOC              (1u << 1)
#define ADC_CR2_EXTTRIG         (1u << 20)
#define ADC_CR2_SWSTART         (1u << 22)
#define ADC_CR2_TSVREFE         (1u << 23)
#define ADC_CHANNEL_MASK        0x1Fu
#define ADC_CHANNEL_TEMP        16u
#define ADC_CHANNEL_VREF        17u
#define ADC_SAMPLE_16_17_MASK   (0x3Fu << 18)
#define ADC_SAMPLE_16_17_LONG   (0x3Fu << 18)

#define INPUT_TICK_MS           15u
#define TEMP_REFRESH_TICKS      33u
#define SAMPLE_COUNT            8u
#define RF_BITS_MAX             96u
#define RF_LEVEL_THRESHOLD      1500u
#define RF_PULSE_CLASSIFY       6u
#define RF_TUNE_STEP_KHZ        1000

enum {
    MODE_TEMPERATURE = 0u,
    MODE_RF = 1u,
};

struct thermo_cfg {
    int16_t offset_tenths;
    uint16_t magic;
};
#define CFG_MAGIC 0x5448

struct globals {
    const app_api_t *api;
    int16_t raw_temp;
    int16_t samples[SAMPLE_COUNT];
    uint8_t sample_idx;
    uint8_t sample_cnt;
    int16_t filtered_raw;
    int16_t comp_offset_tenths;
    int16_t ambient_tenths;
    uint16_t vdd_mv;
    bool valid;
    bool vdd_valid;
    bool show_f;
    bool running;
    bool dirty;
    bool suspended;
    bool cfg_dirty;
    uint8_t mode;
    bool rf_active;
    bool rf_monitor;
    int16_t rf_shift_khz;
    uint16_t rf_last_level;
    uint16_t rf_pulse_count;
    char rf_bits[RF_BITS_MAX + 1u];
    uint16_t rf_bit_count;
};

static struct globals g;
#define A (g.api)

static uint64_t divide(uint32_t n, uint32_t d) { return A->uidivmod(n, d); }

static bool adc_read_channel(volatile uint32_t *adc, uint8_t channel, uint16_t *sample) {
    adc[14] = (adc[14] & ~ADC_CHANNEL_MASK) | channel;
    adc[0] = ~ADC_SR_EOC;
    adc[2] |= ADC_CR2_SWSTART | ADC_CR2_EXTTRIG;
    uint16_t timeout = 0xFFFFu;
    while ((adc[0] & ADC_SR_EOC) == 0u && --timeout != 0u) {}
    if (timeout == 0u) return false;
    *sample = (uint16_t)(adc[20] & 0x0FFFu);
    return true;
}

static void cfg_load(void) {
    g.comp_offset_tenths = 85;
    struct thermo_cfg cfg;
    if (A->cfg_load) {
        A->cfg_load((uint8_t *)&cfg, sizeof(cfg));
        if (cfg.magic == CFG_MAGIC) {
            if (cfg.offset_tenths >= -50 && cfg.offset_tenths <= 200) {
                g.comp_offset_tenths = cfg.offset_tenths;
            }
        }
    }
}

static void cfg_save(void) {
    if (!A->cfg_save) return;
    struct thermo_cfg cfg;
    cfg.offset_tenths = g.comp_offset_tenths;
    cfg.magic = CFG_MAGIC;
    A->cfg_save((const uint8_t *)&cfg, sizeof(cfg));
    g.cfg_dirty = false;
}

static void refresh_temperature(void) {
    volatile uint32_t *const adc = (volatile uint32_t *)ADC1_ADDRESS;
    const uint32_t saved_cr2 = adc[2];
    const uint32_t saved_smpr2 = adc[4];
    const uint32_t saved_sqr3 = adc[14];
    uint16_t temp_sample = 0u, vref_sample = 0u;

    adc[2] = saved_cr2 | ADC_CR2_TSVREFE;
    adc[4] = (saved_smpr2 & ~ADC_SAMPLE_16_17_MASK) | ADC_SAMPLE_16_17_LONG;
    A->delay_ms(1u);

    const bool temp_ok = adc_read_channel(adc, ADC_CHANNEL_TEMP, &temp_sample);
    const bool vref_ok = adc_read_channel(adc, ADC_CHANNEL_VREF, &vref_sample);

    adc[14] = saved_sqr3;
    adc[4] = saved_smpr2;
    adc[2] = saved_cr2;

    if (!vref_ok || vref_sample == 0u) { g.vdd_valid=false; g.valid=false; return; }
    g.vdd_mv = (uint16_t)divide(4095u * 1200u, vref_sample);
    g.vdd_valid = true;
    if (!temp_ok) { g.valid=false; return; }

    const uint32_t sensor_mv = (uint32_t)divide((uint32_t)temp_sample * 1200u, vref_sample);
    int16_t t;
    if (sensor_mv >= 760u) t = (int16_t)(30u + (uint32_t)divide((sensor_mv - 760u) * 10u, 25u));
    else t = (int16_t)(30 - (int32_t)(uint32_t)divide((760u - sensor_mv) * 10u, 25u));

    g.raw_temp = t;
    g.valid = true;
    g.samples[g.sample_idx] = t;
    g.sample_idx = (g.sample_idx + 1) % SAMPLE_COUNT;
    if (g.sample_cnt < SAMPLE_COUNT) g.sample_cnt++;
    int32_t sum=0; for(uint8_t i=0;i<g.sample_cnt;i++) sum+=g.samples[i];
    g.filtered_raw = (int16_t)(sum / g.sample_cnt);
    g.ambient_tenths = (int16_t)(g.filtered_raw * 10 - g.comp_offset_tenths);
}

static void fb_set_pixel(uint8_t x, uint8_t y, bool on) {
    if (x>=128 || y>=64) return;
    uint8_t line = y>>3; uint8_t bit = y&7;
    if (on) A->fb[line][x] |= (1u<<bit);
    else A->fb[line][x] &= ~(1u<<bit);
}
static void fb_fill_rect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, bool on) {
    for(uint8_t dy=0;dy<h;dy++) for(uint8_t dx=0;dx<w;dx++) fb_set_pixel(x+dx,y+dy,on);
}

static uint8_t font_index_for_char(char c) {
    if (c >= '0' && c <= '9') return (uint8_t)(c - '0');
    if (c == '-') return 10u;
    if (c == '.') return 11u;
    if (c == 'C' || c == 'c') return 12u;
    if (c == 'F' || c == 'f') return 13u;
    return 0u;
}

static void draw_char_5x7_scaled(uint8_t x, uint8_t y, char c, uint8_t scale, bool on) {
    const uint8_t glyph_index = font_index_for_char(c);
    uint8_t col_data[5];
    if (glyph_index > 13u) return;
    A->asset_read(FONT5X7_GLYPHS + glyph_index * 5u, col_data, sizeof(col_data));
    for(uint8_t cx=0;cx<5;cx++) {
        uint8_t col=col_data[cx];
        for(uint8_t cy=0;cy<7;cy++) {
            if((col>>cy)&1) fb_fill_rect(x+cx*scale, y+cy*scale, scale, scale, on);
        }
    }
}
static uint8_t draw_big_text(uint8_t x, uint8_t y, const char *s, uint8_t scale){
    uint8_t cur=x; while(*s){ draw_char_5x7_scaled(cur,y,*s,scale,true); cur+=6*scale; s++; } return cur;
}
static void format_temp_big(char *out, int16_t tenths, bool fahrenheit){
    int16_t t=tenths; bool neg=false; if(t<0){neg=true;t=-t;}
    char *p=out; if(neg) *p++='-';
    uint16_t whole=t/10; uint16_t frac=t%10;
    if(whole>=100){*p++='0'+whole/100; whole%=100; *p++='0'+whole/10; *p++='0'+whole%10;}
    else if(whole>=10){*p++='0'+whole/10; *p++='0'+whole%10;}
    else *p++='0'+whole;
    if(frac!=0){*p++='.'; *p++='0'+frac;}
    *p++=fahrenheit?'F':'C'; *p='\0';
}

static uint32_t rf_target_freq_10hz(void) {
    return (uint32_t)((int32_t)A->rx_freq() + (int32_t)g.rf_shift_khz * 100);
}

static void rf_set_freq_from_vfo(void) {
    uint32_t freq10 = rf_target_freq_10hz();
    A->bk_write(0x38u, (uint16_t)(freq10 & 0xFFFFu));
    A->bk_write(0x39u, (uint16_t)(freq10 >> 16u));
    A->tones_off_rx();
    A->set_agc(false);
    A->audio_path(false);
    A->set_af(APP_AF_AM);
}

static void rf_enter(void) {
    g.mode = MODE_RF;
    g.rf_active = true;
    g.rf_monitor = false;
    g.rf_shift_khz = 0;
    g.rf_last_level = 0u;
    g.rf_pulse_count = 0u;
    g.rf_bit_count = 0u;
    g.rf_bits[0] = '\0';
    rf_set_freq_from_vfo();
    A->af_capture_open();
    A->set_af(APP_AF_AM);
    A->audio_path(false);
    g.dirty = true;
}

static void rf_exit(void) {
    if (g.rf_active) {
        A->af_capture_close();
        g.rf_active = false;
    }
    g.mode = MODE_TEMPERATURE;
    g.rf_monitor = false;
    A->audio_path(false);
    A->set_af(APP_AF_MUTE);
    g.dirty = true;
}

static void rf_sample_bitstream(void) {
    if (!g.rf_active) return;
    uint16_t sample = A->af_capture_sample();
    uint8_t level = (sample > RF_LEVEL_THRESHOLD) ? 1u : 0u;

    if (level != g.rf_last_level) {
        g.rf_last_level = level;
        if (g.rf_pulse_count > 1u && g.rf_pulse_count < 64u) {
            char bit = (g.rf_pulse_count > RF_PULSE_CLASSIFY) ? '1' : '0';
            if (g.rf_bit_count < RF_BITS_MAX) {
                g.rf_bits[g.rf_bit_count++] = bit;
                g.rf_bits[g.rf_bit_count] = '\0';
            }
        }
        g.rf_pulse_count = 1u;
    } else {
        g.rf_pulse_count++;
    }
}

static void format_rf_freq(char *out, uint32_t freq10hz) {
    uint32_t hz = freq10hz * 10u;
    uint32_t mhz = hz / 1000000u;
    uint32_t khz = (hz / 1000u) % 1000u;
    uint32_t h = hz % 1000u;
    char *p = out;
    *p++ = '0' + (mhz / 100u) % 10u;
    *p++ = '0' + (mhz / 10u) % 10u;
    *p++ = '0' + mhz % 10u;
    *p++ = '.';
    *p++ = '0' + khz / 100u;
    *p++ = '0' + (khz / 10u) % 10u;
    *p++ = '0' + khz % 10u;
    *p++ = '.';
    *p++ = '0' + h / 100u;
    *p++ = '0' + (h / 10u) % 10u;
    *p++ = '0' + h % 10u;
    *p = '\0';
}

static void draw_rf_debug(void) {
    char freq[16];
    char bits[RF_BITS_MAX + 1u];
    uint8_t n = 0u;
    format_rf_freq(freq, rf_target_freq_10hz());

    A->display_clear();
    A->status_clear();
    A->print_normal("RF OOK/ASK", 0, 0, 0);
    A->print_normal(freq, 0, 0, 1);
    A->print_normal(g.rf_monitor ? "MONITOR ON" : "READY", 0, 0, 2);

    for (uint16_t i = 0u; i < g.rf_bit_count && i < 32u; i++) {
        bits[n++] = g.rf_bits[i];
    }
    bits[n] = '\0';
    A->print_normal(bits[0] ? bits : "WAITING", 0, 0, 4);
    A->print_normal("UP/DN +/-1kHz 2=RF 3=MON", 0, 0, 6);
    A->draw_battery();
    A->blit_status();
    A->blit_full();
}

static void draw(void){
    char buf[16]; char small[32];
    char title[T_TITLE_LEN];
    char adc_err[T_ADC_ERR_LEN];

    if (g.mode == MODE_RF) {
        draw_rf_debug();
        return;
    }

    A->asset_read(T_TITLE, title, sizeof(title));
    A->display_clear();
    A->status_clear();
    A->print_normal(title, 0, 0, 0);
    A->draw_battery();
    A->blit_status();

    if(!g.valid){ draw_big_text(24,14,"--C",3); A->asset_read(T_ADC_ERR, adc_err, sizeof(adc_err)); A->print_normal(adc_err, 0,0,6); A->blit_full(); return; }

    int16_t disp = g.ambient_tenths;
    if(g.show_f){ int32_t f=(int32_t)g.ambient_tenths*9/5+320; disp=(int16_t)f; }

    format_temp_big(buf,disp,g.show_f);
    uint8_t len=0; while(buf[len]) len++;
    uint8_t scale = (len<=4) ? 4 : 3;
    if(len>=6) scale=3;
    uint8_t total_w = len*6*scale;
    uint8_t x0 = (128>total_w) ? (128-total_w)/2 : 0;
    uint8_t y0 = 11;
    draw_big_text(x0,y0,buf,scale);

    {
        char raw_label[T_RAW_LEN];
        char cmp_label[T_CMP_LEN];
        A->asset_read(T_RAW, raw_label, sizeof(raw_label));
        A->asset_read(T_CMP, cmp_label, sizeof(cmp_label));
        char *o=small;
        for(const char *p = raw_label; *p; p++) *o++ = *p;
        int16_t raw=g.filtered_raw;
        bool neg=false;
        if(raw<0){ neg=true; raw=-raw; }
        if(neg) *o++='-';
        if(raw>=10) *o++='0'+(raw/10);
        *o++='0'+(raw%10);
        *o++='C';
        *o++=' ';
        for(const char *p = cmp_label; *p; p++) *o++ = *p;
        int16_t disp_off = -g.comp_offset_tenths;
        if(disp_off==0){
            *o++='0'; *o++='.'; *o++='0';
        } else {
            if(disp_off<0){ *o++='-'; disp_off=-disp_off; }
            else { *o++='+'; }
            int16_t whole = disp_off/10;
            int16_t frac = disp_off%10;
            if(whole>=10){
                *o++='0'+(whole/10);
                *o++='0'+(whole%10);
            } else {
                *o++='0'+whole;
            }
            *o++='.';
            *o++='0'+frac;
        }
        *o='\0';
        A->print_normal(small,0,0,5);
    }
    {
        char vdd_label[T_VDD_LEN];
        A->asset_read(T_VDD, vdd_label, sizeof(vdd_label));
        char *o=small;
        for(const char *p = vdd_label; *p; p++) *o++ = *p;
        uint16_t v=g.vdd_mv;
        *o++='0'+v/1000; *o++='.'; *o++='0'+(v%1000)/100; *o++='0'+(v%100)/10; *o++='V'; *o++=' ';
        if(A->sys_battery_voltage){
            uint16_t bv = *A->sys_battery_voltage;
            *o++='B';*o++=':'; *o++='0'+(bv/1000)%10; *o++='.'; *o++='0'+(bv%1000)/100; *o++='V';
        }
        *o='\0';
        A->print_normal(small,0,0,6);
    }
    A->blit_full();
}

__attribute__((section(".text.entry"), used))
void app_main(const app_api_t *api){
    g.api=api;
    g.sample_idx=0; g.sample_cnt=0; g.show_f=false; g.running=true; g.dirty=true; g.suspended=false; g.cfg_dirty=false;
    g.mode = MODE_TEMPERATURE;
    g.rf_active=false; g.rf_monitor=false; g.rf_shift_khz=0; g.rf_last_level=0u; g.rf_pulse_count=0u; g.rf_bit_count=0u; g.rf_bits[0]='\0';
    for(uint8_t i=0;i<SAMPLE_COUNT;i++) g.samples[i]=0;

    cfg_load();

    A->backlight_on();
    refresh_temperature();
    draw();

    uint8_t previous = APP_KEY_INVALID;
    uint8_t tick_cnt=0;
    uint16_t repeat_ms=0;
    uint16_t save_delay=0;

    while(g.running){
        const uint8_t key = A->get_key();
        if(key==APP_KEY_SAVER){ previous=APP_KEY_INVALID; repeat_ms=0; g.suspended=true; }
        else if(key==APP_KEY_WAKE){ previous=APP_KEY_INVALID; repeat_ms=0; g.suspended=false; refresh_temperature(); g.dirty=true; tick_cnt=0; }
        else if(key==APP_KEY_INVALID){ previous=key; repeat_ms=0; }
        else if(key!=previous){
            A->backlight_on();
            if(key==APP_KEY_2){
                if(g.mode == MODE_TEMPERATURE) rf_enter();
                else rf_exit();
            } else if(key==APP_KEY_3 && g.rf_active){
                g.rf_monitor = !g.rf_monitor;
                A->audio_path(g.rf_monitor);
                g.dirty = true;
            } else if(key==APP_KEY_UP){
                if(g.mode == MODE_RF){
                    g.rf_shift_khz += 1;
                    rf_set_freq_from_vfo();
                    g.rf_bit_count = 0u;
                    g.rf_bits[0] = '\0';
                    g.dirty = true;
                } else {
                    g.comp_offset_tenths-=5;
                    if(g.comp_offset_tenths<-50) g.comp_offset_tenths=-50;
                    if(g.comp_offset_tenths>200) g.comp_offset_tenths=200;
                    g.ambient_tenths=g.filtered_raw*10-g.comp_offset_tenths;
                    g.dirty=true; g.cfg_dirty=true; save_delay=0;
                }
            } else if(key==APP_KEY_DOWN){
                if(g.mode == MODE_RF){
                    g.rf_shift_khz -= 1;
                    rf_set_freq_from_vfo();
                    g.rf_bit_count = 0u;
                    g.rf_bits[0] = '\0';
                    g.dirty = true;
                } else {
                    g.comp_offset_tenths+=5;
                    if(g.comp_offset_tenths>200) g.comp_offset_tenths=200;
                    if(g.comp_offset_tenths<-50) g.comp_offset_tenths=-50;
                    g.ambient_tenths=g.filtered_raw*10-g.comp_offset_tenths;
                    g.dirty=true; g.cfg_dirty=true; save_delay=0;
                }
            } else if(key==APP_KEY_EXIT){
                if(g.mode == MODE_RF) rf_exit();
                if(g.cfg_dirty) cfg_save();
                g.running=false;
            } else if(key==APP_KEY_MENU){
                if(g.mode == MODE_TEMPERATURE){ g.show_f=!g.show_f; g.dirty=true; }
            } else {
                if(g.mode == MODE_TEMPERATURE) A->play_tone(500,60);
            }
            previous=key;
        } else {
            if(g.mode == MODE_TEMPERATURE && (key==APP_KEY_UP || key==APP_KEY_DOWN)){
                if(repeat_ms>200){
                    repeat_ms=0;
                    if(key==APP_KEY_UP) g.comp_offset_tenths-=5; else g.comp_offset_tenths+=5;
                    if(g.comp_offset_tenths>200) g.comp_offset_tenths=200;
                    if(g.comp_offset_tenths<-50) g.comp_offset_tenths=-50;
                    g.ambient_tenths=g.filtered_raw*10-g.comp_offset_tenths;
                    g.dirty=true; g.cfg_dirty=true;
                }
            }
            if(g.mode == MODE_RF && (key==APP_KEY_UP || key==APP_KEY_DOWN)){
                if(repeat_ms>150){
                    repeat_ms=0;
                    if(key==APP_KEY_UP) g.rf_shift_khz += 1; else g.rf_shift_khz -= 1;
                    rf_set_freq_from_vfo();
                    g.rf_bit_count = 0u;
                    g.rf_bits[0] = '\0';
                    g.dirty=true;
                }
            }
        }

        if(g.mode == MODE_RF){
            rf_sample_bitstream();
            if(g.rf_monitor) {
                A->audio_path(true);
                A->set_af(APP_AF_AM);
            }
        }

        if(!g.suspended && g.dirty){ draw(); g.dirty=false; }

        A->delay_ms(INPUT_TICK_MS);
        repeat_ms+=INPUT_TICK_MS;

        if(!g.suspended){
            tick_cnt++;
            if(g.mode == MODE_TEMPERATURE && tick_cnt>=TEMP_REFRESH_TICKS){ tick_cnt=0; refresh_temperature(); g.dirty=true; }
            if(g.cfg_dirty){
                save_delay+=INPUT_TICK_MS;
                if(save_delay>2000){ cfg_save(); save_delay=0; }
            }
        }
        A->backlight_update();
    }
}
