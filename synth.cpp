/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <math.h>

#include "hardware/pll.h"
#include "hardware/clocks.h"
#include "hardware/structs/clocks.h"
#include "hardware/pio.h"
#include "hardware/adc.h"
#include "pico/stdlib.h"
#include "pico/audio.h"
#include "pico/audio_i2s.h"

#define SINE_WAVE_TABLE_LEN 2048
#define OCTAVES_COUNT 4             // Nombre d'octaves
#define NOTES_PER_OCTAVE 12         // 12 demi-tons par octave
#define FREQUENCY_TABLE_LEN (OCTAVES_COUNT * NOTES_PER_OCTAVE)  // 48 notes au total
#define SAMPLES_PER_BUFFER 1156 // Samples / channel
const uint LED_PIN = 25;
const uint LED0_PIN = 20;
const uint LED1_PIN = 21;
const uint LED2_PIN = 22;
const uint LED7_PIN = 19;
const uint LED6_PIN = 15;
const uint LED5_PIN = 14;
const uint LED4_PIN = 13;
const uint LED3_PIN = 12;
uint leds_pins[8] = {LED0_PIN, LED1_PIN, LED2_PIN, LED3_PIN, LED4_PIN, LED5_PIN, LED6_PIN, LED7_PIN};


const uint BUTTON0_PIN = 3;
const uint BUTTON1_PIN = 2;
const uint BUTTON2_PIN = 5;
const uint BUTTON3_PIN = 4;
const uint BUTTON4_PIN = 9;
const uint BUTTON5_PIN = 6;
const uint BUTTON6_PIN = 7;
const uint BUTTON7_PIN = 8;
uint buttons_pins[8] = {BUTTON0_PIN, BUTTON1_PIN, BUTTON2_PIN, BUTTON3_PIN, BUTTON4_PIN, BUTTON5_PIN, BUTTON6_PIN, BUTTON7_PIN};

const uint TONE_GPIO = 27; // GPIO pour l'entrée ADC
const uint VOL_GPIO = 26; // GPIO pour l'entrée ADC

static const uint32_t PIN_DCDC_PSM_CTRL = 23;

audio_buffer_pool_t *ap;
static bool decode_flg = false;
static constexpr int32_t DAC_ZERO = 1;

#define audio_pio __CONCAT(pio, PICO_AUDIO_I2S_PIO)

static audio_format_t audio_format = {
    .sample_freq = 44100,
    .pcm_format = AUDIO_PCM_FORMAT_S32,
    .channel_count = AUDIO_CHANNEL_STEREO
};

static audio_buffer_format_t producer_format = {
    .format = &audio_format,
    .sample_stride = 8
};

static audio_i2s_config_t i2s_config = {
    .data_pin = PICO_AUDIO_I2S_DATA_PIN,
    .clock_pin_base = PICO_AUDIO_I2S_CLOCK_PIN_BASE,
    .dma_channel0 = 0,
    .dma_channel1 = 1,
    .pio_sm = 0
};

static int16_t sine_wave_table[SINE_WAVE_TABLE_LEN];

// Tableau des valeurs de step pour les notes musicales (4 octaves × 12 demi-tons = 48 notes)
// Index 0 = LA1 (110 Hz), Index 24 = LA4 (440 Hz), Index 47 = SOL#4 (831 Hz)
uint32_t frequency_table[FREQUENCY_TABLE_LEN];

// Noms des notes (12 demi-tons)
const char* note_names[NOTES_PER_OCTAVE] = {
    "LA", "LA#", "SI", "DO", "DO#", "RE", "RE#", "MI", "FA", "FA#", "SOL", "SOL#"
};

uint32_t step0 = 0x146A81;  // LA 440 Hz (valeur expérimentale)  
uint32_t step1 = 0x146A81;  // LA 440 Hz (valeur expérimentale)
int freq_index0 = 24;  // Index 24 = LA4 (440 Hz) - milieu de la gamme
int freq_index1 = 24;  // Index 24 = LA4 (440 Hz) - milieu de la gamme
uint32_t pos0 = 0;
uint32_t pos1 = 0;
const uint32_t pos_max = 0x10000 * SINE_WAVE_TABLE_LEN;
uint vol = 20;

#if 0
audio_buffer_pool_t *init_audio() {

    static audio_format_t audio_format = {
        .pcm_format = AUDIO_PCM_FORMAT_S32,
        .sample_freq = 44100,
        .channel_count = 2
    };

    static audio_buffer_format_t producer_format = {
        .format = &audio_format,
        .sample_stride = 8
    };

    audio_buffer_pool_t *producer_pool = audio_new_producer_pool(&producer_format, 3,
                                                                      SAMPLES_PER_BUFFER); // todo correct size
    bool __unused ok;
    const audio_format_t *output_format;
#if USE_AUDIO_I2S
    audio_i2s_config_t config = {
        .data_pin = PICO_AUDIO_I2S_DATA_PIN,
        .clock_pin_base = PICO_AUDIO_I2S_CLOCK_PIN_BASE,
        .dma_channel = 0,
        .pio_sm = 0
    };

    output_format = audio_i2s_setup(&audio_format, &audio_format, &config);
    if (!output_format) {
        panic("PicoAudio: Unable to open audio device.\n");
    }

    ok = audio_i2s_connect(producer_pool);
    assert(ok);
    { // initial buffer data
        audio_buffer_t *buffer = take_audio_buffer(producer_pool, true);
        int32_t *samples = (int32_t *) buffer->buffer->bytes;
        for (uint i = 0; i < buffer->max_sample_count; i++) {
            samples[i*2+0] = 0;
            samples[i*2+1] = 0;
        }
        buffer->sample_count = buffer->max_sample_count;
        give_audio_buffer(producer_pool, buffer);
    }
    audio_i2s_set_enabled(true);
#elif USE_AUDIO_PWM
    output_format = audio_pwm_setup(&audio_format, -1, &default_mono_channel_config);
    if (!output_format) {
        panic("PicoAudio: Unable to open audio device.\n");
    }
    ok = audio_pwm_default_connect(producer_pool, false);
    assert(ok);
    audio_pwm_set_enabled(true);
#elif USE_AUDIO_SPDIF
    output_format = audio_spdif_setup(&audio_format, &audio_spdif_default_config);
    if (!output_format) {
        panic("PicoAudio: Unable to open audio device.\n");
    }
    //ok = audio_spdif_connect(producer_pool);
    ok = audio_spdif_connect(producer_pool);
    assert(ok);
    audio_spdif_set_enabled(true);
#endif
    return producer_pool;
}
#endif


static inline uint32_t _millis(void)
{
	return to_ms_since_boot(get_absolute_time());
}

// Fonction pour obtenir la valeur step à partir d'un index (0 à FREQUENCY_TABLE_LEN-1)
uint32_t get_frequency_step(int index) {
    if (index < 0) index = 0;
    if (index >= FREQUENCY_TABLE_LEN) index = FREQUENCY_TABLE_LEN - 1;
    return frequency_table[index];
}

// Fonction pour obtenir le nom de la note à partir de l'index
const char* get_note_name(int index, int* octave) {
    if (index < 0) index = 0;
    if (index >= FREQUENCY_TABLE_LEN) index = FREQUENCY_TABLE_LEN - 1;
    
    int note = index % NOTES_PER_OCTAVE;
    *octave = 1 + (index / NOTES_PER_OCTAVE);  // Octave 1 à 4
    
    return note_names[note];
}

// Fonction pour obtenir l'index correspondant à une fréquence approximative  
int get_frequency_index(float target_freq) {
    // Recherche la note la plus proche dans le tableau
    float min_diff = 10000.0f;
    int best_index = 24; // Par défaut LA4
    
    for (int i = 0; i < FREQUENCY_TABLE_LEN; i++) {
        // Reconvertir le step en fréquence pour comparaison
        float freq = (frequency_table[i] * 44100.0f) / (0x10000 * SINE_WAVE_TABLE_LEN);
        float diff = fabsf(freq - target_freq);
        if (diff < min_diff) {
            min_diff = diff;
            best_index = i;
        }
    }
    
    return best_index;
}

void i2s_audio_deinit()
{
    decode_flg = false;

    audio_i2s_set_enabled(false);
    audio_i2s_end();

    audio_buffer_t* ab;
    ab = take_audio_buffer(ap, false);
    while (ab != nullptr) {
        free(ab->buffer->bytes);
        free(ab->buffer);
        ab = take_audio_buffer(ap, false);
    }
    ab = get_free_audio_buffer(ap, false);
    while (ab != nullptr) {
        free(ab->buffer->bytes);
        free(ab->buffer);
        ab = get_free_audio_buffer(ap, false);
    }
    ab = get_full_audio_buffer(ap, false);
    while (ab != nullptr) {
        free(ab->buffer->bytes);
        free(ab->buffer);
        ab = get_full_audio_buffer(ap, false);
    }
    free(ap);
    ap = nullptr;
}

audio_buffer_pool_t *i2s_audio_init(uint32_t sample_freq)
{
    audio_format.sample_freq = sample_freq;

    audio_buffer_pool_t *producer_pool = audio_new_producer_pool(&producer_format, 3, SAMPLES_PER_BUFFER);
    ap = producer_pool;

    bool __unused ok;
    const audio_format_t *output_format;

    output_format = audio_i2s_setup(&audio_format, &audio_format, &i2s_config);
    if (!output_format) {
        panic("PicoAudio: Unable to open audio device.\n");
    }

    ok = audio_i2s_connect(producer_pool);
    assert(ok);
    { // initial buffer data
        audio_buffer_t *ab = take_audio_buffer(producer_pool, true);
        int32_t *samples = (int32_t *) ab->buffer->bytes;
        for (uint i = 0; i < ab->max_sample_count; i++) {
            samples[i*2+0] = DAC_ZERO;
            samples[i*2+1] = DAC_ZERO;
        }
        ab->sample_count = ab->max_sample_count;
        give_audio_buffer(producer_pool, ab);
    }
    audio_i2s_set_enabled(true);

    decode_flg = true;
    return producer_pool;
}

int main() {

    stdio_init_all();

    // Set PLL_USB 96MHz
    pll_init(pll_usb, 1, 1536 * MHZ, 4, 4);
    clock_configure(clk_usb,
        0,
        CLOCKS_CLK_USB_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB,
        96 * MHZ,
        48 * MHZ);
    // Change clk_sys to be 96MHz.
    clock_configure(clk_sys,
        CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX,
        CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB,
        96 * MHZ,
        96 * MHZ);
    // CLK peri is clocked from clk_sys so need to change clk_peri's freq
    clock_configure(clk_peri,
        0,
        CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLK_SYS,
        96 * MHZ,
        96 * MHZ);
    // Reinit uart now that clk_peri has changed
    stdio_init_all();
    adc_init();
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    for (int i = 0; i < 8; i++) {
        gpio_init(leds_pins[i]);
        gpio_set_dir(leds_pins[i], GPIO_OUT);
    }
    for (int i = 0; i < 8; i++) {
        gpio_init(buttons_pins[i]);
        gpio_set_dir(buttons_pins[i], GPIO_IN);
        gpio_pull_up(buttons_pins[i]);
    }

    adc_gpio_init(TONE_GPIO);
    adc_gpio_init(VOL_GPIO);

    // DCDC PSM control
    // 0: PFM mode (best efficiency)
    // 1: PWM mode (improved ripple)
    gpio_init(PIN_DCDC_PSM_CTRL);
    gpio_set_dir(PIN_DCDC_PSM_CTRL, GPIO_OUT);
    gpio_put(PIN_DCDC_PSM_CTRL, 1); // PWM mode for less Audio noise

    //2048 samples 
    
    for (int i = 0; i < SINE_WAVE_TABLE_LEN; i++) {
        sine_wave_table[i] = 32767 * cosf(i * 2 * (float) (M_PI / SINE_WAVE_TABLE_LEN));
    }

    // Initialisation du tableau des fréquences musicales (gamme tempérée)
    // 4 octaves × 12 demi-tons = 48 notes de LA1 (110Hz) à SOL#4 (831Hz)
    // Formule: freq = 440 * 2^((note_index - 24) / 12)
    // Index 0=LA1, 12=LA2, 24=LA3(440Hz), 36=LA4, 47=SOL#4
    
    for (int i = 0; i < FREQUENCY_TABLE_LEN; i++) {
        // Calcul de la fréquence pour cette note (gamme tempérée égale)
        // LA4 (440Hz) est à l'index 24, donc décalage de -24
        float semitone_offset = (float)(i - 24);  // Offset par rapport au LA4 (440Hz)
        float frequency = 440.0f * powf(2.0f, semitone_offset / 12.0f);
        
        // Conversion en valeur step
        frequency_table[i] = (uint32_t)((frequency * 0x10000 * SINE_WAVE_TABLE_LEN) / 44100.0f);
    }

    ap = i2s_audio_init(44100);

    int time = 0;
    while (true) {
        int c = getchar_timeout_us(100000);
        if (c >= 0 && c != PICO_ERROR_TIMEOUT) {
            if (c == '-' && vol) vol--;
            if ((c == '=' || c == '+') && vol < 256) vol++;
            
            // Contrôles fins (anciens)
            if (c == '[' && step0 > 0x10000) step0 -= 0x10000;
            if (c == ']' && step0 < (SINE_WAVE_TABLE_LEN / 16) * 0x20000) step0 += 0x10000;
            if (c == '{' && step1 > 0x10000) step1 -= 0x10000;
            if (c == '}' && step1 < (SINE_WAVE_TABLE_LEN / 16) * 0x20000) step1 += 0x10000;
            
            // Nouveaux contrôles avec le tableau de fréquences
            if (c == 'a' && freq_index0 > 0) {
                freq_index0--;
                step0 = get_frequency_step(freq_index0);
            }
            if (c == 's' && freq_index0 < FREQUENCY_TABLE_LEN - 1) {
                freq_index0++;
                step0 = get_frequency_step(freq_index0);
            }
            if (c == 'z' && freq_index1 > 0) {
                freq_index1--;
                step1 = get_frequency_step(freq_index1);
            }
            if (c == 'x' && freq_index1 < FREQUENCY_TABLE_LEN - 1) {
                freq_index1++;
                step1 = get_frequency_step(freq_index1);
            }
            
            // Saut d'octave (12 demi-tons = une octave)
            if (c == 'A' && freq_index0 >= NOTES_PER_OCTAVE) {
                freq_index0 -= NOTES_PER_OCTAVE;  // -1 octave
                step0 = get_frequency_step(freq_index0);
            }
            if (c == 'S' && freq_index0 < FREQUENCY_TABLE_LEN - NOTES_PER_OCTAVE) {
                freq_index0 += NOTES_PER_OCTAVE;  // +1 octave
                step0 = get_frequency_step(freq_index0);
            }
            if (c == 'Z' && freq_index1 >= NOTES_PER_OCTAVE) {
                freq_index1 -= NOTES_PER_OCTAVE;  // -1 octave canal droit
                step1 = get_frequency_step(freq_index1);
            }
            if (c == 'X' && freq_index1 < FREQUENCY_TABLE_LEN - NOTES_PER_OCTAVE) {
                freq_index1 += NOTES_PER_OCTAVE;  // +1 octave canal droit
                step1 = get_frequency_step(freq_index1);
            }
            
            if (c == 'q') break;
            
            // Affichage des notes avec noms
            int octave0, octave1;
            const char* note0 = get_note_name(freq_index0, &octave0);
            const char* note1 = get_note_name(freq_index1, &octave1);
            
            printf("vol=%d, L:%s%d(%d) R:%s%d(%d)    \r", 
                   static_cast<int>(vol), 
                   note0, octave0, static_cast<int>(step0 >> 16),
                   note1, octave1, static_cast<int>(step1 >> 16));
        } else {
            int pushed_button = -1;
            for (int i = 0; i < 8; i++) {
                if (gpio_get(buttons_pins[i]) == 0) { // Bouton appuyé (logique inversée avec pull-up)
                    pushed_button = i;
                    printf("button %d\n", pushed_button);
                    break;
                }
            }
            for(int i = 0; i < 8; i++) {
                gpio_put(leds_pins[i], 0);
            }
            if (pushed_button >= 0) {
                time = pushed_button;
            }

            gpio_put(leds_pins[time], 1);
            
            sleep_ms(10);

            uint start = _millis();
            uint now = start;
            while (now - start < 200) {
                adc_select_input(0); // TONE_GPIO
                uint tone = adc_read();
                //printf("Tone ADC: %d\n", tone);
                freq_index0 = get_frequency_index((tone / 4095.0f) * 1000.0f + 400); // 0-1000Hz
                step0 = get_frequency_step(freq_index0);
                now = _millis();
            }
        }
        time++;
        if(time == 8) {
            time = 0;
        }
    }
    puts("\n");
    return 0;
}

void decode()
{
    audio_buffer_t *buffer = take_audio_buffer(ap, false);
    if (buffer == NULL) { return; }
    int32_t *samples = (int32_t *) buffer->buffer->bytes;
    for (uint i = 0; i < buffer->max_sample_count; i++) {
        int32_t value0 = (vol * sine_wave_table[pos0 >> 16u]) << 8u;
        int32_t value1 = (vol * sine_wave_table[pos1 >> 16u]) << 8u;
        // use 32bit full scale
        samples[i*2+0] = value0 + (value0 >> 16u);  // L
        samples[i*2+1] = value1 + (value1 >> 16u);  // R
        pos0 += step0;
        pos1 += step1;
        if (pos0 >= pos_max) pos0 -= pos_max;
        if (pos1 >= pos_max) pos1 -= pos_max;
    }
    buffer->sample_count = buffer->max_sample_count;
    give_audio_buffer(ap, buffer);
    return;
}

extern "C" {
// callback from:
//   void __isr __time_critical_func(audio_i2s_dma_irq_handler)()
//   defined at my_pico_audio_i2s/audio_i2s.c
//   where i2s_callback_func() is declared with __attribute__((weak))
void i2s_callback_func()
{
    if (decode_flg) {
        decode();
    }
}
}