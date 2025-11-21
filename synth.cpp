/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <assert.h>
#include <vector>

// Prototypes explicites pour calmer l'analyseur si les includes ne sont pas résolus
extern "C" int printf(const char *format, ...);
extern "C" int puts(const char *s);
extern "C" void free(void *ptr);
extern "C" float powf(float, float);
extern "C" float cosf(float);
extern "C" float fabsf(float);

// Assurer disponibilité des fonctions/f constantes math si certains define manquent
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#ifndef M_E
#define M_E 2.71828182845904523536
#endif

#if !defined(__cplusplus)
#error Ce fichier nécessite un compilateur C++ pour les surcharges.
#endif

#include "hardware/pll.h"
#include "hardware/clocks.h"
#include "hardware/structs/clocks.h"
#include "hardware/pio.h"
#include "hardware/adc.h"
#include "pico/stdlib.h"
#include "pico/audio.h"
#include "pico/audio_i2s.h"

#define SINE_WAVE_TABLE_LEN 2048
#define OCTAVES_COUNT 2             // Nombre d'octaves
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
uint adc_mean[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
const uint TONE_GPIO = 27; // GPIO pour l'entrée ADC
const uint VOL_GPIO = 26; // GPIO pour l'entrée ADC

static const uint32_t PIN_DCDC_PSM_CTRL = 23;

uint analog_select_pins[4] = {28,27,10,11};

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
// Per-note gain compensation (Q16 fixed) to boost low frequencies for perceived loudness
uint32_t frequency_gain_q16[FREQUENCY_TABLE_LEN];

// Noms des notes (12 demi-tons)
const char* note_names[NOTES_PER_OCTAVE] = {
    "LA", "LA#", "SI", "DO", "DO#", "RE", "RE#", "MI", "FA", "FA#", "SOL", "SOL#"
};

// -------- Polyphonie dynamique --------
static const int MAX_VOICES = 7; // capacité simultanée maximale
const uint32_t pos_max = 0x10000 * SINE_WAVE_TABLE_LEN;
uint vol = 30;                       // volume global simple
int current_button = 0;              // bouton sélectionné pour réglages
static int output_gain_shift = 6;    // gain post-mix
int base_freq_index[8] = {24,24,24,24,24,24,24,24}; // note de base par bouton

// ---------------- Envelope (Attack / Release) ----------------
// Attack et Release en millisecondes (modifiables via clavier)
static uint32_t attack_ms = 20;     // temps d'attaque par défaut
static uint32_t release_ms = 200;   // temps de relâchement par défaut
static uint32_t decay_ms   = 100;   // temps de decay (descente vers sustain)
static uint32_t sustain_ms = 10; // durée sustain (0..65536)
static uint32_t sustain_level = 48000; // niveau sustain (0..65536)
static uint32_t speed_tuning = 500; // temps entre 2 notes
static uint32_t sequence = 0; // index de la note en cours dans la sequence

enum EnvState { ENV_IDLE, ENV_ATTACK, ENV_DECAY, ENV_SUSTAIN, ENV_RELEASE };
struct Envelope {
    EnvState state;
    uint32_t pos;            // position dans la phase courante (samples)
    uint32_t attack_samples; // durée d'attaque en samples
    uint32_t decay_samples;  // durée de decay en samples
    uint32_t sustain_samples; // durée de sustain en samples
    uint32_t release_samples;// durée de release en samples
    uint32_t level;          // niveau courant 0..65536
    uint32_t release_start_level; // niveau de départ pour la phase de release
    uint32_t sustain_target; // niveau cible pour sustain (copie de sustain_level)
};

// Collection dynamique de voix (remplace le tableau env[])
struct Voice {
    uint32_t step() 
    {
        freq_index = base_freq_index[button_id];
        return frequency_table[freq_index];
    }
    uint32_t freq_index;
    uint32_t position;
    int button_id; // 0..7 si déclenché par bouton, -1 si via clavier
    Envelope env;
};
static std::vector<Voice> active_voices; // voix actives

// Facteur de courbure logarithmique (>1) : plus grand => attaque plus douce au début
static float curve_factor = 9.0f; // utilisée dans log(1 + t*curve_factor)/log(1+curve_factor)
static bool use_log_curve = true; // permet d'activer/désactiver le lissage log

static inline uint32_t _millis(void)
{
	return to_ms_since_boot(get_absolute_time());
}

static inline void envelope_recalc(Envelope &e) {
    e.attack_samples  = (attack_ms  * audio_format.sample_freq) / 1000;
    e.decay_samples   = (decay_ms   * audio_format.sample_freq) / 1000;
    e.release_samples = (release_ms * audio_format.sample_freq) / 1000;
    e.sustain_samples = (sustain_ms * audio_format.sample_freq) / 1000;
    if (e.state == ENV_ATTACK && e.attack_samples == 0) {
        e.level = 65536; e.state = (e.decay_samples ? ENV_DECAY : ENV_SUSTAIN); e.pos = 0;
    }
    if (e.state == ENV_RELEASE && e.release_samples == 0) {
        e.level = 0; e.state = ENV_IDLE; e.pos = 0;
    }
}

float logmap(float x, float min, float max) {
    return min * powf(max / min, x);   // x in [0,1] maps to [min,max] loga   
}

static inline void envelope_recalc_all() { for(auto &v : active_voices) envelope_recalc(v.env); }

static void start_voice(int button_id) {
    if ((int)active_voices.size() >= MAX_VOICES) {
        // recycle voix en release ou idle si possible
        for (size_t i=0;i<active_voices.size();++i) {
            if (active_voices[i].env.state == ENV_RELEASE || active_voices[i].env.state == ENV_IDLE) {
                active_voices.erase(active_voices.begin()+i);
                break;
            }
        }
        if ((int)active_voices.size() >= MAX_VOICES) {
            active_voices.erase(active_voices.begin()); // fallback: retirer la plus ancienne
        }
    }
    Voice v; v.freq_index = base_freq_index[button_id]; v.position = 0; v.button_id = button_id;
    v.env.state = ENV_ATTACK; v.env.pos = 0; v.env.level = 0; v.env.sustain_target = sustain_level; envelope_recalc(v.env);
    active_voices.push_back(v);
}

static inline void envelope_trigger_attack(Envelope &e) {
    e.state = ENV_ATTACK; e.pos = 0; if (e.attack_samples == 0) { e.level = 65536; e.state = ENV_SUSTAIN; }
}
static inline void envelope_trigger_release(Envelope &e) {
    if (e.state == ENV_IDLE) return; 
    // Impose une longueur minimale pour éviter coupure abrupte (click)
    const uint32_t MIN_RELEASE_SAMPLES = 128; // ~2.9ms
    if (e.release_samples < MIN_RELEASE_SAMPLES) e.release_samples = MIN_RELEASE_SAMPLES;
    e.release_start_level = e.level; // mémorise le niveau atteint (attaque potentiellement incomplète)
    e.state = ENV_RELEASE; e.pos = 0; if (e.release_samples == 0) { e.level = 0; e.state = ENV_IDLE; }
}

static void release_voices_for_button(int button_id) {
    for (auto &v : active_voices) {
        if (v.button_id == button_id && v.env.state != ENV_RELEASE && v.env.state != ENV_IDLE) {
            envelope_trigger_release(v.env);
        }
    }
}

static int voices_for_button(int button_id) {
    int i = 0;
    for (auto &v : active_voices) {
        if (v.button_id == button_id && v.env.state != ENV_IDLE) {
            i++;
        }
    }
    return i;
}

auto start_time = 0;

static inline void envelope_step(Envelope &e) {
    switch (e.state) {
        case ENV_IDLE: e.level = 0; break;
        case ENV_ATTACK:
            if (e.attack_samples == 0) { e.level = 65536; e.state = ENV_SUSTAIN; e.pos = 0; break; }
            if (e.pos >= e.attack_samples) { e.level = 65536; e.state = (e.decay_samples ? ENV_DECAY : ENV_SUSTAIN); e.pos = 0;
                //printf("Attack complete %u\n",_millis() - start_time); 
                break; }
            if (use_log_curve) {
                float t = (float)e.pos / (float)e.attack_samples; // 0..1
                // Approximation rationnelle douce (évite logf/powf coûteux):
                // shaped = t / (t + k*(1-t)) où k lié à curve_factor
                float k = curve_factor; // >1 rend début plus lent
                float denom = t + k * (1.0f - t);
                float shaped = (denom > 0.0f) ? (t / denom) : t; // 0..1
                e.level = (uint32_t)(shaped * 65536.0f);
            } else {
                e.level = (uint64_t)65536 * e.pos / e.attack_samples; // linéaire
            }
            ++e.pos;
            break;
        case ENV_DECAY:
            if (e.decay_samples == 0) { e.state = ENV_SUSTAIN; e.pos = 0; e.level = e.sustain_target; break; }
            if (e.pos >= e.decay_samples) { e.level = e.sustain_target; e.state = ENV_SUSTAIN; e.pos = 0; break; }
            if (use_log_curve) {
                float t = (float)e.pos / (float)e.decay_samples; // 0..1
                float k = curve_factor;
                float denom = t + k * (1.0f - t);
                float shaped_up = (denom > 0.0f) ? (t / denom) : t; // 0..1 increasing
                float shaped_down = 1.0f - shaped_up; // 1 -> 0
                float span = (float)(65536 - e.sustain_target);
                e.level = (uint32_t)(e.sustain_target + shaped_down * span);
            } else {
                uint64_t span = 65536 - e.sustain_target;
                e.level = (uint32_t)(e.sustain_target + (span * (uint64_t)(e.decay_samples - e.pos)) / e.decay_samples);
            }
            ++e.pos;
            break;
        case ENV_SUSTAIN:
            if (e.pos >= e.sustain_samples) { e.state = ENV_RELEASE; e.pos = 0; e.release_start_level = e.level; break; }
            ++e.pos;
            break;
        case ENV_RELEASE:
            if (e.release_samples == 0) { e.level = 0; e.state = ENV_IDLE; e.pos = 0; break; }
            if (e.pos >= e.release_samples) { e.level = 0; e.state = ENV_IDLE; e.pos = 0; break; }
            if (use_log_curve) {
                float t = (float)e.pos / (float)e.release_samples; // 0..1
                float k = curve_factor;
                // Utiliser même forme mais inversée: attackShape(t) = t/(t+k*(1-t)); release = 1 - attackShape(t)
                float denom = t + k * (1.0f - t);
                float base = (denom > 0.0f) ? (t / denom) : t;
                float shaped = 1.0f - base;
                e.level = (uint32_t)(shaped * (float)e.release_start_level);
            } else {
                e.level = (uint64_t)e.release_start_level * ( (uint64_t)(e.release_samples - e.pos) ) / e.release_samples; // linéaire depuis niveau courant
            }
            ++e.pos;
            break;
    }
}

// --------------------------------------------------------------

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

    bool ok;
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

uint16_t adc_read(int button) {
    static int buttonToAnalogPin[16] = {
//pitch pot
        14,
        0,
        2,
        4,
        6,
        8,
        10,
        12,
// tunning pot
        15,
        1,
        3,
        5,
        7,
        9,
        11,
        13
    };
    uint pin = buttonToAnalogPin[button];
    gpio_put(analog_select_pins[0], pin & 0x01);
    gpio_put(analog_select_pins[1], pin & 0x02);
    gpio_put(analog_select_pins[2], pin & 0x04);
    gpio_put(analog_select_pins[3], pin & 0x08);
    sleep_us(100);
    return adc_read();
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

    for (int i = 0; i < 4; i++) {
        gpio_init(analog_select_pins[i]);
        gpio_set_dir(analog_select_pins[i], GPIO_OUT);
    }

    adc_gpio_init(26);
    adc_select_input(0);
    // DCDC PSM control
    // 0: PFM mode (best efficiency)
    // 1: PWM mode (improved ripple)
    gpio_init(PIN_DCDC_PSM_CTRL);
    gpio_set_dir(PIN_DCDC_PSM_CTRL, GPIO_OUT);
    gpio_put(PIN_DCDC_PSM_CTRL, 1); // PWM mode for less Audio noise

    //2048 samples 
    
    #ifndef M_PI
    #define M_PI 3.14159265358979323846
    #endif
    for (int i = 0; i < SINE_WAVE_TABLE_LEN; i++) {
        sine_wave_table[i] = 32767 * cosf(i * 2 * (float) (M_PI / SINE_WAVE_TABLE_LEN));
    }
    int last_note_change = _millis();
    // Initialisation du tableau des fréquences musicales (gamme tempérée)
    // 4 octaves × 12 demi-tons = 48 notes de LA1 (110Hz) à SOL#4 (831Hz)
    // Formule: freq = 440 * 2^((note_index - 24) / 12)
    // Index 0=LA1, 12=LA2, 24=LA3(440Hz), 36=LA4, 47=SOL#4
    
    for (int i = 0; i < FREQUENCY_TABLE_LEN; i++) {
        // Calcul de la fréquence pour cette note (gamme tempérée égale)
        // LA4 (440Hz) est à l'index 24, donc décalage de -24
        float semitone_offset = (float)(i - 12);  // Offset par rapport au LA4 (440Hz)
        float frequency = 440.0f * powf(2.0f, semitone_offset / 12.0f);
        
        // Conversion en valeur step
        frequency_table[i] = (uint32_t)((frequency * 0x10000 * SINE_WAVE_TABLE_LEN) / 44100.0f);
    }

    // Précompute un gain pour compenser la sensibilité humaine (boost des graves)
    // gain = (440 / freq) ^ alpha  where alpha ~ 0.28 gives moderate boost
    const float ref = 440.0f;
    const float alpha = 0.28f;
    for (int i = 0; i < FREQUENCY_TABLE_LEN; ++i) {
        float freq = (frequency_table[i] * 44100.0f) / (0x10000 * SINE_WAVE_TABLE_LEN);
        float gain = powf(ref / freq, alpha);
        if (gain < 0.5f) gain = 0.5f;
        if (gain > 3.0f) gain = 3.0f;
        frequency_gain_q16[i] = (uint32_t)(gain * 65536.0f + 0.5f);
    }

    ap = i2s_audio_init(44100);
    adc_select_input(0); // TONE_GPIO
    int scan_index = 0;
    int sequence_pause_ms = 0;
    while (true) {
        int c = getchar_timeout_us(0);
        if (c >= 0 && c != PICO_ERROR_TIMEOUT) {
            if (c == '-' && vol) vol--;
            if ((c == '=' || c == '+') && vol < 256) vol++;
            

            
            // Contrôles dynamiques
            // Sélection du bouton (1..8) pour modifier sa note de base
            if (c >= '1' && c <= '8') { current_button = c - '1'; }
            // Ajustement de la note de base
            if (c == 'a' && base_freq_index[current_button] > 0) { base_freq_index[current_button]--; }
            if (c == 's' && base_freq_index[current_button] < FREQUENCY_TABLE_LEN - 1) { base_freq_index[current_button]++; }
            if (c == 'A' && base_freq_index[current_button] >= NOTES_PER_OCTAVE) { base_freq_index[current_button] -= NOTES_PER_OCTAVE; }
            if (c == 'S' && base_freq_index[current_button] < FREQUENCY_TABLE_LEN - NOTES_PER_OCTAVE) { base_freq_index[current_button] += NOTES_PER_OCTAVE; }
            // Déclencheurs clavier créent/libèrent des voix indépendantes (button_id = -1)
            if (c == 'p') { start_voice(current_button); }
            if (c == 'o') { for (auto &v : active_voices) if (v.button_id == -1) envelope_trigger_release(v.env); }

            // Réglages Attack / Release
            if (c == 't' && attack_ms > 0) { attack_ms -= (attack_ms > 10 ? 10 : 1); envelope_recalc_all(); }
            if (c == 'T' && attack_ms < 5000) { attack_ms += (attack_ms >= 1000 ? 100 : 10); envelope_recalc_all(); }
            // Réglages Decay
            if (c == 'd' && decay_ms > 0) { decay_ms -= (decay_ms > 10 ? 10 : 1); envelope_recalc_all(); }
            if (c == 'D' && decay_ms < 5000) { decay_ms += (decay_ms >= 1000 ? 100 : 10); envelope_recalc_all(); }
            if (c == 'r' && release_ms > 0) { release_ms -= (release_ms > 10 ? 10 : 1); envelope_recalc_all(); }
            if (c == 'R' && release_ms < 10000) { release_ms += (release_ms >= 1000 ? 100 : 10); envelope_recalc_all(); }
            // Réglages Sustain level (u/U)
            if (c == 'u' && sustain_level > 1000) { sustain_level -= 1000; if (sustain_level < 0) sustain_level = 0; envelope_recalc_all(); }
            if (c == 'U' && sustain_level < 65536-1000) { sustain_level += 1000; if (sustain_level > 65536) sustain_level = 65536; envelope_recalc_all(); }

            // Réglage du facteur de courbure (logarithmique). Plus grand => attaque/release plus longue au début.
            if (c == 'f' && curve_factor > 1.1f) { curve_factor -= 0.5f; }
            if (c == 'F' && curve_factor < 50.0f) { curve_factor += 0.5f; }
            if (c == 'l') { use_log_curve = !use_log_curve; }
            // Ajustement du gain global de sortie (post-mix) : g diminue, G augmente
            if (c == 'g' && output_gain_shift > 0) { output_gain_shift--; }
            if (c == 'G' && output_gain_shift < 12) { output_gain_shift++; }
            
            if (c == 'q') break;
            
            // Affichage des notes avec noms
         int octave0; const char* note0 = get_note_name(base_freq_index[current_button], &octave0);
         int active_cnt = 0; for(auto &vv : active_voices) if (vv.env.state != ENV_IDLE) ++active_cnt;
         printf("BTN%d vol=%d actV=%d atk=%ums dec=%ums sus=%u rel=%ums curve=%.1f log=%d gainShift=%d NOTE:%s%d      \r",
             current_button+1,
             (int)vol,
             active_cnt,
             (unsigned)attack_ms,
             (unsigned)decay_ms,
             (unsigned)sustain_level,
             (unsigned)release_ms,
             curve_factor,
             use_log_curve ? 1 : 0,
             output_gain_shift,
             note0, octave0);
        } else {
            static uint8_t prev_btn[8] = {1,1,1,1,1,1,1,1};
            bool any_button_active = false;

            for(int i=0;i<8;i++) {
                int state = gpio_get(buttons_pins[i]); // 1 = relâché (pull-up), 0 = appuyé
                any_button_active |= (state == 0);
                if(state == 0 && prev_btn[i] == 1) {
                    // nouvelle voix sur ce bouton (superposition autorisée)
                    start_voice(i);
                    gpio_put(leds_pins[i], true);
                    start_time = _millis();
                    printf("Button %d pressed, starting voice at %u ms\n", i, start_time);
                } else if (state == 1 && prev_btn[i] == 0) {
                    // relâche toutes les voix de ce bouton
                    release_voices_for_button(i);
                    gpio_put(leds_pins[i], false);
                }
                prev_btn[i] = state;
                int active_voices_count = voices_for_button(i);
                if (active_voices_count > 0) {
                    gpio_put(leds_pins[i], true);
                } else {
                    gpio_put(leds_pins[i], false);
                }
            }
            if(any_button_active == true) {
                sequence_pause_ms = 1000;
                last_note_change = _millis();
            }

                // time = 0;
            sleep_ms(10);
            adc_mean[scan_index] = adc_read(scan_index);
            adc_mean[scan_index] = adc_read(scan_index); 
            adc_mean[scan_index + 8] = adc_read(scan_index + 8);
            adc_mean[scan_index + 8] = adc_read(scan_index + 8);
            uint index = (uint)(((3600 - adc_mean[scan_index]) / 3600.0f) * (FREQUENCY_TABLE_LEN -1) );
            // if(time == 0)printf("Tone %d ADC: %04d setting %04d index: %04d \n", time, adc_mean[time], adc_mean[time + 8], index);
            float freq = (frequency_table[index] * 44100.0f) / (0x10000 * SINE_WAVE_TABLE_LEN);
            base_freq_index[scan_index] = get_frequency_index(freq);
            // now = _millis();
            float max_level = 3600.f;
            if(scan_index == 8 - 8) { attack_ms  = logmap((adc_mean[ 8] / max_level), 20.0f, 1000.0f);  }
            if(scan_index == 9 - 8) { decay_ms   = logmap((adc_mean[9] / max_level),  20.0f, 300.0f);   }
            if(scan_index == 10 - 8) { sustain_level = (uint32_t)logmap((adc_mean[10] / max_level), 500.0f, 65536.0f);  }
            if(scan_index == 11 - 8) { sustain_ms = logmap((adc_mean[11] / max_level),  20.0f, 500.0f);   }
            if(scan_index == 12 - 8) { release_ms = logmap((adc_mean[12] / max_level), 20.0f, 1000.0f);   }
            if(scan_index == 13 - 8) { speed_tuning = (max_level - adc_mean[13])/5; speed_tuning = speed_tuning < 50 ? 50 : speed_tuning > 700 ? 3000000 : speed_tuning; }

            if(scan_index == 0) {printf("Attack: %d ms, Decay: %d ms Sustainlvl: %u sustain: %d ms Release: %d ms speed_tuning : %d \n",
                (unsigned)attack_ms,
                (unsigned)decay_ms,
                (unsigned)sustain_level,
                (unsigned)sustain_ms,
                (unsigned)release_ms,
                speed_tuning); }
        }
        scan_index++;
        if(scan_index == 8) {
            scan_index = 0;
        }

        if(last_note_change + speed_tuning  + sequence_pause_ms < _millis()) {
            last_note_change = _millis();
            start_voice(sequence);
            sequence_pause_ms = 0;
            sequence++;
            if(sequence >= 8) sequence = 0;
        }
        // if(last_note_change + attack_ms + decay_ms + sustain_ms < _millis()) {
        //     gpio_put(leds_pins[sequence], false);
        //     release_voices_for_button(sequence);
        // }
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
        int64_t mix = 0;
        // Itération arrière pour permettre effacement des voix terminées
        for(int idx = (int)active_voices.size()-1; idx >= 0; --idx) {
            Voice &v = active_voices[idx];
            envelope_step(v.env);
            if (v.env.state == ENV_IDLE) {
                active_voices.erase(active_voices.begin()+idx);
                continue;
            }
            uint32_t p = v.position;
            int32_t raw = (vol * sine_wave_table[p >> 16u]);
            // Appliquer compensation de gain par note (Q16)
            uint32_t gq16 = frequency_gain_q16[v.freq_index];
            int32_t raw_g = (int32_t)(((int64_t)raw * gq16) >> 16);
            int32_t env_scaled = (int32_t)(((int64_t)raw_g * v.env.level) >> 16);
            mix += env_scaled;
            v.position += v.step();
            if (v.position >= pos_max) v.position -= pos_max;
        }
        // Suppression du scaling dépendant du nombre de voix pour éviter le saut de gain lors du passage 2->1
        // Soft clip rapide (compression) pour éviter saturation dure
        // Limites approximatives pour conserver dynamique
        const int32_t CLIP_THRESH = 0x3FFFFFFF; // marge sous INT32_MAX
        int32_t value = (int32_t)mix; // déjà en plage 32-bit
        if (value > CLIP_THRESH) {
            int64_t x = value;
            // approximation douce: compresser au-delà du seuil
            value = (int32_t)(CLIP_THRESH + (x - CLIP_THRESH)/8); // compresse 8:1 la portion excédentaire
        } else if (value < -CLIP_THRESH) {
            int64_t x = value;
            value = (int32_t)(-CLIP_THRESH + (x + CLIP_THRESH)/8);
        }
        // Application du gain de sortie ajustable
        int64_t scaled = ((int64_t)value) << output_gain_shift;
        // Saturation pour éviter overflow int32
        if (scaled > INT32_MAX) scaled = INT32_MAX;
        if (scaled < INT32_MIN) scaled = INT32_MIN;
        int32_t out = (int32_t)scaled;
        samples[i*2+0] = out;
        samples[i*2+1] = out; // copie stéréo
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