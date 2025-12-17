# Exemple: Ajouter 3 Samples d'Instruments

Ce guide montre comment ajouter rapidement plusieurs samples avec le système générique.

## Scénario: Piano, Violon, Batterie

### 1. Préparer les fichiers WAV

Supposons que vous avez:
- `piano_middle_c.wav` (Piano C4)
- `violin_a4.wav` (Violon A4)
- `kick_drum.wav` (Grosse caisse)

Convertissez-les:

```bash
python3 convert_wav.py piano_middle_c.wav piano_c4.wav
python3 convert_wav.py violin_a4.wav violin.wav
python3 convert_wav.py kick_drum.wav kick.wav
```

### 2. Modifier CMakeLists.txt

**Avant:**
```cmake
set(SAMPLE_LIST
    sample1
)
```

**Après:**
```cmake
set(SAMPLE_LIST
    sample1
    piano_c4
    violin
    kick
)
```

### 3. Rebuild

```bash
cd build
ninja
```

Output attendu:
```
-- Embedding piano_c4.wav as flash object
-- Embedding violin.wav as flash object
-- Embedding kick.wav as flash object
[2/2] Linking CXX executable synth.elf
```

### 4. Vérifier l'intégration

```bash
arm-none-eabi-size synth.elf
arm-none-eabi-nm synth.elf | grep "_binary.*_wav_start"
```

Vous devriez voir 4 samples (sample1 + vos 3 nouveaux).

### 5. Déclarer les symboles dans sample_data.h

```cpp
extern "C" {
    // Sample 1 (existant)
    extern const unsigned char _binary__Users_jpinkasfeld_synth_sample1_wav_start[];
    extern const unsigned char _binary__Users_jpinkasfeld_synth_sample1_wav_end[];
    
    // Piano C4
    extern const unsigned char _binary__Users_jpinkasfeld_synth_piano_c4_wav_start[];
    extern const unsigned char _binary__Users_jpinkasfeld_synth_piano_c4_wav_end[];
    
    // Violin A4
    extern const unsigned char _binary__Users_jpinkasfeld_synth_violin_wav_start[];
    extern const unsigned char _binary__Users_jpinkasfeld_synth_violin_wav_end[];
    
    // Kick drum
    extern const unsigned char _binary__Users_jpinkasfeld_synth_kick_wav_start[];
    extern const unsigned char _binary__Users_jpinkasfeld_synth_kick_wav_end[];
}
```

**Note:** Vérifiez le chemin exact avec `arm-none-eabi-nm build/piano_c4_wav.o` car il inclut le chemin absolu.

### 6. Créer une banque de samples

Dans `sample_data.h`, ajoutez:

```cpp
struct SampleInfo {
    const int16_t* data;
    unsigned length;
    float root_freq;
    const char* name;
};

// Helper macros pour simplifier
#define GET_SAMPLE_PCM(name) \
    reinterpret_cast<const int16_t*>(_binary__Users_jpinkasfeld_synth_##name##_wav_start + 44)

#define GET_SAMPLE_LEN(name) \
    (reinterpret_cast<const WavHeader*>(_binary__Users_jpinkasfeld_synth_##name##_wav_start)->data_size / 2)

// Banque de samples
inline const SampleInfo* get_sample_bank() {
    static const SampleInfo bank[] = {
        { GET_SAMPLE_PCM(sample1), GET_SAMPLE_LEN(sample1), 440.0f, "Sine" },
        { GET_SAMPLE_PCM(piano_c4), GET_SAMPLE_LEN(piano_c4), 261.63f, "Piano C4" },
        { GET_SAMPLE_PCM(violin), GET_SAMPLE_LEN(violin), 440.0f, "Violin A4" },
        { GET_SAMPLE_PCM(kick), GET_SAMPLE_LEN(kick), 60.0f, "Kick" },
    };
    return bank;
}

inline int get_sample_count() { return 4; }
```

### 7. Utiliser dans synth.cpp

```cpp
static int current_sample = 0; // Index du sample actif

// Dans la boucle clavier:
if (c == 'x') {
    current_sample = (current_sample + 1) % get_sample_count();
    printf("Sample: %s\n", get_sample_bank()[current_sample].name);
}

// Dans start_voice():
if (waveform_type == WAVE_SAMPLE) {
    v.is_sample = true;
    v.sample_pos_q16 = 0;
    
    const SampleInfo* info = &get_sample_bank()[current_sample];
    float target_freq = (frequency_table[v.freq_index] * 44100.0f) / (0x10000 * SINE_WAVE_TABLE_LEN);
    float ratio = target_freq / info->root_freq;
    v.sample_inc_q16 = (uint32_t)(ratio * 65536.0f + 0.5f);
    
    // Stocker pointeur et longueur dans Voice (ajouter ces champs)
    v.sample_data = info->data;
    v.sample_length = info->length;
}

// Dans decode():
if (v.is_sample) {
    uint32_t sidx = v.sample_pos_q16 >> 16u;
    if (sidx >= v.sample_length) {
        v.env.state = ENV_IDLE;
        active_voices.erase(active_voices.begin()+idx);
        continue;
    }
    base_wave = v.sample_data[sidx];  // Lecture depuis pointeur stocké
    v.sample_pos_q16 += v.sample_inc_q16;
}
```

### 8. Extensions Voice struct

Dans `synth.cpp`:

```cpp
struct Voice {
    // ... existant
    
    // Pour multi-samples (ajout)
    const int16_t* sample_data = nullptr;
    uint32_t sample_length = 0;
};
```

## Résultat final

- **Ajout simplifié** : modifier une seule ligne dans CMakeLists.txt
- **Build automatique** : tous les samples sont convertis et liés
- **Sélection runtime** : touche `x` pour changer d'instrument
- **RAM = 0** : tous les samples restent en flash

## Tailles indicatives

| Sample | Durée | Taille Flash |
|--------|-------|--------------|
| Piano C4 (attaque) | 0.5s | ~44 KB |
| Violin (sustained) | 1.0s | ~88 KB |
| Kick drum | 0.2s | ~18 KB |
| **Total** | | **~150 KB** |

Sur RP2350 (4 MB flash), vous pouvez facilement stocker 20-30 samples courts avec boucles.

## Optimisations

### Loop points pour sustain infini

Ajoutez à `SampleInfo`:

```cpp
struct SampleInfo {
    // ... existant
    uint32_t loop_start;  // Index début boucle
    uint32_t loop_end;    // Index fin boucle
    bool has_loop;
};

// Exemples
{ GET_SAMPLE_PCM(piano_c4), GET_SAMPLE_LEN(piano_c4), 261.63f, "Piano C4",
  2205, 8820, true },  // Boucle 0.05s → 0.2s
```

Dans `decode()` lors de `ENV_SUSTAIN`:

```cpp
if (v.is_sample && info->has_loop) {
    uint32_t sidx = v.sample_pos_q16 >> 16u;
    if (sidx >= info->loop_end) {
        v.sample_pos_q16 = info->loop_start << 16u;  // Recommencer boucle
    }
}
```

### Compression IMA ADPCM

Pour réduire flash 4:1, compressez avec `ffmpeg`:

```bash
ffmpeg -i piano_c4.wav -acodec adpcm_ima_wav piano_c4_compressed.wav
```

Puis implémentez décodeur IMA (16 octets état par voix).

## Workflow recommandé

1. **Enregistrer** courts samples (0.2-1.0s)
2. **Marquer** loop points avec Audacity
3. **Convertir** avec `convert_wav.py`
4. **Ajouter** à `SAMPLE_LIST`
5. **Tester** sur hardware
6. **Ajuster** root_freq si pitch incorrect
