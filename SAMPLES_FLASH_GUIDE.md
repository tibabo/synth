# Intégration de Samples WAV depuis Flash

Ce système permet d'embarquer des fichiers WAV directement dans la flash du RP2350, **sans consommation de RAM supplémentaire**. Les samples sont accessibles via le bus XIP (eXecute In Place) mappé en lecture seule.

## Comment ça fonctionne

1. **Conversion WAV → Objet binaire** : `objcopy` transforme le fichier WAV en fichier objet `.o` avec symboles `_binary_*_start` et `_binary_*_end`
2. **Linkage direct** : L'objet est lié dans l'exécutable final et placé en section `.rodata` (flash)
3. **Accès runtime** : Le code lit directement depuis la flash via pointeurs const

## Vérification mémoire

```bash
cd build
arm-none-eabi-size synth.elf
# La section 'data' (RAM initialisée) ne doit PAS augmenter avec l'ajout de samples
# Les samples apparaissent dans 'text' (flash) via .rodata

arm-none-eabi-nm synth.elf | grep sample1_wav
# Les symboles doivent avoir le flag 'R' (read-only data = flash)
```

Exemple de sortie:
```
   text    data     bss     dec     hex filename
 157400       0   10756  168156   290dc synth.elf
                 ^^^^^ RAM reste à 0 même avec 44KB de sample!

10020150 R _binary__Users_jpinkasfeld_synth_sample1_wav_end
                ^^ Flag R = Read-only = Flash
```

## Ajouter un nouveau sample

### Étape 1: Préparer le WAV

Utilisez le script de conversion fourni:

```bash
python3 convert_wav.py mon_sample.wav sample2.wav
```

Le script:
- Convertit en mono si stéréo (mixdown)
- Resample vers 44.1 kHz si nécessaire
- Normalise le niveau (évite clipping)
- Génère un fichier WAV standardisé

### Étape 2: Ajouter à la liste CMake

**C'est maintenant automatique !** Il suffit d'ajouter le nom du sample (sans .wav) à la liste dans `CMakeLists.txt`:

```cmake
set(SAMPLE_LIST
    sample1
    sample2      # <-- Ajouter ici
    piano_c4     # <-- Et d'autres...
    # ...
)
```

La boucle `foreach()` génère automatiquement:
- Règle `objcopy` pour conversion WAV → objet
- Target de dépendance
- Linkage dans l'exécutable

Rebuild:
```bash
ninja -C build
```

### Étape 3: Déclarer les symboles

Dans `sample_data.h`:

```cpp
extern "C" {
    // Sample 1 (existant)
    extern const unsigned char _binary__Users_jpinkasfeld_synth_sample1_wav_start[];
    extern const unsigned char _binary__Users_jpinkasfeld_synth_sample1_wav_end[];
    
    // Sample 2 (nouveau)
    extern const unsigned char _binary__Users_jpinkasfeld_synth_sample2_wav_start[];
    extern const unsigned char _binary__Users_jpinkasfeld_synth_sample2_wav_end[];
}
```

**⚠️ Attention**: Les symboles incluent le chemin absolu. Vérifier avec:
```bash
arm-none-eabi-nm build/sample2_wav.o
```

### Étape 4: Créer accesseurs

```cpp
inline const int16_t* get_sample2_pcm16() {
    return reinterpret_cast<const int16_t*>(_binary__Users_jpinkasfeld_synth_sample2_wav_start + 44);
}

inline unsigned get_sample2_len() {
    const WavHeader* hdr = reinterpret_cast<const WavHeader*>(_binary__Users_jpinkasfeld_synth_sample2_wav_start);
    return hdr->data_size / 2;
}
```

### Étape 5: Sélection dans synth.cpp

Ajouter une variable globale et touche de sélection:

```cpp
static int current_sample_index = 0; // 0 = sample1, 1 = sample2, etc.

// Dans la boucle clavier:
if (c == 'x') { 
    current_sample_index = (current_sample_index + 1) % 2; // 2 samples
}
```

Puis dans `start_voice()`:

```cpp
if (waveform_type == WAVE_SAMPLE) {
    v.is_sample = true;
    v.sample_pos_q16 = 0;
    
    // Sélection du sample actif
    const int16_t* sample_ptr;
    unsigned sample_len;
    
    if (current_sample_index == 0) {
        sample_ptr = get_sample_pcm16();
        sample_len = get_sample_len();
    } else {
        sample_ptr = get_sample2_pcm16();
        sample_len = get_sample2_len();
    }
    
    // Calcul pitch ratio...
}
```

## Optimisations avancées

### Gestion multi-samples avec table

```cpp
struct SampleBank {
    const int16_t* data;
    uint32_t length;
    float root_freq;
    const char* name;
};

static const SampleBank samples[] = {
    { get_sample_pcm16(), get_sample_len(), 440.0f, "Sine" },
    { get_sample2_pcm16(), get_sample2_len(), 261.63f, "Piano C4" },
    { get_sample3_pcm16(), get_sample3_len(), 440.0f, "Saw A4" },
};

static const int NUM_SAMPLES = sizeof(samples) / sizeof(samples[0]);
```

### Support de boucles (sustain infini)

Ajouter à `SampleBank`:

```cpp
struct SampleBank {
    // ... existant
    uint32_t loop_start;  // index de début de boucle
    uint32_t loop_end;    // index de fin de boucle
    bool has_loop;
};
```

Dans `decode()`, phase `ENV_SUSTAIN`:

```cpp
if (v.is_sample && samples[current_sample_index].has_loop) {
    uint32_t sidx = v.sample_pos_q16 >> 16u;
    if (sidx >= samples[current_sample_index].loop_end) {
        // Retour au début de la boucle
        v.sample_pos_q16 = samples[current_sample_index].loop_start << 16u;
    }
}
```

### Compression (IMA ADPCM)

Pour économiser la flash (ratio 4:1), compresser les WAV:

```bash
ffmpeg -i input.wav -acodec adpcm_ima_wav compressed.wav
```

Puis implémenter un décodeur IMA ADPCM à la volée (état 16 octets par voix).

## Contraintes et limites

| Contrainte | Valeur RP2350 | Notes |
|------------|---------------|-------|
| Flash totale | 4 MB | Partagée code + samples |
| RAM (SRAM) | 520 KB | **Non utilisée** par samples |
| XIP cache | 16 KB | Améliore accès répétés |
| Latence XIP | ~8 cycles | Vs 1-2 cycles SRAM |

**Budget mémoire indicatif:**
- 1 seconde mono 44.1kHz PCM16 = **88.2 KB flash**
- 10 secondes = 882 KB (~20% flash)
- Samples courts (attaques) + boucles = optimal

**Performances:**
- Mixer 7 voix sample simultanées: OK
- Pitch shifting ±2 octaves: OK
- Limiter durées (0.5-2s) pour recyclage rapide

## Workflow recommandé

1. **Samples courts** (< 1s): attaques instrumentales, drum hits
2. **Boucles agressives**: sustain répété sur 0.1-0.5s
3. **Multi-sampling**: un sample tous les 12 demi-tons évite pitch shifting extrême
4. **Prévisualisation**: tester sur PC avant flashage avec `aplay` ou `sox`

## Dépannage

**Symboles non trouvés (undefined reference)**
- Vérifier chemin complet dans symboles: `arm-none-eabi-nm build/sampleX_wav.o`
- Adapter les déclarations `extern` en conséquence

**Sample ne joue pas / craquements**
- Vérifier format: mono, 16-bit, 44.1kHz
- Éviter pitch shift > ±24 demi-tons (aliasing)

**Flash overflow**
- Réduire durées samples
- Utiliser compression IMA ADPCM
- Activer optimisation `-Os` dans CMake

**Compilation lente**
- Normal: `objcopy` s'exécute à chaque rebuild si WAV modifié
- Solution: marquer samples comme `GENERATED` et ne pas toucher sauf besoin
