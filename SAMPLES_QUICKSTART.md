# Quick Start: WAV Sample Integration

## ✅ Ce qui est déjà en place

Le système est **opérationnel** avec `sample1.wav` (0.5s sine 440Hz) embarqué en flash:

- **Aucune consommation RAM** : le sample est lu directement depuis flash (XIP)
- **Intégration automatique** : CMake + objcopy convertit le WAV en objet lié
- **Accès transparent** : le code lit via `sample_pcm16[index]` comme un tableau normal

## 🎹 Utilisation

1. **Flash le projet** : `picotool load build/synth.elf -fx`
2. **Cycle les formes d'onde** : touche `w` → sin → car → tri → **sam** (sample)
3. **Déclencher une voix** : bouton matériel ou touche `p`
4. **Pitch shifting** : les potentiomètres pitch ajustent la fréquence de lecture

Le sample joue avec enveloppe ADSR complète et se termine automatiquement.

## 🔧 Ajouter vos samples

### Option 1: Remplacement rapide

Remplacer `sample1.wav` par votre fichier (mono 16-bit 44.1kHz) :

```bash
# Préparer votre WAV
python3 convert_wav.py mon_instrument.wav sample1.wav

# Rebuild (CMake détecte automatiquement le changement)
ninja -C build
```

### Option 2: Ajouter un deuxième sample

1. **Convertir** :
```bash
python3 convert_wav.py piano_c4.wav sample2.wav
```

2. **Éditer `CMakeLists.txt`** (ajouter à la liste) :
```cmake
set(SAMPLE_LIST
    sample1
    sample2     # <-- Ajouter simplement ici !
)
```

3. **Vérifier les symboles générés** :
```bash
ninja -C build sample2_blob_target
arm-none-eabi-nm build/sample2_wav.o
# Noter les symboles _binary_..._start/end
```

4. **Déclarer dans `sample_data.h`** :
```cpp
extern "C" {
    // Sample 2
    extern const unsigned char _binary__Users_jpinkasfeld_synth_sample2_wav_start[];
    extern const unsigned char _binary__Users_jpinkasfeld_synth_sample2_wav_end[];
}

inline const int16_t* get_sample2_pcm16() {
    return reinterpret_cast<const int16_t*>(_binary__Users_jpinkasfeld_synth_sample2_wav_start + 44);
}
```

5. **Implémenter sélection** (voir SAMPLES_FLASH_GUIDE.md pour code détaillé)

## 📊 Vérifier RAM = 0

```bash
cd build
arm-none-eabi-size synth.elf
```

La colonne `data` (RAM initialisée) doit rester constante même après ajout de samples.

## 📚 Documentation complète

Voir **[SAMPLES_FLASH_GUIDE.md](SAMPLES_FLASH_GUIDE.md)** pour:
- Gestion multi-samples avec table
- Support de boucles (sustain infini)
- Compression IMA ADPCM
- Optimisations mémoire
- Troubleshooting

## 🛠️ Fichiers du système

| Fichier | Rôle |
|---------|------|
| `convert_wav.py` | Convertisseur WAV → format standard |
| `sample_data.h` | Déclarations symboles + accesseurs |
| `CMakeLists.txt` | Règles objcopy pour embedding |
| `synth.cpp` | Playback engine (lecture + pitch shifting) |
| `sample1.wav` | Exemple de sample embarqué |

## ⚡ Performance

- **7 voix sample simultanées** : OK
- **Pitch ±24 demi-tons** : OK (avec aliasing léger)
- **Latence XIP** : négligeable à 44.1kHz
- **Flash occupée** : ~88 KB par seconde de sample

## 🎯 Prochaines étapes suggérées

1. ✅ **Système de base** : opérationnel
2. ⏭️ **Multi-samples** : ajouter 3-4 instruments
3. ⏭️ **Boucles** : sustain infini avec loop points
4. ⏭️ **Sélection dynamique** : touche `x` pour changer de sample
5. ⏭️ **Interpolation** : réduire artefacts pitch extrêmes

Questions ? Consultez [SAMPLES_FLASH_GUIDE.md](SAMPLES_FLASH_GUIDE.md) pour détails techniques.
