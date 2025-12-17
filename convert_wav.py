#!/usr/bin/env python3
"""
Convertit un fichier WAV en format adapté pour l'embarquement dans le synthé.

Usage:
    python3 convert_wav.py input.wav [output_name]

Le fichier WAV doit être:
- Mono (1 canal)
- 16-bit PCM
- 44.1 kHz (ou sera resampelé automatiquement)

Le script génère un fichier WAV normalisé prêt à être embarqué via objcopy.
"""

import wave
import struct
import sys
import os

def convert_wav(input_path, output_path=None):
    """Convertit un WAV en format standardisé mono 16-bit 44.1kHz."""
    
    if output_path is None:
        base = os.path.splitext(os.path.basename(input_path))[0]
        output_path = f"{base}_converted.wav"
    
    print(f"📂 Lecture: {input_path}")
    
    with wave.open(input_path, 'rb') as w:
        channels = w.getnchannels()
        sampwidth = w.getsampwidth()
        framerate = w.getframerate()
        nframes = w.getnframes()
        
        print(f"   Format source: {channels}ch, {sampwidth*8}bit, {framerate}Hz, {nframes} frames")
        
        # Lecture données brutes
        raw_data = w.readframes(nframes)
        
        # Conversion en samples int16
        if sampwidth == 1:  # 8-bit unsigned
            samples = list(struct.unpack('B' * nframes * channels, raw_data))
            samples = [(s - 128) * 256 for s in samples]  # Convert to signed 16-bit
        elif sampwidth == 2:  # 16-bit signed
            samples = list(struct.unpack('<' + 'h' * (nframes * channels), raw_data))
        else:
            print(f"❌ Format {sampwidth*8}-bit non supporté")
            return False
        
        # Mixdown vers mono si stéréo
        if channels == 2:
            print("   🔄 Conversion stéréo → mono...")
            mono_samples = []
            for i in range(0, len(samples), 2):
                mono_samples.append((samples[i] + samples[i+1]) // 2)
            samples = mono_samples
            nframes = len(samples)
        elif channels != 1:
            print(f"❌ {channels} canaux non supporté (mono ou stéréo uniquement)")
            return False
        
        # Resampling si nécessaire
        if framerate != 44100:
            print(f"   🔄 Resampling {framerate}Hz → 44100Hz...")
            ratio = 44100.0 / framerate
            new_len = int(nframes * ratio)
            resampled = []
            for i in range(new_len):
                src_pos = i / ratio
                idx = int(src_pos)
                if idx + 1 < len(samples):
                    # Interpolation linéaire
                    frac = src_pos - idx
                    val = int(samples[idx] * (1 - frac) + samples[idx + 1] * frac)
                else:
                    val = samples[idx] if idx < len(samples) else 0
                resampled.append(val)
            samples = resampled
            nframes = new_len
            framerate = 44100
        
        # Normalisation (éviter clipping, garder headroom)
        max_val = max(abs(s) for s in samples)
        if max_val > 0:
            target_peak = 28000  # Headroom pour éviter clipping au mix
            scale = target_peak / max_val
            if scale < 1.0:  # Normaliser uniquement si nécessaire
                print(f"   📊 Normalisation: peak {max_val} → {target_peak}")
                samples = [int(s * scale) for s in samples]
        
        # Écriture WAV normalisé
        print(f"💾 Écriture: {output_path}")
        with wave.open(output_path, 'wb') as out:
            out.setnchannels(1)
            out.setsampwidth(2)
            out.setframerate(44100)
            out.writeframes(struct.pack('<' + 'h' * len(samples), *samples))
        
        duration_ms = int((nframes / 44100.0) * 1000)
        size_kb = os.path.getsize(output_path) / 1024
        
        print(f"✅ Succès!")
        print(f"   Durée: {duration_ms}ms")
        print(f"   Taille: {size_kb:.1f} KB")
        print(f"   Samples: {nframes}")
        
        return True

def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    
    input_path = sys.argv[1]
    output_path = sys.argv[2] if len(sys.argv) > 2 else None
    
    if not os.path.exists(input_path):
        print(f"❌ Fichier introuvable: {input_path}")
        sys.exit(1)
    
    if convert_wav(input_path, output_path):
        print("\n📝 Prochaines étapes:")
        print("   1. Placer le WAV dans le dossier du projet")
        print("   2. Ajouter une règle objcopy dans CMakeLists.txt")
        print("   3. Déclarer les symboles extern dans sample_data.h")
    else:
        sys.exit(1)

if __name__ == '__main__':
    main()
