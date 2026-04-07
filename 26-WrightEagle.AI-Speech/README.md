# Speech Processing Toolkit

This repository contains a collection of speech processing tools including real-time speech recognition, text-to-speech synthesis, and speaker recognition modules.

## Modules Overview

### 1. Real-time Speech Recognition ([vad-whisper.py](./src/asr/vad-whisper.py))

Real-time speech recognition module that combines Whisper ASR model with Silero VAD (Voice Activity Detection) for continuous listening and transcription. Features include:

- Continuous microphone monitoring with VAD-based speech detection
- Real-time audio streaming and resampling
- Whisper model for accurate speech-to-text conversion
- Configurable microphone selection
- CPU-based inference with INT8 quantization

Usage:
```bash
python 1-silero_vad_whisper.py
```

### 2. Text-to-Speech Synthesis ([synthesizer.py](src/tts/synthesizer.py))

High-quality text-to-speech module using VITS-based neural networks. Features include:

- Neural text-to-speech using VITS models
- Real-time audio playback with sounddevice
- Configurable voice models (supports LJSpeech model)
- Easy-to-use API for text synthesis

Usage:
```bash
python 2-TTS-v1.py
```

### 3. Speaker Recognition ([recognizer.py](src/speaker_recognition/recognizer.py))

Advanced speech recognition module with speaker identification capabilities. Features include:

- Multi-speaker recognition and identification
- Speaker embedding extraction using deep learning models
- Real-time speaker diarization
- Integration with Whisper ASR for transcribed speech with speaker labels
- Fallback mechanism for basic audio feature analysis

Usage:
```bash
python 3-recognize-model.py
```

## Environment Setup

Requirements:
- Python 3.10
- Conda environment recommended

Create environment:
```bash
conda create -n speech310 python=3.10
conda activate speech310
```

Install required packages:
```bash
pip install faster-whisper pyaudio resampy torch torchaudio silero-vad sherpa-onnx sounddevice numpy
```

## Configuration

Models are configured in [config.py](config/config.py) file with default settings for CPU inference. Adjust model paths and parameters as needed.

## Notes

- Currently configured for CPU inference with INT8 quantization
- Supports English language recognition by default
- Microphone selection is configurable via device name matching