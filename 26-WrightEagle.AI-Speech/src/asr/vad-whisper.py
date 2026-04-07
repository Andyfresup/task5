import wave
import numpy as np
import pyaudio
import resampy
from faster_whisper import WhisperModel
import torch
import torchaudio
from silero_vad import load_silero_vad, VADIterator
import collections

from config.config import *
WHISPER_MODEL = WHISPER_SMALL

# ============= Find specified microphone =============
def find_input_device(name_part: str):
    p = pyaudio.PyAudio()
    for i in range(p.get_device_count()):
        info = p.get_device_info_by_index(i)
        if name_part.lower() in info["name"].lower() and info["maxInputChannels"] > 0:
            print(f"✓ Found microphone: {info['name']} (index={i})")
            rate = int(info.get("defaultSampleRate", 48000))
            return i, rate
    print("⚠ Microphone not found, using default device")
    index = p.get_default_input_device_info()["index"]
    info = p.get_device_info_by_index(index)
    return index, int(info.get("defaultSampleRate", 48000))


class SpeechRecognizer:
    def __init__(self, mic_name="Newmine"):
        self.model_path = WHISPER_MODEL

        print("Loading Whisper model...")
        self.model = WhisperModel(
            self.model_path,
            # device="cuda",
            # compute_type="float16"
            device = "cpu",
            compute_type="int8"
        )
        print("✓ Whisper model loaded!")

        # Load Silero VAD model
        print("Loading Silero VAD model...")
        self.vad_model = load_silero_vad()
        print("✓ Silero VAD model loaded!")

        # Whisper fixed 16k sample rate
        self.target_rate = 16000

        # ===== Find microphone =====
        self.p = pyaudio.PyAudio()
        self.device_index, self.device_rate = find_input_device(mic_name)
        print(f"🎤 Microphone sample rate: {self.device_rate} Hz")

        # 32ms per frame to ensure 512 samples per frame at 16k sampling to meet vad model requirements
        self.frame_duration_ms = 32
        # Number of samples per frame on device side
        self.frame_samples_device = int(self.device_rate * self.frame_duration_ms / 1000)
        # Number of samples per frame on 16k side
        self.frame_samples_16k = int(self.target_rate * self.frame_duration_ms / 1000)

        # ===== Open microphone stream =====
        self.stream = self.p.open(
            format=pyaudio.paInt16,
            channels=1,
            rate=self.device_rate,
            input=True,
            frames_per_buffer=self.frame_samples_device,
            input_device_index=self.device_index
        )

        # Record and save (optional)
        self.wav_file = wave.open("recorded_audio.wav", "wb")
        self.wav_file.setnchannels(1)
        self.wav_file.setsampwidth(2)
        self.wav_file.setframerate(self.device_rate)

        # ===== Audio buffer (16k)=====
        self.buffer_16k = np.array([], dtype=np.float32)

        # VAD Iterator for streaming detection
        self.vad_iterator = VADIterator(self.vad_model, threshold=0.5)

        # Status tracking
        self.is_speaking = False
    # ============= Main loop =============
    def start_listening(self):
        print("Starting real-time speech recognition (Silero VAD)...\n")

        try:
            while True:
                # Read frame by frame
                data = self.stream.read(self.frame_samples_device, exception_on_overflow=False)
                # Save to wav
                self.wav_file.writeframes(data)

                # Convert to float32 [-1, 1] (device sample rate)
                audio_dev = np.frombuffer(data, dtype=np.int16).astype(np.float32) / 32768.0

                # Resample to 16k, for VAD + Whisper
                audio_16k = resampy.resample(
                    audio_dev,
                    sr_orig=self.device_rate,
                    sr_new=self.target_rate,
                    filter="kaiser_best"  # High quality resampling similar to sinc_best
                )

                # Ensure length is our expected frame size
                if len(audio_16k) > self.frame_samples_16k:
                    audio_16k = audio_16k[:self.frame_samples_16k]
                elif len(audio_16k) < self.frame_samples_16k:
                    pad_len = self.frame_samples_16k - len(audio_16k)
                    audio_16k = np.concatenate(
                        [audio_16k, np.zeros(pad_len, dtype=np.float32)]
                    )

                # Use Silero VAD for detection
                speech_dict = self.vad_iterator(audio_16k, return_seconds=False)

                if speech_dict is not None:
                    if 'start' in speech_dict:
                        # Detected speech start
                        print("Starting to speak...")
                        self.is_speaking = True
                        self.buffer_16k = np.array([], dtype=np.float32)  # Reset buffer

                    if 'end' in speech_dict:
                        # Detected speech end
                        print("Speech ended, starting recognition...")
                        self.is_speaking = False
                        if len(self.buffer_16k) > int(self.target_rate * 0.1):  # At least 0.1 seconds
                            self.transcribe_buffer()

                # If speaking, add audio to buffer
                if self.is_speaking:
                    self.buffer_16k = np.concatenate([self.buffer_16k, audio_16k])

        except KeyboardInterrupt:
            print("Recognition stopped")
            # Before stopping, recognize the last spoken segment
            if self.is_speaking and len(self.buffer_16k) > int(self.target_rate * 0.1):
                self.transcribe_buffer()
        finally:
            self.cleanup()

    # ============= Send to Whisper for recognition =============
    def transcribe_buffer(self):
        audio = self.buffer_16k.astype(np.float32)
        if len(audio) == 0:
            return

        # Keep your original settings: English, beam search, no timestamps
        segments, _ = self.model.transcribe(
            audio,
            beam_size=5,
            language="en",
            without_timestamps=True
        )

        text = "".join(seg.text.strip() + " " for seg in segments).strip()
        if text:
            print("Recognition:", text)

    # ============= Clean up resources =============
    def cleanup(self):
        self.stream.stop_stream()
        self.stream.close()
        self.p.terminate()
        self.wav_file.close()
        print("Recording saved as recorded_audio.wav")

if __name__ == "__main__":
    recognizer = SpeechRecognizer("Newmine")
    recognizer.start_listening()
