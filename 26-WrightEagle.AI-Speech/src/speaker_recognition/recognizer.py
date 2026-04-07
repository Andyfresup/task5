import os

# Set mirror before importing huggingface_hub
os.environ['HF_ENDPOINT'] = 'https://hf-mirror.com'

import wave
import numpy as np
import pyaudio
import resampy
from faster_whisper import WhisperModel
import torch
import torchaudio
from silero_vad import load_silero_vad, VADIterator
import collections
import tempfile
from pyannote.audio import Model, Inference
from pyannote.audio.pipelines.speaker_verification import PretrainedSpeakerEmbedding
import warnings

warnings.filterwarnings("ignore", category=FutureWarning)

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
            device="cpu",
            compute_type="int8"
        )
        print("✓ Whisper model loaded!")

        # Load Silero VAD model
        print("Loading Silero VAD model...")
        self.vad_model = load_silero_vad()
        print("✓ Silero VAD model loaded!")

        # Load speaker identification model
        print("Loading speaker identification model...")
        try:
            self.speaker_model = PretrainedSpeakerEmbedding(
                "speechbrain/spkrec-ecapa-voxceleb",
                device=torch.device("cpu")
            )
            print("✓ Speaker identification model loaded!")
        except Exception as e:
            print(f"⚠ Speaker identification model loading failed: {e}")
            print("Using basic feature method as alternative")
            self.speaker_model = None

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

        # Speaker identification related
        self.speaker_embeddings = []  # Store speaker embeddings
        self.speaker_ids = []  # Store corresponding speaker IDs
        self.next_speaker_id = 0  # Next speaker's ID
        self.embedding_history = {}  # Store embedding history for each speaker ID

    def extract_speaker_embedding(self, audio_data):
        """
        Extract speaker embeddings using pretrained model
        """
        if self.speaker_model is None:
            # If professional model is unavailable, fallback to basic features
            return self.extract_basic_features(audio_data)

        try:
            # Ensure audio data is in correct format and sample rate
            audio_tensor = torch.tensor(audio_data).unsqueeze(0).float()

            # Extract speaker embeddings
            embedding = self.speaker_model(audio_tensor)
            return embedding[0].numpy()  # Return first audio's embedding
        except Exception as e:
            print(f"⚠ Error extracting speaker embedding: {e}")
            # On error, fallback to basic features
            return self.extract_basic_features(audio_data)

    def extract_basic_features(self, audio_data):
        """
        Extract basic audio features as alternative
        """
        if len(audio_data) < 2:
            return np.array([0, 0, 0, 0, 0])

        # Calculate short-term energy
        energy = np.sum(audio_data ** 2) / len(audio_data)

        # Calculate zero crossing rate
        zero_crossings = np.sum(np.abs(np.diff(np.sign(audio_data)))) / (2 * len(audio_data))

        # Calculate audio spectral features
        N = len(audio_data)
        if N == 0:
            return np.array([0, 0, 0, 0, 0])

        # Calculate FFT
        fft_vals = np.fft.fft(audio_data)
        magnitude = np.abs(fft_vals[:N // 2])  # Take positive frequency part

        # Calculate approximate spectral centroid
        freqs = np.arange(len(magnitude))
        if np.sum(magnitude) > 0:
            spectral_centroid = np.sum(freqs * magnitude) / np.sum(magnitude)
        else:
            spectral_centroid = 0

        # Calculate spectral bandwidth (spread)
        if np.sum(magnitude) > 0:
            spectral_spread = np.sqrt(np.sum(((freqs - spectral_centroid) ** 2) * magnitude) / np.sum(magnitude))
        else:
            spectral_spread = 0

        # Calculate spectral skewness
        if np.sum(magnitude) > 0 and spectral_spread > 0:
            spectral_skewness = np.sum(((freqs - spectral_centroid) ** 3) * magnitude) / (
                        np.sum(magnitude) * (spectral_spread ** 1.5))
        else:
            spectral_skewness = 0

        # Combine feature vector
        features = np.array([
            energy,
            zero_crossings,
            spectral_centroid,
            spectral_spread,
            spectral_skewness
        ])

        # Normalize feature vector
        norm = np.linalg.norm(features)
        if norm > 0:
            features = features / norm

        return features

    def identify_speaker(self, audio_data):
        """
        Identify current speaker ID
        """
        # Extract embedding from current audio segment
        current_embedding = self.extract_speaker_embedding(audio_data)

        # If using basic features method, adjust similarity threshold
        if self.speaker_model is None:
            threshold = 0.7  # Threshold for basic features
        else:
            threshold = 0.7  # Threshold for deep embeddings

        if len(self.speaker_embeddings) == 0:
            # If no known speakers, create new speaker
            speaker_id = f"Speaker_{self.next_speaker_id}"
            self.next_speaker_id += 1
            self.speaker_embeddings.append(current_embedding)
            self.speaker_ids.append(speaker_id)
            # Initialize embedding history
            self.embedding_history[speaker_id] = [current_embedding]
            print(f"Discovered new speaker: {speaker_id}")
            return speaker_id

        # Calculate similarity with known speakers
        similarities = []
        for known_embedding in self.speaker_embeddings:
            # Use cosine similarity calculation (more suitable for embedding vectors)
            if len(current_embedding) != len(known_embedding):
                # If dimensions differ, use basic features method
                dot_product = np.dot(current_embedding[:5], known_embedding[:5])
                norm_product = np.linalg.norm(current_embedding[:5]) * np.linalg.norm(known_embedding[:5])
            else:
                dot_product = np.dot(current_embedding, known_embedding)
                norm_product = np.linalg.norm(current_embedding) * np.linalg.norm(known_embedding)

            if norm_product == 0:
                similarity = 0
            else:
                similarity = dot_product / norm_product
            # Convert to distance (1 - similarity)
            distance = 1 - similarity
            similarities.append(distance)

        # Find the most similar speaker
        min_distance_idx = np.argmin(similarities)
        min_distance = similarities[min_distance_idx]

        if min_distance < threshold:
            # Match with known speaker
            matched_speaker_id = self.speaker_ids[min_distance_idx]
            # Update that speaker's embedding history
            if matched_speaker_id in self.embedding_history:
                self.embedding_history[matched_speaker_id].append(current_embedding)
                # Keep recent embeddings for averaging
                if len(self.embedding_history[matched_speaker_id]) > 5:
                    self.embedding_history[matched_speaker_id] = self.embedding_history[matched_speaker_id][-5:]
                # If using deep embeddings, update average embedding
                if self.speaker_model is not None:
                    avg_embedding = np.mean(self.embedding_history[matched_speaker_id], axis=0)
                    self.speaker_embeddings[min_distance_idx] = avg_embedding
            return matched_speaker_id
        else:
            # Create new speaker
            speaker_id = f"Speaker_{self.next_speaker_id}"
            self.next_speaker_id += 1
            self.speaker_embeddings.append(current_embedding)
            self.speaker_ids.append(speaker_id)
            # Initialize embedding history
            self.embedding_history[speaker_id] = [current_embedding]
            print(f"Discovered new speaker: {speaker_id}")
            return speaker_id

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
                        # Speech start detected
                        print("Started speaking...")
                        self.is_speaking = True
                        self.buffer_16k = np.array([], dtype=np.float32)  # Reset buffer

                    if 'end' in speech_dict:
                        # Speech end detected
                        print("Finished speaking, starting recognition...")
                        self.is_speaking = False
                        if len(self.buffer_16k) > int(self.target_rate * 0.1):  # At least 0.1 seconds
                            self.transcribe_buffer()

                # If speaking, add audio to buffer
                if self.is_speaking:
                    self.buffer_16k = np.concatenate([self.buffer_16k, audio_16k])

        except KeyboardInterrupt:
            print("Recognition stopped")
            # Recognize the last spoken segment before stopping
            if self.is_speaking and len(self.buffer_16k) > int(self.target_rate * 0.1):
                self.transcribe_buffer()
        finally:
            self.cleanup()

    # ============= Send to Whisper for recognition =============
    def transcribe_buffer(self):
        audio = self.buffer_16k.astype(np.float32)
        if len(audio) == 0:
            return

        # Identify speaker
        speaker_id = self.identify_speaker(audio)

        # Keep your original settings: English, beam search, no timestamps
        segments, _ = self.model.transcribe(
            audio,
            beam_size=5,
            language="en",
            without_timestamps=True
        )

        text = "".join(seg.text.strip() + " " for seg in segments).strip()
        if text:
            print(f"[{speaker_id}]: {text}")

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