import sherpa_onnx
import sounddevice as sd


class TTS:
    def __init__(self):
        # Initialize TTS engine
        tts_config = sherpa_onnx.OfflineTtsConfig(
            model=sherpa_onnx.OfflineTtsModelConfig(
                vits=sherpa_onnx.OfflineTtsVitsModelConfig(
                    # model="./models/vits-zh-aishell3/vits-aishell3.onnx",
                    # tokens="./models/vits-zh-aishell3/tokens.txt",
                    # lexicon="./models/vits-zh-aishell3/lexicon.txt",# This file is only needed for Chinese models

                    model="./models/vits-ljs/vits-ljs.onnx",
                    tokens="./models/vits-ljs/tokens.txt",
                    lexicon="./models/vits-ljs/lexicon.txt",

                    # model="./models/vits-vctk/vits-vctk.onnx",
                    # tokens="./models/vits-vctk/tokens.txt",
                )
            )
        )

        self.tts = sherpa_onnx.OfflineTts(tts_config)

    def speak(self, text):
        """
        Text to speech and timing
        Returns processing time (excluding playback time)
        """
        audio = self.tts.generate(text, sid=0, speed=1.0)
        # Play audio (using sounddevice)
        sd.play(audio.samples, audio.sample_rate)
        sd.wait()

if __name__ == "__main__":
    TTSer = TTS()
    TTSer.speak("Robot is ready to help!")