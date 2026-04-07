# config.py
import yaml
from pathlib import Path

# 自动加载同目录下的 config.yaml
_config_file = Path(__file__).parent / "paths.yaml"

with open(_config_file, "r", encoding="utf-8") as f:
    _cfg = yaml.safe_load(f)

# 模型路径
VOSK_en = _cfg["model_paths"]["vosk"]["en"]
VOSK_giga_en = _cfg["model_paths"]["vosk"]["giga_en"]
VOSK_small_en = _cfg["model_paths"]["vosk"]["small_en"]
# VOSK_small_cn = _cfg["model_paths"]["vosk"].get("small_cn")  # 如果需要可选字段

WHISPER_MEDIUM = _cfg["model_paths"]["whisper"]["medium"]
WHISPER_LARGE = _cfg["model_paths"]["whisper"]["large"]
WHISPER_SMALL = _cfg["model_paths"]["whisper"]["small"]
WHISPER_BASE = _cfg["model_paths"]["whisper"]["base"]

DIARIZATION_COMMUNITY = _cfg["model_paths"]["diarization"]["community_1"]

# 数据路径
FLAC_PATH = _cfg["data_paths"]["flac"]
TEST_PATH = _cfg["data_paths"]["test"]["root"]
TEST_FILE_PATH = _cfg["data_paths"]["test"]["transcript"]