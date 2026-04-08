"""
Configuration Layer

Supports seamless switching between Cloud and Local modes
"""

from enum import Enum
from typing import Literal


class RunMode(str, Enum):
    """Run mode"""
    CLOUD = "CLOUD"  # Cloud API (current PC development)
    LOCAL = "LOCAL"  # Local Ollama (future Orin deployment)


class Config:
    """
    Global configuration class

    Usage:
    1. Development stage: Keep MODE = RunMode.CLOUD, configure cloud API
    2. Deployment stage: Change to MODE = RunMode.LOCAL, no need to modify other code
    """

    # ==================== Run Mode ====================
    MODE: RunMode = RunMode.CLOUD

    # ==================== Cloud Configuration ====================
    # DeepSeek API (Recommended, cost-effective)
    CLOUD_BASE_URL = "https://api.deepseek.com"
    CLOUD_API_KEY = "sk-xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx"  # Replace with your API Key
    CLOUD_MODEL = "deepseek-chat"

    # Alibaba DashScope (Optional)
    # CLOUD_BASE_URL = "https://dashscope.aliyuncs.com/compatible-mode/v1"
    # CLOUD_API_KEY = "sk-xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx"
    # CLOUD_MODEL = "qwen-turbo"

    # ==================== Local Configuration ====================
    LOCAL_BASE_URL = "http://localhost:11434/v1"
    LOCAL_API_KEY = "ollama"  # Ollama doesn't need real key
    LOCAL_MODEL = "qwen2.5:3b"  # Recommended 3B quantized version

    # ==================== LLM Parameters ====================
    TEMPERATURE = 0.7  # Temperature coefficient (0-1, higher = more random)
    MAX_TOKENS = 512  # Maximum generation length
    TIMEOUT = 30  # Request timeout (seconds)

    # ==================== Robot Configuration ====================
    ROBOT_NAME = "LARA"
    ENABLE_MOCK = True  # True=Mock mode, False=Real ROS
    ENABLE_REPLAN_ON_FAILURE = True
    MAX_REPLAN_ATTEMPTS = 1

    # ==================== Logging Configuration ====================
    LOG_LEVEL = "INFO"  # DEBUG, INFO, WARNING, ERROR
    LOG_FILE = "logs/robot.log"

    @classmethod
    def get_llm_config(cls) -> dict:
        """
        Get LLM configuration for current mode

        Returns:
            dict: Configuration dictionary containing base_url, api_key, model
        """
        if cls.MODE == RunMode.CLOUD:
            return {
                "base_url": cls.CLOUD_BASE_URL,
                "api_key": cls.CLOUD_API_KEY,
                "model": cls.CLOUD_MODEL,
                "temperature": cls.TEMPERATURE,
                "max_tokens": cls.MAX_TOKENS,
                "timeout": cls.TIMEOUT,
            }
        else:  # LOCAL
            return {
                "base_url": cls.LOCAL_BASE_URL,
                "api_key": cls.LOCAL_API_KEY,
                "model": cls.LOCAL_MODEL,
                "temperature": cls.TEMPERATURE,
                "max_tokens": cls.MAX_TOKENS,
                "timeout": cls.TIMEOUT,
            }

    @classmethod
    def is_cloud_mode(cls) -> bool:
        """Check if in cloud mode"""
        return cls.MODE == RunMode.CLOUD

    @classmethod
    def is_local_mode(cls) -> bool:
        """Check if in local mode"""
        return cls.MODE == RunMode.LOCAL


# Try to import local configuration (for overriding defaults, not committed to git)
try:
    import config_local

    # Dynamically update Config class attributes
    for attr in dir(config_local):
        if not attr.startswith('_'):  # skip private attributes
            value = getattr(config_local, attr)
            if hasattr(Config, attr):
                setattr(Config, attr, value)

    print("✓ Loaded local configuration config_local.py")
except ImportError:
    print("⚠ config_local.py not found, using default configuration")
    print("Tip: copy config.py to config_local.py and fill in your API key")
