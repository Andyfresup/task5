"""
Local Configuration Example

1. Rename this file to config_local.py
2. Fill in your API keys
3. config_local.py will automatically override default settings in config.py
"""

from config import RunMode

# ==================== Run Mode ====================
MODE = RunMode.CLOUD  # Cloud API
# MODE = RunMode.LOCAL  # Local Ollama

# ==================== Cloud Configuration ====================

# DeepSeek API 
# Register at: https://platform.deepseek.com/
CLOUD_BASE_URL = "https://api.deepseek.com/v1"
CLOUD_API_KEY = "sk-your-dashscope-api-key-here"
CLOUD_MODEL = "deepseek-chat"

# Alibaba Cloud DashScope (Optional)
# Register at: https://dashscope.console.aliyun.com/
# CLOUD_BASE_URL = "https://dashscope.aliyuncs.com/compatible-mode/v1"
# CLOUD_API_KEY = "sk-your-dashscope-api-key-here"
# CLOUD_MODEL = "qwen-turbo"

# OpenAI (Optional)
# CLOUD_BASE_URL = "https://api.openai.com/v1"
# CLOUD_API_KEY = "sk-your-openai-api-key-here"
# CLOUD_MODEL = "gpt-3.5-turbo"

# ==================== Local Configuration ====================
# No need to modify the following if using local Ollama
LOCAL_BASE_URL = "http://localhost:11434/v1"
LOCAL_API_KEY = "ollama"
LOCAL_MODEL = "qwen3:8b"

# ==================== Other Configuration ====================
TEMPERATURE = 0.7
MAX_TOKENS = 1024
ROBOT_NAME = "LARA"
