"""
LLM Client

Encapsulates OpenAI-compatible interface, supports seamless cloud/local switching
"""

import json
import sys
import os
from typing import Optional, List, Dict, Any
from openai import OpenAI, AsyncOpenAI

# Add src directory to the path to import config
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../..'))
from config import Config

from modules.planning.schemas import RobotDecision, parse_action


class LLMClient:
    """
    LLM Client (synchronous version)

    Supports:
    - Cloud APIs (DeepSeek, DashScope, etc.)
    - Local Ollama
    - Automatic JSON parsing and retry
    """

    def __init__(self, config: Optional[Dict[str, Any]] = None):
        """
        Initialize LLM client

        Args:
            config: Custom config, uses Config.get_llm_config() if None
        """
        if config is None:
            config = Config.get_llm_config()

        self.base_url = config["base_url"]
        self.api_key = config["api_key"]
        self.model = config["model"]
        self.temperature = config.get("temperature", 0.7)
        self.max_tokens = config.get("max_tokens", 512)
        self.timeout = config.get("timeout", 30)

        # Initialize OpenAI client (disable proxy to avoid SOCKS issues)
        import httpx
        self.client = OpenAI(
            base_url=self.base_url,
            api_key=self.api_key,
            timeout=self.timeout,
            http_client=httpx.Client(trust_env=False)  # Disable proxy
        )

        print(f"✓ LLM Client initialized successfully")
        print(f"  Mode: {'Cloud' if Config.is_cloud_mode() else 'Local'}")
        print(f"  Model: {self.model}")
        print(f"  Base URL: {self.base_url}")

    def chat(
        self,
        messages: List[Dict[str, str]],
        temperature: Optional[float] = None,
        max_tokens: Optional[int] = None,
        enable_thinking: bool = False
    ) -> str:
        """
        Call LLM for chat

        Args:
            messages: Message list, format: [{"role": "user", "content": "..."}]
            temperature: Temperature coefficient (overrides default)
            max_tokens: Max token count (overrides default)
            enable_thinking: Whether to enable thinking mode (Qwen3 only)

        Returns:
            str: LLM response text
        """
        # Add /no_think marker for Qwen3 to disable thinking mode
        if not enable_thinking and "qwen3" in self.model.lower():
            # Add /no_think after last user message
            if messages and messages[-1].get("role") == "user":
                messages[-1]["content"] = messages[-1]["content"] + " /no_think"

        response = self.client.chat.completions.create(
            model=self.model,
            messages=messages,
            temperature=temperature or self.temperature,
            max_tokens=max_tokens or self.max_tokens
        )

        return response.choices[0].message.content

    def get_decision(
        self,
        user_input: str,
        system_prompt: str,
        conversation_history: Optional[List[Dict[str, str]]] = None,
        max_retries: int = 3
    ) -> RobotDecision:
        """
        Get robot decision (core method)

        Args:
            user_input: User input
            system_prompt: System prompt
            conversation_history: Conversation history
            max_retries: Maximum retry count on JSON parsing failure

        Returns:
            RobotDecision: Parsed decision object

        Raises:
            ValueError: If unable to parse after retries
        """
        # Build message list
        messages = [{"role": "system", "content": system_prompt}]

        # Add conversation history
        if conversation_history:
            messages.extend(conversation_history)

        # Add current user input
        messages.append({"role": "user", "content": user_input})

        # Retry mechanism
        last_error = None
        for attempt in range(max_retries):
            try:
                # Call LLM
                response = self.chat(messages)

                # Debug: Print LLM raw output
                if attempt == 0:  # Only print on first attempt
                    print(f"\n📄 LLM raw output:\n{'-'*60}")
                    print(response)
                    print(f"{'-'*60}\n")

                # Try to parse JSON
                decision_dict = self._extract_json(response)

                # If action field exists and not None, parse action
                if decision_dict.get("action") is not None:
                    action_dict = decision_dict["action"]
                    decision_dict["action"] = parse_action(action_dict)

                # Use Pydantic for validation
                decision = RobotDecision(**decision_dict)
                return decision

            except Exception as e:
                last_error = e
                print(f"⚠ Parse failed (attempt {attempt + 1}/{max_retries}): {e}")

                if attempt < max_retries - 1:
                    # Feed the error back to the LLM and ask it to regenerate
                    error_msg = (
                        f"Your output format is incorrect, error: {str(e)}\n"
                        f"Please output strictly in JSON format."
                    )
                    messages.append({"role": "assistant", "content": response})
                    messages.append({"role": "user", "content": error_msg})

        # All retries failed
        raise ValueError(
            f"Failed to parse LLM output after {max_retries} retries. Last error: {last_error}"
        )

    def _extract_json(self, text: str) -> dict:
        """
        Extract JSON from text (supports markdown code blocks)

        Args:
            text: Text containing JSON

        Returns:
            dict: Parsed dictionary

        Raises:
            json.JSONDecodeError: If unable to parse
        """
        # Try direct parse
        try:
            return json.loads(text)
        except json.JSONDecodeError:
            pass

        # Try extracting JSON from markdown code block labeled json
        if "```json" in text:
            start = text.find("```json") + 7
            end = text.find("```", start)
            json_str = text[start:end].strip()
            return json.loads(json_str)

        # Try extracting from a regular code block
        if "```" in text:
            start = text.find("```") + 3
            end = text.find("```", start)
            json_str = text[start:end].strip()
            return json.loads(json_str)

        # All attempts failed, raise original error
        raise json.JSONDecodeError("Unable to extract JSON from text", text, 0)


class AsyncLLMClient:
    """
    LLM Client (asynchronous version)

    For scenarios that require async calls (e.g., web services, ROS nodes)
    """

    def __init__(self, config: Optional[Dict[str, Any]] = None):
        if config is None:
            config = Config.get_llm_config()

        self.base_url = config["base_url"]
        self.api_key = config["api_key"]
        self.model = config["model"]
        self.temperature = config.get("temperature", 0.7)
        self.max_tokens = config.get("max_tokens", 512)
        self.timeout = config.get("timeout", 30)

        # Initialize async OpenAI client (disable proxy to avoid SOCKS issues)
        import httpx
        self.client = AsyncOpenAI(
            base_url=self.base_url,
            api_key=self.api_key,
            timeout=self.timeout,
            http_client=httpx.AsyncClient(trust_env=False)  # Disable proxy
        )

        print(f"✓ Async LLM Client initialized successfully")
        print(f"  Mode: {'Cloud' if Config.is_cloud_mode() else 'Local'}")
        print(f"  Model: {self.model}")

    async def chat(
        self,
        messages: List[Dict[str, str]],
        temperature: Optional[float] = None,
        max_tokens: Optional[int] = None,
        enable_thinking: bool = False
    ) -> str:
        """Asynchronous chat"""
        # Add /no_think marker for Qwen3 to disable thinking mode
        if not enable_thinking and "qwen3" in self.model.lower():
            # Append /no_think after last user message
            if messages and messages[-1].get("role") == "user":
                messages[-1]["content"] = messages[-1]["content"] + " /no_think"

        response = await self.client.chat.completions.create(
            model=self.model,
            messages=messages,
            temperature=temperature or self.temperature,
            max_tokens=max_tokens or self.max_tokens
        )

        return response.choices[0].message.content

    async def get_decision(
        self,
        user_input: str,
        system_prompt: str,
        conversation_history: Optional[List[Dict[str, str]]] = None,
        max_retries: int = 3
    ) -> RobotDecision:
        """Asynchronously get decision"""
        messages = [{"role": "system", "content": system_prompt}]

        if conversation_history:
            messages.extend(conversation_history)

        messages.append({"role": "user", "content": user_input})

        last_error = None
        for attempt in range(max_retries):
            try:
                response = await self.chat(messages)
                decision_dict = self._extract_json(response)

                if decision_dict.get("action") is not None:
                    action_dict = decision_dict["action"]
                    decision_dict["action"] = parse_action(action_dict)

                decision = RobotDecision(**decision_dict)
                return decision

            except Exception as e:
                last_error = e
                print(f"⚠ Parse failed (attempt {attempt + 1}/{max_retries}): {e}")

                if attempt < max_retries - 1:
                    error_msg = (
                        f"Your output format is incorrect, error: {str(e)}\n"
                        f"Please output strictly in JSON format."
                    )
                    messages.append({"role": "assistant", "content": response})
                    messages.append({"role": "user", "content": error_msg})

        raise ValueError(
            f"Failed to parse LLM output after {max_retries} retries. Last error: {last_error}"
        )

    def _extract_json(self, text: str) -> dict:
        """Extract JSON from text"""
        try:
            return json.loads(text)
        except json.JSONDecodeError:
            pass

        if "```json" in text:
            start = text.find("```json") + 7
            end = text.find("```", start)
            json_str = text[start:end].strip()
            return json.loads(json_str)

        if "```" in text:
            start = text.find("```") + 3
            end = text.find("```", start)
            json_str = text[start:end].strip()
            return json.loads(json_str)

        raise json.JSONDecodeError("Unable to extract JSON from text", text, 0)


# ==================== Test Code ====================

if __name__ == "__main__":
    print("=== Test LLM Client ===\n")

    # Create client
    client = LLMClient()

    # Simple chat test
    print("\n--- Test 1: Simple chat ---")
    messages = [
        {"role": "system", "content": "You are a friendly assistant"},
        {"role": "user", "content": "Hello"}
    ]

    try:
        response = client.chat(messages)
        print(f"Reply: {response}\n")
    except Exception as e:
        print(f"Error: {e}\n")

    print("✓ Basic functionality test completed")
    print("\nNote: Please configure the API key in config.py before running")
