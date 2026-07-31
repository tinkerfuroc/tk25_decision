"""
Configuration settings for the GPSR behavior tree.

OPENAI_API_KEY resolution order:
    1. Process env var OPENROUTER_API_KEY (or OPENAI_API_KEY) if set
    2. OPENROUTER_API_KEY in ``BT_ENV_FILE`` or ``$TK25_WS/.env``

Model resolution:
    OPENAI_MODEL (the planner / text model) is read from GPSR_LLM_MODEL, then
    LLM_MODEL (env first, then .env), defaulting to ``openai/gpt-4.1``. Set
    GPSR_LLM_MODEL=deepseek/deepseek-v4-pro in .env to plan with DeepSeek.

    OPENAI_VISION_MODEL (used by the image-based VLM fallback) is read from
    LLM_MODEL, defaulting to ``openai/gpt-4.1``. It is kept SEPARATE because a
    text-only planner model (DeepSeek) cannot answer image queries — the count
    VLM fallback needs a vision-capable model regardless of the planner choice.

All of these go through the same OpenRouter base URL + OPENROUTER_API_KEY, so a
provider-prefixed slug (``deepseek/...``, ``openai/...``) is all that changes.
"""

import os
from pathlib import Path

_WORKSPACE = Path(os.environ.get("TK25_WS", Path.home() / "tk25_ws"))
_ENV_PATH = Path(os.environ.get("BT_ENV_FILE", _WORKSPACE / ".env"))


def _read_env_file(path: Path, key: str):
    if not path.exists():
        return None
    for line in path.read_text().splitlines():
        line = line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        k, _, v = line.partition("=")
        if k.strip() == key:
            return v.strip().strip('"').strip("'") or None
    return None


def _resolve_api_key() -> str:
    for env_var in ("OPENROUTER_API_KEY", "OPENAI_API_KEY"):
        v = os.environ.get(env_var)
        if v:
            return v
    v = _read_env_file(_ENV_PATH, "OPENROUTER_API_KEY")
    if v:
        return v
    return ""


def _resolve_model(env_keys, default: str) -> str:
    """First non-empty value across env_keys, checking process env then .env."""
    for key in env_keys:
        v = os.environ.get(key)
        if v:
            return v
    for key in env_keys:
        v = _read_env_file(_ENV_PATH, key)
        if v:
            return v
    return default


# OpenAI API settings
OPENAI_API_KEY = _resolve_api_key()
# Planner / text model — GPSR_LLM_MODEL (or DEEPSEEK_LLM_MODEL) lets GPSR run a
# different (e.g. DeepSeek) model from the rest of the stack.
OPENAI_MODEL = _resolve_model(
    ("GPSR_LLM_MODEL", "DEEPSEEK_LLM_MODEL", "LLM_MODEL"), "openai/gpt-4.1"
)
# Vision model for image queries (count VLM fallback). MUST be vision-capable —
# never the text-only planner model.
OPENAI_VISION_MODEL = _resolve_model(("LLM_MODEL",), "openai/gpt-4.1")
# Model for the generic VLM / LLM fallbacks (tasks the small trees don't cover,
# general-knowledge questions). Pinned to gpt-4.1 by default and kept separate
# from the planner so the fallbacks stay on a known vision+text model even if
# the planner is swapped to a text-only model.
OPENAI_FALLBACK_MODEL = _resolve_model(("GPSR_FALLBACK_MODEL", "LLM_MODEL"), "openai/gpt-4.1")
OPENAI_TEMPERATURE = 0.2
OPENAI_MAX_TOKENS = 256
