"""
Configuration settings for the GPSR behavior tree.

OPENAI_API_KEY resolution order:
    1. Process env var OPENROUTER_API_KEY (or OPENAI_API_KEY) if set
    2. OPENROUTER_API_KEY line in /home/tinker/tk25_ws/.env
    3. Hardcoded fallback (kept only so config import never crashes)

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

_ENV_PATH = Path("/home/tinker/tk25_ws/.env")


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
    return "sk-or-v1-DEAD-KEY-set-OPENROUTER_API_KEY-or-edit-workspace-.env"


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


def _resolve_llm_timeout_s(default: float = 45.0) -> float:
    """Read GPSR_LLM_TIMEOUT_S (process env only); fall back to ``default``.

    I4 (round-3 adversarial review, M7): the planner client is built with no
    timeout and default (unbounded) retries, so an OpenRouter stall can hang
    a planning attempt indefinitely -- this bounds every request the
    planner's ``openai.OpenAI`` client makes.
    """
    raw = os.environ.get("GPSR_LLM_TIMEOUT_S")
    if raw:
        try:
            return float(raw)
        except ValueError:
            pass
    return default


# I4: seconds before an openai.OpenAI(...) request to the planner/fallback
# LLM gives up -- see GPSRPlanner._new_client (planner.py). Overridable via
# GPSR_LLM_TIMEOUT_S for tests/tuning.
LLM_TIMEOUT_S = _resolve_llm_timeout_s()
