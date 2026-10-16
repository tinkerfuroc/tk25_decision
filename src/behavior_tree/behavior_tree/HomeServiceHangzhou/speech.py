"""Task-specific ASR context and conservative phonetic text correction."""
import re
from functools import lru_cache
from pypinyin import Style, lazy_pinyin
from .config import OWNER_NAMES

# Whisper prompts are examples of expected speech, not decoding instructions.
NAME_PROMPT = "\u6770\u514b\u3002\u5b89\u4e1c\u3002\u674e\u534e\u3002\u6211\u53eb\u6770\u514b\u3002\u6211\u7684\u540d\u5b57\u662f\u5b89\u4e1c\u3002\u6211\u662f\u674e\u534e\u3002"
SWITCH_PROMPT = "\u6253\u5f00\u7535\u5668\u3002\u5173\u95ed\u7535\u5668\u3002\u5f00\u706f\u3002\u5173\u706f\u3002\u6253\u5f00\u3002\u5173\u95ed\u3002\u4e0d\u8981\u5f00\u3002\u522b\u5173\u3002\u4e0d\u7528\u4e86\u3002\u4fdd\u6301\u4e0d\u53d8\u3002"
ASSISTANCE_PROMPT = "\u6211\u6ca1\u4e8b\u3002\u9700\u8981\u5e2e\u52a9\u3002\u6276\u6211\u8d77\u6765\u3002\u8bf7\u53eb\u4eba\u6765\u3002\u8c22\u8c22\uff0c\u4e0d\u7528\u4e86\u3002"
HELP_PROMPT = "\u8bf7\u5e2e\u6211\u62ff\u4e1c\u897f\u3002\u8bf7\u53eb\u4eba\u6765\u3002\u9700\u8981\u5e2e\u52a9\u3002\u4e0d\u7528\u4e86\uff0c\u8c22\u8c22\u3002"


@lru_cache(maxsize=256)
def syllables(text):
    return tuple(lazy_pinyin(text, style=Style.NORMAL, strict=True))


def one_edit_apart(a, b):
    """Accept one insertion, deletion or substitution within one syllable."""
    if a == b:
        return True
    if abs(len(a) - len(b)) > 1:
        return False
    if len(a) == len(b):
        return sum(x != y for x, y in zip(a, b)) == 1
    if len(a) > len(b):
        a, b = b, a
    return any(a == b[:i] + b[i + 1:] for i in range(len(b)))


def match_owner_name(text):
    """Match names only in enrollment; never fuzzy-match arbitrary sentences."""
    if not isinstance(text, str) or not text.strip():
        raise ValueError("No owner name heard")
    # An explicit self-identification after a correction takes precedence.
    introductions = list(re.finditer(r"(?:\u6211\u7684\u540d\u5b57[\u662f\u53eb]|\u6211\u53eb|\u6211\u662f)", text))
    if introductions:
        text = text[introductions[-1].end():]
    if re.search(r"\u4e0d\u662f|\u4e0d\u53eb|\u522b\u53eb|\u4e0d\u8981\u53eb|\u4e0d\u5bf9", text):
        raise ValueError("Owner name was negated or corrected without a replacement")
    runs = re.findall(r"[\u3400-\u4dbf\u4e00-\u9fff]+", text)
    matches = set()
    for name in OWNER_NAMES:
        expected = syllables(name)
        for run in runs:
            actual = syllables(run)
            if any(actual[i:i + len(expected)] == expected
                   for i in range(len(actual) - len(expected) + 1)):
                matches.add(name)
    if len(matches) == 1:
        return next(iter(matches))
    if matches:
        raise ValueError("Multiple owner names heard")
    # Near matches must cover a whole short answer, not a window in a sentence.
    answer = text.strip(" \uff0c\u3002\uff01!\uff1f?\u3001,\n\t")
    answer = re.sub(r"[\u554a\u5440\u5462]$", "", answer)
    if not re.fullmatch(r"[\u3400-\u4dbf\u4e00-\u9fff]+", answer):
        raise ValueError("Owner name is not a contiguous spoken name")
    actual = syllables(answer)
    for name in OWNER_NAMES:
        expected = syllables(name)
        if (len(actual) == len(expected)
                and sum(a != b for a, b in zip(actual, expected)) == 1
                and all(one_edit_apart(a, b) for a, b in zip(actual, expected))):
            matches.add(name)
    if len(matches) != 1:
        raise ValueError("Owner name is unknown or ambiguous")
    return next(iter(matches))


def normalize_switch_transcript(text):
    """Correct exact homophones of switch verbs without changing negation."""
    if not isinstance(text, str):
        return text
    verbs = {syllables(word): word for word in ("\u6253\u5f00", "\u5173\u95ed", "\u5173\u6389", "\u5173\u706f", "\u5f00\u706f")}
    def correct_run(match):
        run = match.group()
        sounds = syllables(run)
        output, index = [], 0
        while index < len(run):
            word = verbs.get(sounds[index:index + 2])
            if word is not None:
                output.append(word)
                index += 2
            else:
                output.append(run[index])
                index += 1
        return "".join(output)
    return re.sub(r"[\u3400-\u4dbf\u4e00-\u9fff]+", correct_run, text)
