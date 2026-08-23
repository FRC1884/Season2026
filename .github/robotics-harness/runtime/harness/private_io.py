"""Symlink-safe private artifact writes for local Phase 0 evidence."""

from __future__ import annotations

import os
import uuid
from pathlib import Path


def write_private_text(path: Path | str, text: str) -> Path:
    """Atomically write UTF-8 text with owner-only file and directory permissions."""

    requested = Path(path).expanduser()
    output = requested.parent.resolve() / requested.name
    output.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
    output.parent.chmod(0o700)
    temporary = output.with_name(f".{output.name}.{uuid.uuid4().hex}.tmp")
    flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL
    if hasattr(os, "O_NOFOLLOW"):
        flags |= os.O_NOFOLLOW
    descriptor = os.open(temporary, flags, 0o600)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            stream.write(text)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, output)
        output.chmod(0o600)
    except BaseException:
        temporary.unlink(missing_ok=True)
        raise
    return output
