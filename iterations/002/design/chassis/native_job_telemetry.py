"""Write observable native-job progress and bind it to a recoverable saved archive."""
from contextlib import contextmanager
from datetime import datetime, timezone
from hashlib import sha256
import json
import os
from pathlib import Path
from time import monotonic


class NativeProgress:
    def __init__(self, destination, source_archive):
        self.folder = Path(destination)
        self.folder.mkdir(parents=True, exist_ok=True)
        self.history = self.folder / "native-progress.jsonl"
        self.latest = self.folder / "native-progress.json"
        self.start = monotonic()
        self.count = 0
        if self.history.exists() or self.latest.exists():
            raise FileExistsError("Progress belongs to an earlier native run")
        self.checkpoint(source_archive)

    def checkpoint(self, path):
        path = Path(path)
        with path.open("rb") as stream:
            fingerprint = sha256()
            for chunk in iter(lambda: stream.read(1024 * 1024), b""):
                fingerprint.update(chunk)
            digest = fingerprint.hexdigest()
        self.last_archive = {"path": str(path), "sha256": digest, "bytes": path.stat().st_size}

    def emit(self, state, operation, article, ordinal, total, started, **fields):
        self.count += 1
        now = monotonic()
        row = {"sequence": self.count, "utc": datetime.now(timezone.utc).isoformat(),
               "state": state, "operation": operation, "article": article,
               "ordinal": ordinal, "total": total, "elapsedSeconds": now-self.start,
               "operationSeconds": now-started, "lastRecoverableArchive": self.last_archive, **fields}
        encoded = json.dumps(row, separators=(",",":")) + "\n"
        with self.history.open("a", encoding="utf-8") as history:
            history.write(encoded)
            history.flush()
            os.fsync(history.fileno())
        temporary = self.latest.with_suffix(".tmp")
        with temporary.open("w", encoding="utf-8") as file:
            file.write(encoded)
            file.flush()
            os.fsync(file.fileno())
        os.replace(temporary, self.latest)

    @contextmanager
    def operation(self, name, article, ordinal, total):
        started = monotonic()
        self.emit("before", name, article, ordinal, total, started)
        try:
            yield
        except BaseException as error:
            self.emit("failed", name, article, ordinal, total, started, error=repr(error))
            raise
        self.emit("after", name, article, ordinal, total, started)
