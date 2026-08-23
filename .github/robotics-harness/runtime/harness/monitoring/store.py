"""Append-only JSON Lines event persistence."""

from __future__ import annotations

import json
import os
from pathlib import Path

from harness.monitoring.models import MonitoringEvent, json_dumps


class EventStoreError(RuntimeError):
    """Raised when persisted monitoring evidence cannot be decoded."""


class EventStore:
    """A transparent local event store suitable for prototypes and tests."""

    def __init__(self, path: Path | str) -> None:
        self.path = Path(path)

    def _last_event_sha256(self) -> str:
        events = self.read_all()
        return events[-1].event_sha256 if events else ""

    def append(self, event: MonitoringEvent) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
        if self.path.is_symlink():
            raise EventStoreError("event store path cannot be a symbolic link")
        payload_event = MonitoringEvent.create(
            event_type=event.event_type,
            session_id=event.session_id,
            student_identifier=event.student_identifier,
            device_identifier=event.device_identifier,
            repository=event.repository,
            branch=event.branch,
            task_identifier=event.task_identifier,
            tool_or_command=event.tool_or_command,
            files_affected=event.files_affected,
            result=event.result,
            risk_level=event.risk_level,
            policy_status=event.policy_status,
            commit_or_pr_reference=event.commit_or_pr_reference,
            reviewer_identifier=event.reviewer_identifier,
            actor_role=event.actor_role,
            identity_assurance=event.identity_assurance,
            status=event.status,
            summary=event.summary,
            references=event.references,
            previous_event_sha256=self._last_event_sha256(),
            metadata=event.metadata,
            timestamp=event.timestamp,
            event_id=event.event_id,
        )
        sealed = MonitoringEvent.create(
            event_type=payload_event.event_type,
            session_id=payload_event.session_id,
            student_identifier=payload_event.student_identifier,
            device_identifier=payload_event.device_identifier,
            repository=payload_event.repository,
            branch=payload_event.branch,
            task_identifier=payload_event.task_identifier,
            tool_or_command=payload_event.tool_or_command,
            files_affected=payload_event.files_affected,
            result=payload_event.result,
            risk_level=payload_event.risk_level,
            policy_status=payload_event.policy_status,
            commit_or_pr_reference=payload_event.commit_or_pr_reference,
            reviewer_identifier=payload_event.reviewer_identifier,
            actor_role=payload_event.actor_role,
            identity_assurance=payload_event.identity_assurance,
            status=payload_event.status,
            summary=payload_event.summary,
            references=payload_event.references,
            previous_event_sha256=payload_event.previous_event_sha256,
            event_sha256=payload_event.payload_sha256(),
            metadata=payload_event.metadata,
            timestamp=payload_event.timestamp,
            event_id=payload_event.event_id,
        )
        payload = json_dumps(sealed.to_dict())
        flags = os.O_WRONLY | os.O_CREAT | os.O_APPEND
        if hasattr(os, "O_NOFOLLOW"):
            flags |= os.O_NOFOLLOW
        descriptor = os.open(
            self.path,
            flags,
            0o600,
        )
        os.chmod(self.path, 0o600)
        with os.fdopen(descriptor, "a", encoding="utf-8") as stream:
            stream.write(payload)
            stream.write("\n")
            stream.flush()

    def read_all(self) -> tuple[MonitoringEvent, ...]:
        if not self.path.exists():
            return ()
        events: list[MonitoringEvent] = []
        prior_sha = ""
        integrity_started = False
        for line_number, line in enumerate(
            self.path.read_text(encoding="utf-8").splitlines(), start=1
        ):
            if not line.strip():
                continue
            try:
                value = json.loads(line)
                if not isinstance(value, dict):
                    raise ValueError("event must be an object")
                event = MonitoringEvent.from_dict(value)
                if event.event_sha256:
                    integrity_started = True
                    if event.previous_event_sha256 != prior_sha:
                        raise EventStoreError(
                            f"invalid event at {self.path}:{line_number}: integrity chain mismatch"
                        )
                    if event.payload_sha256() != event.event_sha256:
                        raise EventStoreError(
                            f"invalid event at {self.path}:{line_number}: integrity digest mismatch"
                        )
                    prior_sha = event.event_sha256
                else:
                    if integrity_started:
                        raise EventStoreError(
                            f"invalid event at {self.path}:{line_number}: "
                            "legacy unhashed event cannot follow integrity-chained events"
                        )
                    prior_sha = ""
                events.append(event)
            except (json.JSONDecodeError, TypeError, ValueError) as error:
                raise EventStoreError(
                    f"invalid event at {self.path}:{line_number}: {error}"
                ) from error
        return tuple(events)
