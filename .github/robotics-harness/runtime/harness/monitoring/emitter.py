"""Optional sanitized monitoring-event emission for standalone services."""

from __future__ import annotations

import os
from pathlib import Path
from typing import Any

from harness.monitoring.models import EventType, MonitoringEvent
from harness.monitoring.store import EventStore

EVENT_STORE_ENV = "HARNESS_MONITORING_EVENT_STORE"


def emit_service_event(
    event_type: EventType,
    *,
    repository: str,
    result: str,
    metadata: dict[str, Any] | None = None,
    branch: str = "",
    task_identifier: str = "",
    event_store: Path | str | None = None,
) -> None:
    """Append one event when an explicit store is configured; otherwise do nothing."""

    configured = str(event_store or os.environ.get(EVENT_STORE_ENV, "")).strip()
    if not configured:
        return
    EventStore(Path(configured)).append(
        MonitoringEvent.create(
            event_type=event_type,
            session_id="repository-governance-services",
            student_identifier="system-service",
            repository=repository,
            branch=branch,
            task_identifier=task_identifier,
            result=result,
            metadata={} if metadata is None else metadata,
        )
    )
