"""Create evidence-bearing, deterministic student diff reviews."""

from __future__ import annotations

import json
import re
import uuid
from dataclasses import asdict, dataclass
from datetime import UTC, datetime
from pathlib import Path

from harness.git import ChangedFile
from harness.monitoring.models import EventType, MonitoringEvent, format_timestamp
from harness.monitoring.store import EventStore
from harness.private_io import write_private_text
from harness.risk import RiskAssessment, RiskClassifier

_PYTHON_FUNCTION_PATTERN = re.compile(
    r"^[+ ]\s*(?:async\s+)?def\s+([A-Za-z_][A-Za-z0-9_]*)\s*\(([^)]*)\)",
    re.MULTILINE,
)
_JAVA_METHOD_PATTERN = re.compile(
    r"^[+ ]\s*"
    r"(?:(?:public|protected|private|static|final|abstract|synchronized|native|default)\s+)*"
    r"(?:<[^>{}]+>\s+)?"
    r"[A-Za-z_$][A-Za-z0-9_$<>,.? \[\]]*\s+"
    r"([A-Za-z_$][A-Za-z0-9_$]*)\s*\(([^;{}]*)\)\s*(?:throws\s+[^{]+)?\{?",
    re.MULTILINE,
)
_JAVA_CONTROL_WORDS = {"if", "for", "while", "switch", "catch", "return", "new"}
_GENERIC_ANSWERS = {
    "i understand",
    "i understand the code",
    "looks good",
    "yes",
    "no",
    "ok",
    "okay",
    "done",
    "n/a",
}
_CAUSAL_WORDS = {
    "because",
    "ensures",
    "guards",
    "if",
    "prevents",
    "so",
    "therefore",
    "when",
}


class AnswerValidationError(ValueError):
    """Raised when a review answer is generic or evidence is incomplete."""


@dataclass(frozen=True, slots=True)
class DiffSection:
    identifier: str
    file: str
    part: int
    line_count: int
    content: str


@dataclass(frozen=True, slots=True)
class ReviewQuestion:
    identifier: str
    prompt: str
    file: str
    section_identifier: str
    category: str = "behaviour"
    anchors: tuple[str, ...] = ()
    expected_concepts: tuple[str, ...] = ()
    follow_up_prompt: str = ""


@dataclass(frozen=True, slots=True)
class QuestionAnswer:
    question_id: str
    answer: str


@dataclass(frozen=True, slots=True)
class AnswerEvaluation:
    question_id: str
    status: str
    score: int
    maximum_score: int
    matched_anchors: tuple[str, ...]
    matched_concepts: tuple[str, ...]
    missing_concepts: tuple[str, ...]
    feedback: str
    follow_up_prompt: str = ""


@dataclass(frozen=True, slots=True)
class VerificationAction:
    action: str
    result: str
    evidence_references: tuple[str, ...]


@dataclass(frozen=True, slots=True)
class ReviewRequest:
    task_identifier: str
    student_identifier: str
    repository: str
    branch: str
    commit: str
    changes: tuple[ChangedFile, ...]
    answers: tuple[str, ...]
    verification_action: str
    verification_result: str
    evidence_references: tuple[str, ...]


@dataclass(frozen=True, slots=True)
class ReviewRecord:
    review_id: str
    task_identifier: str
    student_identifier: str
    repository: str
    branch: str
    commit: str
    changed_files: tuple[str, ...]
    risk_classification: str
    risk_matches: tuple[dict[str, str], ...]
    sections: tuple[DiffSection, ...]
    questions: tuple[ReviewQuestion, ...]
    answers: tuple[QuestionAnswer, ...]
    answer_evaluations: tuple[AnswerEvaluation, ...]
    verification_action: VerificationAction
    completion_state: str
    timestamp: datetime

    def to_dict(self) -> dict[str, object]:
        value = asdict(self)
        value["timestamp"] = format_timestamp(self.timestamp)
        return value


def split_diff(changes: tuple[ChangedFile, ...], max_lines: int = 80) -> tuple[DiffSection, ...]:
    """Split each file diff into bounded, sequential review sections."""

    if max_lines < 10:
        raise ValueError("max_lines must be at least 10")
    sections: list[DiffSection] = []
    for change in changes:
        lines = change.patch.splitlines()
        if not lines:
            lines = [f"diff for {change.path} has no textual content"]
        for offset in range(0, len(lines), max_lines):
            part = offset // max_lines + 1
            content = "\n".join(lines[offset : offset + max_lines])
            sections.append(
                DiffSection(
                    identifier=f"section-{len(sections) + 1}",
                    file=change.path,
                    part=part,
                    line_count=len(content.splitlines()),
                    content=content,
                )
            )
    return tuple(sections)


def _questions_for_section(section: DiffSection) -> list[str]:
    functions = [
        *_PYTHON_FUNCTION_PATTERN.findall(section.content),
        *[
            (name, inputs)
            for name, inputs in _JAVA_METHOD_PATTERN.findall(section.content)
            if name not in _JAVA_CONTROL_WORDS
        ],
    ]
    functions = list(dict.fromkeys(functions))
    questions: list[str] = []
    if functions:
        names = ", ".join(name for name, _inputs in functions[:3])
        questions.append(f"Which changed function ({names}) is relevant, and what does it do?")
        first_name, inputs = functions[0]
        questions.append(
            f"What inputs does {first_name} receive"
            + (f" ({inputs.strip()})" if inputs.strip() else "")
            + ", and which outputs or robot behaviour can change?"
        )
    else:
        questions.append(
            f"What behaviour or repository rule can change because of the edits in {section.file}?"
        )
    lowered = section.content.lower()
    if "min(" in lowered or "clamp" in lowered or "limit" in lowered:
        questions.append("Which condition or calculation limits the requested value?")
    if "max_drive_speed" in lowered or "/safety/" in section.file.lower():
        questions.append(
            "Which file contains the protected maximum, and who must approve changes to it?"
        )
        questions.append("What could happen if the protected speed check were removed?")
    if "test" in section.file.lower() or "assert" in lowered:
        questions.append("Which test validates this behaviour, and what failure would it catch?")
    else:
        questions.append("What test or verification action demonstrates the changed behaviour?")
    return questions[:4]


def _tokenize(value: str) -> tuple[str, ...]:
    tokens: list[str] = []
    for token in re.findall(r"[A-Za-z0-9_]+", value):
        lowered = token.lower()
        tokens.append(lowered)
        parts = re.findall(r"[A-Z]?[a-z]+|[A-Z]+(?![a-z])|\d+", token)
        tokens.extend(part.lower() for part in parts if part)
        tokens.extend(part.lower() for part in token.split("_") if part)
    return tuple(dict.fromkeys(tokens))


def _section_methods(section: DiffSection) -> list[tuple[str, str]]:
    return list(
        dict.fromkeys(
            [
                *_PYTHON_FUNCTION_PATTERN.findall(section.content),
                *[
                    (name, inputs)
                    for name, inputs in _JAVA_METHOD_PATTERN.findall(section.content)
                    if name not in _JAVA_CONTROL_WORDS
                ],
            ]
        )
    )


def _section_constants(section: DiffSection) -> tuple[str, ...]:
    matches = re.findall(
        r"\b([A-Z][A-Z0-9_]{2,})\b",
        section.content,
    )
    return tuple(dict.fromkeys(matches[:6]))


def _question_budget(*, risk_level: str, section_count: int) -> int:
    base = {
        "low": 2,
        "medium": 3,
        "high": 4,
        "critical": 5,
    }.get(risk_level, 3)
    return min(max(base, 1), max(base + section_count - 1, base), 7)


def _build_question(
    *,
    section: DiffSection,
    category: str,
    prompt: str,
    anchors: tuple[str, ...],
    expected_concepts: tuple[str, ...],
    follow_up_prompt: str,
) -> ReviewQuestion:
    return ReviewQuestion(
        identifier="",
        prompt=prompt,
        file=section.file,
        section_identifier=section.identifier,
        category=category,
        anchors=anchors,
        expected_concepts=expected_concepts,
        follow_up_prompt=follow_up_prompt,
    )


def _questions_with_metadata(section: DiffSection) -> list[ReviewQuestion]:
    methods = _section_methods(section)
    constants = _section_constants(section)
    lowered = section.content.lower()
    file_stem = Path(section.file).stem
    primary_method = methods[0][0] if methods else file_stem
    primary_inputs = methods[0][1].strip() if methods else ""
    method_anchors = tuple(item for item in (primary_method, file_stem, *constants[:2]) if item)
    questions: list[ReviewQuestion] = [
        _build_question(
            section=section,
            category="design_rationale",
            prompt=(
                f"Why was `{primary_method}` changed in {section.file}, and what concrete "
                "behaviour does this diff now enforce?"
            ),
            anchors=method_anchors,
            expected_concepts=tuple(
                item
                for item in (
                    primary_method,
                    file_stem,
                    "behaviour",
                    "change",
                )
                if item
            ),
            follow_up_prompt=(
                f"Point to the exact changed condition in `{primary_method}` and describe the "
                "before/after behaviour."
            ),
        )
    ]
    if primary_inputs:
        questions.append(
            _build_question(
                section=section,
                category="control_flow",
                prompt=(
                    f"How do the inputs to `{primary_method}` ({primary_inputs}) flow through the "
                    "changed code, and which branch or calculation now decides the output?"
                ),
                anchors=method_anchors,
                expected_concepts=tuple(
                    item
                    for item in (
                        primary_method,
                        "input",
                        "output",
                        "branch",
                        "calculation",
                    )
                    if item
                ),
                follow_up_prompt=(
                    f"Name the exact branch, guard, or calculation inside `{primary_method}` "
                    "that controls the result."
                ),
            )
        )
    if "min(" in lowered or "clamp" in lowered or "limit" in lowered or constants:
        questions.append(
            _build_question(
                section=section,
                category="safety_constraints",
                prompt=(
                    f"Which safety or configuration bound in {section.file} constrains the new "
                    "behaviour, and what hardware or robot consequence would follow if that "
                    "constraint were bypassed?"
                ),
                anchors=tuple(item for item in (*constants, file_stem, primary_method) if item),
                expected_concepts=tuple(
                    item
                    for item in (
                        *constants[:2],
                        "limit",
                        "safety",
                        "constraint",
                    )
                    if item
                ),
                follow_up_prompt=(
                    "Identify the exact constant or guard and explain the unsafe outcome if it "
                    "were removed."
                ),
            )
        )
    if (
        any(token in lowered for token in ("assert", "test", "junit"))
        or "test" in section.file.lower()
    ):
        questions.append(
            _build_question(
                section=section,
                category="tests",
                prompt=(
                    f"Which test in {section.file} or its paired source now proves the intended "
                    "behaviour, and what regression would it catch if the implementation drifted?"
                ),
                anchors=tuple(
                    item for item in (file_stem, primary_method, "test", "assert") if item
                ),
                expected_concepts=("test", "assert", "regression", primary_method),
                follow_up_prompt=(
                    "Name the exact assertion or scenario and the failure it would report."
                ),
            )
        )
    else:
        questions.append(
            _build_question(
                section=section,
                category="verification",
                prompt=(
                    f"What verification action should demonstrate the changed behaviour in "
                    f"{section.file}, and what result would show the change is correct?"
                ),
                anchors=tuple(item for item in (file_stem, primary_method, "test") if item),
                expected_concepts=("verify", "result", "behaviour", primary_method),
                follow_up_prompt=(
                    "Tie the verification to one changed method or condition "
                    "and the expected result."
                ),
            )
        )
    if any(token in lowered for token in ("throw", "catch", "null", "blank", "optional", "fail")):
        questions.append(
            _build_question(
                section=section,
                category="failure_behaviour",
                prompt=(
                    f"What does the changed code in {section.file} do on invalid, missing, or "
                    "unexpected input, and why is that failure behaviour safe?"
                ),
                anchors=tuple(
                    item for item in (file_stem, primary_method, "invalid", "safe") if item
                ),
                expected_concepts=("invalid", "input", "safe", "failure"),
                follow_up_prompt=(
                    "Describe the exact invalid-input path and the resulting safe behaviour."
                ),
            )
        )
    return questions


def generate_questions(
    sections: tuple[DiffSection, ...],
    *,
    risk_level: str = "low",
    max_questions: int | None = None,
) -> tuple[ReviewQuestion, ...]:
    limit = max_questions or _question_budget(risk_level=risk_level, section_count=len(sections))
    candidates: list[ReviewQuestion] = []
    seen_categories: set[str] = set()
    prioritized_sections = sorted(
        sections,
        key=lambda section: (
            0
            if any(
                token in section.content.lower() or token in section.file.lower()
                for token in ("safety", "disable", "interlock", "limit", "command")
            )
            else 1
            if not (
                "test" in section.file.lower()
                or section.file.lower().endswith((".md", ".rst", ".adoc"))
            )
            else 2,
            section.file,
            section.part,
        ),
    )
    for section in prioritized_sections:
        for question in _questions_with_metadata(section):
            if len(candidates) >= limit:
                break
            if question.category in seen_categories and len(sections) == 1:
                continue
            seen_categories.add(question.category)
            candidates.append(question)
        if len(candidates) >= limit:
            break
    if not candidates:
        for section in sections[:1]:
            candidates.append(
                _build_question(
                    section=section,
                    category="behaviour",
                    prompt=(
                        f"What behaviour or repository rule can change because of the edits in "
                        f"{section.file}?"
                    ),
                    anchors=(Path(section.file).stem,),
                    expected_concepts=("behaviour", Path(section.file).stem),
                    follow_up_prompt="Describe one concrete before/after effect from the diff.",
                )
            )
    return tuple(
        ReviewQuestion(
            identifier=f"question-{index}",
            prompt=question.prompt,
            file=question.file,
            section_identifier=question.section_identifier,
            category=question.category,
            anchors=question.anchors,
            expected_concepts=question.expected_concepts,
            follow_up_prompt=question.follow_up_prompt,
        )
        for index, question in enumerate(candidates, start=1)
    )


def validate_answer(answer: str) -> None:
    normalized = " ".join(answer.lower().strip().rstrip(".!").split())
    if normalized in _GENERIC_ANSWERS or normalized.startswith("i understand"):
        raise AnswerValidationError(
            "generic confirmation is not evidence; describe the code or behaviour"
        )
    words = re.findall(r"[A-Za-z0-9_]+", normalized)
    if len(words) < 5 or len(set(words)) < 4:
        raise AnswerValidationError(
            "answer must contain at least five substantive words about the code"
        )


def evaluate_answer(
    question: ReviewQuestion,
    answer: str,
    *,
    minimum_score: int = 3,
) -> AnswerEvaluation:
    validate_answer(answer)
    tokens = set(_tokenize(answer))
    anchor_tokens = {token for token in _tokenize(" ".join(question.anchors)) if len(token) > 2}
    concept_tokens = {
        token for token in _tokenize(" ".join(question.expected_concepts)) if len(token) > 2
    }
    matched_anchors = tuple(anchor for anchor in question.anchors if _tokenize(anchor)[0] in tokens)
    matched_concepts = tuple(
        concept for concept in question.expected_concepts if _tokenize(concept)[0] in tokens
    )
    score = 0
    words = _tokenize(answer)
    if len(words) >= 8:
        score += 1
    if matched_anchors or anchor_tokens & tokens:
        score += 1
    if matched_concepts or len(concept_tokens & tokens) >= max(1, min(2, len(concept_tokens))):
        score += 1
    if any(word in tokens for word in _CAUSAL_WORDS):
        score += 1
    missing = tuple(
        concept for concept in question.expected_concepts if concept not in matched_concepts
    )
    passed = score >= minimum_score and bool(matched_anchors or matched_concepts)
    feedback = (
        "answer is aligned with the changed code"
        if passed
        else (
            "answer needs stronger code grounding; reference the changed symbol, the controlling "
            "condition, and the concrete behaviour or test impact"
        )
    )
    follow_up = "" if passed else question.follow_up_prompt
    return AnswerEvaluation(
        question_id=question.identifier,
        status="aligned" if passed else "weak",
        score=score,
        maximum_score=4,
        matched_anchors=matched_anchors,
        matched_concepts=matched_concepts,
        missing_concepts=missing,
        feedback=feedback,
        follow_up_prompt=follow_up,
    )


class ReviewEngine:
    """Generate, validate, persist, and report a completed diff walkthrough."""

    def __init__(
        self,
        *,
        classifier: RiskClassifier | None = None,
        output_directory: Path | str = "artifacts/reviews",
        event_store: EventStore | None = None,
    ) -> None:
        self.classifier = classifier or RiskClassifier()
        self.output_directory = Path(output_directory)
        self.event_store = event_store

    def prepare(
        self, changes: tuple[ChangedFile, ...]
    ) -> tuple[tuple[DiffSection, ...], tuple[ReviewQuestion, ...], RiskAssessment]:
        if not changes:
            raise ValueError("the diff contains no changed files")
        sections = split_diff(changes)
        return sections, generate_questions(sections), self.classifier.classify(changes)

    def complete(self, request: ReviewRequest) -> tuple[ReviewRecord, Path, Path]:
        sections, questions, assessment = self.prepare(request.changes)
        if len(request.answers) != len(questions):
            raise AnswerValidationError(
                f"expected {len(questions)} answers, received {len(request.answers)}"
            )
        evaluations = tuple(
            evaluate_answer(question, answer)
            for question, answer in zip(questions, request.answers, strict=True)
        )
        weak = [evaluation for evaluation in evaluations if evaluation.status != "aligned"]
        if weak:
            raise AnswerValidationError(
                "answers did not align with changed code: "
                + ", ".join(evaluation.question_id for evaluation in weak)
            )
        if not request.verification_action.strip():
            raise AnswerValidationError("at least one verification action is required")
        if not request.verification_result.strip():
            raise AnswerValidationError("the verification result is required")
        if not request.evidence_references:
            raise AnswerValidationError("verification requires at least one evidence reference")

        record = ReviewRecord(
            review_id=str(uuid.uuid4()),
            task_identifier=request.task_identifier,
            student_identifier=request.student_identifier,
            repository=request.repository,
            branch=request.branch,
            commit=request.commit,
            changed_files=tuple(change.path for change in request.changes),
            risk_classification=assessment.risk.label,
            risk_matches=tuple(match.to_dict() for match in assessment.matches),
            sections=sections,
            questions=questions,
            answers=tuple(
                QuestionAnswer(question.identifier, answer)
                for question, answer in zip(questions, request.answers, strict=True)
            ),
            answer_evaluations=evaluations,
            verification_action=VerificationAction(
                action=request.verification_action,
                result=request.verification_result,
                evidence_references=request.evidence_references,
            ),
            completion_state="completed",
            timestamp=datetime.now(UTC),
        )
        record_path = self.output_directory / f"{record.review_id}.json"
        report_path = self.output_directory / f"{record.review_id}.md"
        write_private_text(
            record_path,
            json.dumps(record.to_dict(), indent=2, sort_keys=True) + "\n",
        )
        write_private_text(report_path, self.mentor_report(record))
        self._record_events(record, record_path)
        return record, record_path, report_path

    def _record_events(self, record: ReviewRecord, record_path: Path) -> None:
        if self.event_store is None:
            return
        self.event_store.append(
            MonitoringEvent.create(
                event_type=EventType.LEARNING_LOOP_STARTED,
                session_id=f"learning-{record.review_id}",
                student_identifier=record.student_identifier,
                repository=record.repository,
                branch=record.branch,
                task_identifier=record.task_identifier,
                commit_or_pr_reference=record.commit,
                result="started",
            )
        )
        self.event_store.append(
            MonitoringEvent.create(
                event_type=EventType.LEARNING_LOOP_COMPLETED,
                session_id=f"learning-{record.review_id}",
                student_identifier=record.student_identifier,
                repository=record.repository,
                branch=record.branch,
                task_identifier=record.task_identifier,
                commit_or_pr_reference=record.commit,
                result="completed",
                metadata={"review_record": str(record_path)},
            )
        )

    @staticmethod
    def mentor_report(record: ReviewRecord) -> str:
        lines = [
            f"# Learning-loop review {record.review_id}",
            "",
            f"- Task: `{record.task_identifier}`",
            f"- Student identifier: `{record.student_identifier}`",
            f"- Repository: `{record.repository}`",
            f"- Branch / commit: `{record.branch}` / `{record.commit}`",
            f"- Risk: **{record.risk_classification}**",
            f"- Completion: **{record.completion_state}**",
            "",
            "## Changed files",
            "",
        ]
        lines.extend(f"- `{path}`" for path in record.changed_files)
        lines.extend(["", "## Walkthrough evidence", ""])
        answer_by_id = {answer.question_id: answer.answer for answer in record.answers}
        evaluation_by_id = {
            evaluation.question_id: evaluation for evaluation in record.answer_evaluations
        }
        for question in record.questions:
            evaluation = evaluation_by_id[question.identifier]
            lines.extend(
                [
                    f"### {question.identifier}: {question.file}",
                    "",
                    question.prompt,
                    "",
                    f"> {answer_by_id[question.identifier]}",
                    "",
                    (
                        f"Evaluation: **{evaluation.status}** "
                        f"({evaluation.score}/{evaluation.maximum_score}) — "
                        f"{evaluation.feedback}"
                    ),
                    "",
                ]
            )
        lines.extend(
            [
                "## Verification",
                "",
                f"- Action: `{record.verification_action.action}`",
                f"- Result: {record.verification_action.result}",
                "- Evidence:",
            ]
        )
        lines.extend(
            f"  - `{reference}`" for reference in record.verification_action.evidence_references
        )
        return "\n".join(lines) + "\n"
