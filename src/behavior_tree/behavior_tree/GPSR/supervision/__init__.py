"""Multimodal execution supervision for GPSR."""

from .models import (
    ArtifactRef,
    BtAssessment,
    CaptureRequest,
    EffectRisk,
    Escalation,
    GlobalAction,
    GlobalPlanDecision,
    NodeContract,
    RecoveryKind,
    RecoveryProposal,
    ReportedStatus,
    SnapshotBundle,
    SubtaskStatus,
    SuccessMode,
    SupervisionMode,
    SupervisorConfig,
    Verdict,
    VerificationDecision,
    WorldChange,
)
from .clients import (
    OpenRouterSupervisorClient,
    ScriptedSupervisorClient,
    SupervisorClient,
)
from .context import (
    ContextProvider,
    FixtureContextProvider,
    StaticContextProvider,
    gpsr_named_pose,
)
from .contracts import NodeContractRegistry, default_node_contracts
from .controller import MissionSupervisor, SupervisorIntervention
from .recovery import RecoveryLedger, RecoveryMacroCompiler
from .scenarios import (
    SCENARIO_CASES,
    ScenarioCase,
    ScenarioStage,
    build_capture_request,
    iter_stages,
)

__all__ = [
    "ArtifactRef",
    "BtAssessment",
    "CaptureRequest",
    "EffectRisk",
    "Escalation",
    "GlobalAction",
    "GlobalPlanDecision",
    "ContextProvider",
    "FixtureContextProvider",
    "MissionSupervisor",
    "NodeContract",
    "NodeContractRegistry",
    "OpenRouterSupervisorClient",
    "RecoveryKind",
    "RecoveryLedger",
    "RecoveryMacroCompiler",
    "RecoveryProposal",
    "ReportedStatus",
    "SCENARIO_CASES",
    "ScenarioCase",
    "ScenarioStage",
    "ScriptedSupervisorClient",
    "SnapshotBundle",
    "StaticContextProvider",
    "SubtaskStatus",
    "SuccessMode",
    "SupervisionMode",
    "SupervisorConfig",
    "SupervisorClient",
    "SupervisorIntervention",
    "Verdict",
    "VerificationDecision",
    "WorldChange",
    "default_node_contracts",
    "build_capture_request",
    "gpsr_named_pose",
    "iter_stages",
]
