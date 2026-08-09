"""Explicit effect-node registry used by GPSR supervision."""
from __future__ import annotations

from dataclasses import replace
from typing import Any, Iterable

from .models import EffectRisk, NodeContract


class NodeContractRegistry:
    """Resolve only explicitly allow-listed external-effect node classes.

    Exact base classes are also honoured, which covers trusted subclasses such
    as ``BtNode_GraspWithPose`` without relying on class-name substrings.
    """

    def __init__(self, contracts: Iterable[NodeContract] = ()) -> None:
        self._contracts = {contract.class_name: contract for contract in contracts}

    def register(self, contract: NodeContract) -> None:
        self._contracts[contract.class_name] = contract

    def contract_for(self, node_or_type: Any) -> NodeContract | None:
        node_type = node_or_type if isinstance(node_or_type, type) else type(node_or_type)
        for candidate in node_type.__mro__:
            contract = self._contracts.get(candidate.__name__)
            if contract is not None:
                if candidate is node_type:
                    return contract
                return replace(contract, class_name=node_type.__name__)
        return None

    def require(self, node_or_type: Any) -> NodeContract:
        contract = self.contract_for(node_or_type)
        if contract is None:
            name = node_or_type.__name__ if isinstance(node_or_type, type) else type(node_or_type).__name__
            raise KeyError(f"no GPSR effect contract registered for {name}")
        return contract

    def names(self) -> frozenset[str]:
        return frozenset(self._contracts)


def default_node_contracts() -> NodeContractRegistry:
    contracts: list[NodeContract] = []

    def add(
        names: Iterable[str],
        effect: str,
        risk: EffectRisk,
        expected: str,
        evidence: tuple[str, ...],
        *,
        recovery: bool = True,
    ) -> None:
        for name in names:
            contracts.append(
                NodeContract(
                    class_name=name,
                    effect=effect,
                    risk=risk,
                    expected_postcondition=expected,
                    evidence_modalities=evidence,
                    allow_local_recovery=recovery,
                )
            )

    add(
        (
            "BtNode_GotoAction",
            "BtNode_Approach",
            "BtNode_NavBack",
            "BtNode_FollowAction",
            "BtNode_GoToLuggage",
        ),
        "navigation",
        EffectRisk.REVERSIBLE_MOTION,
        "The mobile base reached the requested destination or relative pose.",
        ("map", "front_camera", "status"),
    )
    add(
        ("BtNode_CaptureCurrentPose",),
        "localization",
        EffectRisk.OBSERVATION,
        "A current map-frame robot pose was captured.",
        ("map", "blackboard", "status"),
    )
    add(
        (
            "BtNode_ScanFor",
            "BtNode_ScanForGeneralist",
            "BtNode_ObjectScan",
            "BtNode_TrackPerson",
            "BtNode_TrackPersonAction",
            "BtNode_FindObj",
            "BtNode_FeatureExtraction",
            "BtNode_SeatRecommend",
            "BtNode_SeatRecommendBbox",
            "BtNode_FeatureMatching",
            "BtNode_GetPointCloud",
            "BtNode_FindPlacingLocation",
            "BtNode_DoorDetection",
            "BtNode_ScanForWavingPerson",
            "BtNode_ScanForWavingPersonNew",
            "BtNode_GetImage",
        ),
        "perception",
        EffectRisk.OBSERVATION,
        "The requested observation was completed and its result was written to the blackboard.",
        ("front_camera", "wrist_camera", "blackboard", "status"),
    )
    add(
        (
            "BtNode_TurnPanTilt",
            "BtNode_TurnTo",
            "BtNode_MaintainEyeContact",
            "BtNode_RecoveryScan",
        ),
        "camera_motion",
        EffectRisk.OBSERVATION,
        "The camera or gaze mechanism reached the requested view.",
        ("front_camera", "status"),
    )
    add(
        (
            "BtNode_MoveArm",
            "BtNode_MoveArmSingle",
            "BtNode_CartesianMove",
            "BtNode_JointMoveAction",
            "BtNode_PointTo",
            "BtNode_PointToPoseStamped",
        ),
        "arm_motion",
        EffectRisk.REVERSIBLE_MOTION,
        "The arm reached the requested joint or Cartesian pose without unsafe contact.",
        ("front_camera", "wrist_camera", "arm", "status"),
    )
    add(
        (
            "BtNode_Grasp",
            "BtNode_GraspWithPose",
            "BtNode_Drop",
            "BtNode_Place",
            "BtNode_GripperAction",
            "BtNode_ScanAndPlace",
            "BtNode_FoldClothingAction",
            "BtNode_FoldClothingDn",
        ),
        "manipulation",
        EffectRisk.IRREVERSIBLE,
        "The requested object transfer or gripper interaction occurred exactly once and remains safe.",
        ("front_camera", "wrist_camera", "arm", "blackboard", "status"),
        recovery=False,
    )
    add(
        (
            "BtNode_Announce",
            "BtNode_TTSCN",
            "BtNode_AnnounceFromBB",
            "BtNode_ReacqAnnounce",
        ),
        "human_output",
        EffectRisk.OBSERVATION,
        "The intended message was delivered by the speech service.",
        ("blackboard", "status"),
    )
    add(
        (
            "BtNode_WaitForStart",
            "BtNode_GraspRequest",
            "BtNode_GetConfirmation",
            "BtNode_GetConfirmationAction",
            "BtNode_Listen",
            "BtNode_ListenAction",
        ),
        "human_input",
        EffectRisk.OBSERVATION,
        "The requested human response was captured and stored.",
        ("blackboard", "status"),
    )
    return NodeContractRegistry(contracts)


__all__ = ["NodeContractRegistry", "default_node_contracts"]
