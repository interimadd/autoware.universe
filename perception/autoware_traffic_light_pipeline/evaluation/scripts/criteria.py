#!/usr/bin/env python3

# Copyright 2026 TIER IV, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Minimal port of driving_log_replayer_v2's criteria/perception.py.

Trimmed to the methods that apply to traffic light classification2d evaluation (num_tp,
num_gt_tp, label, metrics_score) and the Distance filter (the only Filter kind the DLR traffic
light scenarios in this repo use). See evaluate_result_implementation_plan.md's Phase 2 for why
this is ported rather than depended on: driving_log_replayer_v2 itself is not a dependency of
this package (its scenario model is pydantic and pulls in the whole DLR node stack), only the
plain scoring logic underneath it.
"""

from __future__ import annotations

from abc import ABC
from abc import abstractmethod
from copy import deepcopy
from enum import Enum
from numbers import Number
from typing import TYPE_CHECKING

import numpy as np
from perception_eval.tool.utils import filter_frame_by_distance

if TYPE_CHECKING:
    from perception_eval.evaluation.result.perception_frame_result import PerceptionFrameResult
    from traffic_light_label import DistanceRange


class SuccessFail(Enum):
    SUCCESS = "Success"
    FAIL = "Fail"

    def __str__(self) -> str:
        return self.value

    def is_success(self) -> bool:
        return self == SuccessFail.SUCCESS

    def __and__(self, other: SuccessFail) -> SuccessFail:
        return SuccessFail.SUCCESS if self.is_success() and other.is_success() else SuccessFail.FAIL


class CriteriaLevel(Enum):
    """PERFECT == 100.0, HARD >= 75.0, NORMAL >= 50.0, EASY >= 25.0, CUSTOM >= value you specify."""

    PERFECT = 100.0
    HARD = 75.0
    NORMAL = 50.0
    EASY = 25.0
    CUSTOM = None

    def is_valid(self, score: Number) -> bool:
        return score >= self.value

    @classmethod
    def from_scenario_value(cls, value: str | Number) -> CriteriaLevel:
        if isinstance(value, str):
            name = value.upper()
            if name not in cls.__members__ or name == "CUSTOM":
                error_msg = f"CriteriaLevel must be perfect, hard, normal or easy, got: {value}"
                raise ValueError(error_msg)
            return cls.__members__[name]
        if isinstance(value, Number):
            min_range, max_range = 0.0, 100.0
            if not (min_range <= value <= max_range):
                error_msg = f"Custom CriteriaLevel must be [0.0, 100.0], got: {value}"
                raise ValueError(error_msg)
            custom = cls.CUSTOM
            custom._value_ = float(value)
            return custom
        error_msg = f"Unsupported CriteriaLevel value: {value!r}"
        raise TypeError(error_msg)


class CriteriaMethod(Enum):
    NUM_TP = "num_tp"
    NUM_GT_TP = "num_gt_tp"
    LABEL = "label"
    METRICS_SCORE = "metrics_score"

    @classmethod
    def from_str(cls, value: str) -> CriteriaMethod:
        name = value.upper()
        if name not in cls.__members__:
            error_msg = f"Unsupported CriteriaMethod: {value}"
            raise ValueError(error_msg)
        return cls.__members__[name]


class CriteriaMethodImpl(ABC):
    def __init__(self, level: CriteriaLevel) -> None:
        self.level = level

    def get_result(self, frame: PerceptionFrameResult) -> tuple[SuccessFail, float] | None:
        """Return None (Not Available) when the frame has neither a matched nor an unmatched object."""
        if not self.has_objects(frame):
            return None
        score = self.calculate_score(frame)
        return (
            (SuccessFail.SUCCESS, score)
            if self.level.is_valid(score)
            else (SuccessFail.FAIL, score)
        )

    @staticmethod
    def has_objects(frame: PerceptionFrameResult) -> bool:
        num_success = frame.pass_fail_result.get_num_success()
        num_fail = frame.pass_fail_result.get_num_fail()
        return num_success + num_fail > 0

    @staticmethod
    @abstractmethod
    def calculate_score(frame: PerceptionFrameResult) -> float: ...


class NumTP(CriteriaMethodImpl):
    name = CriteriaMethod.NUM_TP

    @staticmethod
    def calculate_score(frame: PerceptionFrameResult) -> float:
        num_success = frame.pass_fail_result.get_num_success()
        num_objects = num_success + frame.pass_fail_result.get_num_fail()
        return 100.0 * num_success / num_objects if num_objects != 0 else 100.0


class NumGtTP(CriteriaMethodImpl):
    name = CriteriaMethod.NUM_GT_TP

    @staticmethod
    def calculate_score(frame: PerceptionFrameResult) -> float:
        num_success = frame.pass_fail_result.get_num_success()
        num_gt = frame.pass_fail_result.get_num_gt()
        return 100.0 * num_success / num_gt if num_gt != 0 else 100.0


class Label(CriteriaMethodImpl):
    name = CriteriaMethod.LABEL

    @staticmethod
    def calculate_score(frame: PerceptionFrameResult) -> float:
        is_label_corrects = [
            result.is_label_correct
            for result in frame.object_results
            if result.ground_truth_object is not None
        ]
        return 100.0 if len(is_label_corrects) == 0 else 100.0 * float(np.mean(is_label_corrects))


class MetricsScore(CriteriaMethodImpl):
    name = CriteriaMethod.METRICS_SCORE

    @staticmethod
    def calculate_score(frame: PerceptionFrameResult) -> float:
        scores = [
            acc.accuracy
            for score in frame.metrics_score.classification_scores
            for acc in score.accuracies
            if not np.isnan(acc.accuracy)
        ]
        return 100.0 * sum(scores) / len(scores) if len(scores) != 0 else 0.0


_METHOD_IMPLS: dict[CriteriaMethod, type[CriteriaMethodImpl]] = {
    CriteriaMethod.NUM_TP: NumTP,
    CriteriaMethod.NUM_GT_TP: NumGtTP,
    CriteriaMethod.LABEL: Label,
    CriteriaMethod.METRICS_SCORE: MetricsScore,
}


class CriteriaFilter:
    """Distance-only filter.

    The Region filter DLR also supports is unused by this repo's scenarios, so it is
    intentionally left unported -- add it here if a scenario needs it.
    """

    def __init__(self, distance_range: DistanceRange | None) -> None:
        self.distance_range = distance_range

    def filter_frame_result(self, frame: PerceptionFrameResult) -> PerceptionFrameResult:
        if self.distance_range is None:
            return deepcopy(frame)
        return filter_frame_by_distance(frame, self.distance_range.lower, self.distance_range.upper)


class PerceptionCriteria:
    """One scenario Criterion: a distance filter plus one (method, level) pass/fail rule.

    Unlike DLR's version this only supports a single CriteriaMethod/CriteriaLevel pair per
    Criterion (every traffic light scenario in this repo uses exactly one), not the list form DLR
    also accepts.
    """

    def __init__(
        self, method: CriteriaMethod, level: CriteriaLevel, distance_range: DistanceRange | None
    ) -> None:
        self.method_impl = _METHOD_IMPLS[method](level)
        self.criteria_filter = CriteriaFilter(distance_range)

    def get_result(
        self,
        frame: PerceptionFrameResult,
    ) -> (
        tuple[SuccessFail, float, PerceptionFrameResult] | tuple[None, None, PerceptionFrameResult]
    ):
        """Return (None, None, filtered_frame) for a NoGTNoObj frame (excluded from the pass rate)."""
        filtered_frame = self.criteria_filter.filter_frame_result(frame)
        method_result = self.method_impl.get_result(filtered_frame)
        if method_result is None:
            return None, None, filtered_frame
        success_fail, score = method_result
        return success_fail, score, filtered_frame
