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

"""Ported label-string conversion, so the estimated side matches what DLR feeds perception_eval.

Ported from driving_log_replayer_v2's traffic_light.py (TRAFFIC_LIGHT_LABEL_MAPPINGS /
get_traffic_light_label_str). Kept dependency-free (pure function of the message's color/shape
fields) so it can be unit-tested without perception_eval or a ROS runtime.

This workspace's autoware_perception_msgs renamed TrafficSignalElement to TrafficLightElement, but
the color/shape/status constant values are unchanged, so this module type-hints against the plain
element fields (color: int, shape: int) instead of importing either message type.
"""

from __future__ import annotations

from typing import NamedTuple
from typing import Protocol

# autoware_perception_msgs/msg/TrafficLightElement constants (color).
COLOR_RED = 1
COLOR_AMBER = 2
COLOR_GREEN = 3

# autoware_perception_msgs/msg/TrafficLightElement constants (shape).
SHAPE_CIRCLE = 1
SHAPE_LEFT_ARROW = 2
SHAPE_RIGHT_ARROW = 3
SHAPE_UP_ARROW = 4
SHAPE_UP_LEFT_ARROW = 5
SHAPE_UP_RIGHT_ARROW = 6
SHAPE_DOWN_ARROW = 7
SHAPE_DOWN_LEFT_ARROW = 8
SHAPE_DOWN_RIGHT_ARROW = 9


class TrafficLightElementLike(Protocol):
    color: int
    shape: int


TRAFFIC_LIGHT_LABEL_MAPPINGS: list[tuple[frozenset, str]] = [
    (frozenset({"green"}), "green"),
    (frozenset({"green", "straight"}), "green_straight"),
    (frozenset({"green", "left"}), "green_left"),
    (frozenset({"green", "right"}), "green_right"),
    (frozenset({"yellow"}), "yellow"),
    (frozenset({"yellow", "straight"}), "yellow_straight"),
    (frozenset({"yellow", "left"}), "yellow_left"),
    (frozenset({"yellow", "right"}), "yellow_right"),
    (frozenset({"yellow", "straight", "left"}), "yellow_straight_left"),
    (frozenset({"yellow", "straight", "right"}), "yellow_straight_right"),
    (frozenset({"red"}), "red"),
    (frozenset({"red", "straight"}), "red_straight"),
    (frozenset({"red", "left"}), "red_left"),
    (frozenset({"red", "right"}), "red_right"),
    (frozenset({"red", "straight", "left"}), "red_straight_left"),
    (frozenset({"red", "straight", "right"}), "red_straight_right"),
    (frozenset({"red", "straight", "left", "right"}), "red_straight_left_right"),
    (frozenset({"red", "right", "diagonal"}), "red_rightdiagonal"),
    (frozenset({"red", "left", "diagonal"}), "red_leftdiagonal"),
]


def get_traffic_light_label_str(elements: list[TrafficLightElementLike]) -> str:  # noqa: C901
    """Convert a TrafficLightGroup's elements into the label string perception_eval expects.

    Mirrors driving_log_replayer_v2.traffic_light.get_traffic_light_label_str(). Unmapped
    combinations (including the empty set, e.g. an all-off signal) return "unknown".
    """
    label_infos: set[str] = set()
    for element in elements:
        if element.shape == SHAPE_CIRCLE:
            if element.color == COLOR_RED:
                label_infos.add("red")
            elif element.color == COLOR_AMBER:
                label_infos.add("yellow")
            elif element.color == COLOR_GREEN:
                label_infos.add("green")
            continue

        if element.shape == SHAPE_UP_ARROW:
            label_infos.add("straight")
        elif element.shape == SHAPE_LEFT_ARROW:
            label_infos.add("left")
        elif element.shape == SHAPE_RIGHT_ARROW:
            label_infos.add("right")
        elif element.shape in (SHAPE_UP_LEFT_ARROW, SHAPE_DOWN_LEFT_ARROW):
            label_infos.add("left")
            label_infos.add("diagonal")
        elif element.shape in (SHAPE_UP_RIGHT_ARROW, SHAPE_DOWN_RIGHT_ARROW):
            label_infos.add("right")
            label_infos.add("diagonal")

    for info_set, label in TRAFFIC_LIGHT_LABEL_MAPPINGS:
        if label_infos == info_set:
            return label

    return "unknown"


class DistanceRange(NamedTuple):
    lower: float
    upper: float


def parse_distance_range(value: str | None) -> DistanceRange | None:
    """Parse a scenario yaml `Filter.Distance` string into (lower, upper).

    Mirrors driving_log_replayer_v2.traffic_light.Filter.validate_distance_range(): "min-max"
    (upper omitted -> sys.float_info.max), None means "do not filter by distance".
    """
    if value is None:
        return None

    import sys

    err_msg = f"{value} is not valid distance range, expected ordering min-max with min < max."

    s_lower, s_upper = value.split("-")
    upper = sys.float_info.max if s_upper == "" else float(s_upper)
    lower = float(s_lower)

    if lower >= upper:
        raise ValueError(err_msg)
    return DistanceRange(lower, upper)
