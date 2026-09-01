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

"""Pure-function tests for traffic_light_label.py.

No ROS/perception_eval needed, run with plain `pytest evaluation/scripts/test_traffic_light_label.py`.
"""

from dataclasses import dataclass
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))

from traffic_light_label import DistanceRange  # noqa: E402
from traffic_light_label import get_traffic_light_label_str  # noqa: E402
from traffic_light_label import parse_distance_range  # noqa: E402


@dataclass
class FakeElement:
    color: int
    shape: int


CIRCLE = 1
RED, AMBER, GREEN = 1, 2, 3
LEFT_ARROW, RIGHT_ARROW, UP_ARROW = 2, 3, 4
UP_LEFT_ARROW, UP_RIGHT_ARROW = 5, 6
DOWN_LEFT_ARROW, DOWN_RIGHT_ARROW = 8, 9


def circle(color: int) -> FakeElement:
    return FakeElement(color=color, shape=CIRCLE)


def arrow(shape: int) -> FakeElement:
    return FakeElement(color=0, shape=shape)


@pytest.mark.parametrize(
    ("elements", "expected"),
    [
        ([circle(RED)], "red"),
        ([circle(AMBER)], "yellow"),
        ([circle(GREEN)], "green"),
        ([circle(RED), arrow(UP_ARROW)], "red_straight"),
        ([circle(RED), arrow(LEFT_ARROW)], "red_left"),
        ([circle(RED), arrow(RIGHT_ARROW)], "red_right"),
        (
            [circle(RED), arrow(UP_ARROW), arrow(LEFT_ARROW), arrow(RIGHT_ARROW)],
            "red_straight_left_right",
        ),
        ([circle(RED), arrow(UP_LEFT_ARROW)], "red_leftdiagonal"),
        ([circle(RED), arrow(UP_RIGHT_ARROW)], "red_rightdiagonal"),
        ([circle(RED), arrow(DOWN_LEFT_ARROW)], "red_leftdiagonal"),
        ([circle(RED), arrow(DOWN_RIGHT_ARROW)], "red_rightdiagonal"),
        ([circle(GREEN), arrow(UP_ARROW)], "green_straight"),
        ([], "unknown"),
        ([arrow(UP_ARROW)], "unknown"),  # arrow with no color is not a valid mapping
    ],
)
def test_get_traffic_light_label_str(elements: list[FakeElement], expected: str) -> None:
    assert get_traffic_light_label_str(elements) == expected


def test_parse_distance_range_none() -> None:
    assert parse_distance_range(None) is None


def test_parse_distance_range_bounded() -> None:
    assert parse_distance_range("0.0-10.0") == DistanceRange(0.0, 10.0)


def test_parse_distance_range_unbounded_upper() -> None:
    result = parse_distance_range("210.0-")
    assert result.lower == 210.0
    assert result.upper == sys.float_info.max


def test_parse_distance_range_invalid_ordering() -> None:
    with pytest.raises(ValueError, match="not valid distance range"):
        parse_distance_range("10.0-5.0")
