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

"""Distance-binned accuracy evaluation of a traffic light recognition result bag.

Matches a result bag against a t4dataset's annotation, using the same matching/scoring logic
driving_log_replayer_v2 (DLR) uses (perception_eval). See evaluate_result_implementation_plan.md's
Phase 2 for the design; this is the rclpy-free "batch" reimplementation of DLR's
traffic_light_evaluator_node.py.

Usage:
    ros2 run autoware_traffic_light_pipeline evaluate_traffic_light_recognition.py
        --scenario <DLR scenario yaml>
        --dataset <t4dataset dir>
        --result-bag <result bag, e.g. run_traffic_light_pipeline_evaluation's --output-bag>
        --output-dir <output dir>

See evaluation/scripts/README.md for the environment this needs (perception_eval is not a rosdep
of this workspace) and evaluation/README.md for how to produce --result-bag.
"""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
from dataclasses import field
import json
import logging
from pathlib import Path
import sys
from typing import Any

import yaml

# Both sibling modules install next to this script (see CMakeLists.txt); make sibling imports work
# regardless of cwd, the same way a script run via `ros2 run` would expect.
sys.path.insert(0, str(Path(__file__).resolve().parent))

from criteria import CriteriaLevel  # noqa: E402
from criteria import CriteriaMethod  # noqa: E402
from criteria import PerceptionCriteria  # noqa: E402
from traffic_light_label import get_traffic_light_label_str  # noqa: E402
from traffic_light_label import parse_distance_range  # noqa: E402

logging.basicConfig(level=logging.INFO, format="%(levelname)s: %(message)s")
logger = logging.getLogger("evaluate_traffic_light_recognition")


# --- scenario -------------------------------------------------------------------------------


@dataclass
class Criterion:
    name: str
    pass_rate: float
    method: CriteriaMethod
    level: CriteriaLevel
    distance_range: Any  # traffic_light_label.DistanceRange | None
    criteria: PerceptionCriteria = field(init=False)

    def __post_init__(self) -> None:
        self.criteria = PerceptionCriteria(self.method, self.level, self.distance_range)


def load_criteria(scenario: dict) -> list[Criterion]:
    criteria = []
    for i, c in enumerate(scenario["Evaluation"]["Conditions"]["Criterion"]):
        method_value = c["CriteriaMethod"]
        level_value = c["CriteriaLevel"]
        if isinstance(method_value, list) or isinstance(level_value, list):
            error_msg = (
                "This tool only supports a single CriteriaMethod/CriteriaLevel per Criterion "
                f"(criterion index {i} has a list)."
            )
            raise ValueError(error_msg)
        criteria.append(
            Criterion(
                name=c.get("criteria_name") or f"criteria_{i}",
                pass_rate=float(c["PassRate"]),
                method=CriteriaMethod.from_str(method_value),
                level=CriteriaLevel.from_scenario_value(level_value),
                distance_range=parse_distance_range(c["Filter"].get("Distance")),
            ),
        )
    return criteria


# --- lanelet2 map -----------------------------------------------------------------------------


def load_lanelet2_map(dataset_path: Path):  # noqa: ANN201
    """Load <dataset>/map/lanelet2_map.osm.

    MGRS-only, matching this repo's C++ evaluation tools (run_traffic_light_pipeline_evaluation /
    run_traffic_light_recognition_evaluation).
    """
    from autoware_lanelet2_extension_python.projection import MGRSProjector
    import autoware_lanelet2_extension_python.regulatory_elements  # noqa: F401  (registers reg elems)
    import lanelet2

    map_dir = dataset_path / "map"
    projector_info_path = map_dir / "map_projector_info.yaml"
    with projector_info_path.open() as f:
        projector_info = yaml.safe_load(f)
    projector_type = projector_info.get("projector_type")
    if projector_type != "MGRS":
        error_msg = f"Only MGRS-projected maps are supported, got projector_type: {projector_type}"
        raise ValueError(error_msg)
    projection = MGRSProjector(lanelet2.io.Origin(0.0, 0.0))
    return lanelet2.io.load((map_dir / "lanelet2_map.osm").as_posix(), projection)


def get_min_traffic_light_distance(
    traffic_lights: list, p2d
) -> tuple[float, float, float, float]:  # noqa: ANN001
    """Find the regulatory element linestring closest to `p2d`.

    Ported from DLR's TrafficLightEvaluator.get_min_traffic_light_distance(). Returns the
    (x, y, z) center of the closest linestring, plus that distance.
    """
    from lanelet2.geometry import distance as ll2_distance
    from lanelet2.geometry import to2D

    min_distance = sys.float_info.max
    min_tl = None
    for tl in traffic_lights:
        cur_distance = ll2_distance(to2D(tl), p2d)
        if cur_distance < min_distance:
            min_tl = tl
            min_distance = cur_distance
    min_tl_center = tuple(
        (getattr(min_tl[0], attr) + getattr(min_tl[1], attr)) / 2.0 for attr in ("x", "y", "z")
    )
    return (*min_tl_center, min_distance)


# --- tf ----------------------------------------------------------------------------------------


def load_transform_buffer(input_bag_path: Path):  # noqa: ANN201
    """Load every /tf, /tf_static message from the dataset's input_bag.

    Builds a standalone tf2_ros.Buffer (no node/executor needed). Mirrors the C++ tools'
    load_transform_buffer(): a 24h cache so the whole bag's transforms stay resolvable for the
    whole run.
    """
    from rclpy.duration import Duration as RclpyDuration
    from rclpy.serialization import deserialize_message
    import rosbag2_py
    from tf2_msgs.msg import TFMessage
    from tf2_ros import Buffer

    buffer = Buffer(cache_time=RclpyDuration(seconds=24 * 60 * 60))

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=input_bag_path.as_posix(), storage_id=""),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr", output_serialization_format="cdr"
        ),
    )
    reader.set_filter(rosbag2_py.StorageFilter(topics=["/tf", "/tf_static"]))
    authority = "evaluate_traffic_light_recognition"
    while reader.has_next():
        topic_name, data, _ = reader.read_next()
        message = deserialize_message(data, TFMessage)
        is_static = topic_name == "/tf_static"
        for transform in message.transforms:
            if is_static:
                buffer.set_transform_static(transform, authority)
            else:
                buffer.set_transform(transform, authority)
    return buffer


def lookup_map_to_baselink(tf_buffer, stamp):  # noqa: ANN001, ANN201
    """Look up map->base_link at `stamp`, i.e. ego's pose expressed in the map frame.

    Mirrors DLR's DLREvaluatorV2.lookup_transform(); on failure returns an identity
    TransformStamped and logs, the same fallback DLR uses.
    """
    from geometry_msgs.msg import TransformStamped
    from rclpy.time import Time as RclpyTime
    from tf2_ros import TransformException

    try:
        return tf_buffer.lookup_transform("map", "base_link", RclpyTime.from_msg(stamp))
    except TransformException as ex:
        logger.info("Could not transform map to base_link: %s", ex)
        return TransformStamped()


# --- perception_eval frame construction ------------------------------------------------------


def unix_time_microsec_from_ros_timestamp(stamp) -> int:  # noqa: ANN001
    return stamp.sec * 1_000_000 + stamp.nanosec // 1000


def get_traffic_light_pos_and_dist(
    lanelet_map,  # noqa: ANN001
    traffic_light_uuid: str,
    ego_position_map,  # noqa: ANN001 (geometry_msgs/Vector3-like: .x, .y)
    max_distance: float,
) -> tuple[float, float, float, float]:
    """Compute the GT side's map-frame position and distance.

    Returns the (x, y, z) of the GT object's regulatory element (nearest linestring to ego) plus
    the distance. Ported from DLR's get_traffic_light_pos_and_dist().
    """
    from lanelet2.core import BasicPoint2d

    rtn_distance = max_distance + 1.0
    try:
        int_uuid = int(traffic_light_uuid)
    except ValueError:
        return (0.0, 0.0, 0.0, rtn_distance)
    traffic_light_obj = lanelet_map.regulatoryElementLayer.get(int_uuid)
    p2d = BasicPoint2d(ego_position_map.x, ego_position_map.y)
    return get_min_traffic_light_distance(traffic_light_obj.trafficLights, p2d)


def get_traffic_light_pos(
    lanelet_map,  # noqa: ANN001
    traffic_light_group_id: int,
    cam2map,  # noqa: ANN001
    ego_position_map,  # noqa: ANN001
) -> tuple[float, float, float]:
    """Compute the estimated side's camera-frame position.

    Returns the (x, y, z) of `traffic_light_group_id`'s regulatory element (nearest linestring to
    ego), in the camera frame. Ported from DLR's get_traffic_light_pos().
    """
    from lanelet2.core import BasicPoint2d

    traffic_light_obj = lanelet_map.regulatoryElementLayer.get(traffic_light_group_id)
    p2d = BasicPoint2d(ego_position_map.x, ego_position_map.y)
    tl_x, tl_y, tl_z, _ = get_min_traffic_light_distance(traffic_light_obj.trafficLights, p2d)
    return cam2map.inv().transform((tl_x, tl_y, tl_z))


def list_dynamic_object_2d_from_msg(
    lanelet_map,  # noqa: ANN001
    label_converter,  # noqa: ANN001
    unix_time_us: int,
    traffic_light_groups: list,  # noqa: ANN001 (list of TrafficLightGroup)
    cam2map,  # noqa: ANN001
    ego_position_map,  # noqa: ANN001
):  # noqa: ANN201
    """Ported from DLR's list_dynamic_object_2d_from_ros_msg()."""
    from perception_eval.common.object2d import DynamicObject2D
    from perception_eval.common.schema import FrameID

    estimated_objects = []
    for signal in traffic_light_groups:
        label = label_converter.convert_label(get_traffic_light_label_str(signal.elements))
        confidence = max(signal.elements, key=lambda e: e.confidence).confidence
        position = get_traffic_light_pos(
            lanelet_map, signal.traffic_light_group_id, cam2map, ego_position_map
        )
        estimated_objects.append(
            DynamicObject2D(
                unix_time=unix_time_us,
                frame_id=FrameID.CAM_TRAFFIC_LIGHT,
                semantic_score=confidence,
                semantic_label=label,
                roi=None,
                uuid=str(signal.traffic_light_group_id),
                position=position,
            ),
        )
    return estimated_objects


# --- summary ------------------------------------------------------------------------------------


@dataclass
class CriterionAggregate:
    name: str
    pass_rate: float
    frames: int = 0
    passed: int = 0
    no_gt_no_obj: int = 0
    score_sum: float = 0.0
    tp: int = 0
    fp: int = 0
    fn: int = 0

    def rate(self) -> float:
        return 0.0 if self.frames == 0 else 100.0 * self.passed / self.frames

    def avg_score(self) -> float:
        return 0.0 if self.frames == 0 else self.score_sum / self.frames

    def is_success(self) -> bool:
        return self.rate() >= self.pass_rate


def write_summary(aggregates: list[CriterionAggregate], output_dir: Path) -> bool:
    rows = []
    for agg in aggregates:
        rows.append(
            {
                "criterion": agg.name,
                "frames": agg.frames,
                "no_gt_no_obj": agg.no_gt_no_obj,
                "TP": agg.tp,
                "FP": agg.fp,
                "FN": agg.fn,
                "avg_score": round(agg.avg_score(), 2),
                "pass_rate[%]": round(agg.rate(), 2),
                "required_pass_rate[%]": agg.pass_rate,
                "result": "PASS" if agg.is_success() else "FAIL",
            },
        )

    with (output_dir / "summary.csv").open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()) if rows else [])
        writer.writeheader()
        writer.writerows(rows)

    header = list(rows[0].keys()) if rows else []
    lines = ["| " + " | ".join(header) + " |", "| " + " | ".join(["---"] * len(header)) + " |"]
    lines += ["| " + " | ".join(str(row[h]) for h in header) + " |" for row in rows]
    table_md = "\n".join(lines)
    overall = all(agg.is_success() for agg in aggregates)
    summary_md = f"# Traffic light recognition evaluation summary\n\n{table_md}\n\n"
    summary_md += f"**Overall: {'PASS' if overall else 'FAIL'}**\n"
    (output_dir / "summary.md").write_text(summary_md)

    print(table_md)  # noqa: T201
    print(f"\nOverall: {'PASS' if overall else 'FAIL'}")  # noqa: T201
    return overall


# --- main -----------------------------------------------------------------------------------


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario", required=True, type=Path, help="DLR scenario yaml")
    parser.add_argument("--dataset", required=True, type=Path, help="t4dataset directory")
    parser.add_argument(
        "--result-bag", required=True, type=Path, help="rosbag holding the topic to evaluate"
    )
    parser.add_argument(
        "--topic",
        default=None,
        help="Topic to evaluate (default: scenario's Evaluation.degradation_topic)",
    )
    parser.add_argument("--output-dir", required=True, type=Path, help="Where to write results")
    return parser.parse_args()


def main() -> int:  # noqa: C901, PLR0915
    args = parse_args()

    dataset_path = args.dataset.expanduser().resolve()
    output_dir = args.output_dir.expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    with args.scenario.expanduser().open() as f:
        scenario = yaml.safe_load(f)
    evaluation = scenario["Evaluation"]
    if evaluation["UseCaseName"] != "traffic_light":
        error_msg = f"Not a traffic_light scenario: UseCaseName={evaluation['UseCaseName']}"
        raise ValueError(error_msg)

    topic = args.topic or evaluation.get("degradation_topic")
    if topic is None:
        error_msg = "--topic not given and scenario has no degradation_topic"
        raise ValueError(error_msg)

    criteria_list = load_criteria(scenario)

    p_cfg = evaluation["PerceptionEvaluationConfig"]
    evaluation_config_dict = dict(p_cfg["evaluation_config_dict"])
    # Required by perception_eval to select the TrafficLightLabel converter (see
    # evaluate_result_implementation_plan.md's Phase 2 step 2).
    evaluation_config_dict["label_prefix"] = "traffic_light"
    evaluation_config_dict["count_label_number"] = True
    camera_type = p_cfg["camera_type"]
    max_distance = float(evaluation_config_dict["max_distance"])

    # perception_eval-only imports (criteria.py already imports it at module scope, so this does
    # not gain --help-without-the-overlay -- it's just grouped here with the rest of the
    # perception_eval setup for readability). See evaluation/scripts/README.md.
    from perception_eval.config import PerceptionEvaluationConfig
    from perception_eval.evaluation.result.perception_frame_config import CriticalObjectFilterConfig
    from perception_eval.evaluation.result.perception_frame_config import PerceptionPassFailConfig
    from perception_eval.manager import PerceptionEvaluationManager

    eval_config = PerceptionEvaluationConfig(
        dataset_paths=[dataset_path.as_posix()],
        frame_id=camera_type,
        result_root_directory=(output_dir / "perception_eval_log").as_posix(),
        evaluation_config_dict=evaluation_config_dict,
        load_raw_data=False,
    )
    critical_cfg = CriticalObjectFilterConfig(
        evaluator_config=eval_config,
        target_labels=evaluation["CriticalObjectFilterConfig"]["target_labels"],
    )
    pass_fail_cfg = PerceptionPassFailConfig(
        evaluator_config=eval_config,
        target_labels=evaluation["PerceptionPassFailConfig"]["target_labels"],
    )
    manager = PerceptionEvaluationManager(evaluation_config=eval_config)

    lanelet_map = load_lanelet2_map(dataset_path)
    tf_buffer = load_transform_buffer(dataset_path / "input_bag")

    from autoware_perception_msgs.msg import TrafficLightGroupArray
    from perception_eval.common.schema import FrameID
    from rclpy.serialization import deserialize_message
    import rosbag2_py

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(
            uri=args.result_bag.expanduser().resolve().as_posix(), storage_id=""
        ),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr", output_serialization_format="cdr"
        ),
    )
    reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))

    skip_counter = 0
    num_frames = 0
    aggregates = [CriterionAggregate(name=c.name, pass_rate=c.pass_rate) for c in criteria_list]
    result_lines: list[dict] = []

    while reader.has_next():
        _, data, _ = reader.read_next()
        msg: TrafficLightGroupArray = deserialize_message(data, TrafficLightGroupArray)

        map_to_baselink = lookup_map_to_baselink(tf_buffer, msg.stamp)
        unix_time_us = unix_time_microsec_from_ros_timestamp(msg.stamp)

        gt_frame = manager.get_ground_truth_now_frame(unix_time_us)
        if gt_frame is None:
            skip_counter += 1
            continue

        cam2map = gt_frame.transforms[(FrameID.CAM_TRAFFIC_LIGHT, FrameID.MAP)]
        ego_position_map = map_to_baselink.transform.translation

        for obj in gt_frame.objects:
            x, y, z, _dist = get_traffic_light_pos_and_dist(
                lanelet_map, obj.uuid, ego_position_map, max_distance
            )
            obj.set_position(cam2map.inv().transform((x, y, z)))

        estimated_objects = list_dynamic_object_2d_from_msg(
            lanelet_map,
            manager.evaluator_config.label_converter,
            unix_time_us,
            msg.traffic_light_groups,
            cam2map,
            ego_position_map,
        )

        frame_result = manager.add_frame_result(
            unix_time=unix_time_us,
            ground_truth_now_frame=gt_frame,
            estimated_objects=estimated_objects,
            critical_object_filter_config=critical_cfg,
            frame_pass_fail_config=pass_fail_cfg,
        )
        num_frames += 1

        frame_line = {"frame_name": frame_result.frame_name, "unix_time": unix_time_us}
        for criterion, agg in zip(criteria_list, aggregates, strict=True):
            result, score, filtered_frame = criterion.criteria.get_result(frame_result)
            if result is None:
                agg.no_gt_no_obj += 1
                frame_line[criterion.name] = {"NoGTNoObj": True}
                continue
            agg.frames += 1
            agg.score_sum += score
            agg.tp += len(filtered_frame.pass_fail_result.tp_object_results)
            agg.fp += len(filtered_frame.pass_fail_result.fp_object_results)
            agg.fn += len(filtered_frame.pass_fail_result.fn_objects)
            if result.is_success():
                agg.passed += 1
            frame_line[criterion.name] = {"result": str(result), "score": score}
        result_lines.append(frame_line)

    with (output_dir / "result.jsonl").open("w") as f:
        for line in result_lines:
            f.write(json.dumps(line) + "\n")

    logger.info(
        "Processed %d frames (%d skipped: no ground truth frame within 75ms)",
        num_frames,
        skip_counter,
    )
    if num_frames == 0:
        logger.error(
            "No frame was evaluated at all -- check --topic/camera_type/frame timestamps "
            "(see evaluate_result_implementation_plan.md's failure-pattern section).",
        )
        return 1

    overall_pass = write_summary(aggregates, output_dir)
    return 0 if overall_pass else 1


if __name__ == "__main__":
    sys.exit(main())
