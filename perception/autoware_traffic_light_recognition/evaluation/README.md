# TrafficLightRecognition オフライン評価ツール

t4dataset のパスを指定すると、`TrafficLightRecognition::run()`（本パッケージの ROS 非依存コア）
を全フレームに対して実行し、その結果を rosbag に書き出すツールです。

`autoware_traffic_light_component_test` の `run_traffic_light_pipeline` のうち、前段パイプライン
（本パッケージのコア）に相当する部分だけを移植したものです。以下は移植していません。

- 後段（multi_camera_fusion / arbiter / crosswalk_traffic_light_estimator）の評価
- `ParameterLoader` によるコンポーネント別 param.yaml のマージ
  （`TrafficLightRecognitionConfig` が既にフラットなので、評価用 yaml 1 枚に直接書く）

`rclcpp::init` / executor / DDS は一切使わず、コアライブラリと rosbag2 のみで動作します。

## 使い方

```bash
ros2 run autoware_traffic_light_recognition run_traffic_light_recognition_evaluation \
  --config $(ros2 pkg prefix --share autoware_traffic_light_recognition)/evaluation/config/x2_v4.4.evaluation.yaml \
  --dataset <t4dataset のパス> \
  --output-bag result/output_bag
```

データセットのレイアウトは固定です（Component Test ハーネスと同じ規約）。

```text
<dataset>/input_bag
<dataset>/map/lanelet2_map.osm      # MGRS 座標系のみ対応
<dataset>/map/map_projector_info.yaml
```

## 入出力

|      | 内容                                                                                           |
| ---- | ---------------------------------------------------------------------------------------------- |
| 入力 | `<dataset>/input_bag` の `camera_info` / `image_raw(/compressed)` / `/tf` / `/tf_static`       |
| 出力 | `run()` の `merged_signals`（`TrafficLightArray`）と `selected_rois`（`TrafficLightRoiArray`） |

- 画像と camera_info は本番の `message_filters::ExactTime` と同じくヘッダスタンプ完全一致で
  ペアリングし、相手のいないメッセージは捨てます。
- 出力トピック名は評価用 yaml の `cameras[].output_topics` で指定します（本番のトピック名）。
- 各メッセージはヘッダスタンプの時刻で書き込むため、同じデータセットからは常に同じ bag が
  得られます（実行時刻には依存しません）。
- 出力 bag のストレージ形式は入力 bag と同じものを自動で使います。
- `run()` が失敗したフレームは stderr に出して読み飛ばします（Node が捨てるのと同じ挙動）。

## 評価用 yaml

`config/x2_v4.4.evaluation.yaml` を参照してください。`cameras[]` に 1 エントリ書くごとに
`TrafficLightRecognition` インスタンスが 1 つ生成されます。`recognition:` 以下は全カメラ共通で、
`config/traffic_light_recognition.param.yaml` と `launch/traffic_light_recognition.launch.xml`
の値をそのまま持ってきたものです。カメラごとに変わるのは
`map_based_detector.min/max_timestamp_offset` だけなので、そこだけ `cameras[]` 側にあります。

model_path / label_path 類は環境依存の絶対パスなので、手元の `autoware_data` に合わせて
書き換えてください。
