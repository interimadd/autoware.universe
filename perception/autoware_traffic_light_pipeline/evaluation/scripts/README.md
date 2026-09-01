# 精度評価スクリプト

`run_traffic_light_pipeline_evaluation`（[../README.md](../README.md)）が書き出した bag を、
t4dataset の `annotation/` を Ground Truth として距離ビンごとに精度評価します。DLR
(driving_log_replayer_v2) の `traffic_light_evaluator_node.py` から rclpy を剥がし、DLR の
scenario yaml をそのまま読めるようにしたものです。

## 環境

このパッケージ自体のビルドに `perception_eval` は不要ですが、**このスクリプトの実行時にだけ**
必要です。`rosdep`/`package.xml` には含めていません（本ワークスペースに存在しないため）。実行前に
`perception_eval` の入ったオーバーレイを別途 source してください。手元にあるものは例えば:

```bash
source /opt/ros/humble/setup.bash
source <this_ws>/install/setup.bash
source /home/takahisaishikawa/pilot-auto/install/setup.bash   # perception_eval が入っている
```

`perception_eval` を持っていない場合は
`pip install ~/driving_log_replayer_v2/dependency/autoware_perception_evaluation`
でも動きます（`requires-python >=3.10,<3.13`、`numpy<2` / `nuscenes-devkit` / `shapely` が入ります）。

## 使い方

```bash
ros2 run autoware_traffic_light_pipeline evaluate_traffic_light_recognition.py \
  --scenario <DLR scenario yaml> \
  --dataset <t4dataset のパス> \
  --result-bag <run_traffic_light_pipeline_evaluation の --output-bag> \
  --output-dir result/evaluation
```

`--topic` を省略すると scenario の `Evaluation.degradation_topic` を使います
（通常は `/perception/traffic_light_recognition/internal/traffic_signals`）。

scenario yaml は DLR のものをそのまま使えます。pydantic では読まず `pyyaml` で読むだけなので、DLR
自体はインストールされていなくても構いません。ただし `CriteriaMethod` / `CriteriaLevel` はリスト
指定（DLR が対応している複数条件の AND）には対応していません。1 Criterion = 1 method + 1 level
のみです。`Filter` も `Distance` のみ対応（`Region` は本リポジトリの traffic_light scenario では
使われていないため未実装）。

## 出力

`--output-dir` 以下に:

- `result.jsonl` — フレームごとの Criterion 別スコア
- `summary.md` / `summary.csv` — 距離ビン（Criterion）× {frames, TP, FP, FN, 平均スコア, 合格率,
  PASS/FAIL} の表
- 標準出力に同じ表 + 全 Criterion の AND による総合判定（exit code もこれに従う: 0=PASS, 1=FAIL）

## 妥当性チェック

このデータセットの `input_bag` に実車で録った本番相当の topic が既に入っている場合、同じ
スクリプトを `--result-bag <dataset>/input_bag --topic <本番 topic>` で走らせ、DLR で同じ
scenario を回した結果と距離ビン別スコアが一致するかを突き合わせられます。これで、このスクリプトの
評価ロジックの正しさを Phase 1（`run_traffic_light_pipeline_evaluation`）の出力品質と切り離して
検証できます。

一致しない場合の疑い所は順に: `frame_id=CAM_TRAFFIC_LIGHT`、`label_prefix`/`count_label_number`、
GT/推定双方への `set_position`、TF lookup 時刻。距離ビン全部で TP=0 なら uuid かラベルか
フレームIDのミスマッチ、`get_ground_truth_now_frame` の skip 数が多いならスタンプ単位（μs）か
75ms 許容の問題を疑ってください。

## テスト

`criteria.py` / `traffic_light_label.py` は perception_eval にもROSにも依存しない純関数なので、
`pytest` だけで検証できます:

```bash
python3 -m pytest evaluation/scripts/test_traffic_light_label.py -q
```
