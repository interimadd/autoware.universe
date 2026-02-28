# Autoware Universe ゼロベース再設計提案

## 目次

1. [現状分析](#1-現状分析)
2. [特定された構造的課題](#2-特定された構造的課題)
3. [再設計の全体ビジョン](#3-再設計の全体ビジョン)
4. [レイヤードアーキテクチャ](#4-レイヤードアーキテクチャ)
5. [ミドルウェア層の再設計](#5-ミドルウェア層の再設計)
6. [ドメイン別モジュール再構成](#6-ドメイン別モジュール再構成)
7. [ビルドシステムの刷新](#7-ビルドシステムの刷新)
8. [リアルタイム・安全性アーキテクチャ](#8-リアルタイム安全性アーキテクチャ)
9. [テスト・品質保証戦略](#9-テスト品質保証戦略)
10. [E2E AI統合アーキテクチャ](#10-e2e-ai統合アーキテクチャ)
11. [移行戦略](#11-移行戦略)
12. [まとめ](#12-まとめ)

---

## 1. 現状分析

### 1.1 プロジェクト規模

| 指標 | 現状値 |
|------|--------|
| 総パッケージ数 | 244 |
| C++/Hソースファイル数 | 2,852 |
| 総コード行数 | 137,000+ |
| Pythonファイル数 | 110 |
| CUDAファイル数 | 62 |
| テストファイル数 | 321 |
| 起動(Launch)ファイル数 | 235 |
| パラメータYAMLファイル数 | 315 |
| CI/CDワークフロー数 | 34+ |
| 外部依存リポジトリ数 | 14 |

### 1.2 ドメイン別パッケージ分布

```
Planning(計画)       ████████████████████████████  65 (26.6%)
Perception(認識)     ██████████████████████        53 (21.7%)
System(システム)     ██████████                    26 (10.7%)
Control(制御)        ████████                      21  (8.6%)
Common(共通)         ███████                       18  (7.4%)
Localization(位置推定) ██████                       14  (5.7%)
Sensing(センシング)  ██████                        14  (5.7%)
Visualization(可視化) █████                        12  (4.9%)
Evaluator(評価)      ███                            7  (2.9%)
Simulator(シミュ)    ███                            7  (2.9%)
Vehicle(車両)        ██                             4  (1.6%)
Map/E2E/Examples     █                              3  (1.2%)
```

### 1.3 依存関係の集中度

最も依存されている上位パッケージ（被依存数）:

| パッケージ | 被依存パッケージ数 | 全体に占める割合 |
|------------|-------------------|------------------|
| rclcpp | 218 | 89% |
| rclcpp_components | 167 | 68% |
| **autoware_utils** | **132** | **54%** |
| geometry_msgs | 97 | 40% |
| autoware_perception_msgs | 78 | 32% |
| autoware_motion_utils | 77 | 32% |
| autoware_planning_msgs | 73 | 30% |
| tf2 / tf2_ros | 72 / 66 | 30% / 27% |
| autoware_vehicle_info_utils | 58 | 24% |
| pluginlib | 51 | 21% |

**`autoware_utils` が全パッケージの54%から依存されている**点が最大のアーキテクチャ上のボトルネックである。

---

## 2. 特定された構造的課題

### 課題1: ユーティリティ結合の爆発（Build Cascade問題）

`autoware_utils` と `autoware_motion_utils` のヘッダファイルへの些細な変更が、プロジェクト全体の132〜77パッケージの再コンパイルを引き起こす。Boost.Geometryなどの重量級テンプレートライブラリの利用も相まって、フルビルドには16〜32GBのスワップが推奨されるほど重い。

**定量的影響:**
- `behavior_velocity_planner` や `behavior_path_planner` のビルドは特にメモリ・時間を消費
- CI上の差分ビルドでも、コアパッケージの変更時は非常に長時間を要する

### 課題2: ミドルウェアのレイテンシ

ROS2/DDS (FastDDS) のシリアライゼーションオーバーヘッドにより、大規模センサデータの伝送レイテンシが約38ミリ秒に達する。対照的に、Apollo CyberRTの共有メモリ方式は0.27マイクロ秒（約14万倍高速）。

**具体的問題:**
- LiDARポイントクラウドが徐々に6Hzまで劣化するケースが報告
- 外部ツール（RViz2, `ros2 topic delay`）のサブスクライブだけでパイプラインが遅延
- 数百ノードの`/tf`サブスクリプションによるCPU使用率の増大

### 課題3: 命名規則の不統一

`autoware_*` プレフィックスのパッケージが226個に対し、歴史的経緯から `tier4_*` プレフィックスが12個残存。外部開発者にとって、どれが公式インターフェースでどれがベンダー固有かの判別が困難。

### 課題4: Planning領域の過剰な細分化

65パッケージ（全体の26.6%）がPlanningに集中し、`behavior_path_planner` だけで12の子パッケージ（プラグイン）、`behavior_velocity_planner` に14のモジュールが存在する。単一パッケージで最大37個の `<depend>` タグを持つものも存在する。

さらに、Autowareは不活性シナリオに対しても軌道計算を実行する並列実行モデルを採用しており、計算リソースの浪費が発生する。

### 課題5: リアルタイム保証の欠如

バニラLinux上で動作するため、ページフォールト、動的メモリ確保、非決定的スケジューリングが発生し得る。安全認証（ISO 26262）に必要な決定的動作を保証できない。

### 課題6: 設定管理の分散

315以上のパラメータYAMLファイルがパッケージごとに分散配置されている。中央集約的なパラメータ管理がなく、車両・環境ごとの設定の一括変更が困難。

### 課題7: テストカバレッジの偏り

321のテストファイルに対して244パッケージが存在し、特に安全クリティカルな制御・計画ドメインのテストカバレッジが十分でない可能性がある。統合テスト・E2Eテストの仕組みが弱い。

### 課題8: Core/Universeの境界が曖昧

Autoware CoreへのUniverse成熟パッケージのポーティングが進行中だが、どのパッケージが「安定・プロダクション品質」でどれが「実験的」かの境界が流動的であり、外部ユーザーが信頼度を判断しにくい。

---

## 3. 再設計の全体ビジョン

### 3.1 設計哲学

```
┌─────────────────────────────────────────────────────────────────┐
│                     設計原則 (Design Principles)                 │
├─────────────────────────────────────────────────────────────────┤
│ 1. ゼロコピー通信ファースト  (Zero-Copy Communication First)     │
│ 2. コンパイル時間予算制      (Compilation Time Budget)           │
│ 3. 安全等級別レイヤー分離    (Safety-Level Layered Isolation)    │
│ 4. 交換可能コンポーネント    (Swappable Components)             │
│ 5. E2E AI ネイティブ統合     (E2E AI Native Integration)        │
│ 6. 車両非依存抽象化          (Vehicle-Agnostic Abstraction)     │
│ 7. 設定の一元管理            (Centralized Configuration)        │
│ 8. 段階的移行可能性          (Incremental Migration Path)       │
└─────────────────────────────────────────────────────────────────┘
```

### 3.2 ターゲットアーキテクチャ概観

```
┌──────────────────────────────────────────────────────────────────────────┐
│                          Application Layer                               │
│  ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌──────────────┐  │
│  │ Sensing  │ │Perception│ │ Planning │ │ Control  │ │   System     │  │
│  │ Pipeline │ │ Pipeline │ │ Pipeline │ │ Pipeline │ │ Orchestrator │  │
│  └────┬─────┘ └────┬─────┘ └────┬─────┘ └────┬─────┘ └──────┬───────┘  │
│       │            │            │            │               │          │
├───────┴────────────┴────────────┴────────────┴───────────────┴──────────┤
│                        Component Contract Layer                          │
│  ┌────────────────────────────────────────────────────────────────────┐  │
│  │  Interface Definitions (IDL) + Safety Contracts + QoS Policies    │  │
│  └────────────────────────────────────────────────────────────────────┘  │
├─────────────────────────────────────────────────────────────────────────┤
│                     Communication Middleware Layer                        │
│  ┌─────────────┐ ┌─────────────┐ ┌──────────────┐ ┌─────────────────┐  │
│  │ Zero-Copy   │ │  Managed    │ │   Priority   │ │  Deterministic  │  │
│  │ Shared Mem  │ │  Transport  │ │  Scheduler   │ │  Executor       │  │
│  └─────────────┘ └─────────────┘ └──────────────┘ └─────────────────┘  │
├─────────────────────────────────────────────────────────────────────────┤
│                       Platform Abstraction Layer                         │
│  ┌──────────┐ ┌──────────┐ ┌────────────┐ ┌────────────┐              │
│  │  RTOS    │ │  Linux   │ │   GPU/NPU  │ │  Sensor    │              │
│  │  HAL     │ │  RT HAL  │ │   HAL      │ │  HAL       │              │
│  └──────────┘ └──────────┘ └────────────┘ └────────────┘              │
└─────────────────────────────────────────────────────────────────────────┘
```

---

## 4. レイヤードアーキテクチャ

### 4.1 4層アーキテクチャ

現状の「フラットなドメイン分割」から、明確な4層構造に変更する。

#### Layer 0: Platform Abstraction Layer (PAL)

**目的:** ハードウェア・OS差異の吸収

```
platform/
├── hal_sensor/          # センサ抽象化（LiDAR, Camera, Radar, IMU, GNSS）
├── hal_vehicle/         # 車両CAN/Ethernet抽象化
├── hal_compute/         # GPU/NPU/FPGAコンピュートデバイス抽象化
├── hal_time/            # 時刻同期・タイムスタンプ抽象化
└── hal_storage/         # マップ・ログストレージ抽象化
```

**現状との差分:** 現状の `vehicle/` (4パッケージ) と `sensing/` の一部を統合し、ハードウェアとの境界を単一レイヤーに集約する。

**メリット:**
- 新センサの追加が HAL インターフェースの実装だけで完結
- RTOS移行時にこの層のみ変更すればよい
- シミュレータがこの層をモック実装するだけでE2Eテスト可能

#### Layer 1: Communication & Runtime Layer (CRL)

**目的:** プロセス間通信、スケジューリング、監視の基盤

```
runtime/
├── transport/
│   ├── zero_copy_shm/       # 共有メモリゼロコピー伝送
│   ├── managed_dds/         # DDS（外部通信用、ローカルは非使用）
│   └── negotiated_qos/      # 適応的QoS制御
├── executor/
│   ├── priority_executor/   # 優先度ベース実行器
│   ├── chain_aware_sched/   # チェーン認識スケジューラ (PiCAS方式)
│   └── deadline_monitor/    # デッドラインミス検出
├── lifecycle/
│   ├── node_manager/        # ノードライフサイクル管理
│   ├── health_monitor/      # ヘルスモニタリング
│   └── failover/            # フェイルオーバー制御
└── diagnostics/
    ├── trace/               # LTTngベーストレーシング
    └── metrics/             # レイテンシ・スループット計測
```

**現状との差分:** 現状は各ノードがデフォルトROS2エグゼキュータをそのまま使用し、`/tf` を暗黙的にサブスクライブしている。新設計では:
- **ノード内通信はDDSを迂回し共有メモリ直結** (Agnocast/SIM方式)
- **TransformListenerをシングルトン化** し、コンテナ内で共有
- **チェーン認識スケジューリング** で感知→認識→計画→制御のE2Eレイテンシを最適化

#### Layer 2: Component Contract Layer (CCL)

**目的:** ドメイン間のインターフェース定義と安全契約

```
contracts/
├── interfaces/
│   ├── sensing_output.idl       # センシング出力インターフェース
│   ├── perception_output.idl    # 認識出力インターフェース
│   ├── planning_output.idl      # 計画出力インターフェース
│   ├── control_output.idl       # 制御出力インターフェース
│   ├── localization_output.idl  # 位置推定出力インターフェース
│   └── map_service.idl          # マップサービスインターフェース
├── safety_contracts/
│   ├── timing_constraints.yaml  # 各パイプラインの許容レイテンシ
│   ├── data_freshness.yaml      # データ鮮度要件
│   └── fallback_policies.yaml   # フォールバックポリシー
└── qos_profiles/
    ├── safety_critical.yaml     # 制御・ブレーキ向けQoS
    ├── best_effort.yaml         # 可視化向けQoS
    └── reliable_sensor.yaml     # センサデータ向けQoS
```

**現状との差分:** 現状は39種のメッセージパッケージが `autoware_*_msgs`、`tier4_*_msgs`、標準ROS `*_msgs` に分散している。新設計では:
- **全インターフェースを `contracts/` に一元管理**
- メッセージ定義だけでなく、**タイミング制約・データ鮮度要件を契約として定義**
- 各コンポーネントは契約に対してプログラムし、実装に依存しない

#### Layer 3: Application Layer (AL)

**目的:** 自動運転の機能実装

```
applications/
├── sensing/              # ~8パッケージ（現14→統合）
│   ├── pointcloud/       # LiDAR前処理（CPU/GPU統合）
│   ├── image/            # カメラ前処理
│   ├── radar/            # レーダー前処理
│   └── imu_gnss/         # IMU/GNSS処理
├── perception/           # ~20パッケージ（現53→統合）
│   ├── detection/        # 3Dオブジェクト検出（統合パイプライン）
│   ├── tracking/         # オブジェクト追跡
│   ├── prediction/       # 行動予測
│   ├── segmentation/     # 意味的セグメンテーション
│   ├── traffic_light/    # 信号認識
│   └── fusion/           # マルチモーダル融合
├── localization/         # ~6パッケージ（現14→統合）
│   ├── ndt_matching/     # NDTスキャンマッチング
│   ├── visual_loc/       # 視覚位置推定
│   ├── gnss_loc/         # GNSS位置推定
│   └── pose_fusion/      # ポーズ融合・仲裁
├── planning/             # ~15パッケージ（現65→大幅統合）
│   ├── mission/          # ミッション計画
│   ├── behavior/         # 行動計画（統合モノリス）
│   ├── motion/           # 運動計画
│   ├── trajectory/       # 軌道最適化
│   └── freespace/        # 自由空間計画
├── control/              # ~8パッケージ（現21→統合）
│   ├── lateral/          # 横方向制御（MPC/PurePursuit統合）
│   ├── longitudinal/     # 縦方向制御（PID/MPC統合）
│   ├── emergency/        # 緊急制御
│   └── validation/       # 制御出力検証
├── map/                  # ~3パッケージ
│   ├── map_loader/       # マップ読込
│   ├── map_server/       # マップ配信
│   └── map_utils/        # マップユーティリティ
└── system/               # ~10パッケージ（現26→統合）
    ├── orchestrator/     # システムオーケストレータ
    ├── mode_manager/     # 走行モード管理
    ├── monitor/          # システム監視
    └── api_gateway/      # 外部API
```

**パッケージ数の変化: 244 → 約70〜80（約67%削減）**

---

## 5. ミドルウェア層の再設計

### 5.1 現状の問題

```
現状: Node A → [シリアライズ] → DDS → [デシリアライズ] → Node B
      レイテンシ: ~38ms (大規模センサデータ)
      CPU使用率: /tf の暗黙サブスクリプション × 数百ノード
```

### 5.2 ハイブリッドトランスポート設計

```
新設計:

ケース1: 同一プロセス内（コンポーザブルノード）
  Node A → [ポインタ渡し] → Node B
  レイテンシ: ~0 (ゼロコピー)

ケース2: 同一マシン・異プロセス
  Node A → [共有メモリ + メタデータのみDDS] → Node B
  レイテンシ: ~0.3μs (Agnocast/SIM方式)

ケース3: 異なるマシン（クラウド・リモート監視）
  Node A → [DDS over network] → Node B
  レイテンシ: ネットワーク依存
```

**実装方針:**
- `runtime/transport/zero_copy_shm/`: Agnocast をベースにした共有メモリトランスポート
- センサーデータ（PointCloud, Image）は**常にゼロコピー経路**を使用
- DDS は外部通信・リモートモニタリング専用に限定
- CUDAメモリはcuda_blackboard経由でGPU間転送

### 5.3 TransformListener の最適化

```cpp
// 現状: 各ノードがそれぞれ tf2_ros::TransformListener を生成
// → /tf サブスクリプション × 数百 = CPU負荷爆発

// 新設計: コンテナ内シングルトン + マネージドバッファ
// managed_transform_buffer を全ノードで共有
class ManagedTransformProvider {
  // コンテナ単位で1つのTransformListenerを保持
  // 各ノードはlookupTransformのみを呼び出し
  // /tf サブスクリプション数を劇的に削減
};
```

### 5.4 優先度ベース実行モデル

```
┌──────────────────────────────────────────────────┐
│            Priority-based Executor                │
├──────────────────────────────────────────────────┤
│ Priority 0 (最高): 緊急停止・安全監視             │
│ Priority 1:        制御出力 (20ms周期)            │
│ Priority 2:        計画更新 (100ms周期)           │
│ Priority 3:        認識処理 (GPU非同期)           │
│ Priority 4:        位置推定 (50ms周期)            │
│ Priority 5 (最低): 可視化・ロギング・診断         │
└──────────────────────────────────────────────────┘
```

現状はすべてのノードが同一優先度で実行され、可視化ノードが安全クリティカルな制御ノードとCPU時間を争う。新設計では PiCAS (Priority-driven Chain-Aware Scheduling) に基づき、感知→認識→計画→制御のE2Eチェーンに対してデッドラインを割り当てる。

---

## 6. ドメイン別モジュール再構成

### 6.1 Perception: 53 → ~20パッケージ

**問題:** 53パッケージは個々のアルゴリズム実装ごとに独立パッケージ化されている。例えば、LiDAR検出だけで `centerpoint`, `transfusion`, `frnet`, `apollo_instance_segmentation`, `ptv3` の5パッケージが存在する。

**再設計方針: ストラテジーパターンによる統合**

```
perception/
├── detection/
│   ├── lidar_detector/          # 統合パッケージ
│   │   ├── strategies/
│   │   │   ├── centerpoint/     # 差し替え可能なストラテジー
│   │   │   ├── transfusion/
│   │   │   ├── frnet/
│   │   │   └── ptv3/
│   │   ├── detector_factory.hpp # ファクトリーで選択
│   │   └── lidar_detector_node.cpp
│   ├── camera_detector/         # 統合パッケージ
│   │   ├── strategies/
│   │   │   ├── yolox/
│   │   │   ├── bevdet/
│   │   │   └── bevformer/
│   │   └── camera_detector_node.cpp
│   └── radar_detector/
├── tracking/
│   ├── multi_object_tracker/    # 統合（現在の3パッケージ）
│   └── radar_tracker/
├── prediction/
│   └── behavior_predictor/      # map_based + simpl統合
├── traffic_light/
│   └── traffic_light_pipeline/  # 現在の11パッケージを1パイプラインに
├── fusion/
│   ├── sensor_fusion/           # カメラ+LiDAR+レーダー統合
│   └── object_fusion/           # オブジェクトレベル融合
└── segmentation/
    └── ground_segmentation/     # CPU/CUDA版を自動選択
```

**メリット:**
- アルゴリズムの追加・差替が設定ファイルの変更だけで可能
- TensorRT/CUDAの共通コードを一箇所に集約（`autoware_tensorrt_common` の機能をライブラリ化）
- パッケージ数53→20で依存関係の複雑度が大幅低減

### 6.2 Planning: 65 → ~15パッケージ

**問題:** Planningが最大のドメインで、`behavior_path_planner` (12子パッケージ) と `behavior_velocity_planner` (14モジュール) が過剰に分離されている。

**再設計方針: 統合ビヘイビアプランナー**

```
planning/
├── mission_planner/              # ルート計画（現1パッケージ）
├── behavior_planner/             # ★統合ビヘイビアプランナー
│   ├── path_behaviors/           # 以下は同一パッケージ内モジュール
│   │   ├── lane_following.cpp
│   │   ├── lane_change.cpp
│   │   ├── avoidance.cpp
│   │   ├── goal_planner.cpp
│   │   └── start_planner.cpp
│   ├── velocity_behaviors/
│   │   ├── traffic_light.cpp
│   │   ├── crosswalk.cpp
│   │   ├── intersection.cpp
│   │   ├── stop_line.cpp
│   │   └── speed_bump.cpp
│   ├── behavior_tree/            # BehaviorTree実行エンジン
│   └── scenario_manager.cpp      # シナリオ選択（遅延評価方式）
├── motion_planner/               # 運動計画（trajectory_optimizer等統合）
├── freespace_planner/            # 自由空間計画
├── trajectory_postprocessor/     # 軌道後処理・平滑化
└── planning_validator/           # 計画妥当性検証（統合）
```

**重要な設計変更: 遅延シナリオ評価**

```
現状: 全シナリオを並列計算 → シナリオセレクタが1つ選択（計算の無駄）
新設計: 状態評価 → 必要なシナリオのみ活性化 → 計算（Apolloの逐次方式に近い）

StateEvaluator
  → isApproachingIntersection() ? activate(IntersectionModule) : skip
  → isLaneChangeNeeded()       ? activate(LaneChangeModule)   : skip
  → 常時: activate(LaneFollowingModule)
```

### 6.3 Control: 21 → ~8パッケージ

```
control/
├── lateral_controller/           # MPC + PurePursuit（切替可能）
├── longitudinal_controller/      # PID + MPC（切替可能）
├── emergency_controller/         # 緊急停止（AEB統合）
├── vehicle_cmd_gate/             # コマンドゲート（モード切替）
├── control_validator/            # 制御出力検証（統合）
├── collision_checker/            # 衝突検出（3パッケージ→1統合）
├── shift_decider/                # ギア選択
└── operation_mode_controller/    # 運転モード管理
```

### 6.4 Common/Utils: 18 → ~5パッケージ

**問題:** `autoware_utils` が全体の54%から依存される「神ライブラリ」化。

**再設計方針: 機能単位での分割**

```
core_libs/
├── autoware_geometry/       # 幾何計算（Boost.Geometry依存を局所化）
├── autoware_math/           # 線形代数・フィルタ・補間
├── autoware_types/          # 共通型定義（ヘッダオンリー、超軽量）
├── autoware_time/           # 時刻・タイムスタンプユーティリティ
└── autoware_vehicle_model/  # 車両モデル・パラメータ
```

**核心: `autoware_geometry` に Boost.Geometry 依存を封じ込め**

```cpp
// 現状: autoware_utils.hpp をインクルードすると
//       Boost.Geometry全体のテンプレートインスタンス化が発生
// → ビルド時間の主要因

// 新設計: 必要な関数のみを明示的にインクルード
// autoware_geometry は extern template + 明示的インスタンス化を使用
// → クライアント側のテンプレートインスタンス化を抑制

// geometry/include/autoware/geometry/polygon.hpp
extern template bool intersects<Polygon2d>(const Polygon2d&, const Polygon2d&);

// geometry/src/polygon.cpp
template bool intersects<Polygon2d>(const Polygon2d&, const Polygon2d&);
```

---

## 7. ビルドシステムの刷新

### 7.1 現状の問題

- フルビルドに16〜32GBスワップが必要
- `autoware_utils` ヘッダ変更時に132パッケージが再コンパイル
- CI差分ビルドがコアパッケージ変更時に長時間化
- CUDAパッケージの分離が不完全

### 7.2 新ビルドアーキテクチャ

#### ビルドティア制度

```yaml
# build_tiers.yaml - ビルド順序と依存を明示的に管理
tiers:
  tier0_platform:        # ~5分
    packages: [hal_sensor, hal_vehicle, hal_compute, hal_time]
    cache: true           # CIでプリビルドキャッシュ配布

  tier1_runtime:         # ~3分
    packages: [transport, executor, lifecycle, diagnostics]
    depends_on: [tier0_platform]
    cache: true

  tier2_contracts:       # ~1分（IDLコンパイルのみ）
    packages: [contracts]
    depends_on: []
    cache: true

  tier3_core_libs:       # ~3分
    packages: [autoware_geometry, autoware_math, autoware_types]
    depends_on: [tier2_contracts]
    cache: true

  tier4_applications:    # ~15分（並列ビルド）
    packages: [sensing, perception, localization, planning, control, map, system]
    depends_on: [tier3_core_libs, tier1_runtime]
    parallel: true        # ドメイン間は独立並列ビルド

  tier5_tools:           # ~5分
    packages: [visualization, evaluator, simulator, examples]
    depends_on: [tier4_applications]
    optional: true        # 開発ツールは任意
```

#### ヘッダ肥大化対策

```cmake
# 新CMakeLists.txtパターン
autoware_package()

# ビルド時間分析を標準装備
if(ENABLE_BUILD_TIME_ANALYSIS)
  add_compile_options(-ftime-trace)  # Clangビルド時間トレース
endif()

# ヘッダインクルード数チェック（CIで強制）
# 1パッケージが autoware_geometry を含めて良いヘッダ数の上限を設定
add_custom_target(check_include_depth
  COMMAND ${CMAKE_SOURCE_DIR}/tools/check_include_depth.py
          --max-depth 3
          --max-headers-per-tu 50
)
```

#### CUDAパッケージの完全分離

```
# CUDA依存パッケージは別リポジトリに完全分離
autoware_universe_cuda/
├── perception_cuda/
│   ├── ground_segmentation_cuda/
│   ├── lidar_centerpoint/
│   ├── lidar_transfusion/
│   ├── bevfusion/
│   └── tensorrt_common/
└── sensing_cuda/
    └── cuda_pointcloud_preprocessor/
```

CUDAなし環境でもAutowareの基本機能がビルド・実行可能にする。

### 7.3 CI/CDの再構成

```yaml
# 新CIパイプライン
# 34+ワークフロー → 5つの主要パイプライン

pipelines:
  quick_check:           # ~5分（PRオープン時）
    - lint (clang-format, cppcheck, spell-check)
    - tier2_contracts (IDL変更チェック)
    - affected_tier_only  # 変更に影響されるティアのみビルド

  standard_build:        # ~20分（PR承認前）
    - full_build (tier0-4)
    - unit_tests
    - integration_tests

  nightly:               # ~60分（毎晩）
    - full_build_with_cuda
    - full_test_suite
    - coverage_report
    - performance_benchmark

  release:               # リリース時
    - full_build_all_platforms (x86_64, aarch64)
    - safety_test_suite
    - documentation_build

  weekly_analysis:       # 毎週
    - full_clang_tidy
    - dependency_audit
    - build_time_regression_check
```

---

## 8. リアルタイム・安全性アーキテクチャ

### 8.1 混合クリティカリティ設計

```
┌─────────────────────────────────────────────────────────────┐
│                   ASIL-D (Safety Critical)                   │
│  ┌─────────────┐ ┌──────────────┐ ┌──────────────────────┐  │
│  │ Emergency   │ │   Control    │ │  Vehicle Command     │  │
│  │ Stop        │ │   Output     │ │  Gate                │  │
│  └─────────────┘ └──────────────┘ └──────────────────────┘  │
├─────────────────────────────────────────────────────────────┤
│                   ASIL-B (High Integrity)                    │
│  ┌─────────────┐ ┌──────────────┐ ┌──────────────────────┐  │
│  │ Planning    │ │ Localization │ │  System              │  │
│  │ Validator   │ │ Monitor      │ │  Health Monitor      │  │
│  └─────────────┘ └──────────────┘ └──────────────────────┘  │
├─────────────────────────────────────────────────────────────┤
│                   QM (Quality Managed)                       │
│  ┌─────────────┐ ┌──────────────┐ ┌──────────────────────┐  │
│  │ Perception  │ │ Planning     │ │  Localization        │  │
│  │ (ML-based)  │ │ (Behavior)   │ │  (NDT/Visual)        │  │
│  └─────────────┘ └──────────────┘ └──────────────────────┘  │
├─────────────────────────────────────────────────────────────┤
│                   Non-Safety (Best Effort)                   │
│  ┌─────────────┐ ┌──────────────┐ ┌──────────────────────┐  │
│  │ Logging     │ │ Visualization│ │  Remote Monitoring   │  │
│  └─────────────┘ └──────────────┘ └──────────────────────┘  │
└─────────────────────────────────────────────────────────────┘
```

### 8.2 プロセス分離モデル

```
現状: 全ノードが同一Linux環境で非分離実行

新設計: クリティカリティ別のプロセスコンテナ

[Safety Container] ← RT_PREEMPT or RTOS
  ├── emergency_stop_node (SCHED_FIFO, priority=99)
  ├── control_output_node (SCHED_FIFO, priority=90)
  ├── vehicle_cmd_gate    (SCHED_FIFO, priority=85)
  └── watchdog_node       (SCHED_FIFO, priority=99)

[Perception Container] ← CPU + GPU affinity
  ├── lidar_detector      (CPU core 4-7, GPU 0)
  ├── camera_detector     (CPU core 8-11, GPU 1)
  └── fusion_node         (CPU core 12-13)

[Planning Container] ← Standard priority
  ├── behavior_planner    (CPU core 14-17)
  └── motion_planner      (CPU core 18-19)

[Best-Effort Container] ← Lowest priority
  ├── visualization       (any available core)
  ├── logging             (any available core)
  └── remote_api          (any available core)
```

### 8.3 安全契約フレームワーク

```yaml
# contracts/safety_contracts/control_pipeline.yaml
pipeline: sensing → perception → planning → control → actuation
constraints:
  end_to_end_latency_ms: 100     # E2Eレイテンシ上限
  control_output_period_ms: 20    # 制御出力周期
  max_allowed_misses: 2           # 許容デッドラインミス回数
  fallback_on_miss: emergency_stop # ミス時のフォールバック

per_stage:
  sensing:
    max_latency_ms: 10
    data_freshness_ms: 50
  perception:
    max_latency_ms: 30
    min_detection_rate_hz: 10
  planning:
    max_latency_ms: 40
    trajectory_validity_ms: 500
  control:
    max_latency_ms: 5
    output_jitter_ms: 2
```

---

## 9. テスト・品質保証戦略

### 9.1 テストピラミッド

```
                    ┌───────┐
                    │  E2E  │  ← シナリオベースシミュレーション（~20テスト）
                   ─┤       ├─
                  / └───────┘ \
                 /               \
                ┌─────────────────┐
                │  Integration    │  ← パイプライン結合テスト（~100テスト）
               ─┤                 ├─
              / └─────────────────┘ \
             /                       \
            ┌─────────────────────────┐
            │     Component Tests     │  ← ノード単位テスト（~300テスト）
           ─┤                         ├─
          / └─────────────────────────┘ \
         /                               \
        ┌─────────────────────────────────┐
        │          Unit Tests             │  ← 関数・クラス単位（~2000テスト）
        └─────────────────────────────────┘
```

### 9.2 契約ベーステスト

```cpp
// contracts/ で定義されたインターフェースから自動生成されるテスト
// 例: perception_output.idl から自動生成

TEST(PerceptionContract, OutputFreshnessWithinBound) {
  // perception_output は 100ms以内のタイムスタンプを持つこと
  auto output = perception_node->get_latest_output();
  auto age = now() - output.header.stamp;
  EXPECT_LE(age, Duration::from_millis(100));
}

TEST(PerceptionContract, DetectionRateAboveMinimum) {
  // 検出レートが 10Hz 以上であること
  auto rate = measure_output_rate(perception_node, Duration::from_secs(10));
  EXPECT_GE(rate, 10.0);
}
```

### 9.3 パフォーマンス回帰テスト

```yaml
# CI上で毎晩実行されるベンチマーク
benchmarks:
  e2e_latency:
    scenario: straight_road_with_obstacles
    metric: sensing_to_control_latency_ms
    threshold: 100
    regression_tolerance: 10%

  build_time:
    metric: total_build_time_minutes
    threshold: 30
    regression_tolerance: 15%

  memory_usage:
    scenario: urban_driving_10min
    metric: peak_rss_mb
    threshold: 8192
    regression_tolerance: 10%
```

---

## 10. E2E AI統合アーキテクチャ

### 10.1 現状と未来の共存

Autowareは従来のモジュラーパイプライン（感知→認識→計画→制御）からEnd-to-End AIモデル（入力→出力を単一NNで）への移行期にある。TIER IVは2026年から日本50箇所で拡散モデルベースのE2Eシステムを展開予定。

**新設計ではモジュラーとE2Eの共存をファーストクラスでサポート:**

```
┌──────────────────────────────────────────────────────────────┐
│                    Mode Selector                              │
│  ┌────────────────────────┐  ┌─────────────────────────────┐ │
│  │   Modular Pipeline     │  │    E2E Neural Pipeline      │ │
│  │                        │  │                             │ │
│  │ Sensing → Perception   │  │  Sensor Input               │ │
│  │     → Planning         │  │      ↓                      │ │
│  │     → Control          │  │  Diffusion Model            │ │
│  │                        │  │      ↓                      │ │
│  │ (解釈可能・検証可能)    │  │  Trajectory Output          │ │
│  │                        │  │                             │ │
│  │                        │  │ (高性能・適応的)             │ │
│  └──────────┬─────────────┘  └─────────────┬───────────────┘ │
│             │                              │                 │
│             └──────────┬───────────────────┘                 │
│                        ▼                                     │
│  ┌──────────────────────────────────────────────────────┐    │
│  │   Safety Monitor & Arbitrator                        │    │
│  │   (両パイプラインの出力を比較・安全性を検証)          │    │
│  └──────────────────────────────────────────────────────┘    │
│                        ▼                                     │
│  ┌──────────────────────────────────────────────────────┐    │
│  │   Vehicle Command Output                             │    │
│  └──────────────────────────────────────────────────────┘    │
└──────────────────────────────────────────────────────────────┘
```

### 10.2 E2E統合のための設計要件

```
applications/
├── e2e/
│   ├── model_runtime/        # ONNX/TensorRT推論ランタイム
│   ├── diffusion_planner/    # 拡散モデルベースプランナー
│   ├── vla_planner/          # Vision-Language-Action モデル
│   ├── model_registry/       # モデルバージョン管理
│   └── safety_monitor/       # E2E出力の安全性検証
│       ├── trajectory_bounds_checker.cpp
│       ├── comfort_checker.cpp
│       └── modular_cross_validator.cpp  # モジュラー出力との比較
```

**安全監視の原則:** E2Eモデルが出力する軌道が物理的制約（加速度上限、曲率上限、道路境界）を逸脱しないかをルールベースで常時チェック。モジュラーパイプラインの出力と大きく乖離する場合はモジュラー側にフォールバック。

---

## 11. 移行戦略

### 11.1 段階的移行ロードマップ

ゼロから完全に書き直すのは現実的ではない。以下の段階的アプローチを提案する。

```
Phase 1 (3ヶ月): Foundation
  ├── contracts/ の定義（インターフェースIDL + 安全契約）
  ├── core_libs/ の分割（autoware_utils → 5パッケージ）
  ├── runtime/transport/ のゼロコピー実装（Agnocast統合）
  └── ビルドティア制度の導入

Phase 2 (6ヶ月): Core Migration
  ├── Planning の統合（65 → 15パッケージ）
  ├── Perception のストラテジーパターン化
  ├── Control のプロセス分離
  └── 優先度ベースエグゼキュータの導入

Phase 3 (6ヶ月): Safety & Performance
  ├── 混合クリティカリティプロセス分離
  ├── パフォーマンスベンチマークCI導入
  ├── 契約ベーステストの自動生成
  └── RT_PREEMPT対応の検証

Phase 4 (3ヶ月): E2E Integration
  ├── E2Eパイプラインの統合
  ├── モジュラー/E2E アービトレータの実装
  └── 安全監視フレームワークの完成
```

### 11.2 後方互換性ブリッジ

```cpp
// 移行期間中、旧インターフェースを新インターフェースにブリッジ
// contracts/bridge/legacy_msg_adapter.hpp

// 旧: autoware_perception_msgs/msg/DetectedObjects
// 新: contracts/perception_output (IDL)
// → アダプターノードで自動変換

class LegacyPerceptionBridge : public rclcpp::Node {
  // 旧メッセージ型のサブスクリプション
  // → 新契約型への変換
  // → 新トピックへのパブリッシュ
};
```

### 11.3 命名規則の統一

```
移行計画:
  tier4_*_rviz_plugin    → autoware_*_rviz_plugin     (12パッケージ)
  tier4_*_msgs           → autoware_internal_*_msgs    (外部依存)
  tier4_api_utils        → autoware_api_utils          (1パッケージ)
```

全パッケージを `autoware_` プレフィックスに統一し、ベンダー固有パッケージは `autoware_vendor_*` に格納。

---

## 12. まとめ

### 現状 vs 再設計の比較

| 側面 | 現状 | 再設計 |
|------|------|--------|
| **パッケージ数** | 244 | ~75 (69%削減) |
| **ドメイン通信** | DDS全面使用 (38ms遅延) | ゼロコピー共有メモリ (<1μs) |
| **ビルド時間 (推定)** | 30-60分 (フル) | ~15分 (ティア並列) |
| **core_libs変更の影響範囲** | 132パッケージ | ~20パッケージ (分割後) |
| **Planning複雑度** | 65パッケージ/37依存 | 15パッケージ/遅延シナリオ評価 |
| **リアルタイム保証** | なし | 混合クリティカリティ+優先度スケジューリング |
| **E2E AI対応** | 1パッケージ (実験的) | ファーストクラスサポート |
| **安全契約** | 暗黙的 | 明示的 (YAML+IDL定義) |
| **命名規則** | autoware_* + tier4_* 混在 | autoware_* 統一 |
| **設定管理** | 315 YAML分散 | 階層的一元管理 |
| **テスト戦略** | ユニットテスト主体 | テストピラミッド+契約テスト+E2Eシナリオ |

### 再設計で解決される根本課題

1. **ビルド時間の劇的短縮** — ユーティリティ分割 + ティアキャッシュ + extern template
2. **通信レイテンシの桁違い改善** — DDSバイパス + 共有メモリ
3. **Planning計算効率の向上** — 遅延シナリオ評価で不要な計算を排除
4. **安全性の設計時保証** — 混合クリティカリティ + 安全契約 + プロセス分離
5. **E2E AIへの円滑な移行** — モジュラーとE2Eの並行実行・比較検証
6. **新規参入障壁の低減** — パッケージ数69%削減 + 明確なレイヤー構造
7. **車両横展開の容易化** — HAL層によるハードウェア抽象化

### 最終的な設計判断のポイント

この再設計は「理想のアーキテクチャ」を描くものであり、現実のAutowareプロジェクトが直面する**10年の技術的負債**と**500+企業のエコシステム**を踏まえると、一度に実現することは不可能である。しかし、Phase 1〜4の段階的移行により、18ヶ月かけて実現可能な計画として提案する。

特に Phase 1 の「contracts/ 定義」と「autoware_utils の分割」は、後続のすべてのフェーズの基盤となるため、最優先で着手すべきである。

---

## 参考文献・情報源

- [Autoware Foundation GitHub](https://github.com/autowarefoundation/autoware)
- [Autoware Universe GitHub](https://github.com/autowarefoundation/autoware_universe)
- [Autoware公式サイト](https://autoware.org/)
- [Jung et al. "Open-Source Autonomous Driving Software Platforms: Comparison of Autoware and Apollo" (arXiv:2501.18942)](https://arxiv.org/html/2501.18942v1)
- [TIER IV E2Eアーキテクチャ発表](https://autoware.org/advancing-open-source-end-to-end-ai-for-autonomous-driving-at-scale/)
- [Autoware Performance Troubleshooting](https://autowarefoundation.github.io/autoware-documentation/main/support/troubleshooting/performance-troubleshooting/)
- [Build Process Acceleration Issue #4821](https://github.com/autowarefoundation/autoware.universe/issues/4821)
- [Compile Time Analysis Issue #2773](https://github.com/autowarefoundation/autoware_universe/issues/2773)
- [CI Cache Issue #10932](https://github.com/autowarefoundation/autoware_universe/issues/10932)
- [TransformListener Performance Issue #5385](https://github.com/autowarefoundation/autoware/issues/5385)
- [Point Cloud Pipeline Performance Issue #6032](https://github.com/autowarefoundation/autoware.universe/issues/6032)
- [Repository Renaming Discussion #5684](https://github.com/orgs/autowarefoundation/discussions/5684)
- [Autoware Core Implementation Discussion #5365](https://github.com/orgs/autowarefoundation/discussions/5365)
- [CUDA Package Separation Issue #3135](https://github.com/autowarefoundation/autoware_universe/issues/3135)
- [ROS2 Real-Time Performance Optimization (Chinese Journal of Mechanical Engineering)](https://cjme.springeropen.com/articles/10.1186/s10033-023-00976-5)
- [Mixed-Criticality Scheduling in ROS2 (IJRAH)](https://www.ijrah.com/index.php/ijrah/article/download/669/842/1839)
- [Autoware_Perf: Tracing Framework (ScienceDirect)](https://www.sciencedirect.com/science/article/abs/pii/S1383762121002344)
- [ROS2 Performance Evaluation for ADS (arXiv:2411.11607)](https://arxiv.org/html/2411.11607v1)
- [SIM: Shared-Memory Bypass for ROS2 (arXiv:2510.11448)](https://arxiv.org/pdf/2510.11448)
- [Autoware Workshop at IV 2026](https://autoware.org/iv2026/)
