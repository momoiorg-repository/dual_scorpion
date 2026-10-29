# mocopi収録の定量評価

このドキュメントは、mocopi + 頭カメラで収録した人の動作が、ロボットの再生計画として
どこまで成立しているかを数値で確認するための指標を定義します。出力はJSONレポートで、
複数レポートを1つのCSV比較表にまとめられます。

## 評価の枠組み（内部整合性と外部Ground Truth）

**重要な区別**: mocopiの測定値だけを基準に計算した残差は、mocopi自体の絶対位置精度や
実機の追従誤差を表しません。本システムの指標は次の3層に分かれます。

1. **データ品質**（収録の時刻同期・欠落）: PCの受信時刻とサンプル数から計算する内部指標。
   露光時刻・無線遅延・モジュール自体の推定精度は含みません。
2. **内部整合性**（モデル上の整合）: mocopi→ロボット座標変換→IK→FKの残差。目標は
   測定値（mocopi手姿勢）であり、ロボットや人の実位置の外部測定ではありません。
   MuJoCo上の位置誤差が小さくても、実機のたわみ・URDF誤差・衝突は測っていません。
3. **外部Ground Truth**（実測に基づく誤差）: 外部測定点（`--reference-csv`）や、
   実機エンコーダー読み取り値との比較。実機エンコーダー＋URDFによるTCP推定も
   独立した外部計測ではないため、準外部として区別します。

`--fake` で生成した合成データの数値は実測値ではなく、配管の疎通確認用です。

## A. 収録の品質（`evaluate` → `dual_scorpion.mocopi.quality.v1`）

入力: 収録フォルダ（`camera.jsonl`、`mocopi.jsonl`、`session.json`、任意で `slam.jsonl`）。
カメラ時刻とmocopi時刻は、いずれもPCが受信した時刻です。

| 指標 | 定義（計算式） | 単位 | 意味と限界 |
|---|---|---|---|
| `camera_fps` / `mocopi_fps` | (N−1)/(t_last−t_first) | Hz | 実効サンプルレート。時刻はPC受信時刻 |
| `camera_to_mocopi_abs_offset_ms` | 各カメラフレーム時刻 t_c に対し \|t_m − t_c\|×1000（t_m は最近傍のmocopi受信時刻）。min/mean/p95/max | ms | 同期ずれの分布。露光・無線遅延は含まない |
| `camera_frames_outside_tolerance` | \|t_m−t_c\| > `max_sync_s` のフレーム数 | フレーム | 変換時に除外されうるフレーム |
| `grid_samples` / `unsynchronized_samples` | 固定FPSのグリッド点の総数と、同期条件を満たさず除外した数 | フレーム | 変換で使えた割合 = 1 − unsync/grid |
| `usable_segments` / `usable_dataset_frames` | 連続する同期区間の数 / Σ max(0, len(segment)−1) | 区間・フレーム | LeRobotの (state, next action) ペア数上限 |
| `mocopi_discarded_at_capture` / `mocopi_bad_packets` | 収録中に破棄したフレーム / 不完全パケット数 | 件 | mocopi通信の健全性 |
| `hands.left/right.bounds_min_m` / `bounds_max_m` | 各軸の最小・最大 | m | 手の到達範囲。座標系は `hands_coordinate_frame` |
| `hands.*.path_length_m` | Σ‖p_{i+1}−p_i‖ | m | 手の総移動距離 |
| `hands.*.speed_m_s` | ‖p_{i+1}−p_i‖・fps（SLAM時）/ Δt（mocopi時） | m/s | 速度の分布 |
| `hands.*.outside_workspace_frames` / `fraction` | 手が作業領域box外のフレーム数 / 割合 | フレーム | `--workspace` 指定時のみ。テープ位置の実測YAMLが必要 |
| `camera_to_slam_abs_offset_ms`、`camera_frames_without_valid_slam`、`camera_stream_missing_frames` | カメラ↔SLAM時刻差、SLAM喪失または同期外のフレーム数、bridge連番の欠落数 | ms・フレーム | SLAM使用時のみ |

評価条件: 既定 `--fps 30`、`--max-sync-s 0.08`。作業領域は自動推定せず、未指定なら
範囲外判定を行いません。

## B. 生成軌道の運動学的品質（`replay-sim` → `dual_scorpion.mocopi.simulation_metrics.v1`）

入力: LeRobotデータセットのエピソードと、選択したモデル（新CAD MJCFまたは旧URDF）。
各フレームで人の手・上腕の目標へIKを解き、返った関節角をFKで再計算して比較します。

| 指標 | 定義（計算式） | 単位 | 意味と限界 |
|---|---|---|---|
| `tracking.*.position_m` | ‖p_FK(q) − p_target‖（各更新、min/mean/p95/max） | m | 内部整合性。人が示した手の位置にどれだけ近い関節角を生成できたか |
| `tracking.*.orientation_deg` | 測地角 acos((tr(R_err)−1)/2)（R_err=R_FK R_target^T）の度換算 | deg | 手首回転の追従。位置優先のため逸脱しうる |
| `tracking.*.max_joint_step_deg` | 1更新あたり max_k \|q_k − q_{k−1}\| | deg | 指令の滑らかさ。上限は `robot.max_joint_step_deg`（既定0.5） |
| `tracking.*.joint_limit_margin_deg` | min_k(min(q_k−lower_k, upper_k−q_k)) | deg | **旧URDFのみ**。新CADモデルは限界未測定のため `null` |
| `ik_failures` | 安全ゲートでHOLDしたフレーム数（リスト、`.count` で数えられる） | 件 | NaN・未到達・急変による計画の穴 |
| `workspace_clamped_frames` | 手の目標が到達可能領域へ圧縮されたフレーム数 | フレーム | **旧URDF（`workspace_clamp_measured: true`）のみ**。新CADモデルのプレビューにはclamp段がなく、`null`（未評価） |
| `retarget_clamped_frames` | 手の目標が相対動作の上限を超えたフレーム数 | フレーム | 人の動作がロボットの作業領域を超えた量の代理指標（両モデルで計算） |
| `frames` / `requested_frames` / `source_frames` | 生成フレーム数 / 要求フレーム数 / 元フレーム数 | フレーム | `--slowdown` 倍で保持 |
| `start_pose` | MuJoCo IKを開始した設定 (`work` / `backwards` / `config`) | — | 初期姿勢依存の比較に必要な実験条件。開始姿勢そのものの良し悪しを示さない |
| `collision_check.contact_count` | MuJoCo接触判定で `contact.dist < 0` となった接触の総数 | 件 | **新CAD MJCFのみ**。床と親子関係にないロボットリンク間を対象 |
| `collision_check.frames_with_collisions` | 1フレームでも接触があった更新回数 | フレーム | 同上。`--slowdown` の繰り返しフレームも1更新として数える |
| `collision_check.max_penetration_m` | max(−contact.dist) | m | モデル上の最大めり込み深さ。0は「同梱モデル内では接触なし」 |
| `collision_check.body_pairs` | ボディ名ペアごとの接触数 | 件 | どの部分が接触したかを特定するための内訳 |
| `feasibility` | `ik_failures`・`max_position_error_m`・`max_joint_step_observed_deg` と各上限をまとめた判定 | — | `within_preview_tolerance` は「IK失敗0・位置誤差≤`position_tolerance_m`・関節ステップ≤`max_joint_step_limit_deg`」のとき true。新CADでは「モデル上の衝突0」も条件に加わる。プレビュー判定であり、実機の安全を保証しない |

評価条件: `--slowdown N` は各元フレームをN更新保持します（速度を落とすほどIKが改善しやすい）。
レポートの `model_kind`、`hardware_replay_allowed`、`mujoco_model_sha256`（または
`urdf_sha256`）が計画とモデルの同一性を示します。
`convert-joints` は `feasibility` とは別に、既定25 mmの位置誤差上限（`--max-sim-error-m`）で
変換を拒否できます。判定基準はレポート内の `criteria` と合わせて必ず確認してください。

新CADモデルの関節限界が未測定である理由: MJCFは全関節が `limited="false"` で出力され、
実機の可動範囲・原点・回転方向の対応も未確認です。このため計算上の±180度を仮置きし、
実機再生を拒否します（`hardware_replay_allowed: false`）。実機の可動範囲に基づく
違反率は、限界値を実測して設定するまで算出できません（架空の値を入れない方針）。

新CADモデルの接触判定は、MuJoCoの kinematic state から `mj_forward` の接触リストを読み、
物理シミュレーションを1ステップも実行しません。同梱MJCFは親リンクと子リンクの接触を
除外しているため、報告される接触は床または親子関係にないリンク同士です。このモデルに
テーブル・人・工具などの外部障害物は含まれないため、`collision_check` の0は
「同梱モデル内では接触なし」を意味し、実機作業空間の衝突安全を示しません。

## C. 実機再生の誤差（`replay-real` → `dual_scorpion.mocopi.real_metrics.v1`）

入力: 実機のエンコーダー読み取り値と送信指令（実行時）、計画。

| 指標 | 定義（計算式） | 単位 | 意味と限界 |
|---|---|---|---|
| `frames` / `measured_fps` | 読み取りサンプル数 / (n−1)/(t_last−t_first) | 件・Hz | 実際の制御ループの速度 |
| `tracking.*.max_joint_error_deg` | max_k \|measured_k − planned_k\| | deg | 実機の指令追従誤差（計画 vs エンコーダー） |
| `tracking.*.tcp_position_error_m` | ‖p_FK(measured) − p_target‖ | m | エンコーダー＋URDFによる準外部推定。たわみ・URDF誤差を含む |
| `tracking.*.tcp_orientation_error_deg` | 測地角(R_FK(measured), R_target) | deg | 同上 |
| `status` / `failure` / `hold_error` | `complete` / `aborted` / 停止理由 | — | 中断時も取得済み範囲を保存 |

安全ゲート: 開始位置の2度以内一致、関節あたり1更新0.5度、毎秒15度、計画から5度超の遅れで停止。

## D. 実機とシミュレーションの比較（`compare-real` → `dual_scorpion.mocopi.comparison.v1`）

| 指標 | 定義（計算式） | 単位 | 意味と限界 |
|---|---|---|---|
| `tracking.*.joint_max_error_deg` | max_k \|measured_k − planned_k\|（レポート内フレームごと） | deg | 関節レベルの実機追従 |
| `tracking.*.tcp_sim_vs_encoder_fk_m` | ‖p_FK(measured) − p_sim‖ | m | 同じURDFを使う推定同士の差。独立計測ではない |
| `tracking.*.tcp_target_vs_encoder_fk_m` | ‖p_FK(measured) − p_target‖ | m | 人が示した目標と実機推定位置の差 |
| `tracking.*.orientation_sim_vs_encoder_fk_deg` | 測地角(R_sim, R_FK(measured)) | deg | 手先姿勢の推定同士の差 |
| `external_reference.measurements.*.target_error_m` | ‖p_ext − p_target‖ | m | **外部Ground Truth**。外部計測点と目標の距離 |
| `external_reference.measurements.*.fk_model_error_m` | ‖p_ext − p_FK(measured)‖ | m | **外部Ground Truth**。URDFモデル誤差の直接的指標 |

`--reference-csv`（列: `frame,side,x_m,y_m,z_m`）は、実機レポートのフレーム番号と`tcp_link`
対応点を、計画と同じ座標系・単位（m）で記入します。時点・座標系を合わせられない測定値は
比較しません。いずれの指標も、計測点が手先の代表点でない場合は回転精度を評価しません。

## E. 再現と比較の手順

```bash
# 1件の収録を評価（品質）
uv run --no-sync --active python -m telegrip.mocopi.dataset evaluate \
  --input outputs/mocopi/sessions/episode-001 --output outputs/mocopi/reports/episode-001.json

# MuJoCo再生（内部整合性）
uv run --no-sync --active python -m telegrip.mocopi.dataset replay-sim \
  --dataset outputs/mocopi/datasets/demo-001 --episode 0 --headless --fast --slowdown 3 \
  --plan outputs/mocopi/plans/e1.json --report outputs/mocopi/reports/e1-sim.json

# 初期姿勢依存の比較。同じデータ・モデル・slowdownで開始姿勢だけ変更する
uv run --no-sync --active python -m telegrip.mocopi.dataset replay-sim \
  --dataset outputs/mocopi/datasets/demo-001 --episode 0 --headless --fast --slowdown 3 \
  --start-pose work --plan outputs/mocopi/plans/e1-work.json \
  --report outputs/mocopi/reports/e1-work-sim.json
uv run --no-sync --active python -m telegrip.mocopi.dataset replay-sim \
  --dataset outputs/mocopi/datasets/demo-001 --episode 0 --headless --fast --slowdown 3 \
  --start-pose backwards --plan outputs/mocopi/plans/e1-backwards.json \
  --report outputs/mocopi/reports/e1-backwards-sim.json

# 複数レポートを1つの比較表へ（品質・シミュレーション・実機・比較のどのレポートも可）
uv run --no-sync --active python -m telegrip.mocopi.dataset metrics-csv \
  --input outputs/mocopi/reports/episode-001.json \
  --input outputs/mocopi/reports/e1-work-sim.json \
  --input outputs/mocopi/reports/e1-backwards-sim.json \
  --output outputs/mocopi/reports/metrics-table.csv
```

CSVは1レポート1行で、ネストした値は `tracking.left.position_m.p95` のようなドット区切り名、
リストは `.count` 列になります。値のないセルは空です。列の並びは入力と独立に決まり、
同じ入力からは常に同じCSVが生成されます（再現・比較用）。数値はJSONの値をそのまま
書き出すため、丸めによる差は生じません。グラフ化はCSVをpandas等で読み込み、
`tracking.*.position_m.p95` などをエピソード間で並べるのが基本操作です。
初期姿勢依存の比較では、`start_pose` 列で2つの実行を区別し、IK失敗数・位置誤差・
関節ステップ・対応する計画JSONの最終関節角差を確認します。この比較は選択した初期姿勢に
対するIKの内部整合性の感度であり、どちらの初期姿勢が物理的に安全かを示すものではありません。

数値指標の前に、データセットそのものの構造検証を行えます（読み込み・寸法・映像デコード・
`action` が次フレーム状態であること・未測定モデルが実機再生可能と主張していないこと）。

```bash
uv run --no-sync --active python -m telegrip.mocopi.dataset verify \
  --dataset outputs/mocopi/datasets/demo-001 --output outputs/mocopi/reports/demo-001-verify.json
```

## 参考: 現時点で評価できない項目

| 項目 | 理由 | 追加に必要なもの |
|---|---|---|
| mocopi単体の絶対位置精度 | 外部基準がない | モーションキャプチャ等の同時計測 |
| 実機手先の物理的な絶対精度 | 外部計測が未実施 | `--reference-csv` 用の実測点 |
| 実機の関節可動範囲違反率 | 実機の限界値が未測定 | 実機での限界測定とURDF/MJCFへの反映 |
| グリッパー動作 | 収録していない | グリッパー計測の追加（設計上の拡張点） |
| カメラ露光・無線の遅延 | PC受信時刻のみ記録 | 同期信号やハードウェアタイムスタンプ |
