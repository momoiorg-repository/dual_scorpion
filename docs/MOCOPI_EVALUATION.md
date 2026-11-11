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
| `mocopi_discarded_at_capture` / `mocopi_bad_packets` / `mocopi_waiting_for_skdf_packets` | 収録中に破棄したフレーム / 骨格定義受信後の不完全パケット / `skdf`待ちパケット | 件 | mocopi通信の健全性。起動待ちの `skdf`前パケットは `bad_packets` とは別に数える |
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
| `tracking.*.max_joint_step_deg` | 1更新あたり max_k \|q_k − q_{k−1}\| | deg | 指令の滑らかさ。新CADの表示プレビューでは `robot.preview_max_joint_step_deg`（既定20）、実機検証では `robot.max_joint_step_deg`（既定0.5）を使う |
| `tracking.*.joint_limit_margin_deg` | min_k(min(q_k−lower_k, upper_k−q_k)) | deg | **旧URDFのみ**。新CADモデルは限界未測定のため `null` |
| `ik_failures` | 安全ゲートでHOLDしたフレーム数（リスト、`.count` で数えられる） | 件 | NaN・未到達・急変による計画の穴 |
| `workspace_clamped_frames` | 手の目標が到達可能領域へ圧縮されたフレーム数 | フレーム | **旧URDF（`workspace_clamp_measured: true`）のみ**。新CADモデルのプレビューにはclamp段がなく、`null`（未評価） |
| `retarget_clamped_frames` | 手の目標が相対動作の上限を超えたフレーム数 | フレーム | 人の動作がロボットの作業領域を超えた量の代理指標（両モデルで計算） |
| `frames` / `requested_frames` / `source_frames` | 生成フレーム数 / 要求フレーム数 / 元フレーム数 | フレーム | `--slowdown` 倍で保持 |
| `start_pose` / `retarget_start_pose` / `home_start_pose` | IKを開始した姿勢 / Retarget用初期姿勢 / 机回避を含むホーム姿勢 | — | `start_pose`はIK開始条件。`home_start_pose`は物理レイアウト用で、人間動作の初期姿勢へ流用しない |
| `collision_check.contact_count` | MuJoCo接触判定で `contact.dist < 0` となった接触の総数 | 件 | **新CAD MJCFのみ**。床、親子関係にないロボットリンク間、およびYAMLで明示指定した箱形状を対象 |
| `collision_check.frames_with_collisions` | 1フレームでも接触があった更新回数 | フレーム | 同上。`--slowdown` の繰り返しフレームも1更新として数える |
| `collision_check.max_penetration_m` | max(−contact.dist) | m | モデル上の最大めり込み深さ。0は「指定されたモデル内では接触なし」 |
| `collision_check.body_pairs` | ボディ名ペアごとの接触数 | 件 | どの部分が接触したかを特定するための内訳 |
| `obstacle_count` / `collision_check.obstacle_names` | YAMLで指定された箱の個数と名前 | 個・名前 | 実験条件。障害物寸法が実測かどうかはこの値では保証されない |
| `obstacles_sha256` | 正規化した障害物定義のSHA-256 | — | 計画・レポート・関節角データセット間で同一障害物シーンを照合するための識別子 |
| `feasibility` | `ik_failures`・`max_position_error_m`・`max_joint_step_observed_deg` と各上限をまとめた判定 | — | `within_preview_tolerance` は「IK失敗0・位置誤差≤`position_tolerance_m`・関節ステップ≤`max_joint_step_limit_deg`」のとき true。新CADでは「モデル上の衝突0」も条件に加わる。`max_joint_step_limit_deg` は表示プレビュー用、`hardware_max_joint_step_deg` は実機検証用。プレビュー判定であり、実機の安全を保証しない |

評価条件: `--slowdown N` は各元フレームをN更新保持します（速度を落とすほどIKが改善しやすい）。
レポートの `model_kind`、`hardware_replay_allowed`、`mujoco_model_sha256`（または
`urdf_sha256`）が計画とモデルの同一性を示します。
`convert-joints` は `feasibility` とは別に、既定25 mmの位置誤差上限（`--max-sim-error-m`）で
変換を拒否できます。判定基準はレポート内の `criteria` と合わせて必ず確認してください。

### ロボット用データセットの品質状態

`build-robot` はPlanと検証レポートを保存し、検証を通ったフレームだけを関節角データセットへ
書き出します。IK失敗・自己接触がある場合は出力データセットを作りません。位置は許容内でも
手首姿勢誤差が30度を超えるフレームは削除せず、`observation.robot.warning` と
`validation.orientation_warning_ranges` に残します。この場合、`verify` は構造検証の成功と
姿勢品質を区別して `status: warning` を返します。メタデータには元データ・Plan・レポート・
MJCFのハッシュ、Retarget設定、軸対応、実測移動量比、TCPリンク、グリッパー固定状態も保存します。

新CADモデルの関節限界が未測定である理由: MJCFは全関節が `limited="false"` で出力され、
実機の可動範囲・原点・回転方向の対応も未確認です。このため計算上の±180度を仮置きし、
実機再生を拒否します（`hardware_replay_allowed: false`）。実機の可動範囲に基づく
違反率は、限界値を実測して設定するまで算出できません（架空の値を入れない方針）。

新CADモデルの接触判定は、MuJoCoの kinematic state から `mj_forward` の接触リストを読み、
物理シミュレーションを1ステップも実行しません。同梱MJCFは親リンクと子リンクの接触を
除外しているため、報告される接触は床・親子関係にないリンク同士・YAMLで指定した箱です。
未入力のテーブル・人・工具は評価されないため、`collision_check` の0は「指定されたモデル内
では接触なし」を意味し、実機作業空間の衝突安全や未計測障害物の不存在を示しません。

## C. IKモードの比較（`ik-compare` → `dual_scorpion.mocopi.ik_comparison.v1`）

入力: 同じ人間用mocopiデータセットの同一エピソード。`ik-compare` は次の4条件を別々に
MuJoCoで実行し、同じフレーム番号で比較します。元データセットは変更しません。

| モード | 手先位置 | 手先回転 | 上腕・肘の補助目標 |
| --- | --- | --- | --- |
| `default` | 使用 | 使用（従来経路） | 使用（従来の`upper-arm`） |
| `position-only` | 使用 | 使わない | 使わない |
| `hand-6dof` | 使用 | 使用（15度以内） | 使わない |
| `hand-6dof-upper-arm` | 使用 | 使用（15度以内） | 6DoFを優先した冗長自由度で使用 |

`default` は既存のMocopi IKをそのまま使う参考条件です。研究の主比較は `hand-6dof` と
`hand-6dof-upper-arm` で、両者は同じTCP目標、関節限界、位置・回転許容値、初期関節姿勢、
プレビュー関節ステップを使います。違いは上腕・肘の補助目標の有無だけです。この条件一致は
`comparison/fairness_check.json` に、TCP目標の最大差、初期関節角の最大差、制限値の一致、
C条件の成功フレームが許容内かどうか、B/Cのロボット関節軌道差（平均・最大・RMS）として
保存します。

既存 `default` は従来どおり、上腕姿勢を渡すときの初回IKで手先回転を目標にせず、
必要に応じて位置のみへフォールバックする経路を維持します。一方、研究用のB/Cは同じ
6DoF初期解から開始し、Cだけがその後に冗長自由度で上腕・肘姿勢を補正します。この差は
既存動作を変更しないためのもので、B/Cの比較条件には持ち込みません。

| 指標 | 定義（計算式） | 単位 | 意味と限界 |
|---|---|---|---|
| `position_error_mm` | ‖p_FK(q) − p_target‖×1000 | mm | 計画上のTCP目標とMuJoCo FKの差。実機の絶対精度ではない |
| `orientation_error_deg` | 測地角(R_FK, R_target) | deg | 手首姿勢の差。`position-only` は回転をIK目標にしないため参考値 |
| `ik_success_rate` | 成功フレーム数 / 評価フレーム数 | 割合 | 位置許容値と、6DoF条件では回転許容値を満たした割合 |
| `unreachable_count` / `unreachable_rate` | 位置誤差が許容値を超えたフレーム数 / その割合 | フレーム・割合 | 到達不能・IK未収束。失敗を成功として保存しない |
| `hold_count` / `fallback_count` | HOLDしたフレーム数 / 位置のみへフォールバックした数 | フレーム | 研究用モードはフォールバックを禁止し、発生時は失敗として記録 |
| `shoulder_elbow_angle_error_deg` | 肩→肘方向ベクトル間の角度差 | deg | 人体とロボットの上腕方向を共通座標系で比較。Mocopi肘は人体モデル推定 |
| `arm_plane_angle_error_deg` | 肩・肘・手首で作る腕平面の法線間の角度差 | deg | 腕の面の向き。肘が一直線に近いフレームは欠損になりうる |
| `swivel_angle_error_deg` | 肩→手首軸まわりの肘角の差（[-180,180]度に折返し） | deg | 同じTCPでも異なる肘姿勢を区別する。人体の関節角そのものではない |
| `upper_arm_orientation_error_deg` | 初期上腕フレームで対応付けた後の測地角誤差 | deg | 上腕の回転。初期対応付けに依存する |
| `posture_tcp_position_delta_mm` | C条件で上腕姿勢補正の前後におけるTCP位置の変化 | mm | 6DoF目標を優先できたかの確認。採用した補正は0.5 mm以内 |
| `elbow_pose_different_frames` | 肩→肘方向誤差が10度を超えた成功フレーム数 | フレーム | 人間と異なる肘姿勢になった回数 |
| `joint_speed_abs_deg_s` / `joint_acceleration_abs_deg_s2` | 時刻に対する関節角の1次・2次差分の絶対値 | deg/s・deg/s² | 計画軌道の滑らかさ。実機の速度・加速度指令値ではない |
| `joint_reversal_count` | 連続する関節速度の符号反転（両方15 deg/s以上） | 回 | 急な姿勢反転の目安 |
| `joint_limit_margin_deg` | min(min(q−lower), min(upper−q)) | deg | Full MJCFの仮関節限界への余裕。実機の限界は未測定 |
| `self_collision_contacts` / `cross_arm_collision_contacts` / `obstacle_collision_contacts` | モデル上の接触数 | 件 | 床・非隣接リンク・宣言した箱のみ。未入力の机・人・工具は評価しない |
| `frames` / `missing` | 評価フレーム数 / 欠損した値の数 | フレーム | 評価不能な姿勢は補間せず欠損として数える |

`summary.csv` と `summary.json` には各指標のMean、Median、標準偏差、RMSE、P95、最大、最小、
件数、欠損数を保存します。`per_frame_comparison.csv` は全モード・左右・全フレームの値を
持ちます。図は `comparison/graphs/` にPNG（300 dpi）とSVGで保存し、軸・単位・凡例を明記します。
`report.md` は実験条件、データ・モデルのハッシュ、指標定義、数値、失敗区間、限界を記載し、
数値に基づかない優劣の結論は自動生成しません。

この比較はシミュレーション上の運動学評価です。実機エンコーダー、実機のたわみ、実機の
関節限界、未入力の障害物は含みません。上腕・肘の姿勢はMocopiの人体モデル推定を含みます。

## D. 実機再生の誤差（`replay-real` → `dual_scorpion.mocopi.real_metrics.v1`）

入力: 実機のエンコーダー読み取り値と送信指令（実行時）、計画。

| 指標 | 定義（計算式） | 単位 | 意味と限界 |
|---|---|---|---|
| `frames` / `measured_fps` | 読み取りサンプル数 / (n−1)/(t_last−t_first) | 件・Hz | 実際の制御ループの速度 |
| `tracking.*.max_joint_error_deg` | max_k \|measured_k − planned_k\| | deg | 実機の指令追従誤差（計画 vs エンコーダー） |
| `tracking.*.tcp_position_error_m` | ‖p_FK(measured) − p_target‖ | m | エンコーダー＋URDFによる準外部推定。たわみ・URDF誤差を含む |
| `tracking.*.tcp_orientation_error_deg` | 測地角(R_FK(measured), R_target) | deg | 同上 |
| `status` / `failure` / `hold_error` | `complete` / `aborted` / 停止理由 | — | 中断時も取得済み範囲を保存 |

安全ゲート: 開始位置の2度以内一致、関節あたり1更新0.5度、毎秒15度、計画から5度超の遅れで停止。

## E. 実機とシミュレーションの比較（`compare-real` → `dual_scorpion.mocopi.comparison.v1`）

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

## F. 再現と比較の手順

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

# IKモード比較（default / position-only / hand-6dof / hand-6dof-upper-arm）
uv run --no-sync --active python -m telegrip.mocopi.dataset ik-compare \
  --dataset outputs/mocopi/datasets/demo-001 --episode 0 \
  --output outputs/ik_comparison/experiment_001 --preview-max-step-deg 20

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
`metrics-plot` は追加依存なしでSVGを作成し、指定したドット区切り指標ごとに
1パネルを描画します。未取得値は `missing` と明示され、平滑化や補間は行いません。
SVGの `<desc>` にレポートパスと指標名が保存されるため、図の由来をCSVと照合できます。
初期姿勢依存の比較では、`start_pose` 列で2つの実行を区別し、IK失敗数・位置誤差・
関節ステップ・対応する計画JSONの最終関節角差を確認します。この比較は選択した初期姿勢に
対するIKの内部整合性の感度であり、どちらの初期姿勢が物理的に安全かを示すものではありません。

数値指標の前に、データセットそのものの構造検証を行えます（読み込み・寸法・映像デコード・
`action` が次フレーム状態であること・未測定モデルが実機再生可能と主張していないこと）。
人の姿勢データでは、保存されたカメラ・mocopi・固定FPS格子時刻も検証します。判定は
エピソード内の相対時間に対して行い、LeRobotの `timestamp` との格子差、カメラ/mocopi間の
時刻差、フレーム番号の単調性を報告します。

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
