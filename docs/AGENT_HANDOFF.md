# Agent Handoff — mocopiベース Leaderレス収録システム

最終更新: 2026-10-29（作業ブランチ `dev_mocopi_convert_dataset`、HEADは`git log -1`で確認）

このドキュメントは、別のエージェント（OpenAI Codex 等）がそのまま作業を再開できるようにするための
現在地の記録です。作業単位が完了するたびに「現在のブランチとコミット」「実行済みテスト」
「次の作業」の節を更新してください。Git の状態と本ドキュメントだけで再開できる状態を保ちます。

## 最終目標

mocopi（頭・左右の腕）と頭部装着 Logitech C270 の映像から、Leaderロボットを使わずに
双腕 Dual Scorpion の動作データを収集し、LeRobot Dataset へ変換する。
MuJoCo と実機で再生・確認でき、研究として説明できる定量評価を持つシステムにする。
グリッパー開閉データは現時点では収録せず、後から追加できる設計にしておく
（架空の実測値を生成しない）。

達成条件と現状:

| # | 条件 | 現状 |
|---|---|---|
| 1 | MocopiとHead Cameraのデータを収録・読み込みできる | 実装済み。`--fake` と保存済み記録のリプレイで検証済み。実機での収録は未検証 |
| 2 | 人間の動作をロボットの関節動作へ変換できる | 実装済み。MuJoCo上のIKで検証済み |
| 3 | LeRobot Datasetを生成・読み込み・検証できる | 実装済み。テストで往復検証済み |
| 4 | MuJoCoでDatasetからの再生を確認できる | 実装済み。既定は新CADモデル（full）、旧URDFは `--sim-model legacy` |
| 5 | 実機への安全な再生経路と実施状況の明示 | 実装済み（`replay-real` の検証ゲートと `--execute`）。実機実行は未実施 |
| 6 | 再現可能な定量評価の実装と結果取得 | 実装済み。JSONレポート一式＋`metrics-csv` 比較表＋`docs/MOCOPI_EVALUATION.md`（指標定義） |
| 7 | READMEと使用手順 | `MOCOPI_START_GUIDE.md`・`MOCOPI_DATASET_GUIDE.md`・`README.md`・`HOW_TO_USE.md` を更新済み |
| 8 | 実装・検証結果がGitに保存されている | 逐次 commit / push。未 push がある場合は本ドキュメントの下記を確認 |

## 現在のブランチとコミット

- 作業ブランチ: `dev_mocopi_convert_dataset`（`origin/dev_mocopi_convert_dataset` と同期済み）
- 最新コミット: `e51a360 feat: render mocopi metric comparison plots` に続き、
  この文書の更新を含む今回のキャプチャ時刻検証コミットが最新になる
- 直近の流れ: `a96461b`（URDF差し替え）→ `836556d`（新CADモデルのMuJoCo導入）→
  `141fb51`（doctorのLFS診断）→ `b9c4164`（full-model IKの実返り値検証とstart-pose記録）
  → `4dd4b60`（Full CADのモデル内衝突評価）→ `bb6bc2c`（jitter/dropout境界テスト）
  → `aad68b5`（ハンドオフ整理）→ `860e520`（120秒ストレステスト）
  → `44b8582`（実機診断の記録）→ `a773ed2`（実機状態の記録）
  → `780150b`（operator-declared obstacle scene）→ `e51a360`（metrics SVG）
  → 今回（Human Datasetのキャプチャ時刻検証）

URDF差し替え（元の要求1〜4）は完了済み。`/home/syun/for_sim` の内容が
`dual_scorpion_Full_urdf/` へ反映され、次の4ブランチすべてで同一であることを確認済み:

| ブランチ | リモート最新 | 内容 |
|---|---|---|
| `origin/sim` | `c73f98c` | 差し替え済み（旧内容を削除） |
| `origin/main` | `1f64c14` | 差し替え済み |
| `origin/dev` | `90c29a1` | 差し替え済み |
| `origin/dev_mocopi_convert_dataset` | `git ls-remote` で確認 | 差し替え済み＋新モデル統合 |

ローカルの `main`・`dev` はリモートより遅れている（fast-forward 可能、force push 不要）。
差し替えは Git LFS を含む。新しいクローンでは `git lfs install && git lfs pull` を実行すること。

## アーキテクチャ（このリポジトリでの配置）

- `telegrip_teleoperation/telegrip/mocopi/` — mocopi受信・校正・retarget・IK・MuJoCoプレビュー
  - `full_mujoco.py` — 新CADモデル（`dual_scorpion_Full_urdf/mujoco/scorpion.xml`）でのIK/FK/表示
  - `mujoco_sim.py` — 旧URDF（左右別）でのプレビュー（`--sim-model legacy`）
- `telegrip_teleoperation/telegrip/mocopi/dataset/` — 収録・評価・変換・再生
  - `capture.py` / `convert.py` — 生記録とLeRobot v3変換
  - `evaluate.py` / `compare.py` — 定量評価
  - `replay.py` / `joint_dataset.py` — MuJoCo再生・計画・関節角データセット
  - `real.py` / `real_dataset.py` — 実機検証ゲート・実行・フィードバック変換
- 設定: `config/mocopi/tracking.yaml`（`robot.mujoco_full_model` を含む）
- テスト: `tests/teleoperators/test_mocopi_tracking.py`（61件）、`test_mocopi_dataset.py`（14件）、
  `test_mocopi_full_model.py`（8件）、`test_mocopi_stress.py`（1件）

## 完成した機能（実装・検証済み）

1. **収録** `telegrip.mocopi.dataset capture`
   - C270映像とmocopi（頭・左右の手・上腕/肘の骨）をPC時刻付きで保存。既定はロボット非接続・IKなし
   - `--fake`（合成入力）、`--head-source slam`（外部SLAMで頭を補正）に対応
   - 失敗時は `session.json` が `aborted` になり正常扱いしない
2. **評価（生記録）** `evaluate`
   - 実効FPS、カメラ↔mocopi時刻差（平均・p95・最大）、許容差超え数、使用可能フレーム数、
     手の移動範囲・距離・速度、任意の作業領域外フレーム数
3. **LeRobot v3変換** `convert`
   - `observation.state` 49値（頭・肩・肘・手の位置と回転）、`action` 14値（次の人の手姿勢）
   - `meta/mocopi_dataset.json` に座標系・単位・カメラ情報・reference-id・除外数を保存
   - グリッパー項目は出力しない（`gripper_recorded: false`）
4. **MuJoCo再生と計画** `replay-sim`
   - 既定は新CADモデル。`observation.state` の手姿勢をIKで関節角へ変換し、計画JSONと
     評価レポートJSONを出力。`--sim-model legacy` で旧URDF、`--mujoco-model` で別MJCF
   - IK失敗・関節ステップ・手先位置/姿勢誤差・HOLDを記録。新モデルは関節限界が未測定のため
     `joint_limit_margin_deg` は `null`（仮の可動範囲を実測値として出さない）
5. **関節角データセット** `convert-joints`
   - 計画からLeRobotデータセットを生成（`observation.state`/`action` 各14関節角、度）
   - 元記録とモデルのハッシュ照合、誤差上限、合成/実データ混在拒否
6. **実機検証ゲート** `replay-real`（`--execute` なしは検証のみ）
   - モデルのハッシュ、関節更新速度、シミュレーション目標との差を確認
   - 新CADモデルの計画は `hardware_replay_allowed: false` として実機再生を明示的に拒否
7. **実機フィードバック** `convert-real` / `compare-real`
   - 実機レポートから読み取り/指令データセットを生成。外部実測TCPのCSV入力で
     Ground Truth 比較が可能（未計測なら推定値のみ）
8. **URDF差し替え**（sim/main/dev/dev_mocopi_convert_dataset へ反映済み）
9. **評価の比較表と指標定義** — `metrics-csv` が保存済みレポート（品質・MuJoCo・実機・比較）を
   1つのCSVへまとめる。ネスト値はドット区切り、リストは `.count`、未取得は空セル。
   `docs/MOCOPI_EVALUATION.md` に計算式・単位・入力・評価条件・限界を定義し、
   内部整合性の評価と外部Ground Truthが必要な評価を区別している
10. **データセットの読み込み検証** — `verify` がLeRobot APIで読み込み、スキーマ・寸法・
    映像デコード・時刻格子・`action` = 次フレーム状態の検算・未測定モデルが実機再生可能と
    主張していないことを確認。失敗時は終了コード1と `errors` を返す
11. **実行可能性の判定** — `replay-sim` レポートの `feasibility` に、IK失敗数・最大位置誤差・
    最大関節ステップと各上限、判定基準を記録し `within_preview_tolerance` を返す
    （プレビュー範囲の運動学判定。衝突・実機安全を保証しない）
12. **グリッパー拡張の設計点** — `docs/MOCOPI_DESIGN.md` に収録・変換・IK・実機再生・評価の
    接続点を整理（`GripperEncoderSource` Protocolが既存）。未計測区間は作らない方針を維持
13. **`doctor` の環境診断** — `mujoco_importable`、新CADモデルの `exists`、メッシュの
    `mesh_state`（`ok` / `lfs_pointer` / `missing`）、`visual_urdf_exists` を追加。
    LFS未取得を実行前に検出できる
14. **Full CAD IKの実返り値検証と初期姿勢記録** — MuJoCo IKは各更新の
    `robot.max_joint_step_deg` を最適化境界に適用し、返された関節角をFKで再計算した
    残差を安全判定に使う。オプティマイザーが大きな解を出してから後処理で切詰め、
    その切詰め前の残差で成功扱いにする偽陽性を防ぐ。`replay-sim --start-pose work|backwards|config`
    で同一データの初期姿勢依存実験を再現でき、選択した `start_pose` は計画・評価レポート・
    関節角データセットのメタデータに保存する。速い合成動作はIK失敗として明示され、
    `replay-real` と `convert-joints` の検証で拒否される
15. **Full CADのモデル内衝突評価** — `replay-sim` は同梱MJCFの床と親子関係にない
    リンク間のMuJoCo接触を物理ステップなしで評価する。レポートの `collision_check`
    に接触数・接触フレーム数・最大めり込み深さ（m）・ボディ組合せ内訳を保存し、
    計画JSONの各フレームにも接触数と最大めり込みを保存する。接触があったFull CAD
    計画、および接触結果のない旧いFull CAD計画は `replay-real` / `convert-joints`
    で拒否する。検証時は計画の関節角から接触結果を再計算し、JSONの接触数・最大めり込みと
    照合する。同梱モデルにテーブル・人・工具などの外部障害物は含まれないため、
    この判定は実機作業空間の衝突安全を保証しない
16. **jitter・ドロップ時のエピソード境界検証** — 合成収録に対して受信時刻の
    ジッタと4フレーム連続のmocopiドロップを注入し、品質評価・LeRobot変換・
    エピソード分割を通して検証する。各エピソードの固定FPS格子が保存され、
    ドロップ区間が時間圧縮されないことをテストで確認している
17. **120秒ストレス確認** — `test_mocopi_stress.py` が論理時間120秒・5 FPSの
    合成セッションに±12 msのジッタと、3箇所の4フレーム連続mocopiドロップを注入する。
    12件の不良/ドロップを記録し、4エピソードへ分割したLeRobot Datasetを生成・検証する。
    各エピソードは0.2秒の固定格子を維持し、ドロップ境界は1.2秒として保存される
18. **operator-declared obstacle scene** — `replay-sim --obstacles` がYAMLの箱形状
    （中心・半サイズ・任意回転）をFull CAD MuJoCoモデルに追加する。寸法は実測入力のみで
    自動推定しない。正規化した定義・SHA-256・個数を計画JSONに保存し、評価レポートと
    関節角データセットにもハッシュ・個数・名前を保存する。`replay-real` と
    `convert-joints` は同じ障害物シーンを再構築して接触を再計算し、不一致・接触が
    あれば拒否する。`verify` も関節角データセット内の障害物ハッシュと個数を検証する。
    旧URDFモデルではこのオプションを拒否する
19. **依存ゼロのSVG比較グラフ** — `metrics-plot` がJSONレポートから指定指標ごとの
    棒グラフSVGを出力する。CSVと同じドット区切り指標名を使い、未取得値は
    `missing` と明示する。数値は平滑化・補間せず、SVGのメタデータにレポートパスと
    指標名を保存する。Matplotlibなどの追加依存は不要
20. **Human Datasetのキャプチャ時刻検証** — `verify` が人の姿勢データについて
    カメラ/mocopi/固定FPS格子時刻の有限性、エピソード内相対格子とLeRobot timestampの一致、
    カメラ/mocopi時刻差が `max_sync_s` 以内であること、フレーム番号の単調性を確認する。
    レポートに格子誤差とカメラ/mocopi誤差の最大値を保存する

## 実装途中の機能

- **実機でのmocopi/C270/SLAM収録**: コードは完成。実機パケット・取り付け校正・レイテンシの
  実測検証が未実施
- **長時間・ノイズ耐性の系統実験**: 論理120秒の合成ストレスは検証済み。
  実機由来の長時間記録・露出や無線遅延を含む実測条件は未検証
- **実機 `--execute` 再生**: コードは完成。安全確認ができるまで実行しない。新CADモデルでは
  設計上拒否される
- **実環境ジオメトリの実測**: 箱形状の障害物シーン入力・ハッシュ・衝突検証は実装済み。
  ただし現在の研究室環境のテーブル・治具・テープ範囲の実寸法は未計測のため、
  実環境に一致するシーンはまだ作成していない

## 未着手の機能

- グリッパー開閉の収録・変換（設計上の拡張点: `joint7` を固定、`gripper_recorded: false` を
  メタデータに記録済み。将来 `action` の拡張が必要）
- RL/模倣学習向けのポリシー学習（本システムはデータ生成まで）

## 実行済みテストと結果（2026-10-29）

ROS環境がshellに入っていると `PYTHONPATH` 経由で `/opt/ros/jazzy` が混入し pytest が
壊れるため、テストは ROS 変数を外して実行する:

```bash
cd /home/syun/open_pj/dual_scorpion
env -u PYTHONPATH -u LD_LIBRARY_PATH -u AMENT_PREFIX_PATH -u CMAKE_PREFIX_PATH \
  .venv/bin/python -m pytest tests/teleoperators -q
```

| テスト | 結果 |
|---|---|
| `tests/teleoperators/test_mocopi_tracking.py` | 61 passed |
| `tests/teleoperators/test_mocopi_dataset.py` | 15 passed（`metrics-csv`・`metrics-plot`・`verify`・キャプチャ時刻検証・start-pose記録・jitter/dropout境界を含む） |
| `tests/teleoperators/test_mocopi_full_model.py` | 8 passed（IK実返り値検証・モデル内衝突評価・障害物シーン・`doctor` を含む） |
| `tests/teleoperators/test_mocopi_stress.py` | 1 passed（論理120秒・ジッタ・3箇所のドロップ） |
| `tests/teleoperators` 全体 | 88 passed, 2 skipped（skip は追加依存が必要な既存テスト） |
| `uvx ruff@0.14.1 check` / `format --check`（mocopi・tests） | PASS（0.14.1 は `.pre-commit-config.yaml` と同一） |

追加の合成end-to-end確認はテストで実施: `capture --fake` → `evaluate` → `convert` →
`replay-sim --headless --fast`（旧URDFと新CAD）→ `convert-joints` → `verify` →
`replay-real`（検証のみ）。速い合成動作では新CADのIK失敗が発生することも確認し、
その計画が実機検証で拒否されることをテストしている。別の静止合成データでは
新CAD計画・関節角データセット・検証ゲートの成功経路を確認した。

## 現在発生しているエラー

- テスト失敗・既知の実行時エラーはなし
- 実機mocopiのUDP受信は現在失敗: `mocopi-check --duration 2` でフレームなし
  （終了コード2）。PC側のUDP 12351受信準備はできているが、スマホ側SENDが
  動作していない可能性がある。物理的なmocopi送信開始・アプリ設定・
  ネットワーク確認が必要
- 環境上の注意:
  - root `.venv` に `mujoco` 3.14.0 と `lerobot`・`telegrip` が入っている。
    `telegrip_teleoperation/.venv` には mujoco がない
  - `ruff` はvenvに未導入。`uvx ruff@0.14.1 ...` を使う
  - ROS を source した shell では `lark` 不足等で pytest が失敗する（上記の env -u を使う）

## 実機診断（2026-10-29、モーター非接触）

- `camera-list`: HD Webcam（C270と同一デバイス表示）を `/dev/video0` で検出。
  MJPG 640x480 30 FPSを含む複数モードを列挙できた。`/dev/video1` はメタデータ用で
  キャプチャ不能。設定ファイルの既定 `/dev/video2` は現在の接続順では存在しない
- C270 1フレーム読み取り: `/dev/video0` で成功。`shape=(480, 640, 3)`,
  `dtype=uint8`。画像は保存していない
- `doctor`: MuJoCo 3.14 import可、新CAD MJCFとメッシュは `ok`、
  visual URDF存在、ハードウェア制御は `disabled`
- `doctor` の注意: `rclpy_importable=false`、`head_calibration_exists=false`。
  したがってSLAM/頭校正を使う実機収録前にROS 2環境と頭校正の準備が必要
- `mocopi-check --duration 2`: UDP 12351でmocopiフレームなし。カメラは読み取れるが、
  現在mocopi実パケット受信まで到達していない
- 再確認 `mocopi-check --duration 5`: 2026-10-29に再実行してもUDP 12351で
  mocopiフレームなし（終了コード2）。同一の外部ブロックが継続している

## 次に実行するべき具体的な作業

1. 実機検証（mocopi実packet、C270露出、SLAM、`replay-real --execute`）:
   まずスマホのmocopi SENDを開始し、`mocopi-check --duration 5` で受信を確認する。
   現在の接続ではC270に `--camera /dev/video0` を指定する。安全確認が取れるまで
   モーターを動かさない。実施したら本ドキュメントへ結果を記録
2. 実機由来の長時間記録が取得できたら、同じ評価・変換・検証コマンドで結果を記録する

## 実機が必要な作業と実機なしで可能な作業の分離

**実機が必要**: mocopi実packet受信、C270/mocopi/SLAMを使った実環境収録、実機露出・
無線遅延の実測、`replay-real --execute` によるモーター駆動、実機エンコーダー比較、
外部TCP実測。ユーザーは現在研究室にいないため、これらは安全確認とともに後日実施する。

**実機なしで継続可能**: 合成収録・保存済みデータの品質評価、LeRobot変換・検証、
MuJoCo IK/FK・衝突・障害物シーン評価、レポート/CSV、文書・テスト・不具合修正。
実機がなくてもGoal全体をblockedにしない。残りのソフトウェア作業がある間はこの節に
現在状態を追記して継続する。

**実機mocopiの現在状態**: UDP 12351でフレームを受信できていない。再開時はスマホ側SENDと
ネットワークを確認する。この状態は実機検証の保留理由であり、ソフトウェア開発の停止理由ではない。

## 必要な実行コマンド（環境設定）

```bash
cd /home/syun/open_pj/dual_scorpion
export PATH="$HOME/.local/bin:$PATH"
export VIRTUAL_ENV="$PWD/.venv"
export PYTHONPATH="$PWD/telegrip_teleoperation${PYTHONPATH:+:$PYTHONPATH}"
git lfs install && git lfs pull          # 新CADモデルのメッシュ取得
uv run --no-sync --active python -m telegrip.mocopi camera-list
```

合成データでの一周（実機不要・推奨の疎通確認）:

```bash
uv run --no-sync --active python -m telegrip.mocopi.dataset capture --fake \
  --output outputs/mocopi/sessions/fake-001 --duration 0.8 --task 'demo' --reference-id fake
uv run --no-sync --active python -m telegrip.mocopi.dataset evaluate \
  --input outputs/mocopi/sessions/fake-001 --output outputs/mocopi/reports/fake-001.json
uv run --no-sync --active python -m telegrip.mocopi.dataset convert \
  --input outputs/mocopi/sessions/fake-001 --output outputs/mocopi/datasets/fake-001 \
  --repo-id local/mocopi-fake --fps 30
uv run --no-sync --active python -m telegrip.mocopi.dataset replay-sim \
  --dataset outputs/mocopi/datasets/fake-001 --episode 0 --headless --fast --slowdown 3 \
  --plan outputs/mocopi/plans/fake-001.json --report outputs/mocopi/reports/fake-001-sim.json
uv run --no-sync --active python -m telegrip.mocopi.dataset convert-joints \
  --dataset outputs/mocopi/datasets/fake-001 --plan outputs/mocopi/plans/fake-001.json \
  --output outputs/mocopi/datasets/fake-001-joints --repo-id local/mocopi-fake-joints
uv run --no-sync --active python -m telegrip.mocopi.dataset replay-real \
  --plan outputs/mocopi/plans/fake-001.json --allow-synthetic
```

## ハードウェア・環境上の制約

- 実機（mocopi、C270、Dual Scorpion、stella_vslam）が接続された状態での動作は未検証。
  このマシンで実機に依存しない検証はすべて `--fake` と保存済み記録で行っている
- 新CADモデル（`dual_scorpion_Full_urdf`）は関節可動限界が未測定
  （MJCFで `limited="false"`、質量・ゲインも仮値）。±180度は計算用の仮値であり、
  実機の可動範囲ではない。よって新CADモデルの計画はシミュレーション専用
  （`hardware_replay_allowed: false`）とし、実機再生を拒否する
- 実機のモーター駆動は安全確認ができるまで実行しない。`replay-real --execute` は
  ポート・robot ID・レポート先を要求し、開始位置の2度以内の一致を確認してから動く
- `outputs/` はGit対象外（収録画像に周囲の人が映るため）。比較・評価結果は
  必要に応じて `docs/` や `outputs/mocopi/reports/` に置き、共有するものだけ commit する
