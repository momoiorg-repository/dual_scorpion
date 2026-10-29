# Agent Handoff — mocopiベース Leaderレス収録システム

最終更新: 2026-10-29（作業ブランチ `dev_mocopi_convert_dataset`、commit `836556d`）

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
| 6 | 再現可能な定量評価の実装と結果取得 | 実装済み（JSONレポート一式）。CSVエクスポートは未実装（次作業） |
| 7 | READMEと使用手順 | `MOCOPI_START_GUIDE.md`・`MOCOPI_DATASET_GUIDE.md`・`README.md`・`HOW_TO_USE.md` を更新済み |
| 8 | 実装・検証結果がGitに保存されている | 逐次 commit / push。未 push がある場合は本ドキュメントの下記を確認 |

## 現在のブランチとコミット

- 作業ブランチ: `dev_mocopi_convert_dataset`（`origin/dev_mocopi_convert_dataset` と同期済み）
- 最新コミット: `836556d feat: replay mocopi plans in the full native Scorpion model`
- 直近の流れ: `a96461b`（URDF差し替え）→ `836556d`（新CADモデルのMuJoCo導入）

URDF差し替え（元の要求1〜4）は完了済み。`/home/syun/for_sim` の内容が
`dual_scorpion_Full_urdf/` へ反映され、次の4ブランチすべてで同一であることを確認済み:

| ブランチ | リモート最新 | 内容 |
|---|---|---|
| `origin/sim` | `c73f98c` | 差し替え済み（旧内容を削除） |
| `origin/main` | `1f64c14` | 差し替え済み |
| `origin/dev` | `90c29a1` | 差し替え済み |
| `origin/dev_mocopi_convert_dataset` | `836556d` | 差し替え済み＋新モデル統合 |

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
- テスト: `tests/teleoperators/test_mocopi_tracking.py`（61件）、`test_mocopi_dataset.py`（11件）、
  `test_mocopi_full_model.py`（3件）

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

## 実装途中の機能

- **定量評価のCSVエクスポート**: JSONレポートは完成。研究での比較表に使うCSV出力と、
  指標の定義（計算式・単位・入力・評価条件・意味、内部整合性と外部Ground Truthの区別）を
  記したドキュメントを追加する予定（本ドキュメント作成時点で未実装）
- **実機でのmocopi/C270/SLAM収録**: コードは完成。実機パケット・取り付け校正・レイテンシの
  実測検証が未実施
- **実機 `--execute` 再生**: コードは完成。安全確認ができるまで実行しない。新CADモデルでは
  設計上拒否される

## 未着手の機能

- グリッパー開閉の収録・変換（設計上の拡張点: `joint7` を固定、`gripper_recorded: false` を
  メタデータに記録済み。将来 `action` の拡張が必要）
- 衝突判定付きの実機再生計画（現状は運動学のみ）
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
| `tests/teleoperators/test_mocopi_dataset.py` | 11 passed |
| `tests/teleoperators/test_mocopi_full_model.py` | 3 passed |
| `uvx ruff@0.14.1 check` / `format --check`（mocopi・tests） | PASS（0.14.1 は `.pre-commit-config.yaml` と同一） |

追加で実行した手動の end-to-end 確認（合成データ、実機非接続）:
`capture --fake` → `evaluate` → `convert` → `replay-sim --headless --fast --slowdown 2`
（新CADモデル、IK失敗0、位置誤差 max 約26mm）→ `convert-joints` → `replay-real`（検証のみ）まで動作。

## 現在発生しているエラー

- テスト失敗・既知の実行時エラーはなし
- 環境上の注意:
  - root `.venv` に `mujoco` 3.14.0 と `lerobot`・`telegrip` が入っている。
    `telegrip_teleoperation/.venv` には mujoco がない
  - `ruff` はvenvに未導入。`uvx ruff@0.14.1 ...` を使う
  - ROS を source した shell では `lark` 不足等で pytest が失敗する（上記の env -u を使う）

## 次に実行するべき具体的な作業

1. 定量評価の仕上げ（優先）:
   - レポート（quality / sim / comparison / real feedback）を横断比較できるCSV出力を追加
   - 指標の定義（計算式・単位・入力・条件・意味、外部Ground Truthと内部整合性の区別）を
     `docs/MOCOPI_EVALUATION.md` として文書化し、README と `MOCOPI_DATASET_GUIDE.md` からリンク
   - テストを追加し commit / push
2. 新CADモデルの計画の安全側の扱いを再確認（`hardware_replay_allowed` の伝搬、ハッシュ照合）
3. 実機なしで検証できる範囲の拡充（ノイズ耐性、長時間の記録、エピソード分割の境界）
4. 実機検証（mocopi実packet、C270露出、SLAM、`replay-real --execute`）:
   安全確認が取れるまでモーターを動かさない。実施したら本ドキュメントへ結果を記録
5. グリッパー拡張の設計メモ（収録開始時のタイムスタンプ同期方法を含む）

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
