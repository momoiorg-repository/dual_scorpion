# Dual Scorpion を LeLab で使う

## ソフトウェアの準備

この手順は Dual Scorpion の作業ディレクトリから実行します。開発ブランチを取得し、Python 3.12 以降と Node.js 22.12 以降を用意してください。CI は Python 3.12 と Node.js 22.23.4 を使います。

```bash
git fetch origin
git switch feat/lerobot-upgrade-lelab
git pull --ff-only
export UV_PROJECT_ENVIRONMENT="$PWD/.venv-lelab"
uv sync --locked --all-packages --extra dual_scorpion --extra lelab
git lfs pull --include="dual_scorpion_Full_urdf/urdf/meshes/visual/*.stl"
```

`UV_PROJECT_ENVIRONMENT` で LeLab 用の環境を作ります。Linux x86-64 で CPU のみを使う場合は `uv sync` に `--extra cpu` を、Pi0.5 を使用する場合は `--extra pi` を追加します。GPU を使う場合は、インストールされる PyTorch と NVIDIA ドライバーの対応を確認してください。LeRobot v0.7.0 では Linux の標準 CUDA ビルドが変更されています。[LeRobot v0.7.0 リリースノート](https://github.com/huggingface/lerobot/releases/tag/v0.7.0)

```bash
uv run --no-sync python -c 'import lelab, lerobot; print(lerobot.__version__, lerobot.__file__); print(lelab.__version__, lelab.__file__)'
npm --prefix lelab/frontend ci --ignore-scripts
npm --prefix lelab/frontend run build
uv run --no-sync lelab --no-open
```

最初のコマンドはバージョンと読み込み先を表示します。`0.7.0` の LeRobot がこのリポジトリの `src/lerobot/` から読み込まれていることを確認してください。ブラウザ用のバンドルをビルドしたら、起動中のターミナルを開いたままページにアクセスします。[LeLab を開く](http://localhost:8000/)

## 物理 Leader と Follower を接続する

1. トップページのロボット選択欄で名前を入力してロボットを作成します。歯車から設定を開き、**Robot model** で **Dual Scorpion** を選びます。SO-101 では **SO-101** を選択します。
2. **Detect serial ports** で接続候補を取得します。Leader left、Leader right、Follower left、Follower right に各アームのポートを入力します。Linux では `ls -l /dev/serial/by-id/` で固定の識別子も確認できます。
3. **Device Type** を切り替え、Leader と Follower のそれぞれで **Advanced** から既存の較正を指定します。標準の LeRobot 較正ディレクトリを使用します。Follower の旧 `so101_dual_follower` フォルダーにある較正も選択できます。`HF_LEROBOT_CALIBRATION` を使う環境では LeLab 起動前に設定します。
4. 新規較正が必要な場合は、作業範囲を確保して両腕を支持し、手動で動かせる状態にします。Leader または Follower を選んで **Start Calibration** を押し、左右の関節とグリッパーを安全な可動域内で動かします。全関節の移動を表示で確認し、完了を選びます。新規の較正はそのデバイス用の左右を含む JSON として保存されます。
5. 設定画面でカメラのスキャンを開始し、カメラ名、解像度、FPS、回転を確認します。学習済みポリシーでは、収録時と同じ名前、寸法、向きで割り当てます。
6. トップページに戻り、対象のロボットを選んで **Teleoperate** を押します。左右の Leader を操作し、Follower のフィードバックを確認します。較正がバスに記録された値と異なる場合は、接続時に照合エラーが表示されます。

LeLab は一度に1つのロボット操作を受け付けます。Dual Scorpion の操作は最大30 Hz、1指令で最大0.5°、グリッパーは最大0.5ポイントに制限します。Follower は位置モード、較正範囲、目標の読み返しを確認してからトルクを有効にします。GUI は通信異常や停止時にデバイスの解放を試みます。

## 収録、可視化、再生

- **収録**ではデータセット名、タスク、FPS、エピソード数を設定します。カメラを確認して開始し、画面のエピソード確定操作でテイクを保存します。終了後にフレーム数とカメラを確認します。Hub へのアップロードは任意です。
- **Dataset Viewer** ではエピソードの動画、フレーム、関節の動きを確認できます。Scorpion のエピソードで **Show joint viewer** を開くと、同期済みの300 mmモデルを表示します。実機側の角度を URDF と照合するには、ロボット設定の **Verified URDF mapping** に `config/mocopi/hardware/` 内の実測済みマッピングのファイル名を指定します。左右のグリッパーを動かして表示する場合は、同じ YAML の `view_gripper_deg.left` と `view_gripper_deg.right` にそれぞれ `closed` と `open` の実測モデル角を度で記録します。未マッピングの関節は画面で実測の数値を確認できます。
- **Replay** は物理 Follower を動かします。対象は関節名が一致する16次元収録です。実行前にエピソードの先頭姿勢へ両腕を合わせてください。開始差が腕で2°、グリッパーで5ポイントを超えると再生を停止します。再生中は **Stop** で中断できます。

LeLab は標準の `HF_LEROBOT_HOME` 配下に置かれた LeRobot v3.0 データセットを検出します。既存の Mocopi 変換データを使う場合は、そのツリーを新しい名前でキャッシュの `local/` 配下にコピーできます。

```bash
cache_dir="${HF_LEROBOT_HOME:-${HF_HOME:-$HOME/.cache/huggingface}/lerobot}"
mkdir -p "$cache_dir/local"
cp -aT --no-clobber outputs/mocopi/datasets/episode-001-ft-motor16 "$cache_dir/local/mocopi-motor16"
```

このコピー例は Linux 用です。`meta/info.json`、Parquet と動画が存在する場合は動画も含めてセットで扱います。旧 v2.1 は変換用の作業ディレクトリにコピーし、Hub への送信を無効にして変換します。`legacy_source` を元データセットの実際のパスに置き換えてください。[公式の移行ガイド](https://huggingface.co/docs/lerobot/porting_datasets_v3)

```bash
legacy_source="$cache_dir/owner/old-dataset"
migration_dir="$(mktemp -d)"
cp -aT "$legacy_source" "$migration_dir/dataset" && \
  HF_HUB_OFFLINE=1 uv run --no-sync python -m lerobot.scripts.convert_dataset_v21_to_v30 \
    --repo-id=local/converted-v30 --root="$migration_dir/dataset" --push-to-hub=false
```

`$migration_dir/dataset/meta/info.json` の `codebase_version` が `v3.0` であることと、エピソード数・フレーム数を確認します。変換したディレクトリを `$cache_dir/local/` 配下の新しい名前にコピーすると Dataset Viewer から開けます。

Full CAD の14次元データはモデル内の関節角を記録します。旧モデルのデータを300 mm モデルで実機再生する前に、[Mocopi ガイド](../MOCOPI_DATASET_GUIDE.md#full-cadの14関節データを実機で再生する) に沿って元の記録から別の出力先へ軌道を生成し、モデルと較正に対応する実測マッピングを作成します。モデルハッシュの照合はこの移行中も有効です。

## Training と Rollout

**Training** でローカルの LeRobot データセットとポリシーを選択します。CPU で疎通を試す場合は、ACT、Local、少ないステップ数とバッチサイズを設定します。学習済みのチェックポイントから **Run on Robot** を開始する際は、選択した Follower、カメラ、ポリシーの入力と出力、制御時間を確認します。Rollout はロボットを動かすため、操作者が現場で停止できる状態で開始してください。

テレオペ、収録、再生、Rollout を終了するときは各画面の **Stop** を使用し、動作と接続状態の停止を確認します。較正は **Cancel Calibration** で中断します。停止後に腕が自重で下がる場合に備え、支持した状態でコネクターを外します。サーバーを終了するには起動したターミナルで Ctrl+C を押します。

既存 CLI のヘルプはモーターを動かさずに確認できます。

```bash
uv run --no-sync lerobot-calibrate --help
uv run --no-sync lerobot-teleoperate --help
uv run --no-sync lerobot-record --help
uv run --no-sync lerobot-replay --help
uv run --no-sync lerobot-train --help
uv run --no-sync lerobot-rollout --help
uv run --no-sync robot-policy --help
uv run --no-sync python -m telegrip.mocopi.dataset --help
```

LeRobot v0.7.0 の学習 CLI は `--trackio.*` を使用します。収録時に RGB コーデックを指定する場合は `--dataset.rgb_encoder.vcodec=h264` を使います。既存の CLI 学習結果を使う場合は **Import model** からチェックポイントを参照します。[LeRobot v0.7.0 リリースノート](https://github.com/huggingface/lerobot/releases/tag/v0.7.0)

## CI とローカルテスト

GitHub Actions の **LeRobot and LeLab** は開発ブランチへの push、PR、手動起動に対応します。変更パスで Python と Frontend の実行範囲を判定し、共通依存関係では両方を実行します。終了時の **Integration checks** は両ジョブの結果を確認します。Python と npm は各ロックファイルを使ってキャッシュし、同じブランチの古い実行をキャンセルします。[ワークフロー](../.github/workflows/lerobot-lelab.yml)

```bash
uv sync --locked --all-packages --extra cpu --extra mocopi --extra test --extra grpcio-dep --extra pi --extra lelab-test
HF_HUB_OFFLINE=1 LEROBOT_TEST_DEVICE=cpu uv run --no-sync pytest -q \
  tests/robots/test_dual_scorpion_follower.py tests/teleoperators/test_dual_scorpion_leader.py \
  tests/scripts/test_dual_scorpion_cli_registration.py tests/teleoperators/test_mocopi*.py \
  tests/teleoperators/test_agent_harness.py tests/remote_policy
(cd lelab && HF_HUB_OFFLINE=1 uv run --no-sync pytest -q tests)
(cd lelab/frontend && npm exec -- tsc --noEmit -p tsconfig.app.json && npm run lint && npm test && npm run build)
gh run list --workflow lerobot-lelab.yml --branch feat/lerobot-upgrade-lelab
```

ROS の pytest プラグインを読み込む端末では、テストコマンドの先頭に `env -u PYTHONPATH` を付けます。通常 CI は Ubuntu、CPU、Mock を使います。実機を使用する検証と GPU 学習は現場で実施します。[GitHub Actions の実行履歴](https://github.com/momoiorg-repository/dual_scorpion/actions/workflows/lerobot-lelab.yml)
