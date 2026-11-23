# Universal Robot Policy Inference

`robot-policy` は、ロボットの観測を GPU ワーカーへ渡し、返された関節チャンクをローカルで検証する CLI です。GPU では π0.5、GR00T N1.7、または登録した Policy Adapter を選べます。ローカルの Robot Adapter が観測と実行判定を担当します。

- [アーキテクチャとプロトコル](architecture.md)
- [Colab Backend 実装仕様](colab-backend.md)
- [Colab 側 Agent に渡すプロンプト](colab-agent-prompt.md)

## ハードウェアなしで確認

リポジトリルートで実行します。

```bash
uv sync --locked
uv run --no-sync robot-policy robots
uv run --no-sync robot-policy policies
uv run --no-sync robot-policy doctor --robot mock

PROFILE=config/robot_policy/mock.yaml
uv run --no-sync robot-policy capture --config "$PROFILE" --output /tmp/observation.npz
uv run --no-sync robot-policy predict --config "$PROFILE" --input /tmp/observation.npz --output /tmp/prediction.npz
uv run --no-sync robot-policy validate --config "$PROFILE" --input /tmp/prediction.npz --observation /tmp/observation.npz
uv run --no-sync robot-policy run --config "$PROFILE"
```

Mock は 2 次元の相対アクションと独立したグリッパー絶対値を返します。チャンクを 4 ステップで検証します。

## 収録済み Dual Scorpion データ

データセットのメタデータと動画を含むローカルルートを指定します。Dual Scorpion プロファイルに対応する、腕が度、グリッパーが LeRobot `RANGE_0_100` で収録されたデータを選んでください。一般の LeRobot データセットでは収録時の単位をメタデータから判別できないため、選んだ Robot Adapter の単位を付けます。画像時刻はデータセットのフレーム時刻を使います。

```bash
uv sync --locked --extra dataset
DATASET_ID=syun88/dual_scorpion_pick_place_cube_20261119_142553
DATASET_ROOT="${HF_LEROBOT_HOME:-${HF_HOME:-$HOME/.cache/huggingface}/lerobot}/$DATASET_ID"

uv run --no-sync robot-policy capture --robot dual_scorpion \
  --dataset-root "$DATASET_ROOT" --dataset-id "$DATASET_ID" --frame 0 \
  --output /tmp/dual-observation.npz
```

π0.5 の場合は同じ環境に `pi` を追加します。設定例は CUDA を使います。CPU で動かす場合はプロファイルをコピーし、`policy.options.device` と推論の期限を設定します。チェックポイントとトークナイザーへの Hugging Face アクセスが必要です。

```bash
uv sync --locked --extra pi --extra dataset
uv run --no-sync robot-policy predict --config config/robot_policy/pi05.yaml \
  --robot dual_scorpion --input /tmp/dual-observation.npz --output /tmp/pi05-output.npz

uv run --no-sync robot-policy validate --robot dual_scorpion \
  --input /tmp/pi05-output.npz --observation /tmp/dual-observation.npz
```

GR00T N1.7 は NVIDIA SDK 環境から同じ観測を読みます。起動コマンドと依存関係は共通の Colab 仕様にあります。 [Colab Backend 実装仕様](colab-backend.md)

## GPU ワーカーに接続

gRPC は両端で `grpcio-dep` を使います。下の例はループバック用です。両方のプロセスに同じランダムな `ROBOT_POLICY_TOKEN` を環境変数で設定します。研究室 GPU などでループバック外にリッスンする場合は TLS が必須です。 [gRPC と TLS の設定](architecture.md#grpc)

```bash
# π0.5 worker
uv sync --locked --extra pi --extra grpcio-dep
uv run --no-sync robot-policy serve --config config/robot_policy/pi05.yaml

# Client, with its gRPC dependency installed
uv run --no-sync robot-policy capabilities --config config/robot_policy/remote.example.yaml
uv run --no-sync robot-policy predict --config config/robot_policy/remote.example.yaml \
  --input /tmp/dual-observation.npz --output /tmp/remote-output.npz
```

`remote.example.yaml` の `policy.id` はワーカーの `policy.capabilities.policy_id` と照合されます。GR00T N1.7 に切り替えるときはワーカー側のプロファイルとクライアントの ID を変更します。クライアントにモデル依存パッケージは不要です。

## Dual Scorpion のローカル検証

```bash
uv sync --locked --all-packages --extra dual_scorpion
uv run --no-sync robot-policy doctor --config config/robot_policy/dual_scorpion.live.example.yaml
```

実機の観測を取得する場合は、ローカル HarnessConfig でフォロワー ID と校正、ポート、3 台のカメラを指定します。`robot.options.camera_ids` が Harness の ID を `head`、`right_wrist`、`left_wrist` に対応付けます。`doctor` はポートを開かず、デバイスパスと校正ファイルを調べます。実機 `capture` はエンコーダーとカメラを読み取ります。 [Harness の設定](../../SCORPION_REAL_AGENT_HARNESS.md)

```bash
uv run --no-sync robot-policy capture --config config/robot_policy/dual_scorpion.live.example.yaml \
  --task 'Pick up the cube and place it in the target area' --output /tmp/dual-live-observation.npz
```

`validate` には元の観測ファイルを渡します。検査を通過すると JSON の `checks` と、実行を保留する条件を示す `blockers` が出ます。異常な値やチャンクは終了コード 2 で拒否されます。収録済みの観測やファイルから読み直した観測は解析用です。

Dual Scorpion の実行 API は、ローカルで取得した新鮮な観測と、検証済みの腕の計画に対する一回限りの Operation Grant を要求します。登録した測定シーン、校正、左右の URDF の識別子が一致し、チャンク中のグリッパー目標が現在値のままである場合に Harness の腕実行経路を使用します。グリッパーを変更するチャンクは Dry-run で検証できますが、実行を拒否します。 [実行条件と計画 ID](architecture.md#dual-scorpion-の実行境界)

## 検証結果

| 対象 | この実装での確認 |
| --- | --- |
| Mock | NPZ と gRPC の End-to-End、認証、TLS、期限、古い応答と不一致の拒否、Dry-run。 |
| Dual Scorpion | 収録済み LeRobot データの 16 次元・3 カメラを読み込み。校正、衝突判定、正確な計画への Grant と停止はハードウェアを模擬して検証。 |
| π0.5 | 固定リビジョンの実チェックポイントを CPU で読み込み、上記データセットの先頭フレームから 50×16 を生成。この実行では `right_joint0.pos` の 1→2 ステップ間が 0.799 度で、0.5 度の上限によりローカル検証が拒否。 |
| GR00T N1.7 | 固定リビジョンのモデル設定、モダリティ、統計の幅を確認。SDK の形状と復元済み単位の扱いはテストダブルで検証。SDK が未導入で、ローカルの GPU は NVIDIA の推論要件を下回るため実推論は未確認。 |

受入テストの依存関係は `uv sync --locked --all-packages --extra dual_scorpion --extra pi --extra grpcio-dep` で入ります。`uv run --no-sync pytest -q tests/remote_policy` で実行できます。`ROBOT_POLICY_DATASET_ROOT` を設定すると収録済みデータを使う統合テストも実行します。実データはローカルから読み、テスト時にダウンロードしません。 [NVIDIA の推論環境要件](https://github.com/NVIDIA/Isaac-GR00T/blob/main/getting_started/hardware_recommendation.md)
