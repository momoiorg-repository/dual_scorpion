# Colab Backend 実装仕様

## 実行方式

Colab のマネージドランタイムでは、ノートブックのセルから NPZ ファイルで推論します。ノートブックに観測ファイルを置き、`predict` で結果を書き出し、ロボットホストへ戻します。Google はマネージドランタイムからリモートプロキシへの接続を禁止しています。また、GPU の種類、セッション時間、使用量の上限は変動します。この構成では固定の入口や常設ワーカーを前提にしません。 [Google Colab FAQ](https://research.google.com/colaboratory/faq.html)

| 環境 | 入出力 |
| --- | --- |
| マネージド Colab | ノートブックのファイル入出力、または権限を設定した私有 Drive の NPZ ファイル。 |
| 自分で運用する研究室 GPU またはローカル GPU | 同じ `predict`、または `serve` と認証付き gRPC。 |
| Colab のローカルランタイム | 自分で管理するホストの構成に従う。到達可能な GPU ホストで必要な場合は認証付き gRPC を使う。 |

推論環境はロボットのポート、校正、Operation Grant を扱いません。元の観測ファイルはロボットホストで保管します。サンプル画像や指示をノートブックに置くときは、データに合った共有範囲にします。

## ワーカーの準備

1. この機能を含むリポジトリのスナップショットをノートブックで利用できるようにします。`robot-policy` の同じコードを両端で使います。
2. GPU の割り当て、CUDA、Python 3.12、チェックポイントとトークナイザーへの Hugging Face アクセスを確認します。Hugging Face のトークンは Colab のシークレットまたはランタイムの環境を使います。
3. 入力を `/content/observation.npz` に置きます。最初の実行には読み込みと初回推論に十分な時間を設定します。
4. モデルごとに次のコマンドを使用します。設定例の checkpoint と revision、機体、特徴の対応、制御 FPS はペアになっています。別のチェックポイントを使う場合はその学習時設定に合わせます。

### π0.5

Dual Scorpion リポジトリルートを `/content/dual_scorpion` とします。GPU を選び、`nvidia-smi` で環境を確認します。設定例は、LeRobot の保存済み Pre/Postprocessor と 3 台のカメラを使う 50×16 の出力です。

```bash
cd /content/dual_scorpion
uv sync --locked --python 3.12 --extra pi
uv run --no-sync robot-policy predict --config config/robot_policy/pi05.yaml \
  --robot dual_scorpion --input /content/observation.npz --output /content/pi05-output.npz
```

使用する学習済みポリシーには重みと Pre/Postprocessor の設定、統計ファイルが必要です。Google の PaliGemma トークナイザーが必要な場合はノートブックでアクセスを確認します。 [LeRobot π0.5](https://huggingface.co/docs/lerobot/en/pi05) · [Dual Scorpion π0.5 checkpoint](https://huggingface.co/syun88/dual_scorpion_pi05_pick_place_cube)

### GR00T N1.7

NVIDIA の N1.7 環境を別に用意します。上流の dGPU 設定は Python 3.12、NumPy 1.26.4、Transformers 4.57.3 を固定しています。LeRobot の `pi` 環境には別の固定版があるため、NVIDIA の環境で共通 CLI モジュールを `PYTHONPATH` から読みます。NVIDIA は推論に 16 GiB 以上の対応 GPU を案内しています。例のチェックポイントは Flash Attention を使用するので、そのカーネルを利用できる GPU と SDK を選びます。 [NVIDIA の dGPU 導入手順](https://github.com/NVIDIA/Isaac-GR00T/blob/main/scripts/deployment/README.md) · [GPU 要件](https://github.com/NVIDIA/Isaac-GR00T/blob/main/getting_started/hardware_recommendation.md)

```bash
git clone --recurse-submodules https://github.com/NVIDIA/Isaac-GR00T.git /content/Isaac-GR00T
git -C /content/Isaac-GR00T checkout 708617f5716c63fc0a327391492ea2b91977ae90
git -C /content/Isaac-GR00T submodule update --init --recursive
uv sync --project /content/Isaac-GR00T --locked --python 3.12

PYTHONPATH=/content/dual_scorpion/src uv run --project /content/Isaac-GR00T --with 'PyYAML>=6,<7' \
  python -m lerobot.remote_policy predict \
  --config /content/dual_scorpion/config/robot_policy/groot_n17.yaml \
  --robot dual_scorpion --input /content/observation.npz --output /content/groot-output.npz
```

NVIDIA の実行環境が必要とする PyTorch、CUDA、Flash Attention、`nvidia/Cosmos-Reason2-2B` へのアクセスを確認します。`NEW_EMBODIMENT` と保存済みモダリティに従い、`head` を `front` に対応させ、状態を左右の腕とグリッパーの 4 群に分けます。`Gr00tPolicy.get_action` が実行する復元後に 16×16 の絶対位置チャンクを組み立てます。プロファイルの checkpoint をローカルのディレクトリに変更して実行することもできます。 [NVIDIA Policy API](https://github.com/NVIDIA/Isaac-GR00T/blob/main/getting_started/policy.md) · [Dual Scorpion GR00T N1.7 checkpoint](https://huggingface.co/syun88/dual_scorpion_groot_n17_pick_place_cube)

### 自分で運用する GPU ホストでのサービス

π0.5 では LeRobot に `grpcio-dep` を追加し、`robot-policy serve` を起動します。GR00T N1.7 では上の環境に `--with 'grpcio==1.73.1'` を加え、`python -m lerobot.remote_policy serve` を起動します。外部からアクセスする場合は TLS とランダムな Bearer Token を設定します。認証の設定、期限、能力照合は同じプロトコルを使用します。 [gRPC の設定](architecture.md#grpc)

## 結果の確認

ロボットホストへ結果 NPZ を戻し、どの観測から生成したかを照合します。

```bash
uv run --no-sync robot-policy validate --robot dual_scorpion \
  --input /path/to/prediction.npz --observation /path/to/observation.npz
```

記録する項目はリポジトリのコミット、モデルとリビジョン、SDK 版、GPU 名、前処理から出力までの時間、リクエストと観測の ID、結果の次元・有限性・単位、ローカル Dry-run の結果です。拒否された場合は該当理由を記録します。推論が完了したこと、ローカル検証を通過したこと、実機で安全に実行できることはそれぞれ別の判定です。

このリポジトリではローカル CPU で実チェックポイントの π0.5 出力を取得しました。Colab 上での実行と GR00T N1.7 の学習済みモデルによる推論は未検証です。
