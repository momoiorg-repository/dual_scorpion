# Colab 側 Agent 用プロンプト

以下を Colab の作業 Agent に渡してください。対応するリポジトリのチェックアウトと観測 NPZ が必要です。

```text
Dual Scorpion の Universal Robot Policy Inference 実装を、GPU を使用する Colab
ノートブックで実行してください。目的は保存済みの観測を使う π0.5 と
Isaac-GR00T N1.7 の 1 フレーム推論の検証です。

作業に使うもの
- Universal Inference の実装コミットを含む Dual Scorpion リポジトリ
- docs/robot_policy/colab-backend.md と docs/robot_policy/architecture.md
- ロボットホストの robot-policy capture が出力した observation.npz
- config/robot_policy/pi05.yaml と config/robot_policy/groot_n17.yaml

実行手順
1. リポジトリのコミット、GPU と VRAM、Python、CUDA、モデルへのアクセスを確認して記録してください。
   Hugging Face トークンは Colab シークレットから渡し、出力やノートブックに表示しないでください。
2. マネージド Colab のノートブックで認証済みのファイルアクセスを用い、観測 NPZ を配置してください。
   固定エンドポイント、トンネル、プロキシ、長時間サーバーを使わず、セルで predict を実行してください。
3. π0.5 は LeRobot の pi 環境と保存済みの Pre/Postprocessor を使用してください。
   GR00T N1.7 は独立した NVIDIA SDK 環境で、既存の Gr00tPolicy.get_action を使う
   groot_n17 Adapter を実行してください。チェックポイントに依存する設定はプロファイルで指定してください。
4. 実モデルごとに、同じ観測 NPZ から出力 NPZ を 1 個生成してください。
   予定されている形状は π0.5 が (50, 16)、GR00T N1.7 が (16, 16) です。
   能力、観測との照合、有限性、物理単位、後処理までの処理時間を確認してください。
5. 出力 NPZ と実行の証跡をロボットホストで使える形で報告してください。
   保存済みの元観測と結果で robot-policy validate の Dry-run を行える状態にしてください。
   モーター、操作許可、校正の作成や書き換えはこの作業の対象外です。

機体やモデルの組み合わせが合わない場合は停止し、該当する能力と差分を記録してください。
GPU、依存関係、アクセス権などで実モデルが動かなかった場合はエラーと理由を報告してください。
Mock の結果を提出する場合は、実モデルの結果と区別してください。
```
