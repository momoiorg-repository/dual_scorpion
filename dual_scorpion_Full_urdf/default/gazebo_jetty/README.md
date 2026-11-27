# Gazebo Sim

Gazebo Sim と必要なシステムプラグインをインストールした環境で、このフォルダーから実行します。

```sh
sh run.sh
```

モデルとワールドは SDF 1.12 です。`models/scorpion/model.sdf` に衝突形状、ジョイント、仮の位置制御器があります。`initial_state.json` は Fusion 表示姿勢の記録です。Gazebo 起動時の初期姿勢は中立姿勢です。

SDF の構造とメッシュパスを検査していますが、Gazebo Sim 自体での起動検証は `source/validation.json` の記載を確認してください。
