# MuJoCo

Python 環境に `mujoco` と `numpy` を用意し、このフォルダーで実行します。

```sh
python run.py
python run.py --pose fusion_current
python run.py --headless --steps 1000
```

`scorpion.xml` は衝突形状、親子リンク間の衝突除外、仮の位置制御器、中立姿勢とFusion表示姿勢のキーフレームを含みます。
