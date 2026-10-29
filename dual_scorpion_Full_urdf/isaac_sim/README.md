# Isaac Sim / OpenUSD

`scorpion.usd` は中立姿勢、`scorpion_current_pose.usda` は Fusion の表示姿勢です。`scorpion.usda` は読みやすいテキスト形式の元ステージです。

Isaac Sim の `python.sh` から `run.py` を実行できます。例えば、インストール先に応じて次のように実行します。

```sh
<Isaac-Simのディレクトリ>/python.sh /Users/syun/open_pj/scorpion_ver_parts/for_sim/isaac_sim/run.py --pose fusion_current
```

`--headless --steps 1000` も指定できます。OpenUSD でステージ構造を検査していますが、Isaac Sim / PhysX 自体での起動検証は `source/validation.json` の記載を確認してください。
