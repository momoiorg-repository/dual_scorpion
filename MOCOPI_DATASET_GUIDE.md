# mocopiと頭カメラのデータを記録する

この作業では、頭に装着したC270の映像と、mocopiの頭・左右の腕の姿勢を記録します。
記録した素材は、LeRobot形式へ変換して、同期や動作範囲を数値で評価できます。
MuJoCoと実機での再生はこのブランチで続けて実装します。

手首のmocopi姿勢は記録します。グリッパーの開閉量や実ロボットの関節角は記録しません。
記録時にロボットへ接続したり、IKを計算したりしません。人が作業するときは、実機の腕が届く
範囲に貼ったテープを目印にしてください。テープの位置はPCへ自動入力されません。

## 準備

既存の [MOCOPI_START_GUIDE.md](MOCOPI_START_GUIDE.md) の手順1・3に従って環境とmocopiを準備します。
スマホ側は「上半身集中」を選び、ANKLE/LとANKLE/Rを左右の二の腕に装着して校正してください。
HEADは頭、WRIST/L・WRIST/Rは左右の手首、HIPは腰です。
C270は頭に固定し、PCにUSB接続します。

```bash
cd /home/syun/open_pj/dual_scorpion
export PATH="$HOME/.local/bin:$PATH"
export VIRTUAL_ENV="$PWD/.venv"
export PYTHONPATH="$PWD/telegrip_teleoperation${PYTHONPATH:+:$PYTHONPATH}"
uv run --no-sync --active python -m telegrip.mocopi camera-list
```

`camera-list` の映像取得用デバイスを選んでください。USBの接続順で番号は変わります。

## まず機器なしで試す

```bash
uv run --no-sync --active python -m telegrip.mocopi.dataset capture \
  --fake --output outputs/mocopi/sessions/fake-001 --duration 3 --task 'コップを移動する'
```

`--fake` は合成画像と合成姿勢を作ります。保存先は毎回、新しいフォルダ名にしてください。

## C270とmocopiで記録する

```bash
uv run --no-sync --active python -m telegrip.mocopi.dataset capture \
  --camera /dev/video2 --output outputs/mocopi/sessions/episode-001 \
  --duration 30 --task 'コップを移動する'
```

`/dev/video2` はカメラ一覧の結果に合わせて変更します。コマンドの起動後、スマホのSENDを
一度停止して再開します。mocopiのフレーム受信後に記録が始まり、指定秒数で終了します。
保存先を変えて、作業ごとに繰り返します。途中でカメラ等が故障した場合は `session.json` が
`aborted` になり、正常な記録として扱いません。

| ファイル | 内容 |
| --- | --- |
| `images/*.jpg` | 頭カメラからの画像 |
| `camera.jsonl` | 各画像のPC上での取得時刻とファイル名 |
| `mocopi.jsonl` | 各mocopiフレームのPC受信時刻と頭・両腕を含む骨の姿勢 |
| `session.json` | 成功状態、作業名、カメラ設定、座標系、データの由来 |

この記録ではmocopiの頭姿勢を保存します。頭カメラのvSLAM姿勢による補正はまだ記録していません。
カメラ時刻は画像を読み終えたPC時刻、mocopi時刻はUDPを受信したPC時刻であり、
カメラの露光時刻やセンサーの送信時刻そのものではありません。遅延の実測と評価が必要です。
画像には周囲の人や部屋が映ります。`outputs/` はGitの対象外です。

## 記録を定量評価する

```bash
uv run --no-sync --active python -m telegrip.mocopi.dataset evaluate \
  --input outputs/mocopi/sessions/episode-001 \
  --output outputs/mocopi/reports/episode-001.json
```

画像・mocopiそれぞれの実効FPS、時刻差の平均・95パーセンタイル・最大値、許容差を超えた画像数、
変換に使えるフレーム数、左右の手の移動範囲・移動距離・速度が出ます。
時刻差は**PC側で測った差**であり、露光・無線遅延や位置精度の実測値ではありません。

テープで囲った範囲を同じmocopi座標系で実測できた場合は、別途YAMLに左右の範囲を記入し、
`--workspace 設定ファイル.yaml` を追加すると、手が範囲外だったフレーム数も出せます。
書式は [workspace.example.yaml](config/mocopi/workspace.example.yaml) を参照してください。
例の数値は実際のテープ位置ではありません。範囲は自動推定せず、未指定なら範囲外判定を省きます。

## LeRobot形式へ変換する

LeRobotのdataset依存関係を用意します。

```bash
uv pip install --python .venv/bin/python -e '.[dataset]'
```

複数の作業を一つのデータセットにまとめる場合は `--input` を繰り返します。
出力先は新しいフォルダを指定します。既存フォルダには上書きしません。

```bash
uv run --no-sync --active python -m telegrip.mocopi.dataset convert \
  --input outputs/mocopi/sessions/episode-001 \
  --output outputs/mocopi/datasets/demo-001 \
  --repo-id local/mocopi-demo --fps 30
```

画像と姿勢の記録時刻に最も近い値を固定FPSへ対応付けます。既定の許容差は0.08秒で、
同期できない部分は除外してエピソードを分割します。除外数と重複フレーム数は
出力先の `meta/mocopi_dataset.json` に保存します。実際の記録時刻も各フレームに残します。

| LeRobotの項目 | 内容 |
| --- | --- |
| `observation.images.head` | 頭カメラの動画 |
| `observation.state` | 頭、左右の肩・肘・手の位置と回転（49値） |
| `action` | 次フレームに観測された左右の人間の手の位置と回転（14値） |
| `observation.capture.*` | 画像・mocopi・固定FPSそれぞれの時刻、mocopiフレーム番号 |

位置はmocopi座標系のメートル、回転は `qx, qy, qz, qw` の順です。
`action` は**人間の手の次の姿勢**であり、実ロボットの関節角やモーター指令ではありません。
URDFは記録・変換に使わず、後のロボット再生時に選びます。グリッパーの項目はありません。
`--fake` の記録と実機で取得した記録は、誤って混ぜないよう通常は同じ変換に指定できません。
