# mocopiと頭カメラのデータを記録する

この作業では、頭に装着したC270の映像と、mocopiの頭・左右の腕の姿勢を記録します。
現在はまず素材を保存する段階です。LeRobot形式への変換、定量評価、MuJoCoと実機での再生は
このブランチで続けて実装します。

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
