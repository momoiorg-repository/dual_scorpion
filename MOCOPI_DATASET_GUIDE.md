# mocopiと頭カメラのデータを記録する

この作業では、頭に装着したC270の映像と、mocopiの頭・左右の腕の姿勢を記録します。
記録した素材は、LeRobot形式へ変換して、同期や動作範囲を数値で評価できます。
MuJoCoでの再生と、検証した関節計画を実機へ再生する機能を用意しています。
実機での動作はまだ未検証です。

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
  --fake --output outputs/mocopi/sessions/fake-001 --duration 3 --task 'コップを移動する' \
  --reference-id workbench-session-A
```

`--fake` は合成画像と合成姿勢を作ります。保存先は毎回、新しいフォルダ名にしてください。

## C270とmocopiで記録する

```bash
uv run --no-sync --active python -m telegrip.mocopi.dataset capture \
  --camera /dev/video2 --output outputs/mocopi/sessions/episode-001 \
  --duration 30 --task 'コップを移動する' --reference-id workbench-session-A
```

`/dev/video2` はカメラ一覧の結果に合わせて変更します。コマンドの起動後、スマホのSENDを
一度停止して再開します。mocopiのフレーム受信後に記録が始まり、指定秒数で終了します。
保存先を変えて、作業ごとに繰り返します。途中でカメラ等が故障した場合は `session.json` が
`aborted` になり、正常な記録として扱いません。
`--reference-id` は、そのmocopiの基準座標と校正が同じ間に付ける任意の名前です。
複数回の記録をまとめる場合は同じ名前を指定し、スマホの再校正や座標のリセット後は別の名前にしてください。
これは人が指定する識別子で、基準座標の一致を自動的に保証するものではありません。

| ファイル | 内容 |
| --- | --- |
| `images/*.jpg` | 頭カメラからの画像 |
| `camera.jsonl` | 各画像のPC上での取得時刻とファイル名 |
| `mocopi.jsonl` | 各mocopiフレームのPC受信時刻と頭・両腕を含む骨の姿勢 |
| `session.json` | 成功状態、作業名、カメラ設定、座標系、データの由来 |

このコマンドはmocopiの頭姿勢を保存します。vSLAMで頭を補正する記録は次の手順です。
カメラ時刻は画像を読み終えたPC時刻、mocopi時刻はUDPを受信したPC時刻であり、
カメラの露光時刻やセンサーの送信時刻そのものではありません。遅延の実測と評価が必要です。
画像には周囲の人や部屋が映ります。`outputs/` はGitの対象外です。

### 頭カメラのvSLAMを使って記録する

[MOCOPI_START_GUIDE.md](MOCOPI_START_GUIDE.md) の手順4〜6に従ってC270のレンズ校正、
stella_vslamの導入、頭とカメラの校正を済ませます。ターミナルAのROS bridgeに
`--dataset-image-port 12354` を追加して起動します。

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=1
uv run --no-sync --active python -m telegrip.mocopi.ros_bridge \
  --config config/mocopi/tracking.yaml --camera /dev/video2 --dataset-image-port 12354
```

ターミナルBでstella_vslamを起動し、Aに `SLAM=OK` が出たことを確認します。
頭とカメラの校正が済んでいなければ、同じ地図で先に `calibrate-head` を実行します。
A・Bを動かしたまま、手順1の環境設定をしたターミナルCで記録します。

```bash
uv run --no-sync --active python -m telegrip.mocopi.dataset capture \
  --head-source slam --image-port 12354 --output outputs/mocopi/sessions/episode-slam-001 \
  --duration 30 --task 'コップを移動する' --reference-id workbench-session-A
```

画像はAからPC内部のローカル通信で受け取ります。mocopiのSENDを停止→再開すると、
mocopiと有効なSLAM姿勢の受信を待ってから記録します。カメラを2回開く必要はありません。
`slam.jsonl` にSLAMカメラ姿勢・追跡状態を、`head_calibration.yaml` に頭への変換を保存します。
レンズ補正済みの画像を保存し、レンズ校正値も記録フォルダへコピーします。
異なるSLAM地図やカメラ校正を検出した場合は停止します。
カメラ・mocopi・SLAM間で同期できない部分やSLAM追跡喪失部分は、変換時に除外します。
実機のC270・mocopi・SLAMを組み合わせた動作は、まだ未検証です。

機器なしでこの経路を試すには `--fake --head-source slam` を指定します。

## 記録を定量評価する

```bash
uv run --no-sync --active python -m telegrip.mocopi.dataset evaluate \
  --input outputs/mocopi/sessions/episode-001 \
  --output outputs/mocopi/reports/episode-001.json
```

画像・mocopiそれぞれの実効FPS、時刻差の平均・95パーセンタイル・最大値、許容差を超えた画像数、
変換に使えるフレーム数、左右の手の移動範囲・移動距離・速度が出ます。
SLAM使用時は追跡喪失数、画像とSLAMの時刻差、bridgeからの画像連番で分かる欠落数も出ます。
時刻差は**PC側で測った差**であり、露光・無線遅延や位置精度の実測値ではありません。

テープで囲った範囲をレポートの `hands_coordinate_frame` と同じ座標系で実測できた場合は、
別途YAMLに左右の範囲を記入し、
`--workspace 設定ファイル.yaml` を追加すると、手が範囲外だったフレーム数も出せます。
書式は [workspace.example.yaml](config/mocopi/workspace.example.yaml) を参照してください。
例の数値は実際のテープ位置ではありません。範囲は自動推定せず、未指定なら範囲外判定を省きます。

## LeRobot形式へ変換する

LeRobotのdataset依存関係を用意します。

```bash
uv pip install --python .venv/bin/python -e '.[dataset]'
```

複数の作業を一つのデータセットにまとめる場合は `--input` を繰り返します。
同じ `--reference-id` を付けた記録を指定します。SLAM使用時は頭の校正値も同じ必要があります。
一致を検証できない過去の記録をまとめる場合は、座標のずれを確認したうえで
`--allow-unverified-reference` を指定できます。mocopi頭姿勢とSLAM補正頭姿勢の記録は別々に変換します。
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
LeRobot側の動画はH.264で圧縮されます。元のJPEG画像も記録フォルダに残ります。

| LeRobotの項目 | 内容 |
| --- | --- |
| `observation.images.head` | 頭カメラの動画 |
| `observation.state` | 頭、左右の肩・肘・手の位置と回転（49値） |
| `observation.mocopi.raw_state` | 補正前のmocopi姿勢（49値） |
| `action` | 次フレームに観測された左右の人間の手の位置と回転（14値） |
| `observation.capture.*` | 画像・mocopi・固定FPSそれぞれの時刻、画像とmocopiのフレーム番号 |
| `observation.slam.camera_pose` | SLAM使用時のみ、SLAM地図上のカメラ姿勢 |

状態とactionの位置はメートル、回転は `qx, qy, qz, qw` の順です。状態の座標系は、
mocopiのみの場合はmocopi座標系、SLAM使用時は校正済みの世界座標系です。
`observation.slam.camera_pose` の位置は単眼SLAMの尺度未補正の地図単位で、メートルではありません。
尺度と頭・カメラの対応、元のカメラ校正値は `meta/mocopi_dataset.json` に記録します。
`action` は**人間の手の次の姿勢**であり、実ロボットの関節角やモーター指令ではありません。
URDFは記録・変換に使わず、後のロボット再生時に選びます。グリッパーの項目はありません。
`--fake` の記録と実機で取得した記録は、誤って混ぜないよう通常は同じ変換に指定できません。

## MuJoCoで記録を再生し、動作を評価する

```bash
uv run --no-sync --active python -m telegrip.mocopi.dataset replay-sim \
  --dataset outputs/mocopi/datasets/demo-001 --episode 0 \
  --plan outputs/mocopi/plans/episode-001.json \
  --report outputs/mocopi/reports/episode-001-sim.json
```

記録した頭カメラ映像とMuJoCo画面が開きます。エピソード0の最初の人の姿勢を、
設定したロボット初期姿勢へ合わせ、各フレームの手・上腕をIKで再生します。
表示なしで数値だけ確認するときは `--headless --fast` を追加します。

レポートには左右の目標手先とFK結果の位置・回転誤差、関節の1更新あたりの動き、
関節可動範囲までの余裕、IK失敗、手の目標が制限されたフレーム数が入ります。
これはシミュレーション上の比較で、実機の追従誤差や衝突の有無を測ったものではありません。
計画JSONにはグリッパーを含まない左右7関節ずつの角度を記録します。
位置誤差が大きい場合は、再生をゆっくりにして計画し直せます。例えば `--slowdown 3` は
各記録フレームを3更新保持し、再生時間を3倍にします。出力ファイルは新しい名前にしてください。

URDFを更新したら、`--left-urdf 新しい左腕.urdf --right-urdf 新しい右腕.urdf` を追加します。
変更したURDFのSHA-256が計画と評価結果に保存されます。既存実装に合わせて関節名
`joint0`〜`joint6`、`gripper`、手先リンク名 `tcp_link` が必要です。
新しいURDFに外観が含まれない場合、リンク間を簡略な線で表示します。
同梱のassembly CAD外観が見つからない環境でも、簡略表示で姿勢を確認できます。

## 実機の再生計画を検証する

まずMuJoCoで映像と腕の動き、生成された計画とレポートを確認し、次を実行します。

```bash
uv run --no-sync --active python -m telegrip.mocopi.dataset replay-real \
  --plan outputs/mocopi/plans/episode-001.json
```

このコマンドはモーターを接続せず、URDFのハッシュ、各関節の可動範囲・速度、
シミュレーション上の目標との差を検証します。記録が `--fake` の場合は、検証時に
`--allow-synthetic` が必要です。位置誤差の既定上限は25mmです。超えた場合は、
MuJoCoの再生で `--slowdown 3` などを指定して計画を作り直してください。
別のURDFを使った計画では、検証にも同じ `--left-urdf` / `--right-urdf` を指定します。

## 実機で再生する

実機の左右ポート、校正済みrobot ID、使用したURDF、モーター回転方向を先に確認します。
記録時のテープ範囲は自動検出されず、再生計画は衝突判定を行いません。
人・障害物との距離、可動範囲、緊急停止手段を確認し、近くで停止できる状態で実行してください。
本機能は実機未検証です。通常の [AGENT_GUIDE.md](AGENT_GUIDE.md) に従って先に実機を校正してください。

```bash
export LEFT_FOLLOWER_PORT=/dev/ttyACM0   # 実際の左腕ポートへ変更
export RIGHT_FOLLOWER_PORT=/dev/ttyACM1  # 実際の右腕ポートへ変更
export ROBOT_ID=my_robot                 # 実際の校正IDへ変更
uv run --no-sync --active python -m telegrip.mocopi.dataset replay-real \
  --plan outputs/mocopi/plans/episode-001.json --execute \
  --left-port "$LEFT_FOLLOWER_PORT" --right-port "$RIGHT_FOLLOWER_PORT" --robot-id "$ROBOT_ID" \
  --report outputs/mocopi/reports/episode-001-real.json
```

実行時は既存の校正ファイルとモーター側の設定を照合し、関節の現在位置が計画の開始位置から
各2度以内であることを確認します。位置モードの左右 `joint0`〜`joint6` だけを使用し、
グリッパーの読み取り・指令は行いません。関節の指令差は1更新0.5度・毎秒15度以内です。
計画から5度を超えて遅れた場合は停止し、取得できた範囲のレポートを保存します。
終了時は実機の現在位置を保持する指令を出し、トルクは有効のまま接続を閉じます。
途中停止は **Ctrl+C** です。通信が失われた場合はソフトから停止指令を届けられないことがあります。

レポートの `measured_fps` と `tracking` に、実際の関節角と目標の差、URDFとエンコーダーから
計算した手先位置・向きの差が出ます。後者は独立した外部計測ではなく、
URDFの誤差やたわみを含む実際の手先位置精度を直接測った値ではありません。
