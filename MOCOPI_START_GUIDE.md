# mocopi・ヘッドカメラで両腕を動かす はじめてのガイド

このガイドでは、人間の左右の手の動きを、画面内のDual Scorpionロボット腕に追従させます。
**現在の出力はシミュレーションです。実機ロボットのモータは動かしません。**

まず機器なしで起動し、次にmocopi、最後にヘッドカメラを追加すると、つまずいた場所を確認しやすくなります。
カメラ付きの実機動作はまだ未検証で、外部SLAMの導入も必要です。
詳しい仕様や導入手順は、既存の [HOW_TO_USE.md](HOW_TO_USE.md) にあります。
表示先は**MuJoCoが既定**です。このガイドは現在の起動方法に合わせています。

**vSLAMは頭の追跡を担当します。** 頭に固定したカメラで位置・向きを推定し、
校正したカメラ→HEADの対応から頭の姿勢を求めます。
mocopiの頭から見た手・肩・肘を、その頭の姿勢に重ねて補正します。
二の腕の動きは、上半身集中モードのmocopiから受け取る上腕・肘の情報を使います。

## どこまで試す？

| 試したいこと | 必要なもの | このガイドの手順 |
| --- | --- | --- |
| 画面内の両腕が動くか確認 | PCだけ | 1 → 2 |
| 自分の手で画面内の両腕を動かす | PC、mocopi、スマホ | 1 → 2 → 3 |
| 頭の移動もカメラで補正して追従 | 上記＋ヘッドカメラ、校正用ボード、ROS2、外部SLAM | 1〜6 |

ここでいう「校正」は、カメラやセンサーの値を合わせるための準備です。
スマホで行うmocopiの校正、カメラのレンズの校正、頭とカメラの対応の校正は、それぞれ別の作業です。

## 1. ターミナルを準備する

**新しいターミナルを開くたびに、最初に次の4行を実行してください。**

```bash
cd /home/syun/open_pj/dual_scorpion
export PATH="$HOME/.local/bin:$PATH"
export VIRTUAL_ENV="$PWD/.venv"
export PYTHONPATH="$PWD/telegrip_teleoperation${PYTHONPATH:+:$PYTHONPATH}"
```

まだこのリポジトリの実行環境を作っていない場合だけ、続けてインストールします。
Python 3.12とuvが必要です。

```bash
uv sync --locked
uv pip install --python .venv/bin/python -e './telegrip_teleoperation[mujoco]' pytest
```

すでに実行環境がある場合も、MuJoCoをまだ入れていなければ次の1行だけ実行してください。

```bash
uv pip install --python .venv/bin/python -e './telegrip_teleoperation[mujoco]'
```

以降は `uv run --no-sync --active` を付けて実行します。
`--no-sync` は、追加で入れたTelegripの依存関係を起動時の同期で削除しないための指定です。

## 2. まずPCだけで起動する

mocopiやカメラはまだ接続しなくて大丈夫です。

```bash
uv run --no-sync --active python -m telegrip.mocopi run --fake --sim
```

画面内の左右の腕が動けば、シミュレーション側の準備はできています。
MuJoCoでは台座・腕・グリッパーの実際のSTL形状を表示します。
手先と目標は、位置・向きを同時に示すXYZ座標軸で表示します。
上腕追従中は、小さい青・オレンジの球が肘の目標、ピンクの点が実際の肘位置です。
手先には向きを示す赤(X)・緑(Y)・青(Z)の矢印を表示します。
短い濃い軸が実際の向き、長い半透明の軸が目標の向きです。
外観は `DS_URDF_IK/assembly_1/urdf/assembly_1.urdf` と同梱メッシュを使用します。
関節軸と可動範囲は、既存IKと一致するTelegripの左右URDFを使用します。
関節姿勢を追従させるプレビューで、重力や接触の動力学は実行しません。
IKの計算は、従来のPyBulletを画面なしで使用しています。
`--fake` は、実際のセンサーの代わりにデモ用の動きを入力する指定です。
終了は **Ctrl+C** です。

画面を表示できない環境では、次のコマンドで5秒間動かして、ターミナルに関節角が出るか確認します。

```bash
uv run --no-sync --active python -m telegrip.mocopi run \
  --fake --sim --headless --duration 5
```

## 3. mocopiだけで自分の手に追従させる

### 3-1. センサーを装着する

今回は、ANKLEセンサーを二の腕に移す**上半身集中モード**を使います。

| センサー | 付ける場所 |
| --- | --- |
| HEAD | 頭 |
| WRIST/L | 左手首 |
| WRIST/R | 右手首 |
| ANKLE/L | 左の二の腕 |
| ANKLE/R | 右の二の腕 |
| HIP | 腰 |

スマホのmocopiアプリで「センサーの接続」画面の **︙ → 高度な機能の有効化 → 上半身集中**を選びます。
位置と向きはアプリの装着案内に合わせ、**この装着でアプリ内の校正をやり直してください。**
二の腕のセンサーは上腕の動きを測ります。筋肉の力や筋電位を測るものではありません。

PC側の設定は [tracking.yaml](config/mocopi/tracking.yaml) の次の項目です。
現在は `upper_body` が既定です。スマホ側も同じモードにしてください。

```yaml
mocopi:
  tracking_mode: upper_body
```

PC側のYAMLを変更しても、スマホのモードは自動では切り替わりません。
装着案内をPCで表示するには、次を実行します。

```bash
uv run --no-sync --active python -m telegrip.mocopi mount-check
```

### 3-2. スマホの送信先を設定する

PCとスマホを同じLANにつなぎ、PCで次を実行します。

```bash
hostname -I
```

表示されたIPアドレスから、スマホと同じLANのIPv4アドレスを選びます。
例は `192.168.1.20` です。実際に表示された値を使用してください。

スマホのmocopiアプリで **Motion → SAVEをSENDに切り替え → ネットワーク設定**を開きます。

| 項目 | 設定する値 |
| --- | --- |
| 送信先IP | PCのLAN IPv4アドレス |
| ポート | `12351` |
| 送信形式 | `mocopi (UDP)` |

### 3-3. 受信できるか確認する

**PC側の受信コマンドを先に起動してから、スマホのCaptureで送信を開始します。**
すでに送信中なら、一度停止して再開してください。

```bash
uv run --no-sync --active python -m telegrip.mocopi mocopi-check --duration 30
```

受信FPSと、頭・左右の腕・手の座標が表示されれば受信できています。
30秒で終了します。次へ進む前に、このコマンドが終了したことを確認してください。

### 3-4. カメラなしで両腕を動かす

```bash
uv run --no-sync --active python -m telegrip.mocopi run --mocopi-only --sim
```

MuJoCoを明示する場合は `--sim-backend mujoco` を追加します。
従来の表示を使う場合は `--sim-backend pybullet` に切り替えられます。

```bash
uv run --no-sync --active python -m telegrip.mocopi run \
  --mocopi-only --sim --sim-backend mujoco --start-pose work --alignment body \
  --arm-posture upper-arm
```

1. コマンドを起動したら、スマホの送信を一度停止して再開します。
2. 両肘を曲げ、両手を体の前の楽な位置へ構えて静止します。この姿勢が追従の基準になります。
3. ターミナルに開始の案内が出たら **Enter** を押します。
4. 左手、右手を順番にゆっくり動かし、それぞれの画面内の腕が追従するか確認します。

Enterを押した後の最新の手の位置を基準にします。準備中に手を動かしても、その移動を開始後の急変とは比較しません。
開始後に動きが大きく飛んだ場合は、従来どおりHOLDになります。

### 人間とロボットの開始姿勢を合わせる

既定の `work` は、ロボットの手先が台座の前に来る初期姿勢です。
以前の後ろ向きの折り畳み姿勢は `--start-pose backwards` で選べます。
自分で指定した関節角を使う場合は、YAMLの `robot.initial_joints_deg` を編集し、`--start-pose config` を指定します。

`--alignment body` は、開始時の左右の肩から体の向きを決めます。
あなたが少し違う方向を向いていても、**手を前へ出す → ロボットの前方、左右 → 同じ左右、上へ上げる → 上方**へ対応します。
左右それぞれの手の開始位置・手首の向きは、そのときのロボット手先に対応付けます。
人間とロボットで絶対位置や手首の初期角度を同じにする必要はありません。
人間の肘角度をロボットの肘角度へそのままコピーする方式ではありません。
開始時の上腕・肘もロボットの現在の構えに対応付け、その後の動きだけを反映します。

追従中でも **Enter** を押すと、ロボットは今の位置に保ったまま、手の基準位置と体の向きを取り直します。
手が動かしづらくなったときは、楽な姿勢へ戻して静止し、Enterを押してください。
最初に決めた体の向きはEnterを押すまで固定されます。途中で体の向きを変えた場合も再基準化してください。
`--alignment world` は、体の向きを自動で合わせず、YAMLの座標変換を直接使う設定です。

**`--mocopi-only` を付けると、頭カメラのvSLAM補正は無効です。**
このモードはmocopiの頭・腕の推定値だけを使います。頭のvSLAMを使う場合は手順4〜6へ進みます。
終了は **Ctrl+C**。カメラ付きへ進む前に終了してください。

### 3-5. 肘・二の腕・後ろへ引く動作を確認する

既定の `--arm-posture upper-arm` では、手先の位置を優先し、
**肩から見た肘の移動と上腕の回転**をIKの補助目標にします。
手先をあまり動かさずに肘を開く動きや、二の腕をひねる動きも、ロボットの構えへ反映します。
人とロボットで腕の長さ・関節配置が異なるため、上腕・肘・手首の向きは到達できる範囲で調整します。
手首の向きは、肘・上腕の追従に合わせてずれる場合があります。

開始時は両肘を曲げ、両手を体の前に構えてEnterを押します。次を片腕ずつゆっくり試してください。

1. 手の位置を近くに保ち、肘を外へ開く・上げる。
2. 二の腕を軸にしてゆっくりひねる。
3. 腕を体の前から後ろへ引く。ロボットでは台座へ近づく方向に対応します。

ターミナルの `IK.left.arm_posture` / `IK.right.arm_posture` を確認します。
`active` は姿勢を調整中、`settled` は目標の近く、`limited` は手先・関節の制約により
その時点で姿勢をこれ以上改善できない状態です。`elbow_error_m` は肘の目標との距離、
`upper_arm_error_deg` は上腕の回転誤差です。関節の移動は既定で1更新0.5度なので、数秒間ゆっくり動かしてください。

### 手首の回転を確認する

現在の既定値は `retarget.orientation_scale: 1.0` です。
Enter時から手を90度回すと、ロボットにも90度の回転を目標として送ります。
以前の `0.5` は手の回転を半分にしていたので、自分のYAMLを使う場合もこの値を確認してください。
上腕追従と同時に使う手首の回転の重みも強めています。

両手を前に構えてEnterを押し、手の位置を近くに保って、片方の手をゆっくりひねる・傾ける動きを試します。
ターミナルの `IK.left/right.hand_rotation_deg` は開始時からの手の回転角、
`target_rotation_deg` はロボットに求めた回転角です。
`arm_posture.wrist_error_deg` と、画面の目標・実際の向きの軸で追従を確認できます。
可動範囲と手先位置の制約により、届かない向きでは回転が小さくなります。
また、関節の速度制限があるので、急に手を回すと追従に時間がかかります。

上腕の入力を確認するには追従を終了してから、次を実行します。

```bash
uv run --no-sync --active python -m telegrip.mocopi mocopi-check --duration 20
```

起動後にスマホの送信を停止→再開します。肩・肘の位置は12/13（左）、16/17（右）、
`upper_arm_rotation_deg` は上腕の回転です。ひねったときに回転の値が変わることを確認してください。
値が変わらなければ、スマホの上半身集中モード、ANKLEの装着向き、アプリ内の再校正を確認します。
以前の手先だけの追従と比較するには、`run` に `--arm-posture hand-only` を付けます。

## 4. カメラを選び、レンズを校正する

### 4-1. 使用するカメラを指定する

ヘッドカメラをPCのUSBへつなぎ、一覧を表示します。

```bash
uv run --no-sync --active python -m telegrip.mocopi camera-list
```

一覧の `id` に表示されたデバイスを選びます。
今回のログでは **C270の映像取得用デバイスは `/dev/video2`** です。
同じ名前の `/dev/video3` は映像取得用として開けません。以降は `/dev/video2` を指定します。
USBの接続順で番号が変わった場合は、一覧で映像を取得できるIDを選び直してください。

```bash
uv run --no-sync --active python -m telegrip.mocopi doctor --camera /dev/video2
```

`selected_camera` に指定した値が出ます。これは設定の確認で、映像の取得は次の撮影で確認します。

`--camera 0` のような数値指定や、`--camera /dev/v4l/by-id/...` のパス指定もできます。
省略した場合は [tracking.yaml](config/mocopi/tracking.yaml) の `camera.device` を使います。
解像度とFPSもこのYAMLで設定し、撮影と追従で同じ設定にしてください。

### 4-2. 校正用の写真を撮る

**横9×縦6の内角、1マス25mm** のチェッカーボードを用意します。
印刷用データは [チェッカーボード配布フォルダ](output/pdf/README.md) にあります。
PDFをA4横・倍率100%で印刷し、縮尺確認用の100mmの目盛りを定規で確認してください。
「内角」は白黒のマスが交わる内側の点です。9×6マスという意味ではありません。
1マスの大きさは印刷後に実測してください。

```bash
uv run --no-sync --active python -m telegrip.mocopi camera-capture \
  --camera /dev/video2 --output outputs/mocopi/checkerboard --count 20 --interval 2
```

案内に従って **Enter** を押すと、2秒ごとに20枚撮影します。
ボードを中央・四隅へ動かし、距離や傾きも変えてください。
撮影画像は `outputs/mocopi/checkerboard` に保存されます。
別のカメラや解像度で撮り直す場合は、古い写真と混ぜず、新しいフォルダを指定してください。

### 4-3. 写真からレンズの校正値を作る

```bash
uv run --no-sync --active python -m telegrip.mocopi calibrate-camera \
  --images outputs/mocopi/checkerboard --board 9 6 --square-m 0.025
```

`--images` は撮影で指定したフォルダと合わせます。
ボードの内角数やマスの大きさが違う場合は `--board` と `--square-m` を実物に合わせてください。
`0.025` は25mmをメートルで表した値です。

成功すると `config/mocopi/calibration/` に次の2ファイルができます。

| ファイル | 用途 |
| --- | --- |
| `c270_intrinsics.yaml` | カメラのレンズと歪みの校正値 |
| `c270_stella.yaml` | SLAMが読み込むカメラ設定 |

## 5. カメラ付き追従の準備をする

**ここからはROS2と外部stella_vslamが必要です。**
Pythonのインストールだけではカメラ付き追従は動きません。
まだ導入していない場合は、既存ガイドの [外部stella_vslamの準備](HOW_TO_USE.md#外部stella_vslamの準備) を先に行ってください。
そこには外部ソースの取得、追跡喪失時の処理を直すパッチ、Dockerのビルド、語彙ファイルの取得がまとまっています。
その外部ビルドとライブ融合は、このPCでは未検証です。

次の手順は、既存ガイドに従ってDockerイメージ `dual-scorpion-stella` を作り、
`outputs/mocopi/orb_vocab.fbow` を用意した場合の起動例です。
ネイティブ導入を使う場合の起動方法も既存ガイドにあります。

カメラを頭のバンドにしっかり固定します。HEADセンサーとカメラの位置関係が途中で変わらないようにしてください。
スマホ側の装着・校正も完了させます。

新しいSLAM地図を作る場合は、[tracking.yaml](config/mocopi/tracking.yaml) の `slam.map_id` を
前回とは違う名前にし、次の頭とカメラの校正をやり直してください。初回の既定値は `head-map-001` です。

## 6. 3つのターミナルでカメラ付き追従を起動する

| ターミナル | 担当 | 起動中にしておくもの |
| --- | --- | --- |
| A | カメラ画像を送る | ROS2 bridge |
| B | カメラから頭の動きを推定する | stella_vslam |
| C | mocopiを受信して校正・両腕追従を行う | 校正が終わったら追従コマンドに切り替える |

**A・B・Cのそれぞれで、先に手順1の4行を実行してください。**

### ターミナルA：カメラを起動

撮影と同じカメラを `--camera` で指定します。

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=1
uv run --no-sync --active python -m telegrip.mocopi.ros_bridge \
  --config config/mocopi/tracking.yaml --camera /dev/video2
```

Aは起動したままにして、Bへ進みます。Bの起動前は `SLAM=LOST/initializing` で構いません。

### ターミナルB：SLAMを起動

```bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=1
docker run --rm -it --network host \
  -e ROS_DOMAIN_ID=42 -e ROS_LOCALHOST_ONLY=1 \
  -v "$PWD:/data" dual-scorpion-stella \
  ros2 run stella_vslam_ros run_slam \
  -v /data/outputs/mocopi/orb_vocab.fbow \
  -c /data/config/mocopi/calibration/c270_stella.yaml \
  --ros-args -p publish_tf:=false -p encoding:=bgr8
```

模様のある壁や机を映し、頭を左右へゆっくり平行移動します。
その場で首を回すだけでは単眼SLAMを初期化できません。
**Aに `SLAM=OK` が出たらCへ進みます。AとBは起動したままにします。**

### ターミナルC：頭とカメラの対応を校正

```bash
uv run --no-sync --active python -m telegrip.mocopi calibrate-head
```

コマンドを起動したら、スマホのSEND/Captureを一度停止して再開してください。
同期データを待つ時間は15秒なので、起動後すぐに再開します。

1. 頭・左手首・右手首・左二の腕・右二の腕・腰について、表示された案内を確認して、その都度 **Enter**。
2. 両手を基準にする位置で静止させ、**Enter**。
3. 15秒間の動作説明が出たら、**Enterで計測開始**。
4. 頭を上下左右へゆっくり動かし、首の左右回転・うなずき・傾きも組み合わせます。カメラは模様のある場所を映し続けます。

成功すると `Saved:` と保存先が表示され、`config/mocopi/calibration/head_tracking.yaml` ができます。
エラーになった場合は、頭の動かし方・カメラの固定・SLAMの状態を確認して再実行します。

### 同じターミナルC：両腕追従を開始

校正コマンドが終了したら、AとBを動かしたまま、Cで実行します。

```bash
uv run --no-sync --active python -m telegrip.mocopi run \
  --sim --start-pose work --alignment body --arm-posture upper-arm
```

1. 再びスマホの送信を停止して再開します。
2. 両肘を曲げ、両手を体の前の楽な基準位置で静止させます。
3. 開始の案内が出たら **Enter**。
4. 左右の手をゆっくり動かし、画面内の両腕が追従するか確認します。

カメラの指定はAで行います。Cの `run` はAから姿勢データを受け取るので、`--camera` は付けません。
この起動では `--mocopi-only` を付けず、**vSLAMで頭を補正し、その頭を基準に手・肩・肘を追従**させます。
腕の動きは手順3-5の3種類で確認してください。

## 止まったとき・終了するとき

データが途切れたりSLAMが頭の動きを見失ったりすると、腕は最後の姿勢を維持します。表示の `HOLD` はこの状態です。
同じ地図・取り付け・mocopiの基準が保たれている場合は、受信とSLAMの復旧を確認し、Cで **Enter** を押して基準位置を取り直します。

カメラを付け直した、mocopiをリセットした、SLAMの地図を作り直した場合は、頭とカメラの校正からやり直してください。
カメラ本体や解像度を変更した場合は、レンズの校正もやり直します。

終了は **C → B → A** の順に、それぞれ **Ctrl+C** を押します。

## よくあるつまずき

| 表示・症状 | 最初に確認すること |
| --- | --- |
| `No module named telegrip` | 手順1の4行を実行したか。初回インストールを済ませたか |
| mocopiデータが来ない／`Waiting for skdf` | PC側の起動後にスマホの送信を停止→再開。同じLAN、IPv4、ポート12351も確認 |
| `Address already in use` | `mocopi-check`、`calibrate-head`、`run`を同時に起動していないか |
| カメラが開けない | `camera-list`のIDと指定値を確認。他のカメラアプリやbridgeを終了してから撮影 |
| C270で `/dev/video3` を開けない | 今回の映像取得用IDは `/dev/video2`。`--camera /dev/video2` で起動 |
| ロボットの初期姿勢が手の構えと違う | `--start-pose work --alignment body` で起動し、両手を前に構えてEnter |
| 体の向きを変えたら操作方向が合わない | 楽な姿勢で静止してEnter。ロボットの今の位置から基準を取り直す |
| 二の腕を動かしても構えが変わらない | `--arm-posture upper-arm` と `arm_posture.mode`、スマホの上半身集中モードを確認。手順3-5で入力を確認 |
| 頭カメラの補正が効かない | A・Bが起動し `SLAM=OK` か。Cの `run` に `--mocopi-only` が付いていないか |
| `MuJoCo is not installed` | 手順1のMuJoCo追加インストールを実行 |
| MuJoCoの画面が出ない | デスクトップで起動する。表示環境がなければ `--headless` で数値を確認 |
| 校正用の写真が足りない | ボード全体を映し、距離・傾き・画面内の位置を変えて撮影 |
| `No module named rclpy` | Aで `source /opt/ros/jazzy/setup.bash` を実行したか |
| `stella_vslam_ros`が見つからない | 手順5の外部SLAMの導入を済ませたか |
| `SLAM=LOST/initializing`のまま | Bが起動しているか。模様のある場所を映し、頭を平行移動しているか |
| 頭とカメラの校正が失敗 | Aの `SLAM=OK`、カメラの固定、上下左右の移動と複数方向の回転を確認 |
| 手を動かしても腕が止まる | `tracking_valid`やIKの状態を確認。復旧後Enter。遠くまで動かしすぎていないか |

さらに詳しい対処は [既存ガイドのTroubleshooting](HOW_TO_USE.md#troubleshooting) を参照してください。
設計や数式は [MOCOPI_DESIGN.md](docs/MOCOPI_DESIGN.md)、実装・検証状況は [MOCOPI_IMPLEMENTATION.md](docs/MOCOPI_IMPLEMENTATION.md) にあります。
