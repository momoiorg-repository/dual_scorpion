# 実装結果（2026-10-05）

実装前の調査・公式source根拠は [MOCOPI_DESIGN.md](MOCOPI_DESIGN.md)、
現在の操作手順は [MOCOPI_START_GUIDE.md](../MOCOPI_START_GUIDE.md)。
以前の詳細ガイド [HOW_TO_USE.md](../HOW_TO_USE.md) は内容を保持しています。

## 追加・変更ファイル

| ファイル | 役割 |
|---|---|
| `telegrip_teleoperation/telegrip/mocopi/geometry.py` | SE(3)、Sony basis変換、Head-relative world hands |
| 同 `receiver.py` | Sony公式binary chunk受信、skeleton FK、UDP順序/不完全packet検査 |
| 同 `camera.py` | 既存LeRobot OpenCVCamera adapter、一覧、checkerboard校正、stella設定生成 |
| 同 `calibration.py` | metric Sim(3)+rigid extrinsic同時推定、品質検査、YAML保存 |
| 同 `sources.py` | CameraSource/GripperEncoderSource、buffer、fake、record/replay |
| 同 `slam.py` | 外部SLAMからのlocal UDP pose受信 |
| 同 `ros_bridge.py` | 校正済みImage、CameraInfo、stella Odometry入力、PoseStamped/TF出力 |
| 同 `prepare_stella.py` | 外部ROS2 wrapperをTracking時だけpose publishする動作へ変更 |
| 同 `retarget.py` | 左右別anchor、開始時の体の向き合わせ、相対位置・姿勢gain、workspace上限 |
| 同 `posture.py` | 上腕・肘の相対動作を対応付け、TCP位置を保つ冗長関節で姿勢を改善 |
| 同 `ik.py` | 既存PyBullet FK/IK/visualizerのbridge、NaN/limit/jump/error HOLD |
| 同 `mujoco_sim.py` | 既存URDFをMuJoCoへ読み込む両腕プレビュー。IKはPyBullet DIRECTを再利用 |
| 同 `runtime.py` | timestamp同期、timeout、map/hash検査、不連続検出、手動rearm |
| 同 `config.py` | YAML読み込み、相対path解決、主要閾値検査、camera index/path override |
| 同 `mounting.py` | 上半身集中/標準modeの宣言に応じた6センサー装着案内 |
| 同 `__main__.py` / `__init__.py` | guided CLI / package |
| `tests/teleoperators/test_mocopi_tracking.py` | root CI配下の54テスト。MuJoCo/PyBullet未導入時は関連テストだけskip |
| `config/mocopi/tracking.yaml` | 受信/camera/SLAM/同期/校正/retarget/robotの設定 |
| `config/mocopi/calibration/README.md` | 校正値の生成案内。実測値を仮値で置き換えない |
| `telegrip_teleoperation/telegrip/__init__.py` / `core/__init__.py` | existing public exportsを保ったlazy import |
| `telegrip_teleoperation/pyproject.toml` | mocopi-track entry point |
| `telegrip_teleoperation/README.md` | mocopi入口 |
| `.gitignore` | 個人校正値・add-on venvを除外 |
| `HOW_TO_USE.md` / `docs/MOCOPI_DESIGN.md` / 本ファイル | 日本語手順・設計・実装結果 |

rootのdependency/lock、既存IK solver/URDF/実機driverは変更していない。

## 検証結果

- 新規54テスト PASS。SE(3)、hierarchy、Head-relative hands、scale、non-identity
  world alignment、extrinsic、観測不能/noisy calibration拒否、左右、anchor/gain、
  同期/replay、UDP、map不一致、hand jump、NaN IK/残差、関節jump、既存両腕追従/HOLD。
- `camera-capture` / `doctor` / ROS bridgeの `--camera` index/pathが既存OpenCVCameraへ届くこと、
  YAMLが変更されないこと、不正index/空文字が拒否されることをmockで検証。
  ROS依存のimport前にbridgeの `--help` を表示できることも確認。
- MuJoCoへ読み込んだ左右URDFのTCP位置・姿勢が既存FKと一致すること、両腕追従、
  tracking loss/NaN IK時のqpos HOLD、Enter前後の手の移動で起動が失敗しないことを検証。
  開始時のsaved-map alignment検査と開始後の急変HOLDは維持。
- 体の向き0/90/-135度で前後左右上下が同じrobot軸へ対応すること、手の初期位置/姿勢の差の吸収、
  再anchor時のrobot姿勢維持、root fallback、不正肩データの拒否を検証。
  C270の映像入力/video2と非capture node/video3を区別するエラー案内もmockで検証。
- 左右それぞれの肘を開く・上腕の軸回転・後方への引き戻しを、実際のURDF/MuJoCoで180更新検証。
  肘/上腕目標への誤差が改善し、最終TCP位置誤差5mm未満、1更新0.5度以内、URDF limits内を確認。
  再anchorで上腕の初期差も吸収。上腕rotation急変・NaN・bone欠落時HOLDと旧hand-only replayを検証。
  頭だけが回転しているとき、vSLAM→Head補正により静止した手・肩・肘が動かないことを検証。
- MuJoCo 3.14.0で17個のSTLを使用したGUIデモ起動と正常終了、EGLでの画像出力を確認。
  実機mocopi入力でのMuJoCo追従は、この変更後には未確認。
- 合成checkerboard画像24枚からintrinsicsを推定し、既知focal lengthとの整合を検証。
  実機intrinsicsの代用品としては保存していない。
- 既存Dual Scorpion leader/follower/CLI registration: 9テスト PASS。
- 追加・変更Pythonのruff check PASS、format完了。
- fake CLI: 真値scale=1.7を推定し、左右7関節が時間とともに変化。
- 記録JSONLと対応校正YAMLからreplay CLI起動成功。
- `run --hardware` は明示的にexit code 2で拒否。
- USB webcam `/dev/video0`: 640×480 BGR frameを既存LeRobot captureで取得。
  camera detectionで30 FPS profileを確認。型番は自動判定していない。
- `/opt/ros/jazzy/setup.bash` をsourceしたPython3.12でROS2/Image/CameraInfo/
  Odometry/TF adapter依存をimport成功。

## 未検証と残課題

Sony SDK/Receiver **2026公式ソース**に基づくreceiverだが、手元のmocopi実packetは取得していない。
公式sourceに沿う合成packetでのテストは実機互換性検証と区別する。
実測checkerboard/取り付けextrinsic、USB/network/exposure遅延、mocopi漂流、
外部stellaのビルドとライブ融合精度は未検証。
Lost時の予測poseをそのままpublishしないため外部wrapperへ必須Tracking guardを適用。
source patch適用とidempotencyを検証したが、C++ビルドは未検証。
motion packetから個々のmocopi sensorの接続状態を判定する機能はない。
このPCにstella_vslam_rosはなく、上流Docker/native導入はHOW_TO_USEの別手順。

SLAMはstella_vslam>=0.3/BSD-2を外部processとして使用するadapterを実装。
Linux/ROS2/UVC/Image/校正/map保存の相性を理由に選定。
ORB-SLAM3はGPLv3、公式exampleのROS世代と本C270のIMU欠如も考慮した。
現時点でライブSLAM込みの実機成功を主張しない。

世界poseはSony Head(10)/l_hand(14)/r_hand(18)と祖先骨を使用。
Sony右手系X-left/Y-up/Z-forwardから内部X-forward/Y-left/Z-upへ変換。
W=Mをmetric worldとし、未尺度SLAMのtranslationには校正scaleを適用、
独立したmetric camera→virtual Head bone extrinsicを合成する。
校正は一定時間の並進+複数軸回転を同期収集し、13変数のrobust fitと観測可能性検査。
raw physical Head sensor中心の追跡精度を保証するものではない。

追記: ANKLEを二の腕へ移すSony上半身集中モードの装着案内を既定にし、mount-checkを追加。
上腕回転が同側Handへ伝わること、足のboneを腕へ読み替えていないこと、
肩・肘world poseのHead-relative変換と左右分離をテストした。
肩・肘xyz、上腕rotationとROS Pose/TFを出力し、肘位置・上腕回転のsoft IK目標を追加。
上半身集中モードの実UDP・推定精度の改善は未検証。

`--arm-posture upper-arm`を既定とし、Sony upper-arm(12/16)のworld回転と
肩→肘ベクトルのneutralからの変化を使用する。腕の長さの比とgainでrobot目標へ写し、
最初のrobot肘位置/上腕rotationへanchorする。Enterでは現在のrobot構えから再anchor。
robot upper-armはURDF joint2 childのlink frame、elbowは既存FK skeletonと同じjoint4 origin。
mesh重心を骨の位置として使用しない。
既存IKでTCP位置を解き、既存FKの数値JacobianとSVDからTCP位置のnull spaceを求める。
その空間で肘位置・上腕回転・手首回転のdamped least-squaresを解き、上腕/肘を手首より重視する。
URDF limitsとlast commandからのslewを適用した後、FKで主目標からの位置変化0.5mm以内と
姿勢objectiveの改善を再確認する。上腕目標のためにTCP位置を大きく動かさない。
手首orientationはsoft objectiveになり、肘/上腕のためにずれる場合がある。
`--arm-posture hand-only`は従来のfull-pose/position-priority IKへ戻す。
旧replayの開始sampleに上腕boneがなければ、その腕だけhand-onlyへfallbackする。
MuJoCoでは実際の肘をピンクのsite、肘目標を左右色の小球で表示。
この追加後の実mocopi装着による追従、実camera/vSLAMの精度は未確認。

IKは既存Telegripの左右solver、左右URDF joint limits、TCP link7を再利用。
初期姿勢は前方workを既定とし、`--start-pose backwards/config`で変更可能。
body alignmentはworld上の肩の左右ベクトルを水平面へ投影してoperator frameを決める。
肩のない旧入力はrootの前方軸を使用し、world modeは従来の固定変換を直接使用。
operator forward/left/upをrobot -Y/+X/+Zへ写す。Enter時に手の位置/回転offsetとheadingを保存し、
開始後は相対動作だけを送る。再Enterは現在のrobot TCPから再anchorし、IKのorientation authorityをリセットする。
出力関節角はdegree、内部poseはSE(3)、rotationはmatrix/xyzw。
SimulationはMuJoCoを既定にし、`--sim-backend pybullet`で従来表示を選択可能。
MuJoCoはURDFの関節軸・取付位置・limitsを再利用し、assembly_1のvisual/origin/materialと17個のSTLを取り込む。
assemblyの右branchはTelegrip左branch（ds_urdf_v4_1/v4）、assembly左branchはTelegrip右branch（link2/5）に対応。
同じlink名だけで対応付けず、各branchのlinkごとに外観を割り当てる。shared baseは1回だけ表示。
`robot.visual_urdf`で外観元を指定し、package://assembly_1/を実ファイルへ解決する。
ゼロ/微小inertiaは読み込み時のcompiler boundsで補正するが、元URDFファイルは変更しない。
PyBullet DIRECTの既存FK/IK出力をMuJoCo qposへ反映し、mj_forwardのみ実行するkinematic preview。
MuJoCo APIの根拠: [公式Python文書](https://mujoco.readthedocs.io/en/stable/python.html)、
[公式URDF読み込み説明](https://mujoco.readthedocs.io/en/stable/modeling.html#urdf-extensions)。
Ubuntu Wayland上ではDISPLAYがあればGLFWのX11 libraryを既定にし、終了時はviewerの描画thread終了を待つ。
実機腕、力学追従、衝突回避、STS3215/innoMaker追加は未実装。

次の作業はHOW_TO_USEの「最初に接続するもの・起動するもの・Enterまとめ」に従い、
実機mocopi packet、camera checkerboard、外部SLAM導入、15秒Head校正、neutral anchorを確認すること。
