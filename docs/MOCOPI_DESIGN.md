# mocopi + Head Camera MVP 設計メモ（2026-10-05）

実装前調査: 本体は LeRobot。`src/lerobot/model/kinematics.py` は汎用placo、
`telegrip_teleoperation/telegrip/core/kinematics.py` は本機専用PyBullet FK/IK。
左右Scorpion URDF、7軸+gripper、TCP link7、左右別joint limitとPyBulletVisualizerが
後者に揃っているため、この独立add-onへ小さいmodule群を追加する。
IKの入力はURDF共通stand/worldでの位置(m)、xyzw quaternion、現在関節角(deg)。
URDFが左右mount offsetを含むため、左右bodyを別base位置へ重複offsetしない。
既存IKは位置優先、workspace圧縮、orientation slew、関節slew、FK検証を行う。
新bridgeはNaN、error、未到達residual、limit違反を検出してholdする。

構成: Sony UDP -> skeleton FK -> head-relative hands;
C270 -> ROS2 Image -> 外部stella_vslam_ros -> Odometry -> monotonic同期;
trajectory calibration -> metric world Head -> world hands -> relative retarget -> 既存IK -> 既存PyBullet。
coreはROS非依存。ROS2は独立processのadapterで、local UDP JSONへcamera poseを渡す。
ROS2がない環境ではfake/replay、または明示的mocopi-only fallbackを使う。
ハードウェア腕driverはimportしない。--hardwareは未実装として拒否する。

Sonyの2026公式ページと公式ソースのみをprotocol根拠にする。
Blender `models.py` commit cef13df82e0cc72d747c0b994506682848a0b60a:
LE uint32 payload-size + 4-byte ASCII chunk、tran=7 float32 (xyzw,xyz)。
Unity `MocopiAvatar.cs` commit f498a5ac97b569aad64b7d598a8880113878425e:
plugin座標 -> Unity: position=(-x,y,z), quaternion=(-x,y,z,-w)。
Unityのleft-handed Y-up Z-forwardから、wireはright-handed X-left Y-up Z-forward。
内部はright-handed X-forward Y-left Z-up、basis B=[[0,0,1],[1,0,0],[0,1,0]]。
Unityは骨階層にlocalRotationを適用、非root位置はskeleton definitionのoffsetを保持。
3ds Max公式 `external/MocopiDataHandler.cpp` のparentIndex表も照合する。
rest/skeleton到着前・不完全frameは追従禁止。推測した身長やbone lengthは使わない。

単眼尺度とextrinsicは同時推定:
R_M_H(i)=R_M_S R_S_C(i) R_C_H,
p_M_H(i)=s R_M_S p_S_C(i)+t_M_S+R_M_S R_S_C(i) t_C_H。
Umeyamaを初期値としたrobust least-squares（log scale, SO(3) rotvec）で13変数をfit。
最後に頭位置/姿勢残差とJacobian rank/conditionを検査し、静止・純回転・単軸運動等の
観測不能な校正は保存しない。mocopi位置のmetric性はbodyモデルに依存し、測量精度は保証しない。
W=M（内部basis）。mapの再初期化後は校正を再実施、同じmapの再利用時は開始整合性を検査。

SLAM選定: stella_vslam>=0.3、BSD-2、Linux、ROS2 wrapper、perspective model、map保存。
ORB-SLAM3は高精度なvisual/inertial/multimap実装だがGPLv3、本C270にはIMUがなく
inertial尺度は使えず、公式examplesはROS1。双方とも低texture/blur/純回転初期化は苦手。
選定はROS2接続とintrinsics生成の容易さも考慮。SLAMコード/語彙はvendorしない。
ROS2 wrapper commit 186f623dd1c24ee83678f8e1da593801f450bbb3 の
`~/camera_pose` はnav_msgs/Odometry、mapとcamera両方がROS basis。
callbackはpose pointer取得時にpublishするが、coreのtracking_moduleはLost時も
予測poseを返す場合がある。外部wrapperのpublish_poseにframe_publisherの
get_tracking_state()=="Tracking" guardを追加する準備ツールを提供し、
lostはそのguardによるpose publish停止とcore timeoutで検出する。

一次情報:
- https://xyn.sony.net/en/developer/mocopi
- https://xyn.sony.net/en/developer/technical/mocopi/techspec (2026-07-09)
- https://xyn.sony.net/en/developer/technical/mocopi/senddata (2026-07-14)
- https://www.sony.co.jp/en/Products/mocopi-dev/en/downloads/DownloadInfo.html
- https://github.com/sony/mocopi-receiver-plugin-blender
- https://github.com/sony/mocopi-receiver-plugin-unity
- https://github.com/sony/mocopi-receiver-plugin-3dsmax
- https://github.com/stella-cv/stella_vslam
- https://github.com/stella-cv/stella_vslam_ros/tree/ros2
- https://github.com/UZ-SLAMLab/ORB_SLAM3

3ds Max hierarchy照合commit: `fb2bf5867f4737efd91d498b15dcb323a9e95977`。
実装結果・ファイル一覧・検証境界は [MOCOPI_IMPLEMENTATION.md](MOCOPI_IMPLEMENTATION.md)。

上半身集中の追加運用: Sony公式mobile beta guide
https://www.sony.co.jp/en/Products/mocopi-dev/jp/documents/beta/HowToBetaFunctions_UpBody.html
でANKLEをUPPER ARMへ移すmodeを確認。推定はスマホ側に任せ、wireのleg boneを
armへ再割当しない。既存FKはupper/lower arm rotationをHandへ伝搬するため流用。
上腕origin=12/16、肘（lower-arm origin）=13/17をHead-relative worldへ変換してdebugに公開。
宣言modeは自動検出/アプリ操作を意味せず、実UDPは未検証。
robot EE目標以外に肘constraintを入れる機能は今回追加しない。
