# assembly_1

`assembly_1` は、既存の `urdf/assembly_1.urdf` と `meshes/` をROS 2 Jazzyで読み込み、RViz2上でURDF、TF、joint_state_publisher_guiの動作を確認するための `ament_cmake` パッケージです。

既存の `launch/assembly_1.launch` はROS 1形式の可能性があるため残しています。ROS 2での表示確認には `launch/display.launch.py` を使います。

## 1. 前提

- Ubuntu 24.04
- ROS2 Jazzyはインストール済み
- Isaac SimではURDF表示とjoint control確認済み
- ROS2側ではTFとMoveIt2 IKを検証する
- Gazebo、MuJoCo、PyBullet、Isaac Sim連携は今回は対象外
- ros2_controlや実機制御は今回は対象外
- MoveIt2はFake Controllerで検証する

## 2. 環境確認

```bash
source /opt/ros/jazzy/setup.bash
ros2 --version
```

この環境で `ros2 --version` が `unrecognized arguments` になる場合は、`ros2 --help` でROS 2 CLIが起動できることを確認してください。

必要パッケージ確認:

```bash
ros2 pkg list | grep robot_state_publisher
ros2 pkg list | grep joint_state_publisher_gui
ros2 pkg list | grep rviz2
ros2 pkg list | grep moveit
ros2 pkg list | grep tf2_tools
```

不足がある場合だけ、以下を実行します。

```bash
sudo apt update
sudo apt install \
  ros-jazzy-rviz2 \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-joint-state-publisher-gui \
  ros-jazzy-tf2-tools \
  ros-jazzy-xacro \
  ros-jazzy-moveit \
  ros-jazzy-moveit-setup-assistant
```

## 3. ビルド方法

このフォルダは以下の位置にある前提です。

```text
~/DS_URDF_IK/assembly_1
```

ビルドは親フォルダから行います。

```bash
cd ~/DS_URDF_IK
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 4. URDF / TF / RViz確認

```bash
ros2 launch assembly_1 display.launch.py
```

別ターミナル:

```bash
cd ~/DS_URDF_IK
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run tf2_ros tf2_echo base_link left_gripper_link
ros2 run tf2_ros tf2_echo base_link right_gripper_link
ros2 run tf2_tools view_frames
```

または、このパッケージ直下で以下を実行します。

```bash
./scripts/check_tf.sh left
./scripts/check_tf.sh right
./scripts/check_tf.sh frames
```

RViz2のFixed Frameはまず `base_link` にしています。このURDFには最上位linkとして `root` があり、`root` から `base_link` へfixed jointで接続されています。もし表示やTFが不安定な場合は、RViz2のFixed Frameを `root` に変えてください。

## 5. joint確認

`joint_state_publisher_gui` でrevolute系jointを動かし、RViz2上でリンクとTFが動くか確認します。Isaac Simではすでにjoint control確認済みなので、ROS2側ではTFが正しく出るかを中心に見ます。

## 6. MoveIt2 Setup Assistant手順

起動:

```bash
source /opt/ros/jazzy/setup.bash
source ~/DS_URDF_IK/install/setup.bash
ros2 launch moveit_setup_assistant setup_assistant.launch.py
```

URDF読み込み:

```text
~/DS_URDF_IK/assembly_1/urdf/assembly_1.urdf
```

Virtual Joint:

- Joint Name: `virtual_joint`
- Parent Frame: `world`
- Child Link: `root`
- Type: `fixed`

もし `root` linkがない場合:

- Child Link: `base_link`

Planning Group 1:

- Group Name: `left_arm`
- Kinematic Solver: `kdl_kinematics_plugin/KDLKinematicsPlugin`
- Add Kin. Chain
- Base Link: `base_link`
- Tip Link: `left_gripper_link`

Planning Group 2:

- Group Name: `right_arm`
- Kinematic Solver: `kdl_kinematics_plugin/KDLKinematicsPlugin`
- Add Kin. Chain
- Base Link: `base_link`
- Tip Link: `right_gripper_link`

End Effectors:

- 最初は設定しなくてよい
- IK検証だけならplanning groupのtip linkで十分

Controllers:

- Fake Controllersを使う
- 実機制御やros2_controlは今回は不要
- Fake Controllerは実機モーターへ送らず、MoveIt2/RViz上でPlan/Executeのアニメーションを見るためのもの
- IK計算とmotion planning自体は実際に行われる

## STL参照

URDF内のmesh filenameは `package://assembly_1/meshes/*.stl` 形式です。現在参照されているSTLはすべて `meshes/` に存在します。
