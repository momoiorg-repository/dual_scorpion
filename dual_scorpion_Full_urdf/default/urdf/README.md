# URDF / ROS 2

`scorpion.urdf` は相対メッシュパス、`scorpion_ros.urdf` は `package://scorpion_description/` を使います。`scorpion.srdf` は隣接リンクの衝突除外を示します。使用するツールに応じて自己衝突とSRDFを設定してください。

ROS 2 ではこのフォルダーを `scorpion_description` パッケージとしてワークスペースの `src` に置き、ビルド後、`ros2 launch scorpion_description display.launch.py` で表示できます。`joint_state_publisher_gui` と RViz 2 が必要です。
