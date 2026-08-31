"""
Run this inside Isaac Sim's Script Editor.

It creates an Action Graph that:
- publishes the selected robot articulation to ROS 2 topic /joint_states
- subscribes to ROS 2 topic /joint_command
- sends received JointState position commands to IsaacArticulationController

Select the robot articulation root in the Stage first, or set ROBOT_PRIM_PATH
manually below.
"""

ROBOT_PRIM_PATH = ""  # Example: "/World/assembly_1"
GRAPH_PATH = "/ROS2_MoveIt_Bridge"
JOINT_STATE_TOPIC = "joint_states"
JOINT_COMMAND_TOPIC = "joint_command"


import omni
import omni.graph.core as og
import omni.kit.commands
import omni.timeline
import usdrt.Sdf
from isaacsim.core.utils import extensions
from pxr import Usd, UsdPhysics


def find_articulation_roots(stage):
    return [
        str(prim.GetPath())
        for prim in stage.Traverse()
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI)
    ]


def find_root_from_selection(stage, roots):
    selection = omni.usd.get_context().get_selection().get_selected_prim_paths()
    for selected in selection:
        selected = str(selected)
        if selected in roots:
            return selected

        matches = [root for root in roots if root.startswith(selected.rstrip("/") + "/")]
        if len(matches) == 1:
            return matches[0]

        prim = stage.GetPrimAtPath(selected)
        while prim and prim.IsValid():
            path = str(prim.GetPath())
            if path in roots:
                return path
            prim = prim.GetParent()

    return None


def resolve_robot_prim_path():
    stage = omni.usd.get_context().get_stage()
    if stage is None:
        raise RuntimeError("No USD stage is open.")

    roots = find_articulation_roots(stage)
    if ROBOT_PRIM_PATH:
        if ROBOT_PRIM_PATH not in roots:
            raise RuntimeError(
                f"ROBOT_PRIM_PATH is not an articulation root: {ROBOT_PRIM_PATH}\n"
                f"Detected articulation roots: {roots}"
            )
        return ROBOT_PRIM_PATH

    selected = find_root_from_selection(stage, roots)
    if selected:
        return selected

    if len(roots) == 1:
        return roots[0]

    raise RuntimeError(
        "Could not choose a robot articulation root automatically.\n"
        "Select the robot articulation root in the Stage, or edit ROBOT_PRIM_PATH.\n"
        f"Detected articulation roots: {roots}"
    )


def create_graph(robot_prim_path):
    timeline = omni.timeline.get_timeline_interface()
    timeline.stop()

    stage = omni.usd.get_context().get_stage()
    if stage.GetPrimAtPath(GRAPH_PATH).IsValid():
        stage.RemovePrim(GRAPH_PATH)

    keys = og.Controller.Keys
    og.Controller.edit(
        {"graph_path": GRAPH_PATH, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("ReadSystemTime", "isaacsim.core.nodes.IsaacReadSystemTime"),
                ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("PublishJointState", "isaacsim.ros2.bridge.ROS2PublishJointState"),
                ("SubscribeJointState", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
                ("ArticulationController", "isaacsim.core.nodes.IsaacArticulationController"),
            ],
            keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "PublishJointState.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "SubscribeJointState.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "ArticulationController.inputs:execIn"),
                ("Context.outputs:context", "PublishJointState.inputs:context"),
                ("Context.outputs:context", "SubscribeJointState.inputs:context"),
                ("ReadSystemTime.outputs:systemTime", "PublishJointState.inputs:timeStamp"),
                ("SubscribeJointState.outputs:jointNames", "ArticulationController.inputs:jointNames"),
                (
                    "SubscribeJointState.outputs:positionCommand",
                    "ArticulationController.inputs:positionCommand",
                ),
                (
                    "SubscribeJointState.outputs:velocityCommand",
                    "ArticulationController.inputs:velocityCommand",
                ),
                ("SubscribeJointState.outputs:effortCommand", "ArticulationController.inputs:effortCommand"),
            ],
            keys.SET_VALUES: [
                ("ArticulationController.inputs:robotPath", robot_prim_path),
                ("PublishJointState.inputs:topicName", JOINT_STATE_TOPIC),
                ("SubscribeJointState.inputs:topicName", JOINT_COMMAND_TOPIC),
                ("PublishJointState.inputs:targetPrim", [usdrt.Sdf.Path(robot_prim_path)]),
            ],
        },
    )


extensions.enable_extension("isaacsim.ros2.bridge")
robot_path = resolve_robot_prim_path()
create_graph(robot_path)

print("=" * 72)
print("Created Isaac ROS 2 MoveIt bridge graph")
print(f"  Graph:        {GRAPH_PATH}")
print(f"  Robot:        {robot_path}")
print(f"  Publish:      /{JOINT_STATE_TOPIC}")
print(f"  Subscribe:    /{JOINT_COMMAND_TOPIC}")
print("Press Play in Isaac Sim, then check from a ROS 2 terminal:")
print("  ros2 topic info /joint_states -v")
print("  ros2 topic info /joint_command -v")
print("=" * 72)
