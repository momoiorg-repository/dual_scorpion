"""Minimal, explicit safety patch to an EXTERNAL official stella ROS2 checkout.

stella tracking_module may return a predicted valid pose even when track() fails.
The unmodified official ROS wrapper checks only the returned pointer. Gate its
publisher on the actual frame_publisher tracking state, so core timeout means lost.
No SLAM code or dependencies are copied into this repository.
"""

import argparse
from pathlib import Path

FUNCTION = "void system::publish_pose(const Eigen::Matrix4d& cam_pose_wc, const rclcpp::Time& stamp) {"
HEADER = "#include <stella_vslam/publish/frame_publisher.h>"
MARKER = "// Dual Scorpion: publish only successfully tracked camera poses."
GUARD = (
    "\n    "
    + MARKER
    + '\n    if (slam_->get_frame_publisher()->get_tracking_state() != "Tracking") {\n        return;\n    }\n'
)


def prepare(checkout):
    path = Path(checkout) / "src/stella_vslam_ros.cc"
    original = path.read_text()
    if MARKER in original:
        if FUNCTION + GUARD not in original or HEADER not in original:
            raise ValueError("Existing tracking guard was changed; inspect external checkout")
        return False
    if original.count(FUNCTION) != 1 or "#include <stella_vslam/publish/map_publisher.h>" not in original:
        raise ValueError("Unsupported stella wrapper source; use the documented pinned ROS2 commit")
    updated = original.replace(
        "#include <stella_vslam/publish/map_publisher.h>",
        "#include <stella_vslam/publish/map_publisher.h>\n" + HEADER,
        1,
    )
    updated = updated.replace(FUNCTION, FUNCTION + GUARD, 1)
    path.write_text(updated)
    return True


def main():
    parser = argparse.ArgumentParser(description="Gate external stella ROS2 poses on Tracking state")
    parser.add_argument("checkout", type=Path)
    args = parser.parse_args()
    try:
        changed = prepare(args.checkout)
        print(
            "Tracking guard added. Rebuild stella_vslam_ros."
            if changed
            else "Tracking guard already present."
        )
    except (OSError, ValueError) as exc:
        parser.exit(2, str(exc) + "\n")


if __name__ == "__main__":
    main()
