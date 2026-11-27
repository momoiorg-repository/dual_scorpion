"""Generate the ROS 2 and MuJoCo entry points from the shared camera URDF."""

import argparse
from copy import deepcopy
from pathlib import Path
from xml.etree import ElementTree as ET

DIRECTORY = Path(__file__).resolve().parent
SOURCE = DIRECTORY / "scorpion_wrist_cameras.urdf"
PACKAGE = DIRECTORY.parents[1]


def xml_bytes(robot: ET.Element) -> bytes:
    ET.indent(robot, space="  ")
    return ET.tostring(robot, encoding="utf-8", xml_declaration=True) + b"\n"


def variants() -> dict[str, bytes]:
    parser = ET.XMLParser(target=ET.TreeBuilder(insert_comments=True))
    source = ET.parse(SOURCE, parser=parser).getroot()

    ros = deepcopy(source)
    for mesh in ros.findall(".//mesh"):
        asset = (DIRECTORY / mesh.attrib["filename"]).resolve()
        mesh.set("filename", f"package://scorpion_description/{asset.relative_to(PACKAGE).as_posix()}")

    mujoco = deepcopy(source)
    base = mujoco.find("link[@name='base_link']/visual/geometry/mesh")
    if base is None:
        raise ValueError("The shared URDF must define the base visual mesh")
    # The existing OBJ preserves the base STL triangulation above MuJoCo's STL face limit.
    base.set("filename", "../../../mujoco/meshes/visual/base_link.obj")
    compiler = ET.SubElement(ET.SubElement(mujoco, "mujoco"), "compiler")
    compiler.attrib.update(strippath="false", fusestatic="false", discardvisual="false")

    srdf = ET.Element("robot", name=source.attrib["name"])
    collidable = {link.attrib["name"] for link in source.findall("link") if link.findall("collision")}
    for joint in source.findall("joint"):
        parent = joint.find("parent").attrib["link"]
        child = joint.find("child").attrib["link"]
        if {parent, child} <= collidable:
            ET.SubElement(srdf, "disable_collisions", link1=parent, link2=child, reason="Adjacent")

    return {
        "scorpion_wrist_cameras_ros.urdf": xml_bytes(ros),
        "scorpion_wrist_cameras_mujoco.urdf": xml_bytes(mujoco),
        "scorpion_wrist_cameras.srdf": xml_bytes(srdf),
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true", help="verify generated files without writing them")
    args = parser.parse_args()

    stale = []
    for filename, expected in variants().items():
        path = DIRECTORY / filename
        if args.check:
            if not path.is_file() or path.read_bytes() != expected:
                stale.append(filename)
        else:
            path.write_bytes(expected)
    if stale:
        parser.exit(1, f"Regenerate these files with export_variants.py: {', '.join(stale)}\n")


if __name__ == "__main__":
    main()
