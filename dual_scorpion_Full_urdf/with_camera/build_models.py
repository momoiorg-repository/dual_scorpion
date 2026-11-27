"""Generate the simulator entry points from the approved wrist-camera URDF."""

import argparse
import math
import os
import shutil
from copy import deepcopy
from pathlib import Path
from tempfile import TemporaryDirectory
from xml.etree import ElementTree as ET

import numpy as np
import trimesh
from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics, Vt
from scipy.spatial.transform import Rotation

DIRECTORY = Path(__file__).resolve().parent
ROOT = DIRECTORY.parent / "default"
VARIANT = ROOT / "urdf/variants/wrist_cameras"
URDF = VARIANT / "scorpion_wrist_cameras.urdf"
SRDF = VARIANT / "scorpion_wrist_cameras.srdf"
SIDES = ("r", "l")
INERTIA_ATTRS = ("ixx", "iyy", "izz", "ixy", "ixz", "iyz")
ZERO = "0 0 0"
GZ_JOINT_STATES = "/scorpion/joint_states"


def write_xml(tree, path):
    path.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(tree, space="  ")
    tree.write(path, encoding="utf-8", xml_declaration=True)
    with path.open("ab") as stream:
        stream.write(b"\n")


def vector(value):
    return tuple(map(float, value.split()))


def numbers(values):
    return " ".join(f"{value:.12g}" for value in values)


def pose(element):
    origin = element.find("origin")
    if origin is None:
        return ZERO, ZERO
    return origin.get("xyz", ZERO), origin.get("rpy", ZERO)


def mesh_path(mesh):
    return (URDF.parent / mesh.attrib["filename"]).resolve(strict=True)


def relative_asset(path, simulator):
    return Path(os.path.relpath(path, DIRECTORY / simulator)).as_posix()


def rgba(visual):
    return visual.find("material/color").attrib["rgba"]


def named(element, tag, name):
    match = element.find(f"{tag}[@name='{name}']")
    if match is None:
        raise ValueError(f"Missing {tag} {name} in {element.tag}")
    return match


def camera_for(robot, side):
    return robot.find(f"gazebo[@reference='{side}_wrist_camera_link']/sensor")


def write_urdf(robot, out):
    generic = deepcopy(robot)
    for mesh in generic.iter("mesh"):
        mesh.set("filename", relative_asset(mesh_path(mesh), "urdf"))
    write_xml(ET.ElementTree(generic), out / "urdf/scorpion.urdf")
    shutil.copyfile(VARIANT / "scorpion_wrist_cameras_ros.urdf", out / "urdf/scorpion_ros.urdf")
    shutil.copyfile(SRDF, out / "urdf/scorpion.srdf")


def add_mjcf_geometry(link, body, assets):
    inertial = link.find("inertial")
    xyz, rpy = pose(inertial)
    if vector(rpy) != (0, 0, 0):
        raise ValueError(f"Expected a link-aligned inertia tensor in {link.attrib['name']}")
    inertia = inertial.find("inertia")
    body.insert(
        0,
        ET.Element(
            "inertial",
            {
                "pos": xyz,
                "mass": inertial.find("mass").attrib["value"],
                "fullinertia": " ".join(inertia.attrib[key] for key in INERTIA_ATTRS),
            },
        ),
    )
    index = 1 + len(body.findall("joint"))
    for element in (*link.findall("visual"), *link.findall("collision")):
        xyz, rpy = pose(element)
        shape = element.find("geometry")[0]
        attrs = {"name": element.attrib["name"], "pos": xyz, "euler": rpy, "type": shape.tag}
        if shape.tag == "mesh":
            path = mesh_path(shape)
            mesh_name = path.stem
            if assets.find(f"mesh[@name='{mesh_name}']") is None:
                file = relative_asset(path, "mujoco")
                ET.SubElement(assets, "mesh", name=mesh_name, file=file)
            attrs["mesh"] = mesh_name
        elif shape.tag == "box":
            attrs["size"] = numbers(value / 2 for value in vector(shape.attrib["size"]))
        elif shape.tag == "cylinder":
            attrs["size"] = numbers((float(shape.attrib["radius"]), float(shape.attrib["length"]) / 2))
        else:
            raise ValueError(f"Unsupported wrist geometry {shape.tag}")
        if element.tag == "visual":
            attrs.update(group="2", contype="0", conaffinity="0", rgba=rgba(element))
        else:
            attrs.update(group="3", rgba="0.3 0.7 0.4 0.2")
        body.insert(index, ET.Element("geom", attrs))
        index += 1


def write_mjcf(robot, out):
    tree = ET.parse(ROOT / "mujoco/scorpion.xml")
    model = tree.getroot()
    model.set("model", robot.attrib["name"])
    # URDF uses fixed-axis roll, pitch, yaw.
    model.find("compiler").set("eulerseq", "XYZ")
    asset = model.find("asset")
    for mesh in asset.findall("mesh"):
        mesh.set("file", relative_asset(ROOT / "mujoco" / mesh.attrib["file"], "mujoco"))

    for side in SIDES:
        wrist_name = f"{side}_link_7"
        wrist = named(model, ".//body", wrist_name)
        wrist.remove(wrist.find("inertial"))
        for geom in wrist.findall("geom"):
            asset.remove(named(asset, "mesh", geom.attrib["mesh"]))
            wrist.remove(geom)
        add_mjcf_geometry(named(robot, "link", wrist_name), wrist, asset)

        mount = named(robot, "joint", f"{side}_wrist_camera_mount")
        xyz, rpy = pose(mount)
        camera_name = mount.find("child").attrib["link"]
        camera_body = ET.SubElement(wrist, "body", name=camera_name, pos=xyz, euler=rpy)
        add_mjcf_geometry(named(robot, "link", camera_name), camera_body, asset)

        optical = named(robot, "joint", f"{side}_wrist_camera_optical_joint")
        xyz, rpy = pose(optical)
        optical_body = ET.SubElement(
            camera_body, "body", name=optical.find("child").attrib["link"], pos=xyz, euler=rpy
        )
        sensor = camera_for(robot, side)
        width = int(sensor.findtext("camera/image/width"))
        height = int(sensor.findtext("camera/image/height"))
        hfov = float(sensor.findtext("camera/horizontal_fov"))
        vfov = math.degrees(2 * math.atan(math.tan(hfov / 2) * height / width))
        # Convert the ROS optical convention (+Z, +Y down) to the render convention (-Z, +Y up).
        ET.SubElement(
            optical_body,
            "camera",
            name=sensor.attrib["name"],
            quat="0 1 0 0",
            fovy=f"{vfov:.12g}",
            resolution=f"{width} {height}",
        )

    visual = ET.SubElement(model, "visual")
    ET.SubElement(visual, "global", offwidth=str(width), offheight=str(height))
    contacts = model.find("contact")
    contacts.clear()
    for pair in ET.parse(SRDF).findall("disable_collisions"):
        ET.SubElement(contacts, "exclude", body1=pair.attrib["link1"], body2=pair.attrib["link2"])
    write_xml(tree, out / "mujoco/scorpion.xml")


def add_sdf_geometry(source, target):
    inertial = source.find("inertial")
    sdf_inertial = ET.SubElement(target, "inertial")
    ET.SubElement(sdf_inertial, "pose").text = " ".join(pose(inertial))
    ET.SubElement(sdf_inertial, "mass").text = inertial.find("mass").attrib["value"]
    inertia = ET.SubElement(sdf_inertial, "inertia")
    for name in INERTIA_ATTRS:
        ET.SubElement(inertia, name).text = inertial.find("inertia").attrib[name]

    for element in (*source.findall("visual"), *source.findall("collision")):
        node = ET.SubElement(target, element.tag, name=element.attrib["name"])
        ET.SubElement(node, "pose").text = " ".join(pose(element))
        shape = element.find("geometry")[0]
        geometry = ET.SubElement(ET.SubElement(node, "geometry"), shape.tag)
        if shape.tag == "mesh":
            file = mesh_path(shape).relative_to(ROOT).as_posix()
            ET.SubElement(geometry, "uri").text = f"model://{file}"
        else:
            for key, value in shape.attrib.items():
                ET.SubElement(geometry, key).text = value
        if element.tag == "visual":
            material = ET.SubElement(node, "material")
            ET.SubElement(material, "ambient").text = rgba(element)
            ET.SubElement(material, "diffuse").text = rgba(element)
        else:
            friction = ET.SubElement(ET.SubElement(ET.SubElement(node, "surface"), "friction"), "ode")
            ET.SubElement(friction, "mu").text = "0.6"
            ET.SubElement(friction, "mu2").text = "0.6"


def write_sdf(robot, out):
    stock = ROOT / "gazebo_jetty"
    tree = ET.parse(stock / "models/scorpion/model.sdf")
    model = tree.find("model")
    for uri in model.iter("uri"):
        uri.text = f"model://scorpion/{uri.text}"

    for side in SIDES:
        wrist_name = f"{side}_link_7"
        wrist = named(model, "link", wrist_name)
        for tag in ("inertial", "visual", "collision"):
            for element in wrist.findall(tag):
                wrist.remove(element)
        add_sdf_geometry(named(robot, "link", wrist_name), wrist)

        mount = named(robot, "joint", f"{side}_wrist_camera_mount")
        camera_name = mount.find("child").attrib["link"]
        camera = ET.Element("link", name=camera_name)
        model.insert(list(model).index(model.find("joint")), camera)
        ET.SubElement(camera, "pose", relative_to=wrist_name).text = " ".join(pose(mount))
        add_sdf_geometry(named(robot, "link", camera_name), camera)

        fixed = ET.SubElement(model, "joint", name=mount.attrib["name"], type="fixed")
        ET.SubElement(fixed, "parent").text = wrist_name
        ET.SubElement(fixed, "child").text = camera_name

        optical = named(robot, "joint", f"{side}_wrist_camera_optical_joint")
        frame = ET.SubElement(
            model, "frame", name=optical.find("child").attrib["link"], attached_to=camera_name
        )
        ET.SubElement(frame, "pose").text = " ".join(pose(optical))
        sensor = deepcopy(camera_for(robot, side))
        prefix = sensor.findtext("topic").rsplit("/", 1)[0]
        camera_settings = sensor.find("camera")
        optical_id = camera_settings.find("optical_frame_id")
        ET.SubElement(sensor, "frame_id").text = optical_id.text
        camera_settings.remove(optical_id)
        ET.SubElement(camera_settings, "camera_info_topic").text = f"{prefix}/camera_info"
        camera.append(sensor)

    joint_states = ET.SubElement(
        model,
        "plugin",
        filename="gz-sim-joint-state-publisher-system",
        name="gz::sim::systems::JointStatePublisher",
    )
    ET.SubElement(joint_states, "topic").text = GZ_JOINT_STATES
    for joint in model.findall("joint[@type='revolute']"):
        ET.SubElement(joint_states, "joint_name").text = joint.attrib["name"]

    target = out / "gazebo_jetty"
    model_dir = target / "models/scorpion_wrist_cameras"
    write_xml(tree, model_dir / "model.sdf")
    config = ET.parse(stock / "models/scorpion/model.config")
    config.find("name").text = "Scorpion with wrist cameras"
    config.find("description").text = "Dual Scorpion with two InnoMaker wrist cameras and 16 actuated joints."
    write_xml(config, model_dir / "model.config")

    world = ET.parse(stock / "worlds/scorpion.sdf")
    world.find("world/include/uri").text = "model://scorpion_wrist_cameras"
    sensors = ET.SubElement(
        world.find("world"), "plugin", filename="gz-sim-sensors-system", name="gz::sim::systems::Sensors"
    )
    ET.SubElement(sensors, "render_engine").text = "ogre2"
    write_xml(world, target / "worlds/scorpion.sdf")

    lines = []
    for sensor in model.findall("link/sensor[@type='camera']"):
        topics = (
            (sensor.findtext("topic"), "Image"),
            (sensor.findtext("camera/camera_info_topic"), "CameraInfo"),
        )
        for topic, message in topics:
            lines.extend(
                (
                    f'- ros_topic_name: "{topic}"',
                    f'  gz_topic_name: "{topic}"',
                    f'  ros_type_name: "sensor_msgs/msg/{message}"',
                    f'  gz_type_name: "gz.msgs.{message}"',
                    "  direction: GZ_TO_ROS",
                )
            )
    lines.extend(
        (
            '- ros_topic_name: "/joint_states"',
            f'  gz_topic_name: "{GZ_JOINT_STATES}"',
            '  ros_type_name: "sensor_msgs/msg/JointState"',
            '  gz_type_name: "gz.msgs.Model"',
            "  direction: GZ_TO_ROS",
        )
    )
    (target / "ros_gz_bridge.yaml").write_text("\n".join(lines) + "\n", encoding="utf-8")


def usd_rotation(rotation):
    x, y, z, w = rotation.as_quat(canonical=True)
    return Gf.Quatf(w, x, y, z)


def set_usd_pose(schema, xyz, rotation):
    xform = UsdGeom.Xformable(schema.GetPrim())
    xform.AddTranslateOp().Set(Gf.Vec3d(*xyz))
    xform.AddOrientOp().Set(usd_rotation(rotation))


def set_usd_mass(prim, inertial):
    it = inertial.find("inertia")
    xx, yy, zz, xy, xz, yz = (float(it.attrib[name]) for name in INERTIA_ATTRS)
    eigenvalues, axes = np.linalg.eigh(((xx, xy, xz), (xy, yy, yz), (xz, yz, zz)))
    for column in range(2):
        axis = axes[:, column]
        if axis[np.argmax(np.abs(axis))] < 0:
            axis *= -1
    axes[:, 2] = np.cross(axes[:, 0], axes[:, 1])
    xyz, rpy = pose(inertial)
    rotation = Rotation.from_euler("xyz", vector(rpy)) * Rotation.from_matrix(axes)
    mass = UsdPhysics.MassAPI.Apply(prim)
    mass.CreateMassAttr(float(inertial.find("mass").attrib["value"]))
    mass.CreateCenterOfMassAttr(Gf.Vec3f(*vector(xyz)))
    mass.CreateDiagonalInertiaAttr(Gf.Vec3f(*eigenvalues))
    mass.CreatePrincipalAxesAttr(usd_rotation(rotation))


def add_usd_geometry(stage, geometry_stage, body_path, link):
    for element in (*link.findall("visual"), *link.findall("collision")):
        prim_path = body_path.AppendChild(element.attrib["name"])
        xyz, rpy = pose(element)
        shape = element.find("geometry")[0]
        if shape.tag == "mesh":
            file = mesh_path(shape)
            asset_path = f"/WristGeometry/{file.stem}"
            if not geometry_stage.GetPrimAtPath(asset_path):
                data = trimesh.load_mesh(file, process=False)
                data.merge_vertices(digits_vertex=12)
                mesh = UsdGeom.Mesh.Define(geometry_stage, asset_path)
                mesh.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(data.vertices.astype(np.float32)))
                mesh.CreateFaceVertexCountsAttr(
                    Vt.IntArray.FromNumpy(np.full(len(data.faces), 3, dtype=np.int32))
                )
                mesh.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(data.faces.ravel().astype(np.int32)))
                mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
            geom = UsdGeom.Mesh.Define(stage, prim_path)
            geom.GetPrim().GetReferences().AddReference(geometry_stage.GetRootLayer().identifier, asset_path)
        elif shape.tag == "box":
            geom = UsdGeom.Cube.Define(stage, prim_path)
            geom.CreateSizeAttr(1)
        elif shape.tag == "cylinder":
            geom = UsdGeom.Cylinder.Define(stage, prim_path)
            geom.CreateRadiusAttr(float(shape.attrib["radius"]))
            geom.CreateHeightAttr(float(shape.attrib["length"]))
            geom.CreateAxisAttr(UsdGeom.Tokens.z)
        else:
            raise ValueError(f"Unsupported wrist geometry {shape.tag}")
        set_usd_pose(geom, vector(xyz), Rotation.from_euler("xyz", vector(rpy)))
        if shape.tag == "box":
            geom.AddScaleOp().Set(Gf.Vec3f(*vector(shape.attrib["size"])))
        if element.tag == "visual":
            red, green, blue, alpha = vector(rgba(element))
            geom.CreateDisplayColorAttr([Gf.Vec3f(red, green, blue)])
            geom.CreateDisplayOpacityAttr([alpha])
        else:
            geom.CreatePurposeAttr(UsdGeom.Tokens.guide)
            geom.MakeInvisible()
            UsdPhysics.CollisionAPI.Apply(geom.GetPrim())
            if shape.tag == "mesh":
                UsdPhysics.MeshCollisionAPI.Apply(geom.GetPrim()).CreateApproximationAttr("convexHull")


def usd_camera_pose(wrist_xyz, wrist_orientation, mount_xyz, mount_rotation):
    axis = wrist_orientation.GetImaginary()
    wrist_rotation = Rotation.from_quat((*axis, wrist_orientation.GetReal()))
    position = np.asarray(wrist_xyz) + wrist_rotation.apply(mount_xyz)
    return position, wrist_rotation * mount_rotation


def write_usd(robot, out):
    target = out / "isaac_sim"
    target.mkdir(parents=True, exist_ok=True)
    stock = ROOT / "isaac_sim/scorpion.usd"
    layer = Sdf.Layer.CreateAnonymous("scorpion.usda")
    layer.subLayerPaths = [str(stock)]
    stage = Usd.Stage.Open(layer)
    stage.SetDefaultPrim(stage.GetPrimAtPath("/Scorpion"))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1)
    UsdPhysics.SetStageKilogramsPerUnit(stage, 1)

    geometry_stage = Usd.Stage.CreateInMemory()
    UsdGeom.Scope.Define(geometry_stage, "/WristGeometry")
    captured = Sdf.Layer.CreateAnonymous("scorpion_current_pose.usda")
    captured.TransferContent(Sdf.Layer.FindOrOpen(str(stock.with_name("scorpion_current_pose.usda"))))

    for side in SIDES:
        wrist_name = f"{side}_link_7"
        wrist_path = Sdf.Path(f"/Scorpion/{wrist_name}")
        wrist = stage.GetPrimAtPath(wrist_path)
        for child in wrist.GetChildren():
            if child.IsA(UsdGeom.Mesh):
                child.SetActive(False)
        link = named(robot, "link", wrist_name)
        set_usd_mass(wrist, link.find("inertial"))
        add_usd_geometry(stage, geometry_stage, wrist_path, link)

        mount = named(robot, "joint", f"{side}_wrist_camera_mount")
        xyz, rpy = pose(mount)
        mount_xyz = vector(xyz)
        mount_rotation = Rotation.from_euler("xyz", vector(rpy))
        camera_name = mount.find("child").attrib["link"]
        camera_path = Sdf.Path(f"/Scorpion/{camera_name}")
        camera_body = UsdGeom.Xform.Define(stage, camera_path)
        position, rotation = usd_camera_pose(
            wrist.GetAttribute("xformOp:translate").Get(),
            wrist.GetAttribute("xformOp:orient").Get(),
            mount_xyz,
            mount_rotation,
        )
        set_usd_pose(camera_body, position, rotation)

        captured_link = Sdf.CreatePrimInLayer(captured, camera_path)
        captured_xyz = captured.GetAttributeAtPath(wrist_path.AppendProperty("xformOp:translate")).default
        captured_orientation = captured.GetAttributeAtPath(
            wrist_path.AppendProperty("xformOp:orient")
        ).default
        position, rotation = usd_camera_pose(captured_xyz, captured_orientation, mount_xyz, mount_rotation)
        Sdf.AttributeSpec(captured_link, "xformOp:translate", Sdf.ValueTypeNames.Double3).default = Gf.Vec3d(
            *position
        )
        Sdf.AttributeSpec(captured_link, "xformOp:orient", Sdf.ValueTypeNames.Quatf).default = usd_rotation(
            rotation
        )

        UsdPhysics.RigidBodyAPI.Apply(camera_body.GetPrim()).CreateRigidBodyEnabledAttr(True)
        link = named(robot, "link", camera_name)
        set_usd_mass(camera_body.GetPrim(), link.find("inertial"))
        add_usd_geometry(stage, geometry_stage, camera_path, link)
        UsdPhysics.FilteredPairsAPI.Apply(wrist).GetFilteredPairsRel().AddTarget(camera_path)

        fixed = UsdPhysics.FixedJoint.Define(stage, f"/Scorpion/joints/{mount.attrib['name']}")
        fixed.CreateBody0Rel().SetTargets([wrist_path])
        fixed.CreateBody1Rel().SetTargets([camera_path])
        fixed.CreateLocalPos0Attr(Gf.Vec3f(*mount_xyz))
        fixed.CreateLocalRot0Attr(usd_rotation(mount_rotation))
        fixed.CreateLocalPos1Attr(Gf.Vec3f(0))
        fixed.CreateLocalRot1Attr(Gf.Quatf(1))

        optical = named(robot, "joint", f"{side}_wrist_camera_optical_joint")
        optical_path = camera_path.AppendChild(optical.find("child").attrib["link"])
        optical_frame = UsdGeom.Xform.Define(stage, optical_path)
        xyz, rpy = pose(optical)
        set_usd_pose(optical_frame, vector(xyz), Rotation.from_euler("xyz", vector(rpy)))
        sensor = camera_for(robot, side)
        camera = UsdGeom.Camera.Define(stage, optical_path.AppendChild(sensor.attrib["name"]))
        camera.AddOrientOp().Set(Gf.Quatf(0, 1, 0, 0))
        width = int(sensor.findtext("camera/image/width"))
        height = int(sensor.findtext("camera/image/height"))
        hfov = float(sensor.findtext("camera/horizontal_fov"))
        horizontal_aperture = 20.955
        camera.CreateHorizontalApertureAttr(horizontal_aperture)
        camera.CreateVerticalApertureAttr(horizontal_aperture * height / width)
        camera.CreateFocalLengthAttr(horizontal_aperture / (2 * math.tan(hfov / 2)))
        clip = (float(sensor.findtext(f"camera/clip/{limit}")) for limit in ("near", "far"))
        camera.CreateClippingRangeAttr(Gf.Vec2f(*clip))
        camera.GetPrim().SetCustomDataByKey("resolution", Gf.Vec2i(width, height))

    # The preview lighting stays outside the robot's default prim when it is referenced into another stage.
    light = UsdLux.DistantLight.Define(stage, "/CameraPreviewSun")
    light.CreateIntensityAttr(700)
    light.AddRotateXYZOp().Set(Gf.Vec3f(25, -30, 0))
    UsdLux.DomeLight.Define(stage, "/CameraPreviewFill").CreateIntensityAttr(150)

    geometry = geometry_stage.GetRootLayer()
    if not geometry.Export(str(target / "wrist_geometry.usdc")):
        raise RuntimeError("Could not export USD wrist geometry")
    portable = Sdf.Layer.CreateAnonymous("scorpion.usda")
    portable.TransferContent(layer)
    portable.subLayerPaths = [relative_asset(stock, "isaac_sim")]
    portable.UpdateCompositionAssetDependency(geometry.identifier, "wrist_geometry.usdc")
    (target / "scorpion.usda").write_text(portable.ExportToString().rstrip() + "\n", encoding="utf-8")
    (target / "scorpion_current_pose.usda").write_text(
        captured.ExportToString().rstrip() + "\n", encoding="utf-8"
    )
    entry = Sdf.Layer.CreateAnonymous("scorpion.usd")
    entry.ImportFromString("""#usda 1.0
(
    defaultPrim = "Scorpion"
    kilogramsPerUnit = 1
    metersPerUnit = 1
    subLayers = [@scorpion.usda@]
    upAxis = "Z"
)
""")
    if not entry.Export(str(target / "scorpion.usd"), args={"format": "usdc"}):
        raise RuntimeError("Could not export the binary USD entry stage")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true", help="Fail if a generated model is missing or stale")
    args = parser.parse_args()

    robot = ET.parse(URDF).getroot()
    with TemporaryDirectory(prefix="scorpion_camera_exports_") as temp:
        output = Path(temp)
        write_urdf(robot, output)
        write_mjcf(robot, output)
        write_sdf(robot, output)
        write_usd(robot, output)
        stale = []
        for source in sorted(path for path in output.rglob("*") if path.is_file()):
            relative = source.relative_to(output)
            destination = DIRECTORY / relative
            if destination.is_file() and source.read_bytes() == destination.read_bytes():
                continue
            if args.check:
                stale.append(str(relative))
            else:
                destination.parent.mkdir(parents=True, exist_ok=True)
                shutil.copyfile(source, destination)
        if stale:
            raise SystemExit("Regenerate stale models:\n" + "\n".join(stale))


if __name__ == "__main__":
    main()
