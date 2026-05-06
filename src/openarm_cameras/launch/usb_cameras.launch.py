import json
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


CAMERA_SPECS = [
    ("head", "cam_head", "/cam/head", "cam_head"),
    ("left", "cam_left_wrist", "/cam/left_wrist", "cam_left_wrist"),
    ("right", "cam_right_wrist", "/cam/right_wrist", "cam_right_wrist"),
]

CAMERA_ALIASES = {
    "head": {"head", "cam_head", "front", "front_cam", "camera_head"},
    "left": {"left", "cam_left_wrist", "left_wrist", "camera_left"},
    "right": {"right", "cam_right_wrist", "right_wrist", "camera_right"},
}


def load_camera_config(config_file):
    if not config_file:
        return {}
    path = Path(config_file).expanduser()
    if not path.exists():
        raise RuntimeError(f"Camera config file not found: {path}")
    with path.open("r", encoding="utf-8") as file_obj:
        return json.load(file_obj)


def find_camera_entry(config, slot):
    cameras = config.get("cameras", {}) if isinstance(config, dict) else {}
    if isinstance(cameras, dict):
        return cameras.get(slot, {})
    if not isinstance(cameras, list):
        return {}

    aliases = CAMERA_ALIASES[slot]
    for camera in cameras:
        names = {
            str(camera.get("slot", "")),
            str(camera.get("name", "")),
            str(camera.get("frame_id", "")),
        }
        namespace = str(camera.get("namespace", ""))
        if names & aliases or f"/{slot}" in namespace:
            return camera
    return {}


def device_from_entry(entry):
    for key in ("device_path", "by_id", "by_path", "path"):
        value = entry.get(key, "") if isinstance(entry, dict) else ""
        if value:
            return value
    return ""


def value_from_config(config, entry, key, fallback):
    if isinstance(entry, dict) and key in entry:
        return str(entry[key])
    defaults = config.get("defaults", {}) if isinstance(config, dict) else {}
    if isinstance(defaults, dict) and key in defaults:
        return str(defaults[key])
    return fallback


def launch_setup(context, *args, **kwargs):
    config_file = LaunchConfiguration("config_file").perform(context)
    camera_config = load_camera_config(config_file)
    width = LaunchConfiguration("width").perform(context)
    height = LaunchConfiguration("height").perform(context)
    fps = LaunchConfiguration("fps").perform(context)
    raw_fps = LaunchConfiguration("raw_fps").perform(context)
    compressed_fps = LaunchConfiguration("compressed_fps").perform(context)
    pixel_format = LaunchConfiguration("pixel_format").perform(context)
    raw_reliability = LaunchConfiguration("raw_reliability").perform(context)
    compressed_reliability = LaunchConfiguration("compressed_reliability").perform(context)
    history = LaunchConfiguration("history").perform(context)
    depth = LaunchConfiguration("depth").perform(context)
    jpeg_quality = LaunchConfiguration("jpeg_quality").perform(context)
    auto_exposure = LaunchConfiguration("auto_exposure").perform(context)
    exposure_time = LaunchConfiguration("exposure_time").perform(context)

    nodes = []
    for slot, camera_name, namespace, frame_id in CAMERA_SPECS:
        entry = find_camera_entry(camera_config, slot)
        device_path = LaunchConfiguration(f"{slot}_device").perform(context)
        if not device_path:
            device_path = device_from_entry(entry)
        if not device_path:
            continue

        camera_width = value_from_config(camera_config, entry, "width", width)
        camera_height = value_from_config(camera_config, entry, "height", height)
        camera_fps = value_from_config(camera_config, entry, "fps", fps)
        camera_raw_fps = value_from_config(camera_config, entry, "raw_fps", raw_fps)
        camera_compressed_fps = value_from_config(camera_config, entry, "compressed_fps", compressed_fps)
        camera_pixel_format = value_from_config(camera_config, entry, "pixel_format", pixel_format)
        camera_raw_reliability = value_from_config(camera_config, entry, "raw_reliability", raw_reliability)
        camera_compressed_reliability = value_from_config(camera_config, entry, "compressed_reliability", compressed_reliability)
        camera_history = value_from_config(camera_config, entry, "history", history)
        camera_depth = value_from_config(camera_config, entry, "depth", depth)
        camera_jpeg_quality = value_from_config(camera_config, entry, "jpeg_quality", jpeg_quality)
        camera_auto_exposure = value_from_config(camera_config, entry, "auto_exposure", auto_exposure)
        camera_exposure_time = value_from_config(camera_config, entry, "exposure_time", exposure_time)

        nodes.append(
            Node(
                package="openarm_cameras",
                executable="opencv_camera_pub",
                name=f"{camera_name}_opencv_pub",
                output="screen",
                arguments=[
                    "--camera-name", camera_name,
                    "--device-path", device_path,
                    "--width", camera_width,
                    "--height", camera_height,
                    "--fps", camera_fps,
                    "--frame-id", frame_id,
                    "--namespace", namespace,
                    "--image-topic", "color/image_raw",
                    "--pixel-format", camera_pixel_format,
                    "--raw-reliability", camera_raw_reliability,
                    "--compressed-reliability", camera_compressed_reliability,
                    "--history", camera_history,
                    "--depth", camera_depth,
                    "--raw-fps", camera_raw_fps,
                    "--compressed-fps", camera_compressed_fps,
                    "--jpeg-quality", camera_jpeg_quality,
                    "--auto-exposure", camera_auto_exposure,
                    "--exposure-time", camera_exposure_time,
                ],
            )
        )

    if not nodes:
        raise RuntimeError("At least one of head_device/left_device/right_device must be set.")
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("config_file", default_value="", description="Optional JSON file with stable USB camera device paths"),
        DeclareLaunchArgument("head_device", default_value="", description="Head USB camera device path"),
        DeclareLaunchArgument("left_device", default_value="", description="Left wrist USB camera device path"),
        DeclareLaunchArgument("right_device", default_value="", description="Right wrist USB camera device path"),
        DeclareLaunchArgument("width", default_value="640", description="Camera width"),
        DeclareLaunchArgument("height", default_value="480", description="Camera height"),
        DeclareLaunchArgument("fps", default_value="30", description="Capture fps"),
        DeclareLaunchArgument("raw_fps", default_value="30", description="Raw image publish fps"),
        DeclareLaunchArgument("compressed_fps", default_value="30", description="Compressed image publish fps"),
        DeclareLaunchArgument("pixel_format", default_value="", description="Optional V4L2 pixel format, e.g. MJPG"),
        DeclareLaunchArgument("raw_reliability", default_value="reliable", description="QoS for raw image topic"),
        DeclareLaunchArgument("compressed_reliability", default_value="best_effort", description="QoS for compressed topic"),
        DeclareLaunchArgument("history", default_value="keep_last", description="QoS history"),
        DeclareLaunchArgument("depth", default_value="1", description="QoS queue depth"),
        DeclareLaunchArgument("jpeg_quality", default_value="80", description="JPEG quality for compressed topic"),
        DeclareLaunchArgument("auto_exposure", default_value="-1", description="V4L2 auto exposure value"),
        DeclareLaunchArgument("exposure_time", default_value="-1", description="V4L2 exposure time value"),
        OpaqueFunction(function=launch_setup),
    ])