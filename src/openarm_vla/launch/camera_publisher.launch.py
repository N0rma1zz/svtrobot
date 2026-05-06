#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


D405_SUPPORTED_PROFILES = {
    (1280, 720): [5, 15, 30],
    (848, 480): [5, 15, 30, 60, 90],
    (640, 480): [5, 15, 30, 60, 90],
    (640, 360): [5, 15, 30, 60, 90],
    (480, 270): [5, 15, 30, 60, 90],
    (424, 240): [5, 15, 30, 60, 90],
}

D435_SUPPORTED_PROFILES = {
    (1920, 1080): [6, 15, 30],
    (1280, 720): [6, 15, 30],
    (848, 480): [6, 15, 30, 60, 90],
    (640, 480): [6, 15, 30, 60, 90],
    (640, 360): [6, 15, 30, 60, 90],
    (480, 270): [6, 15, 30, 60, 90],
    (424, 240): [6, 15, 30, 60, 90],
}

UNSET = "unset"
SUPPORTED_CAMERA_TYPES = {"D405", "D435", "D435I"}
TRUTHY_VALUES = {"1", "true", "yes", "on"}
FALSY_VALUES = {"0", "false", "no", "off"}
CAMERA_CONTROL_ARGUMENTS = {
    "color_auto_exposure": {
        "type": "bool",
        "param": "rgb_camera.enable_auto_exposure",
        "description": "颜色自动曝光，true/false/unset",
    },
    "color_exposure": {
        "type": "int",
        "param": "rgb_camera.exposure",
        "min": 1,
        "max": 10000,
        "description": "颜色手动曝光，范围 1..10000，unset 表示不设置",
    },
    "color_gain": {
        "type": "int",
        "param": "rgb_camera.gain",
        "min": 0,
        "max": 128,
        "description": "颜色手动增益，范围 0..128，unset 表示不设置",
    },
    "color_auto_white_balance": {
        "type": "bool",
        "param": "rgb_camera.enable_auto_white_balance",
        "description": "颜色自动白平衡，true/false/unset",
    },
    "color_white_balance": {
        "type": "int",
        "param": "rgb_camera.white_balance",
        "min": 2800,
        "max": 6500,
        "description": "颜色手动白平衡，范围 2800..6500，unset 表示不设置",
    },
    "color_brightness": {
        "type": "int",
        "param": "rgb_camera.brightness",
        "min": -64,
        "max": 64,
        "description": "颜色亮度，范围 -64..64，unset 表示不设置",
    },
    "color_contrast": {
        "type": "int",
        "param": "rgb_camera.contrast",
        "min": 0,
        "max": 100,
        "description": "颜色对比度，范围 0..100，unset 表示不设置",
    },
    "color_saturation": {
        "type": "int",
        "param": "rgb_camera.saturation",
        "min": 0,
        "max": 100,
        "description": "颜色饱和度，范围 0..100，unset 表示不设置",
    },
    "color_sharpness": {
        "type": "int",
        "param": "rgb_camera.sharpness",
        "min": 0,
        "max": 100,
        "description": "颜色锐度，范围 0..100，unset 表示不设置",
    },
}

TOPIC_SLOT = {
    "cam_left": "left_wrist",
    "cam_right": "right_wrist",
    "cam_head": "head",
}


def normalize_camera_type(cam_type: str) -> str:
    return cam_type.strip().upper()


def parse_optional_bool(raw_value: str, arg_name: str) -> bool | None:
    value = raw_value.strip().lower()
    if value in ("", UNSET):
        return None
    if value in TRUTHY_VALUES:
        return True
    if value in FALSY_VALUES:
        return False
    raise RuntimeError(f"参数 {arg_name} 必须是 true/false/{UNSET}，当前值: {raw_value}")


def parse_optional_int(raw_value: str, arg_name: str) -> int | None:
    value = raw_value.strip().lower()
    if value in ("", UNSET):
        return None
    try:
        return int(raw_value)
    except ValueError as exc:
        raise RuntimeError(f"参数 {arg_name} 必须是整数或 {UNSET}，当前值: {raw_value}") from exc


def validate_integer_range(arg_name: str, value: int | None, min_value: int, max_value: int) -> None:
    if value is None:
        return
    if not (min_value <= value <= max_value):
        raise RuntimeError(f"参数 {arg_name} 超出范围: {value}，允许范围: {min_value}..{max_value}")


def parse_camera_controls(context, camera_name: str, cam_type: str) -> tuple[dict[str, object], list[str]]:
    cam_type_upper = normalize_camera_type(cam_type)
    if cam_type_upper not in SUPPORTED_CAMERA_TYPES:
        raise RuntimeError(f"{camera_name} 不支持的相机类型: {cam_type}")

    if cam_type_upper == "D405":
        param_prefix = "depth_module"
        src_prefix = "rgb_camera"
    else:
        param_prefix = "rgb_camera"
        src_prefix = "rgb_camera"

    controls: dict[str, object] = {}
    applied_logs: list[str] = []
    for option_name, spec in CAMERA_CONTROL_ARGUMENTS.items():
        arg_name = f"{camera_name}_{option_name}"
        raw_value = LaunchConfiguration(arg_name).perform(context)
        if spec["type"] == "bool":
            parsed_value = parse_optional_bool(raw_value, arg_name)
        else:
            parsed_value = parse_optional_int(raw_value, arg_name)
            validate_integer_range(arg_name, parsed_value, spec["min"], spec["max"])

        if parsed_value is not None:
            param_name = spec["param"].replace(src_prefix, param_prefix)
            controls[param_name] = parsed_value
            applied_logs.append(f"{option_name}={parsed_value}")

    color_auto_exposure = controls.get(f"{param_prefix}.enable_auto_exposure")
    color_exposure = controls.get(f"{param_prefix}.exposure")
    color_gain = controls.get(f"{param_prefix}.gain")
    color_auto_white_balance = controls.get(f"{param_prefix}.enable_auto_white_balance")
    color_white_balance = controls.get(f"{param_prefix}.white_balance")

    if color_auto_exposure is True:
        if color_exposure is not None:
            controls.pop(f"{param_prefix}.exposure", None)
            applied_logs.append("color_exposure=ignored(auto)")
        if color_gain is not None:
            controls.pop(f"{param_prefix}.gain", None)
            applied_logs.append("color_gain=ignored(auto)")
    if color_auto_white_balance is True and color_white_balance is not None:
        raise RuntimeError(
            f"{camera_name} 同时设置了颜色自动白平衡=true 与手动白平衡，请改为 false 或移除手动值。"
        )

    if color_auto_exposure is None and (color_exposure is not None or color_gain is not None):
        controls[f"{param_prefix}.enable_auto_exposure"] = False
        applied_logs.append("color_auto_exposure=False(auto)")
    if color_auto_white_balance is None and color_white_balance is not None:
        controls[f"{param_prefix}.enable_auto_white_balance"] = False
        applied_logs.append("color_auto_white_balance=False(auto)")

    return controls, applied_logs


def create_camera_control_arguments() -> list[DeclareLaunchArgument]:
    arguments: list[DeclareLaunchArgument] = []
    for camera_name in ("cam_left", "cam_right", "cam_head"):
        for option_name, spec in CAMERA_CONTROL_ARGUMENTS.items():
            arguments.append(
                DeclareLaunchArgument(
                    f"{camera_name}_{option_name}",
                    default_value=UNSET,
                    description=f"{camera_name} {spec['description']}",
                )
            )
    return arguments


def validate_profile(cam_type: str, width: int, height: int, fps: int) -> tuple[bool, str]:
    cam_type_upper = cam_type.upper()
    if cam_type_upper == "D405":
        profiles = D405_SUPPORTED_PROFILES
    elif cam_type_upper in ("D435", "D435I"):
        profiles = D435_SUPPORTED_PROFILES
    else:
        return False, f"不支持的相机类型: {cam_type}，仅支持 D405、D435 和 D435I"

    resolution = (width, height)
    if resolution not in profiles:
        supported_res = ", ".join([f"{w}x{h}" for w, h in profiles.keys()])
        return False, f"{cam_type_upper} 不支持分辨率 {width}x{height}\n支持的分辨率: {supported_res}"
    supported_fps = profiles[resolution]
    if fps not in supported_fps:
        return False, f"{cam_type_upper} 在分辨率 {width}x{height} 下不支持帧率 {fps}\n支持的帧率: {supported_fps}"
    return True, f"{cam_type_upper} 配置有效: {width}x{height}@{fps}fps"


def create_camera_node(context, name: str, serial: str, cam_type: str, profile: str, extra_camera_params=None):
    serial_value = serial.perform(context) if hasattr(serial, "perform") else serial
    cam_type_value = cam_type.perform(context) if hasattr(cam_type, "perform") else cam_type
    profile_value = profile.perform(context) if hasattr(profile, "perform") else profile
    serial_str = f"_{serial_value}"

    if normalize_camera_type(cam_type_value) == "D405":
        camera_params = {
            "serial_no": serial_str,
            "depth_module.color_profile": profile_value,
            "depth_module.depth_profile": profile_value,
            "enable_color": True,
            "enable_depth": True,
            "align_depth.enable": False,
            "enable_infra1": False,
            "enable_infra2": False,
            "enable_gyro": False,
            "enable_accel": False,
            "pointcloud.enable": False,
        }
        original_color_topic = f"/{name}/{name}/color/image_rect_raw"
        original_depth_topic = f"/{name}/{name}/depth/image_rect_raw"
    else:
        camera_params = {
            "serial_no": serial_str,
            "rgb_camera.color_profile": profile_value,
            "depth_module.depth_profile": profile_value,
            "enable_color": True,
            "enable_depth": True,
            "align_depth.enable": True,
            "enable_infra1": False,
            "enable_infra2": False,
            "enable_gyro": False,
            "enable_accel": False,
            "pointcloud.enable": False,
        }
        original_color_topic = f"/{name}/{name}/color/image_raw"
        original_depth_topic = f"/{name}/{name}/aligned_depth_to_color/image_raw"

    if extra_camera_params:
        camera_params.update(extra_camera_params)

    slot = TOPIC_SLOT[name]
    unified_color_topic = f"/cam/{slot}/color/image_raw"
    unified_depth_topic = f"/cam/{slot}/depth/image_raw"
    remappings = [
        (original_color_topic, unified_color_topic),
        (original_depth_topic, unified_depth_topic),
    ]

    return Node(
        package="realsense2_camera",
        executable="realsense2_camera_node",
        name=name,
        namespace=name,
        parameters=[camera_params],
        remappings=remappings,
        output="screen",
    )


def launch_setup(context, *args, **kwargs):
    width = int(LaunchConfiguration("width").perform(context))
    height = int(LaunchConfiguration("height").perform(context))
    fps = int(LaunchConfiguration("fps").perform(context))
    profile = f"{width}x{height}x{fps}"

    cam_left_serial = LaunchConfiguration("cam_left_serial")
    cam_left_type = LaunchConfiguration("cam_left_type")
    cam_right_serial = LaunchConfiguration("cam_right_serial")
    cam_right_type = LaunchConfiguration("cam_right_type")
    cam_head_serial = LaunchConfiguration("cam_head_serial")
    cam_head_type = LaunchConfiguration("cam_head_type")

    cam_left_type_str = normalize_camera_type(cam_left_type.perform(context))
    cam_right_type_str = normalize_camera_type(cam_right_type.perform(context))
    cam_head_type_str = normalize_camera_type(cam_head_type.perform(context))

    cameras_to_validate = [
        ("cam_left", cam_left_type_str),
        ("cam_right", cam_right_type_str),
        ("cam_head", cam_head_type_str),
    ]
    log_actions = []
    has_error = False
    for cam_name, cam_type_str in cameras_to_validate:
        is_valid, message = validate_profile(cam_type_str, width, height, fps)
        if not is_valid:
            has_error = True
            log_actions.append(LogInfo(msg=f"\n[ERROR] {cam_name} ({cam_type_str}) 配置无效:\n{message}\n"))
        else:
            log_actions.append(LogInfo(msg=f"[INFO] {cam_name}: {message}"))

    cam_left_controls, cam_left_control_logs = parse_camera_controls(context, "cam_left", cam_left_type_str)
    cam_right_controls, cam_right_control_logs = parse_camera_controls(context, "cam_right", cam_right_type_str)
    cam_head_controls, cam_head_control_logs = parse_camera_controls(context, "cam_head", cam_head_type_str)

    for cam_name, applied_logs in (
        ("cam_left", cam_left_control_logs),
        ("cam_right", cam_right_control_logs),
        ("cam_head", cam_head_control_logs),
    ):
        if applied_logs:
            log_actions.append(LogInfo(msg=f"[INFO] {cam_name} 颜色参数: {', '.join(applied_logs)}"))

    if has_error:
        raise RuntimeError(
            f"\n配置校验失败！\n当前配置: {width}x{height}@{fps}fps\n"
            f"请检查相机类型和分辨率/帧率是否匹配。"
        )

    cam_left_node = create_camera_node(context, "cam_left", cam_left_serial, cam_left_type, profile, cam_left_controls)
    cam_right_node = create_camera_node(context, "cam_right", cam_right_serial, cam_right_type, profile, cam_right_controls)
    cam_head_node = create_camera_node(context, "cam_head", cam_head_serial, cam_head_type, profile, cam_head_controls)
    return log_actions + [cam_left_node, cam_right_node, cam_head_node]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("width", default_value="640", description="图像宽度 (像素)"),
            DeclareLaunchArgument("height", default_value="480", description="图像高度 (像素)"),
            DeclareLaunchArgument("fps", default_value="15", description="帧率 (Hz)"),
            DeclareLaunchArgument("cam_left_serial", default_value="218622270388", description="左腕相机序列号"),
            DeclareLaunchArgument("cam_left_type", default_value="D405", description="左腕相机类型 (D405、D435 或 D435I)"),
            DeclareLaunchArgument("cam_right_serial", default_value="218622274446", description="右腕相机序列号"),
            DeclareLaunchArgument("cam_right_type", default_value="D405", description="右腕相机类型 (D405、D435 或 D435I)"),
            DeclareLaunchArgument("cam_head_serial", default_value="335522070220", description="头部相机序列号"),
            DeclareLaunchArgument("cam_head_type", default_value="D435", description="头部相机类型 (D405、D435 或 D435I)"),
        ]
        + create_camera_control_arguments()
        + [OpaqueFunction(function=launch_setup)]
    )