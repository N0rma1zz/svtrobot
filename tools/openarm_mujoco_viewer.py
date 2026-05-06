#!/usr/bin/env python3
"""
OpenArm MuJoCo Camera Tuning Viewer.

为每个相机打开独立窗口，实时渲染。
支持 XML 热重载：编辑 XML 中的相机参数后，按 R 键或等待自动检测文件变化，
窗口会自动刷新为新的相机视角。

用法:
    # 多窗口实时预览（每个相机一个窗口）+ XML 热重载
    python openarm_mujoco_viewer.py

    # 保存各相机快照（headless 可用）
    python openarm_mujoco_viewer.py --snapshot

    # 自定义 XML 路径
    python openarm_mujoco_viewer.py --xml openarm_mujoco/v1/openarm_scene_groot.xml

    # MuJoCo 原生交互式 viewer
    python openarm_mujoco_viewer.py --native

操作说明:
    R          - 重新加载 XML（修改相机参数后按此键立即生效）
    S          - 保存当前所有相机截图到 camera_snapshots/
    Q / ESC    - 退出
    XML 文件被修改后也会自动重载（每秒检测一次）
"""

import argparse
import os
import sys
import time

import cv2
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", ".."))

# Default arm pose
DEFAULT_LEFT_ARM = [0.0, -0.8, 0.0, 1.2, 0.0, 0.0, 0.0]
DEFAULT_RIGHT_ARM = [0.0, 0.8, 0.0, 1.2, 0.0, 0.0, 0.0]


def _set_default_pose(m, d):
    """Set robot to default home pose."""
    import mujoco
    joints = ([f"openarm_left_joint{i}" for i in range(1, 8)]
              + [f"openarm_right_joint{i}" for i in range(1, 8)])
    values = DEFAULT_LEFT_ARM + DEFAULT_RIGHT_ARM
    for jname, val in zip(joints, values):
        jid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, jname)
        if jid >= 0:
            d.qpos[m.jnt_qposadr[jid]] = val
    mujoco.mj_forward(m, d)


def _load_scene(xml_path: str):
    """Load MuJoCo model and set default pose. Returns (model, data, camera_names)."""
    import mujoco
    m = mujoco.MjModel.from_xml_path(xml_path)
    d = mujoco.MjData(m)
    _set_default_pose(m, d)
    cam_names = []
    for i in range(m.ncam):
        name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_CAMERA, i)
        cam_names.append(name)
    return m, d, cam_names


def _render_camera(renderer, data, cam_id: int, cam_name: str) -> np.ndarray:
    """Render one camera and add overlay text."""
    renderer.update_scene(data, camera=cam_id)
    img = renderer.render().copy()
    # Green label with camera name
    cv2.putText(img, cam_name, (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
    return img


def _get_camera_info(m) -> str:
    """Get camera info string for console output."""
    import mujoco
    lines = []
    for i in range(m.ncam):
        name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_CAMERA, i)
        body_id = m.cam_bodyid[i]
        body_name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, body_id)
        pos = m.cam_pos[i]
        quat = m.cam_quat[i]
        fovy = m.cam_fovy[i]
        lines.append(f"  [{i}] {name}: body={body_name}, "
                     f"pos=[{pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}], "
                     f"quat=[{quat[0]:.4f}, {quat[1]:.4f}, {quat[2]:.4f}, {quat[3]:.4f}], "
                     f"fovy={fovy:.1f}")
    return "\n".join(lines)


# ─────────── Multi-window live viewer with hot-reload ───────────

def live_camera_viewer(xml_path: str, win_height: int = 360, win_width: int = 480):
    """Open each camera in a separate OpenCV window with XML hot-reload."""
    import mujoco

    xml_path = os.path.abspath(xml_path)
    # Track all XML files involved (scene + included bimanual)
    xml_dir = os.path.dirname(xml_path)
    xml_files = [xml_path]
    # Also watch the included bimanual_groot.xml
    for candidate in ["openarm_bimanual_groot.xml", "openarm_bimanual.xml"]:
        p = os.path.join(xml_dir, candidate)
        if os.path.exists(p):
            xml_files.append(p)

    def get_mtimes():
        return {f: os.path.getmtime(f) for f in xml_files if os.path.exists(f)}

    print("=" * 60)
    print("  OpenArm Camera Tuning Viewer")
    print("=" * 60)
    print(f"  XML: {xml_path}")
    print(f"  Watching: {[os.path.basename(f) for f in xml_files]}")
    print()
    print("  R     = 手动重载 XML")
    print("  S     = 保存截图到 camera_snapshots/")
    print("  Q/ESC = 退出")
    print("  XML 文件修改后自动重载")
    print("=" * 60)

    m, d, cam_names = _load_scene(xml_path)
    renderer = mujoco.Renderer(m, height=win_height, width=win_width)
    last_mtimes = get_mtimes()

    print(f"\n相机列表 ({len(cam_names)} 个):")
    print(_get_camera_info(m))
    print()

    # Create windows and arrange them
    for i, name in enumerate(cam_names):
        win_name = f"[{i}] {name}"
        cv2.namedWindow(win_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(win_name, win_width, win_height)
        # Tile windows: 2 per row
        col = i % 2
        row = i // 2
        cv2.moveWindow(win_name, 50 + col * (win_width + 20), 50 + row * (win_height + 60))

    need_reload = True  # render on first iteration

    while True:
        # Check file changes (every loop, ~1 FPS)
        current_mtimes = get_mtimes()
        if current_mtimes != last_mtimes:
            print("\n[自动重载] 检测到 XML 文件变化，重新加载...")
            need_reload = True
            last_mtimes = current_mtimes

        if need_reload:
            try:
                renderer.close()
                m, d, cam_names = _load_scene(xml_path)
                renderer = mujoco.Renderer(m, height=win_height, width=win_width)
                print(f"  重载成功! 相机数: {len(cam_names)}")
                print(_get_camera_info(m))

                # Close old windows, create new ones
                cv2.destroyAllWindows()
                for i, name in enumerate(cam_names):
                    win_name = f"[{i}] {name}"
                    cv2.namedWindow(win_name, cv2.WINDOW_NORMAL)
                    cv2.resizeWindow(win_name, win_width, win_height)
                    col = i % 2
                    row = i // 2
                    cv2.moveWindow(win_name, 50 + col * (win_width + 20),
                                   50 + row * (win_height + 60))
            except Exception as e:
                print(f"  [重载失败] {e}")
                print("  请修正 XML 后重试 (按 R 或等待自动检测)")
                need_reload = False
                time.sleep(1)
                continue
            need_reload = False

        # Render all cameras
        for i, name in enumerate(cam_names):
            win_name = f"[{i}] {name}"
            img_rgb = _render_camera(renderer, d, i, name)
            img_bgr = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2BGR)
            cv2.imshow(win_name, img_bgr)

        key = cv2.waitKey(500) & 0xFF  # 500ms = ~2 FPS, low CPU usage

        if key == ord('q') or key == 27:  # Q or ESC
            break
        elif key == ord('r'):
            print("\n[手动重载] 重新加载 XML...")
            need_reload = True
        elif key == ord('s'):
            out_dir = "camera_snapshots"
            os.makedirs(out_dir, exist_ok=True)
            for i, name in enumerate(cam_names):
                img_rgb = _render_camera(renderer, d, i, name)
                path = os.path.join(out_dir, f"{name}.png")
                cv2.imwrite(path, cv2.cvtColor(img_rgb, cv2.COLOR_RGB2BGR))
                print(f"  保存: {path}")
            print(f"  截图已保存到 {out_dir}/")

    renderer.close()
    cv2.destroyAllWindows()
    print("\n退出.")


# ─────────── Snapshot mode (headless) ───────────

def save_snapshots(xml_path: str, output_dir: str = "camera_snapshots"):
    """Save snapshot images from all cameras (works headless with EGL)."""
    os.environ.setdefault("MUJOCO_GL", "egl")
    import mujoco

    m, d, cam_names = _load_scene(xml_path)
    os.makedirs(output_dir, exist_ok=True)
    renderer = mujoco.Renderer(m, height=480, width=640)

    print(f"相机列表 ({len(cam_names)} 个):")
    print(_get_camera_info(m))
    print()

    for i, name in enumerate(cam_names):
        renderer.update_scene(d, camera=i)
        img = renderer.render()
        path = os.path.join(output_dir, f"{name}.png")
        cv2.imwrite(path, cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
        print(f"  保存: {path} ({img.shape})")

    renderer.close()
    print(f"\n所有 {len(cam_names)} 个相机截图已保存到 {output_dir}/")


# ─────────── Native MuJoCo viewer ───────────

def launch_native_viewer(xml_path: str):
    """Launch MuJoCo's built-in interactive viewer."""
    import mujoco
    import mujoco.viewer

    m, d, cam_names = _load_scene(xml_path)
    print("MuJoCo 原生 Viewer:")
    print("  鼠标拖拽=旋转, 滚轮=缩放, 双击=选择")
    print("  Tab=切换相机, ] [=下/上一个相机")
    print(_get_camera_info(m))
    mujoco.viewer.launch(m, d)


# ─────────── Main ───────────

if __name__ == "__main__":
    default_xml = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                               "..", "src", "openarm_mujoco", "v1",
                               "openarm_scene_groot.xml")

    parser = argparse.ArgumentParser(
        description="OpenArm MuJoCo 相机调参工具",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  # 多窗口实时预览 + XML 热重载（默认模式）
  python openarm_mujoco_viewer.py

  # 保存各相机截图
  python openarm_mujoco_viewer.py --snapshot

  # MuJoCo 原生交互式 viewer
  python openarm_mujoco_viewer.py --native

  # 指定 XML
  python openarm_mujoco_viewer.py --xml path/to/scene.xml

相机 XML 编辑方法:
  编辑 openarm_mujoco/v1/openarm_bimanual_groot.xml

  front_camera (固定相机):
    <camera name="front_camera" mode="fixed" fovy="60"
      pos="0.7 0.0 0.9" quat="0.924 0.0 0.383 0.0"/>

  wrist_camera (跟随 link7):
    <camera name="left_wrist_camera" pos="0 -0.03 0.1"
      quat="0.5 0.5 -0.5 0.5" fovy="70"/>

  参数说明:
    pos   = 相机位置 (x y z)，相对于所在 body
    quat  = 朝向四元数 (w x y z)
    fovy  = 垂直视场角（度）
    mode  = fixed(世界固定) / track(跟踪目标)
        """)
    parser.add_argument("--xml", type=str, default=default_xml,
                        help="场景 XML 路径")
    parser.add_argument("--snapshot", action="store_true",
                        help="保存截图模式（headless 可用）")
    parser.add_argument("--native", action="store_true",
                        help="启动 MuJoCo 原生 viewer")
    parser.add_argument("--output_dir", type=str, default="camera_snapshots",
                        help="截图输出目录")
    parser.add_argument("--width", type=int, default=480,
                        help="每个相机窗口宽度")
    parser.add_argument("--height", type=int, default=360,
                        help="每个相机窗口高度")
    args = parser.parse_args()

    if args.snapshot:
        save_snapshots(args.xml, args.output_dir)
    elif args.native:
        launch_native_viewer(args.xml)
    else:
        live_camera_viewer(args.xml, args.height, args.width)
