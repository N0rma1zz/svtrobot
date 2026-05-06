// ============================================================
// main_openarm.cpp
//
// VR Teleoperation for OpenArm Bimanual Robot (C++)
// Uses PXREARobotSDK (VR), ROS2 (hardware comm), KDL (IK)
//
// Build:
//   source /opt/ros/humble/setup.bash
//   source ~/openarm_ws/ros2_ws/install/setup.bash
//   cd XRoboToolkit-Teleop-Sample-Cpp
//   mkdir -p build && cd build
//   cmake -DBUILD_UR5=OFF -DBUILD_OPENARM=ON .. && make
//
// Run:
//   # Terminal 1: Start hardware
//   ros2 launch openarm_bringup openarm.bimanual.launch.py \
//     arm_type:=v10 robot_controller:=forward_position_controller
//
//   # Terminal 2: Teleop
//   ./teleop_demo_openarm --urdf /path/to/openarm_bimanual_control.urdf
// ============================================================

#define _USE_MATH_DEFINES
#include <cmath>
#include <iostream>
#include <sstream>
#include <thread>
#include <mutex>
#include <atomic>
#include <array>
#include <vector>
#include <string>
#include <chrono>
#include <csignal>
#include <algorithm>
#include <iomanip>
#include <fstream>
#include <functional>

// VR SDK
#include <PXREARobotSDK.h>
#include <nlohmann/json.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>

// ROS2
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/float64.hpp>
#include <control_msgs/action/gripper_command.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/bool.hpp>

// KDL / URDF
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/frames.hpp>
#include <urdf/model.h>

using json = nlohmann::json;
using GripperAction = control_msgs::action::GripperCommand;

// ============================================================
// Configuration
// ============================================================
constexpr int ARM_DOF = 7;
constexpr double DEFAULT_RATE_HZ = 100.0;
constexpr double GRIPPER_OPEN = 0.044;   // meters
constexpr double GRIPPER_CLOSE = 0.0;
constexpr double DEFAULT_SCALE = 1.0;
constexpr double IK_DAMPING = 0.05;
constexpr int MAX_IK_ITER = 10;
constexpr double IK_POS_TOL = 1e-4;
constexpr double IK_ROT_TOL = 1e-3;
constexpr double MAX_JOINT_VEL = 4.0;   // rad/s per joint
constexpr double NULL_SPACE_GAIN = 0.5; // joint centering gain (prevents wrist drift)
constexpr bool ENABLE_VR_JSON_DEBUG = false;
constexpr bool ENABLE_PERIODIC_STATUS_LOG = false;

// VR -> Robot coordinate transform
// VR:    X=right,   Y=up,   Z=backward
// Robot: X=forward,  Y=left, Z=up
const Eigen::Matrix3d R_VR_TO_ROBOT = (Eigen::Matrix3d() <<
     0,  0, -1,
    -1,  0,  0,
     0,  1,  0).finished();
const Eigen::Quaterniond Q_VR_TO_ROBOT(R_VR_TO_ROBOT);

// ============================================================
// Global state
// ============================================================
std::atomic<bool> g_running{true};

struct VRControllerData {
    std::array<double, 7> pose{};   // x,y,z, qx,qy,qz,qw
    double trigger = 0.0;
    double grip = 0.0;
    // VR buttons (right: A/B, left: X/Y)
    bool button_a = false;
    bool button_b = false;
    double thumbstick_x = 0.0;
    double thumbstick_y = 0.0;
};

struct {
    VRControllerData left, right;
    std::array<double, 7> headset{};
    std::mutex mtx;
} g_vr;

struct ArmHWState {
    std::array<double, ARM_DOF> positions{};
    double gripper_pos = 0.0;
    bool received = false;
    std::mutex mtx;
};

ArmHWState g_left_hw, g_right_hw;

// ============================================================
// Signal handler
// ============================================================
void signalHandler(int signum) {
    std::cout << "\nReceived signal " << signum << ", shutting down...\n";
    g_running = false;
}

// ============================================================
// Parse VR pose string "x,y,z,qx,qy,qz,qw"
// ============================================================
std::array<double, 7> parsePoseStr(const std::string& s) {
    std::array<double, 7> r{};
    std::stringstream ss(s);
    std::string tok;
    for (int i = 0; i < 7 && std::getline(ss, tok, ','); i++)
        r[i] = std::stod(tok);
    return r;
}

// ============================================================
// Try to read a bool button field from JSON with multiple key patterns
// ============================================================
bool tryGetButton(const json& j, const std::vector<std::string>& keys) {
    for (auto& k : keys) {
        if (j.contains(k)) {
            if (j[k].is_boolean()) return j[k].get<bool>();
            if (j[k].is_number()) return j[k].get<double>() > 0.5;
            if (j[k].is_string()) return j[k].get<std::string>() == "true" || j[k].get<std::string>() == "1";
        }
    }
    return false;
}

// Dump JSON keys recursively for debug
void dumpJsonKeys(const json& j, const std::string& prefix = "", int depth = 0) {
    if (depth > 4) return;
    for (auto& [k, v] : j.items()) {
        std::string path = prefix.empty() ? k : prefix + "." + k;
        if (v.is_object()) {
            std::cout << "  [JSON] " << path << " = {object}\n";
            dumpJsonKeys(v, path, depth + 1);
        } else if (v.is_array()) {
            std::cout << "  [JSON] " << path << " = [array, size=" << v.size() << "]\n";
        } else if (v.is_boolean()) {
            std::cout << "  [JSON] " << path << " = " << (v.get<bool>() ? "true" : "false") << " (bool)\n";
        } else if (v.is_number()) {
            std::cout << "  [JSON] " << path << " = " << v.get<double>() << " (number)\n";
        } else if (v.is_string()) {
            auto s = v.get<std::string>();
            if (s.size() > 60) s = s.substr(0, 60) + "...";
            std::cout << "  [JSON] " << path << " = \"" << s << "\" (string)\n";
        }
    }
}

// ============================================================
// VR SDK callback
// ============================================================
void OnVRCallback(void*, PXREAClientCallbackType type, int, void* userData) {
    switch (type) {
    case PXREAServerConnect:
        std::cout << "[VR] Server connected\n";
        break;
    case PXREAServerDisconnect:
        std::cout << "[VR] Server disconnected\n";
        break;
    case PXREADeviceFind:
        std::cout << "[VR] Device found: " << static_cast<const char*>(userData) << "\n";
        break;
    case PXREADeviceMissing:
        std::cout << "[VR] Device missing: " << static_cast<const char*>(userData) << "\n";
        break;
    case PXREADeviceConnect:
        std::cout << "[VR] Device connected\n";
        break;
    case PXREADeviceStateJson: {
        auto& dsj = *static_cast<PXREADevStateJson*>(userData);
        static int vr_cb_count = 0;
        static bool json_structure_dumped = false;
        vr_cb_count++;
        try {
            json data = json::parse(dsj.stateJson);

            // Dump full JSON structure once for debugging
            if (ENABLE_VR_JSON_DEBUG && !json_structure_dumped) {
                json_structure_dumped = true;
                std::cout << "\n========== VR JSON STRUCTURE DUMP ==========\n";
                dumpJsonKeys(data, "", 0);
                // If nested value, dump that too
                if (data.contains("value") && data["value"].is_string()) {
                    try {
                        auto v = json::parse(data["value"].get<std::string>());
                        std::cout << "--- Parsed value ---\n";
                        dumpJsonKeys(v, "value", 0);
                    } catch(...) {}
                }
                std::cout << "========== END JSON DUMP ==========\n\n";
            }

            // Button key candidates
            static const std::vector<std::string> btn_a_keys = {"primaryButton", "button_a", "buttonA", "A", "a", "button_primary", "buttonPrimary"};
            static const std::vector<std::string> btn_b_keys = {"secondaryButton", "button_b", "buttonB", "B", "b", "button_secondary", "buttonSecondary"};

            if (!data.contains("value")) {
                if (ENABLE_VR_JSON_DEBUG && vr_cb_count <= 3) std::cout << "[VR-DBG] No 'value' key. Keys:";
                if (ENABLE_VR_JSON_DEBUG && vr_cb_count <= 3) { for (auto& [k,v] : data.items()) std::cout << " " << k; std::cout << "\n"; }
                // Try parsing as direct format (no nested value)
                if (data.contains("Controller") || data.contains("Head")) {
                    std::lock_guard<std::mutex> lock(g_vr.mtx);
                    if (data.contains("Controller")) {
                        auto& ctrl = data["Controller"];
                        if (ctrl.contains("left")) {
                            auto& l = ctrl["left"];
                            if (l.contains("pose")) g_vr.left.pose = parsePoseStr(l["pose"].get<std::string>());
                            if (l.contains("trigger")) g_vr.left.trigger = l["trigger"].get<double>();
                            if (l.contains("grip")) g_vr.left.grip = l["grip"].get<double>();
                            g_vr.left.button_a = tryGetButton(l, btn_a_keys);
                            g_vr.left.button_b = tryGetButton(l, btn_b_keys);
                        }
                        if (ctrl.contains("right")) {
                            auto& r = ctrl["right"];
                            if (r.contains("pose")) g_vr.right.pose = parsePoseStr(r["pose"].get<std::string>());
                            if (r.contains("trigger")) g_vr.right.trigger = r["trigger"].get<double>();
                            if (r.contains("grip")) g_vr.right.grip = r["grip"].get<double>();
                            g_vr.right.button_a = tryGetButton(r, btn_a_keys);
                            g_vr.right.button_b = tryGetButton(r, btn_b_keys);
                        }
                    }
                    if (data.contains("Head")) {
                        g_vr.headset = parsePoseStr(data["Head"]["pose"].get<std::string>());
                    }
                }
                break;
            }
            auto value = json::parse(data["value"].get<std::string>());

            // Debug: print first few and every 500th
            if (ENABLE_VR_JSON_DEBUG && (vr_cb_count <= 3 || vr_cb_count % 500 == 0)) {
                std::cout << "[VR-DBG #" << vr_cb_count << "] value keys:";
                for (auto& [k,v] : value.items()) std::cout << " " << k;
                std::cout << "\n";
            }

            std::lock_guard<std::mutex> lock(g_vr.mtx);
            if (value.contains("Controller")) {
                if (value["Controller"].contains("left")) {
                    auto& l = value["Controller"]["left"];
                    g_vr.left.pose = parsePoseStr(l["pose"].get<std::string>());
                    g_vr.left.trigger = l["trigger"].get<double>();
                    g_vr.left.grip = l["grip"].get<double>();
                    g_vr.left.button_a = tryGetButton(l, btn_a_keys);
                    g_vr.left.button_b = tryGetButton(l, btn_b_keys);
                }
                if (value["Controller"].contains("right")) {
                    auto& r = value["Controller"]["right"];
                    g_vr.right.pose = parsePoseStr(r["pose"].get<std::string>());
                    g_vr.right.trigger = r["trigger"].get<double>();
                    g_vr.right.grip = r["grip"].get<double>();
                    g_vr.right.button_a = tryGetButton(r, btn_a_keys);
                    g_vr.right.button_b = tryGetButton(r, btn_b_keys);
                }
            }
            if (value.contains("Head")) {
                g_vr.headset = parsePoseStr(value["Head"]["pose"].get<std::string>());
            }
        } catch (const std::exception& e) {
            if (vr_cb_count <= 5)
                std::cerr << "[VR-ERR #" << vr_cb_count << "] " << e.what() << "\n"
                          << "  Raw JSON (first 300): " << std::string(dsj.stateJson).substr(0, 300) << "\n";
        }
        break;
    }
    default:
        break;
    }
}

// ============================================================
// Coordinate transform helpers
// ============================================================
Eigen::Vector3d vrPosToRobot(const std::array<double, 7>& p) {
    return R_VR_TO_ROBOT * Eigen::Vector3d(p[0], p[1], p[2]);
}

Eigen::Quaterniond vrQuatToRobot(const std::array<double, 7>& p) {
    Eigen::Quaterniond q(p[6], p[3], p[4], p[5]); // w,x,y,z
    return Q_VR_TO_ROBOT * q * Q_VR_TO_ROBOT.conjugate();
}

KDL::Frame eigenToKDL(const Eigen::Vector3d& pos, const Eigen::Quaterniond& quat) {
    Eigen::Matrix3d rot = quat.toRotationMatrix();
    return KDL::Frame(
        KDL::Rotation(rot(0,0), rot(0,1), rot(0,2),
                      rot(1,0), rot(1,1), rot(1,2),
                      rot(2,0), rot(2,1), rot(2,2)),
        KDL::Vector(pos.x(), pos.y(), pos.z()));
}

Eigen::Vector3d kdlPos(const KDL::Frame& f) {
    return {f.p.x(), f.p.y(), f.p.z()};
}

Eigen::Quaterniond kdlQuat(const KDL::Frame& f) {
    double x, y, z, w;
    f.M.GetQuaternion(x, y, z, w);
    return Eigen::Quaterniond(w, x, y, z);
}

// ============================================================
// IK Solver (KDL-based damped least-squares differential IK)
// ============================================================
struct ArmIK {
    KDL::Chain chain;
    std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk;
    std::unique_ptr<KDL::ChainJntToJacSolver> jac;
    std::array<double, ARM_DOF> q_min{}, q_max{};

    bool init(const KDL::Tree& tree, const std::string& root,
              const std::string& tip, const urdf::Model& model,
              const std::string& joint_prefix) {
        if (!tree.getChain(root, tip, chain)) {
            std::cerr << "KDL: cannot extract chain " << root << " -> " << tip << "\n";
            return false;
        }
        if (static_cast<int>(chain.getNrOfJoints()) != ARM_DOF) {
            std::cerr << "KDL: expected " << ARM_DOF << " joints, got "
                      << chain.getNrOfJoints() << "\n";
            return false;
        }
        fk = std::make_unique<KDL::ChainFkSolverPos_recursive>(chain);
        jac = std::make_unique<KDL::ChainJntToJacSolver>(chain);

        // Joint limits from URDF
        for (int i = 0; i < ARM_DOF; i++) {
            auto j = model.getJoint(joint_prefix + std::to_string(i + 1));
            if (j && j->limits) {
                q_min[i] = j->limits->lower;
                q_max[i] = j->limits->upper;
            } else {
                q_min[i] = -M_PI;
                q_max[i] = M_PI;
            }
        }

        // Print chain info
        int ji = 0;
        for (unsigned s = 0; s < chain.getNrOfSegments(); s++) {
            auto& seg = chain.getSegment(s);
            if (seg.getJoint().getType() != KDL::Joint::Fixed) {
                std::cout << "  joint " << ji << ": " << seg.getJoint().getName()
                          << "  limits [" << q_min[ji] << ", " << q_max[ji] << "]\n";
                ji++;
            }
        }
        return true;
    }

    KDL::Frame computeFK(const std::array<double, ARM_DOF>& q) {
        KDL::JntArray jq(ARM_DOF);
        for (int i = 0; i < ARM_DOF; i++) jq(i) = q[i];
        KDL::Frame frame;
        fk->JntToCart(jq, frame);
        return frame;
    }

    std::array<double, ARM_DOF> solve(
            const std::array<double, ARM_DOF>& q_current,
            const KDL::Frame& target,
            double dt) {
        KDL::JntArray q(ARM_DOF);
        for (int i = 0; i < ARM_DOF; i++) q(i) = q_current[i];

        // Pre-compute joint midpoints (constant)
        Eigen::VectorXd q_mid(ARM_DOF);
        for (int i = 0; i < ARM_DOF; i++)
            q_mid(i) = 0.5 * (q_min[i] + q_max[i]);

        for (int iter = 0; iter < MAX_IK_ITER; iter++) {
            KDL::Frame cur;
            fk->JntToCart(q, cur);

            KDL::Twist tw = KDL::diff(cur, target);
            if (tw.vel.Norm() < IK_POS_TOL && tw.rot.Norm() < IK_ROT_TOL) break;

            KDL::Jacobian J(ARM_DOF);
            jac->JntToJac(q, J);

            Eigen::Matrix<double, 6, 1> dx;
            dx << tw.vel.x(), tw.vel.y(), tw.vel.z(),
                  tw.rot.x(), tw.rot.y(), tw.rot.z();

            Eigen::MatrixXd Jm = J.data;                  // 6 x 7
            Eigen::MatrixXd JJt = Jm * Jm.transpose();    // 6 x 6
            JJt.diagonal().array() += IK_DAMPING * IK_DAMPING;

            // Task-space step
            auto JJt_ldlt = JJt.ldlt();
            Eigen::VectorXd dq_task = Jm.transpose() * JJt_ldlt.solve(dx);

            // Null-space joint centering (reuse LDLT decomposition)
            Eigen::MatrixXd Jpinv = Jm.transpose() * JJt_ldlt.solve(Eigen::MatrixXd::Identity(6, 6));
            Eigen::MatrixXd N = Eigen::MatrixXd::Identity(ARM_DOF, ARM_DOF) - Jpinv * Jm;
            Eigen::VectorXd dq_null = N * (NULL_SPACE_GAIN * (q_mid - q.data));

            Eigen::VectorXd dq = dq_task + dq_null;

            for (int i = 0; i < ARM_DOF; i++) {
                q(i) += dq(i);
                q(i) = std::clamp(q(i), q_min[i], q_max[i]);
            }
        }

        // Velocity-limit the total step (scale proportionally)
        double max_allowed = MAX_JOINT_VEL * dt;
        double max_delta = 0.0;
        for (int i = 0; i < ARM_DOF; i++)
            max_delta = std::max(max_delta, std::abs(q(i) - q_current[i]));

        double scale = (max_delta > max_allowed) ? max_allowed / max_delta : 1.0;

        std::array<double, ARM_DOF> result;
        for (int i = 0; i < ARM_DOF; i++) {
            result[i] = q_current[i] + (q(i) - q_current[i]) * scale;
            result[i] = std::clamp(result[i], q_min[i], q_max[i]);
        }
        return result;
    }
};

// ============================================================
// Per-arm controller state
// ============================================================
struct ArmCtrl {
    std::string name;
    ArmIK ik;

    // Reference poses set on activation
    Eigen::Vector3d ref_ee_pos{};
    Eigen::Quaterniond ref_ee_quat{Eigen::Quaterniond::Identity()};
    Eigen::Vector3d ref_vr_pos{};
    Eigen::Quaterniond ref_vr_quat{Eigen::Quaterniond::Identity()};

    bool is_active = false;
    std::array<double, ARM_DOF> joint_targets{};
    double prev_gripper_cmd = -1.0;

    // ROS2 interfaces
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pos_pub;
    rclcpp_action::Client<GripperAction>::SharedPtr gripper_client;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr gripper_target_pub;

    int pub_count = 0;

    void publishJoints() {
        auto msg = std_msgs::msg::Float64MultiArray();
        msg.data.assign(joint_targets.begin(), joint_targets.end());
        pos_pub->publish(msg);
        pub_count++;
    }

    void sendGripper(double target) {
        if (std::abs(target - prev_gripper_cmd) < 0.0005) return;
        prev_gripper_cmd = target;

        // Publish gripper target for MuJoCo bridge
        if (gripper_target_pub) {
            auto gm = std_msgs::msg::Float64();
            gm.data = target;
            gripper_target_pub->publish(gm);
        }

        if (!gripper_client) return;

        auto goal = GripperAction::Goal();
        goal.command.position = target;
        goal.command.max_effort = 100.0;
        gripper_client->async_send_goal(goal);
    }
};

// ============================================================
// Joint-state callback: extract named joints from the global
// /joint_states topic
// ============================================================
void jointStateCB(
        const sensor_msgs::msg::JointState::SharedPtr msg,
        ArmHWState& state,
        const std::vector<std::string>& joint_names,
        const std::string& gripper_name) {
    std::lock_guard<std::mutex> lk(state.mtx);
    for (int i = 0; i < ARM_DOF; i++) {
        auto it = std::find(msg->name.begin(), msg->name.end(), joint_names[i]);
        if (it != msg->name.end()) {
            auto idx = std::distance(msg->name.begin(), it);
            state.positions[i] = msg->position[idx];
        }
    }
    auto it = std::find(msg->name.begin(), msg->name.end(), gripper_name);
    if (it != msg->name.end()) {
        auto idx = std::distance(msg->name.begin(), it);
        state.gripper_pos = msg->position[idx];
    }
    state.received = true;
}

// ============================================================
// Process one arm for one control step
// ============================================================
void processArm(ArmCtrl& ctrl, ArmHWState& hw,
                VRControllerData& vr_data, double scale, double dt) {
    // Snapshot hardware state
    std::array<double, ARM_DOF> q_cur;
    {
        std::lock_guard<std::mutex> lk(hw.mtx);
        q_cur = hw.positions;
    }

    // Snapshot VR state
    std::array<double, 7> vr_pose;
    double grip_val, trigger_val;
    {
        std::lock_guard<std::mutex> lk(g_vr.mtx);
        vr_pose = vr_data.pose;
        grip_val = vr_data.grip;
        trigger_val = vr_data.trigger;
    }

    bool was_active = ctrl.is_active;
    ctrl.is_active = grip_val > 0.9;

    // Check VR data validity (all zeros means no data yet)
    bool vr_valid = (vr_pose[0] != 0.0 || vr_pose[1] != 0.0 || vr_pose[2] != 0.0);

    if (ctrl.is_active && vr_valid) {
        if (!was_active) {
            // --- Activation: record reference poses ---
            // Use LAST COMMANDED position (not hardware feedback) so there's
            // no jump when re-activating. After deactivation we keep holding
            // joint_targets, so the arm should already be at that position.
            // Using q_cur would cause a drop because hardware feedback lags
            // or sags slightly under gravity.
            KDL::Frame ee = ctrl.ik.computeFK(ctrl.joint_targets);
            ctrl.ref_ee_pos = kdlPos(ee);
            ctrl.ref_ee_quat = kdlQuat(ee);

            ctrl.ref_vr_pos = vrPosToRobot(vr_pose);
            ctrl.ref_vr_quat = vrQuatToRobot(vr_pose);

            // Don't change joint_targets - keep holding current commanded position

            std::cout << "[" << ctrl.name << "] ACTIVATED  EE=["
                      << std::fixed << std::setprecision(3)
                      << ctrl.ref_ee_pos.x() << " "
                      << ctrl.ref_ee_pos.y() << " "
                      << ctrl.ref_ee_pos.z() << "]\n";
        } else {
            // --- Active: compute target from VR delta ---
            Eigen::Vector3d vr_pos = vrPosToRobot(vr_pose);
            Eigen::Quaterniond vr_quat = vrQuatToRobot(vr_pose);

            // Position delta (scaled)
            Eigen::Vector3d delta_pos = (vr_pos - ctrl.ref_vr_pos) * scale;

            // Orientation delta: delta_q such that current = delta_q * ref
            Eigen::Quaterniond delta_q = vr_quat * ctrl.ref_vr_quat.conjugate();

            // Target EE pose
            Eigen::Vector3d target_pos = ctrl.ref_ee_pos + delta_pos;
            Eigen::Quaterniond target_quat = (delta_q * ctrl.ref_ee_quat).normalized();

            KDL::Frame target_frame = eigenToKDL(target_pos, target_quat);

            // Solve IK from LAST COMMANDED position (not hardware feedback).
            // This matches Placo behavior: the solver maintains its own state
            // and doesn't reset to actual hardware pos each frame.
            // Real hardware has latency; starting from q_cur would cause the
            // IK to "tread water" as q_cur lags behind commands.
            ctrl.joint_targets = ctrl.ik.solve(ctrl.joint_targets, target_frame, dt);
        }
    } else if (was_active && !ctrl.is_active) {
        // --- Deactivation: HOLD last commanded position ---
        // Do NOT snap to q_cur. The arm may still be moving toward
        // joint_targets. Keep commanding the last target so the
        // hardware holds that position instead of going limp.
        std::cout << "[" << ctrl.name << "] DEACTIVATED  (holding position)\n";
    }
    // When idle (not active and was not active), keep publishing
    // the same joint_targets to hold position.

    // Always publish joint commands every frame.
    // ForwardCommandController requires continuous commands for real hardware.
    ctrl.publishJoints();

    // Gripper (always responsive, independent of arm)
    double g_target = std::clamp((1.0 - trigger_val) * GRIPPER_OPEN,
                                 GRIPPER_CLOSE, GRIPPER_OPEN);
    ctrl.sendGripper(g_target);
}

// ============================================================
// main
// ============================================================
int main(int argc, char* argv[]) {
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    // ---- Parse arguments ----
    std::string urdf_path;
    double rate_hz = DEFAULT_RATE_HZ;
    double scale = DEFAULT_SCALE;
    bool enable_left = true, enable_right = true;
    bool no_hardware = false;

    for (int i = 1; i < argc; i++) {
        std::string a = argv[i];
        if (a == "--urdf"       && i+1 < argc) urdf_path = argv[++i];
        else if (a == "--rate"  && i+1 < argc) rate_hz = std::stod(argv[++i]);
        else if (a == "--scale" && i+1 < argc) scale = std::stod(argv[++i]);
        else if (a == "--left-only")  enable_right = false;
        else if (a == "--right-only") enable_left  = false;
        else if (a == "--no-hardware") no_hardware = true;
        else if (a == "--help") {
            std::cout <<
                "OpenArm VR Teleoperation (C++)\n\n"
                "Usage: " << argv[0] << " [OPTIONS]\n\n"
                "Options:\n"
                "  --urdf <path>   URDF file (required)\n"
                "  --rate <hz>     Control rate (default: 50)\n"
                "  --scale <f>     VR workspace scale (default: 1.0)\n"
                "  --left-only     Left arm only\n"
                "  --right-only    Right arm only\n"
                "  --no-hardware   VR-only mode (skip hardware wait)\n"
                "  --help          Show this help\n\n"
                "Requires:\n"
                "  ros2 launch openarm_bringup openarm.bimanual.launch.py \\\n"
                "    arm_type:=v10 robot_controller:=forward_position_controller\n";
            return 0;
        }
    }

    if (urdf_path.empty()) {
        // Try default locations
        std::vector<std::string> candidates = {
            "../ros2_ws/openarm_bimanual_control.urdf",
            "openarm_bimanual_control.urdf",
        };
        for (auto& c : candidates) {
            std::ifstream test(c);
            if (test.good()) { urdf_path = c; break; }
        }
        if (urdf_path.empty()) {
            std::cerr << "ERROR: No URDF specified. Use --urdf <path>\n";
            return 1;
        }
    }

    double dt = 1.0 / rate_hz;

    // ---- Parse URDF ----
    std::cout << "Loading URDF: " << urdf_path << "\n";
    urdf::Model urdf_model;
    if (!urdf_model.initFile(urdf_path)) {
        std::cerr << "Failed to parse URDF\n";
        return 1;
    }

    KDL::Tree kdl_tree;
    if (!kdl_parser::treeFromFile(urdf_path, kdl_tree)) {
        std::cerr << "Failed to build KDL tree\n";
        return 1;
    }

    std::string root = kdl_tree.getRootSegment()->second.segment.getName();
    std::cout << "KDL root: " << root
              << "  segments: " << kdl_tree.getNrOfSegments()
              << "  joints: " << kdl_tree.getNrOfJoints() << "\n";

    // ---- IK solvers ----
    ArmCtrl left_ctrl, right_ctrl;
    left_ctrl.name  = "left_arm";
    right_ctrl.name = "right_arm";

    if (enable_left) {
        std::cout << "\nLeft arm chain:\n";
        if (!left_ctrl.ik.init(kdl_tree, root, "openarm_left_link7",
                               urdf_model, "openarm_left_joint")) {
            std::cerr << "Left arm IK init failed\n"; return 1;
        }
    }
    if (enable_right) {
        std::cout << "\nRight arm chain:\n";
        if (!right_ctrl.ik.init(kdl_tree, root, "openarm_right_link7",
                                urdf_model, "openarm_right_joint")) {
            std::cerr << "Right arm IK init failed\n"; return 1;
        }
    }

    // ---- ROS2 ----
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("openarm_vr_teleop_cpp");

    // Joint names
    std::vector<std::string> left_jnames, right_jnames;
    for (int i = 1; i <= ARM_DOF; i++) {
        left_jnames.push_back("openarm_left_joint" + std::to_string(i));
        right_jnames.push_back("openarm_right_joint" + std::to_string(i));
    }

    // Subscriber (global /joint_states)
    auto js_sub = node->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10,
        [&](sensor_msgs::msg::JointState::SharedPtr msg) {
            if (enable_left)
                jointStateCB(msg, g_left_hw, left_jnames, "openarm_left_finger_joint1");
            if (enable_right)
                jointStateCB(msg, g_right_hw, right_jnames, "openarm_right_finger_joint1");
        });

    // Publishers
    if (enable_left) {
        left_ctrl.pos_pub = node->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/left_forward_position_controller/commands", 10);
        left_ctrl.gripper_client = rclcpp_action::create_client<GripperAction>(
            node, "/left_gripper_controller/gripper_cmd");
        left_ctrl.gripper_target_pub = node->create_publisher<std_msgs::msg::Float64>(
            "/left_gripper_target", 10);
    }
    if (enable_right) {
        right_ctrl.pos_pub = node->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/right_forward_position_controller/commands", 10);
        right_ctrl.gripper_client = rclcpp_action::create_client<GripperAction>(
            node, "/right_gripper_controller/gripper_cmd");
        right_ctrl.gripper_target_pub = node->create_publisher<std_msgs::msg::Float64>(
            "/right_gripper_target", 10);
    }

    // VR button publishers (A/B on right controller, X/Y mapped to left A/B)
    auto pub_right_a = node->create_publisher<std_msgs::msg::Bool>("/pico_right_controller/button_a", 10);
    auto pub_right_b = node->create_publisher<std_msgs::msg::Bool>("/pico_right_controller/button_b", 10);
    auto pub_left_a  = node->create_publisher<std_msgs::msg::Bool>("/pico_left_controller/button_a", 10);
    auto pub_left_b  = node->create_publisher<std_msgs::msg::Bool>("/pico_left_controller/button_b", 10);

    // Spin ROS2 in background
    std::thread ros_thread([&]() { rclcpp::spin(node); });

    // ---- VR SDK ----
    std::cout << "\nInitializing VR SDK...\n";
    PXREAInit(nullptr, OnVRCallback, PXREAFullMask);

    // ---- Wait for joint states (skip if --no-hardware) ----
    bool hw_available = false;
    if (no_hardware) {
        std::cout << "--no-hardware mode: skipping hardware wait, VR buttons only.\n";
    } else {
        std::cout << "Waiting for /joint_states...\n";
        auto t0 = std::chrono::steady_clock::now();
        while (g_running) {
            bool ok = (!enable_left  || g_left_hw.received) &&
                      (!enable_right || g_right_hw.received);
            if (ok) { hw_available = true; break; }
            auto elapsed = std::chrono::steady_clock::now() - t0;
            if (std::chrono::duration_cast<std::chrono::seconds>(elapsed).count() > 15) {
                std::cerr << "\nTimeout! Hardware not found. Continuing in VR-only mode.\n"
                          << "  (Use --no-hardware to skip this wait next time)\n";
                break;
            }
            // Publish VR buttons while waiting for hardware
            {
                std::lock_guard<std::mutex> lk(g_vr.mtx);
                std_msgs::msg::Bool btn_msg;
                btn_msg.data = g_vr.right.button_a;
                pub_right_a->publish(btn_msg);
                btn_msg.data = g_vr.right.button_b;
                pub_right_b->publish(btn_msg);
                btn_msg.data = g_vr.left.button_a;
                pub_left_a->publish(btn_msg);
                btn_msg.data = g_vr.left.button_b;
                pub_left_b->publish(btn_msg);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        if (hw_available) std::cout << "Joint states received.\n";
    }

    // Initialize targets to current position and send initial command
    if (hw_available) {
        if (enable_left)  {
            std::lock_guard<std::mutex> lk(g_left_hw.mtx);
            left_ctrl.joint_targets = g_left_hw.positions;
        }
        if (enable_right) {
            std::lock_guard<std::mutex> lk(g_right_hw.mtx);
            right_ctrl.joint_targets = g_right_hw.positions;
        }

        // Publish initial position commands and wait for hardware to settle
        std::cout << "Sending initial position commands...\n";
        for (int i = 0; i < 20; i++) {
            if (enable_left)  left_ctrl.publishJoints();
            if (enable_right) right_ctrl.publishJoints();
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        std::cout << "Hardware warmup complete (2s).\n";
    }

    // ---- Print instructions ----
    std::cout << "\n" << std::string(70, '=') << "\n"
              << " OpenArm VR Teleoperation (C++)\n"
              << std::string(70, '=') << "\n\n"
              << "  Grip Button:  Hold to activate arm control\n"
              << "  Trigger:      Open/close gripper\n"
              << "  Release Grip: Stop arm (holds position)\n\n"
              << "  Rate: " << rate_hz << " Hz   Scale: " << scale << "\n"
              << "  Arms: " << (enable_left  ? "LEFT " : "")
                            << (enable_right ? "RIGHT" : "") << "\n"
              << "  Press Ctrl+C to stop\n"
              << std::string(70, '=') << "\n\n";

    // ---- Control loop ----
    int loop_n = 0;
    double max_loop_ms = 0.0;
    while (g_running) {
        auto t_start = std::chrono::steady_clock::now();

        if (hw_available) {
            if (enable_left)
                processArm(left_ctrl, g_left_hw, g_vr.left, scale, dt);
            if (enable_right)
                processArm(right_ctrl, g_right_hw, g_vr.right, scale, dt);
        }

        // Publish VR button states
        {
            std::lock_guard<std::mutex> lk(g_vr.mtx);
            std_msgs::msg::Bool btn_msg;
            btn_msg.data = g_vr.right.button_a;
            pub_right_a->publish(btn_msg);
            btn_msg.data = g_vr.right.button_b;
            pub_right_b->publish(btn_msg);
            btn_msg.data = g_vr.left.button_a;
            pub_left_a->publish(btn_msg);
            btn_msg.data = g_vr.left.button_b;
            pub_left_b->publish(btn_msg);
        }

        auto t_after_ik = std::chrono::steady_clock::now();
        double loop_ms = std::chrono::duration<double, std::milli>(t_after_ik - t_start).count();
        max_loop_ms = std::max(max_loop_ms, loop_ms);

        // Periodic status
        loop_n++;
        if (ENABLE_PERIODIC_STATUS_LOG && loop_n % static_cast<int>(rate_hz * 2) == 0) {
            std::lock_guard<std::mutex> lk(g_vr.mtx);
            std::cout << "[#" << loop_n << "] "
                      << "L:" << (left_ctrl.is_active  ? "ACT" : "idl")
                      << " R:" << (right_ctrl.is_active ? "ACT" : "idl")
                      << "  loop_max=" << std::fixed << std::setprecision(1) << max_loop_ms << "ms"
                      << "  pub=" << left_ctrl.pub_count << "/" << right_ctrl.pub_count
                      << std::setprecision(2)
                      << "  Lg=" << g_vr.left.grip << " Rg=" << g_vr.right.grip
                      << "  btn[RA=" << g_vr.right.button_a
                      << " RB=" << g_vr.right.button_b
                      << " LA=" << g_vr.left.button_a
                      << " LB=" << g_vr.left.button_b << "]";
            if (enable_right) {
                std::lock_guard<std::mutex> hlk(g_right_hw.mtx);
                std::cout << std::setprecision(3) << "  Rc=[";
                for (int i = 0; i < 3; i++) std::cout << (i?",":"") << right_ctrl.joint_targets[i];
                std::cout << "] Rh=[";
                for (int i = 0; i < 3; i++) std::cout << (i?",":"") << g_right_hw.positions[i];
                std::cout << "]";
            }
            std::cout << "\n";
            max_loop_ms = 0.0;
        }

        // Sleep to maintain rate
        auto elapsed = std::chrono::steady_clock::now() - t_start;
        auto remaining = std::chrono::duration<double>(dt) - elapsed;
        if (remaining.count() > 0)
            std::this_thread::sleep_for(remaining);
    }

    // ---- Cleanup ----
    std::cout << "\nShutting down...\n";

    // Hold current position
    if (enable_left) {
        std::lock_guard<std::mutex> lk(g_left_hw.mtx);
        left_ctrl.joint_targets = g_left_hw.positions;
        left_ctrl.publishJoints();
    }
    if (enable_right) {
        std::lock_guard<std::mutex> lk(g_right_hw.mtx);
        right_ctrl.joint_targets = g_right_hw.positions;
        right_ctrl.publishJoints();
    }

    PXREADeinit();
    rclcpp::shutdown();
    if (ros_thread.joinable()) ros_thread.join();

    std::cout << "Shutdown complete.\n";
    return 0;
}
