#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include "shared_memory.hpp"
#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>

#include "drake/common/find_runfiles.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace bip = boost::interprocess;

// Capsule constraint: signed distance and outward normal from point p to capsule
struct CapsuleResult {
    double g;               // signed distance (positive = safe)
    Eigen::Vector3d normal; // unit outward normal
    Eigen::Vector3d closest; // closest point on capsule surface
};

CapsuleResult EvaluateCapsule(const Eigen::Vector3d& p,
                               const Eigen::Vector3d& p0,
                               const Eigen::Vector3d& p1,
                               double radius) {
    Eigen::Vector3d seg = p1 - p0;
    double seg_len_sq = seg.squaredNorm();

    double t = 0.0;
    if (seg_len_sq > 1e-12) {
        t = std::clamp((p - p0).dot(seg) / seg_len_sq, 0.0, 1.0);
    }

    Eigen::Vector3d q_closest = p0 + t * seg;
    Eigen::Vector3d diff = p - q_closest;
    double dist = diff.norm();

    CapsuleResult result;
    result.g = dist - radius;

    if (dist > 1e-6) {
        result.normal = diff / dist;
    } else {
        Eigen::Vector3d arbitrary = (std::abs(seg.x()) < 0.9)
            ? Eigen::Vector3d::UnitX() : Eigen::Vector3d::UnitY();
        result.normal = seg.cross(arbitrary).normalized();
    }

    result.closest = q_closest + radius * result.normal;
    return result;
}

// ---------------------------------------------------------------------------
// Kalman filter for capsule state: [p0x, p0y, p0z, p1x, p1y, p1z, radius]
// State vector is 14-dim: [pos(7), vel(7)]
// Measurement vector is 7-dim: [p0(3), p1(3), radius]
// ---------------------------------------------------------------------------
class CapsuleKalmanFilter {
public:
    struct Params {
        double dt = 0.1;
        double pos_process_noise = 1e-4;
        double vel_process_noise = 1e-2;
        double measurement_noise = 1e-3;
        double gate_threshold = 0.10;
        int reinit_after = 20;
    };

    explicit CapsuleKalmanFilter(const Params& params) : params_(params) {
        Reset();
    }

    void UpdateParams(const Params& params) { params_ = params; }

    void Reset() {
        initialized_ = false;
        consecutive_rejections_ = 0;
        x_.setZero();
        P_.setZero();
        P_.diagonal().head<7>().setConstant(1.0);
        P_.diagonal().tail<7>().setConstant(0.1);
    }

    bool IsInitialized() const { return initialized_; }
    Eigen::Matrix<double, 7, 1> GetEstimate() const { return x_.head<7>(); }

    void Update(const Eigen::Matrix<double, 7, 1>& z_meas) {
        if (!initialized_) {
            x_.head<7>() = z_meas;
            x_.tail<7>().setZero();
            initialized_ = true;
            consecutive_rejections_ = 0;
            return;
        }

        Eigen::Matrix<double, 14, 14> F = Eigen::Matrix<double, 14, 14>::Identity();
        for (int i = 0; i < 7; ++i) F(i, i + 7) = params_.dt;

        Eigen::Matrix<double, 14, 14> Q = Eigen::Matrix<double, 14, 14>::Zero();
        Q.diagonal().head<7>().setConstant(params_.pos_process_noise);
        Q.diagonal().tail<7>().setConstant(params_.vel_process_noise);

        Eigen::Matrix<double, 7, 14> H = Eigen::Matrix<double, 7, 14>::Zero();
        H.block<7, 7>(0, 0) = Eigen::Matrix<double, 7, 7>::Identity();

        Eigen::Matrix<double, 7, 7> R = Eigen::Matrix<double, 7, 7>::Zero();
        R.diagonal().setConstant(params_.measurement_noise);

        Eigen::Matrix<double, 14, 1> x_pred = F * x_;
        Eigen::Matrix<double, 14, 14> P_pred = F * P_ * F.transpose() + Q;

        Eigen::Matrix<double, 7, 1> innovation = z_meas - H * x_pred;
        double pos_innovation_norm = innovation.head<6>().norm();

        if (pos_innovation_norm > params_.gate_threshold) {
            consecutive_rejections_++;
            x_ = x_pred;
            P_ = P_pred;
            if (consecutive_rejections_ >= params_.reinit_after) {
                x_.head<7>() = z_meas;
                x_.tail<7>().setZero();
                P_.setZero();
                P_.diagonal().head<7>().setConstant(0.01);
                P_.diagonal().tail<7>().setConstant(0.1);
                consecutive_rejections_ = 0;
            }
            return;
        }

        Eigen::Matrix<double, 7, 7> S = H * P_pred * H.transpose() + R;
        Eigen::Matrix<double, 14, 7> K = P_pred * H.transpose() * S.inverse();
        x_ = x_pred + K * innovation;
        P_ = (Eigen::Matrix<double, 14, 14>::Identity() - K * H) * P_pred;
        consecutive_rejections_ = 0;
    }

private:
    Params params_;
    bool initialized_ = false;
    int consecutive_rejections_ = 0;
    Eigen::Matrix<double, 14, 1> x_;
    Eigen::Matrix<double, 14, 14> P_;
};

// ---------------------------------------------------------------------------
// Main ROS2 node
// ---------------------------------------------------------------------------
class ConstraintVisualizer : public rclcpp::Node {
public:
    ConstraintVisualizer()
        : Node("constraint_visualizer")
    {
        // --- Kalman filter params ---
        this->declare_parameter("kalman.dt", 0.1);
        this->declare_parameter("kalman.pos_process_noise", 1e-4);
        this->declare_parameter("kalman.vel_process_noise", 1e-2);
        this->declare_parameter("kalman.measurement_noise", 1e-3);
        this->declare_parameter("kalman.gate_threshold", 0.10);
        this->declare_parameter("kalman.reinit_after", 20);

        // --- Point-cloud-to-robot transform params (tunable at runtime) ---
        // T_robot_pointcloud: transforms points FROM point cloud frame INTO robot frame
        // Usage: p_robot = R * p_pointcloud + t
        this->declare_parameter("transform.tx", 0.0);
        this->declare_parameter("transform.ty", 0.0);
        this->declare_parameter("transform.tz", 0.0);
        this->declare_parameter("transform.roll", 0.0);    // radians
        this->declare_parameter("transform.pitch", 0.0);   // radians
        this->declare_parameter("transform.yaw", 0.0);

        auto kf_params = LoadKalmanParams();
        capsule_kf_ = std::make_unique<CapsuleKalmanFilter>(kf_params);

        param_cb_handle_ = this->add_on_set_parameters_callback(
            std::bind(&ConstraintVisualizer::OnParamChange, this, std::placeholders::_1));

        // Publishers
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/constraint_viz", 10);
        joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "/joint_states", 10);

        // Set up Drake plant for FK
        const std::string model_file = GetPathOrThrow(
            drake::FindRunfile(
                "drake_franka_driver/franka_description/urdf/panda_arm.urdf"));

        plant_ = std::make_unique<drake::multibody::MultibodyPlant<double>>(0.0);
        drake::multibody::Parser parser(plant_.get());
        parser.AddModelsFromUrl(std::string("file://") + model_file);
        plant_->WeldFrames(
            plant_->world_frame(),
            plant_->GetFrameByName("panda_link0"));
        plant_->Finalize();
        context_ = plant_->CreateDefaultContext();

        // 10Hz timer
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&ConstraintVisualizer::TimerCallback, this));

        RCLCPP_INFO(this->get_logger(), "Constraint visualizer started");
        RCLCPP_INFO(this->get_logger(),
            "For robot model in RViz, run:\n"
            "  ros2 run robot_state_publisher robot_state_publisher "
            "--ros-args -p robot_description:=\"$(cat %s)\"",
            model_file.c_str());
    }

private:
    static std::string GetPathOrThrow(const drake::RlocationOrError& result) {
        if (!result.error.empty()) throw std::runtime_error(result.error);
        return result.abspath;
    }

    CapsuleKalmanFilter::Params LoadKalmanParams() {
        CapsuleKalmanFilter::Params p;
        p.dt = this->get_parameter("kalman.dt").as_double();
        p.pos_process_noise = this->get_parameter("kalman.pos_process_noise").as_double();
        p.vel_process_noise = this->get_parameter("kalman.vel_process_noise").as_double();
        p.measurement_noise = this->get_parameter("kalman.measurement_noise").as_double();
        p.gate_threshold = this->get_parameter("kalman.gate_threshold").as_double();
        p.reinit_after = this->get_parameter("kalman.reinit_after").as_int();
        return p;
    }

    // Build rotation matrix from current transform params
    Eigen::Isometry3d LoadTransform() {
        double tx = this->get_parameter("transform.tx").as_double();
        double ty = this->get_parameter("transform.ty").as_double();
        double tz = this->get_parameter("transform.tz").as_double();
        double roll = this->get_parameter("transform.roll").as_double();
        double pitch = this->get_parameter("transform.pitch").as_double();
        double yaw = this->get_parameter("transform.yaw").as_double();

        Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
        T.linear() = (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())
                     * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
                     * Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())).toRotationMatrix();
        T.translation() = Eigen::Vector3d(tx, ty, tz);
        return T;
    }

    // Apply transform to a 3D point (point cloud frame -> robot frame)
    Eigen::Vector3d TransformPoint(const Eigen::Isometry3d& T, const Eigen::Vector3d& p) {
        return T * p;
    }

    rcl_interfaces::msg::SetParametersResult OnParamChange(
            const std::vector<rclcpp::Parameter>& /*params*/) {
        auto kf_params = LoadKalmanParams();
        capsule_kf_->UpdateParams(kf_params);

        auto T = LoadTransform();
        RCLCPP_INFO(this->get_logger(),
            "Params updated — transform: t=(%.3f, %.3f, %.3f) rpy=(%.3f, %.3f, %.3f)",
            T.translation().x(), T.translation().y(), T.translation().z(),
            this->get_parameter("transform.roll").as_double(),
            this->get_parameter("transform.pitch").as_double(),
            this->get_parameter("transform.yaw").as_double());

        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        return result;
    }

    void TimerCallback() {
        // Read shared memory
        bip::managed_shared_memory shm_segment;
        SharedMemoryData* shm = nullptr;
        try {
            shm_segment = bip::managed_shared_memory(bip::open_only, "MySharedMemory");
            shm = shm_segment.find<SharedMemoryData>("SharedData").first;
        } catch (const std::exception& e) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "Shared memory not available: %s", e.what());
            return;
        }
        if (!shm) return;

        Eigen::VectorXd q(7);
        Eigen::Vector3d raw_centroid, raw_p0, raw_p1;
        double raw_radius = 0.0;
        bool have_joints = false;
        bool have_capsule = false;

        {
            bip::scoped_lock<bip::interprocess_mutex> lock(shm->mutex);

            if (shm->data_ready && shm->data.size() >= 7) {
                for (int i = 0; i < 7; ++i) q[i] = shm->data[i];
                have_joints = true;
            }

            // Layout: [label, cx, cy, cz, p0x, p0y, p0z, p1x, p1y, p1z, radius]
            if (shm->clusters_ready && shm->clusters.size() >= 11) {
                raw_centroid = Eigen::Vector3d(shm->clusters[1], shm->clusters[2], shm->clusters[3]);
                raw_p0 = Eigen::Vector3d(shm->clusters[4], shm->clusters[5], shm->clusters[6]);
                raw_p1 = Eigen::Vector3d(shm->clusters[7], shm->clusters[8], shm->clusters[9]);
                raw_radius = shm->clusters[10];
                have_capsule = true;
            }
        }

        if (!have_joints) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "Waiting for joint data");
            return;
        }

        // --- Publish JointState for robot_state_publisher ---
        {
            sensor_msgs::msg::JointState js;
            js.header.stamp = this->now();
            js.name = {"panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4",
                        "panda_joint5", "panda_joint6", "panda_joint7"};
            js.position.resize(7);
            for (int i = 0; i < 7; ++i) js.position[i] = q[i];
            joint_pub_->publish(js);
        }

        if (!have_capsule) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "Waiting for capsule data");
            return;
        }

        // Log raw values so we can see what's coming in
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "RAW centroid=(%.3f,%.3f,%.3f) p0=(%.3f,%.3f,%.3f) p1=(%.3f,%.3f,%.3f) r=%.4f",
            raw_centroid.x(), raw_centroid.y(), raw_centroid.z(),
            raw_p0.x(), raw_p0.y(), raw_p0.z(),
            raw_p1.x(), raw_p1.y(), raw_p1.z(),
            raw_radius);

        // --- Apply point-cloud-to-robot transform ---
        Eigen::Isometry3d T = LoadTransform();
        Eigen::Vector3d transformed_centroid = TransformPoint(T, raw_centroid);
        Eigen::Vector3d transformed_p0 = TransformPoint(T, raw_p0);
        Eigen::Vector3d transformed_p1 = TransformPoint(T, raw_p1);
        // Radius is scalar, not affected by rigid transform

        // --- Kalman filter the transformed capsule ---
        Eigen::Matrix<double, 7, 1> z_meas;
        z_meas << transformed_p0, transformed_p1, raw_radius;
        capsule_kf_->Update(z_meas);
        if (!capsule_kf_->IsInitialized()) return;

        Eigen::Matrix<double, 7, 1> filtered = capsule_kf_->GetEstimate();
        Eigen::Vector3d cap_p0 = filtered.head<3>();
        Eigen::Vector3d cap_p1 = filtered.segment<3>(3);
        double cap_radius = std::max(filtered(6), 0.001);

        // FK: set joint positions and get link positions
        plant_->SetPositions(context_.get(), q);

        const std::vector<std::string> link_names = {
            "panda_link1", "panda_link2", "panda_link3", "panda_link4",
            "panda_link5", "panda_link6", "panda_link7", "panda_link8"
        };

        std::vector<Eigen::Vector3d> link_positions;
        for (const auto& name : link_names) {
            link_positions.push_back(
                plant_->EvalBodyPoseInWorld(*context_,
                    plant_->GetBodyByName(name)).translation());
        }

        // Build marker array
        visualization_msgs::msg::MarkerArray markers;
        auto stamp = this->now();
        int id = 0;

        // --- Capsule body (cylinder) ---
        {
            Eigen::Vector3d mid = (cap_p0 + cap_p1) / 2.0;
            Eigen::Vector3d axis = cap_p1 - cap_p0;
            double length = axis.norm();

            auto marker = MakeMarker(stamp, id++, visualization_msgs::msg::Marker::CYLINDER);
            marker.pose.position.x = mid.x();
            marker.pose.position.y = mid.y();
            marker.pose.position.z = mid.z();

            if (length > 1e-6) {
                Eigen::Vector3d z_axis = Eigen::Vector3d::UnitZ();
                Eigen::Vector3d cap_axis = axis / length;
                Eigen::Vector3d rot_axis = z_axis.cross(cap_axis);
                double rot_angle = std::acos(std::clamp(z_axis.dot(cap_axis), -1.0, 1.0));
                if (rot_axis.norm() > 1e-6) {
                    Eigen::AngleAxisd aa(rot_angle, rot_axis.normalized());
                    Eigen::Quaterniond quat(aa);
                    marker.pose.orientation.x = quat.x();
                    marker.pose.orientation.y = quat.y();
                    marker.pose.orientation.z = quat.z();
                    marker.pose.orientation.w = quat.w();
                }
            }
            marker.scale.x = 2.0 * cap_radius;
            marker.scale.y = 2.0 * cap_radius;
            marker.scale.z = length;
            marker.color.r = 1.0; marker.color.a = 0.3;
            markers.markers.push_back(marker);
        }

        // --- Capsule endcaps ---
        for (int i = 0; i < 2; ++i) {
            const Eigen::Vector3d& pt = (i == 0) ? cap_p0 : cap_p1;
            auto marker = MakeMarker(stamp, id++, visualization_msgs::msg::Marker::SPHERE);
            marker.pose.position.x = pt.x();
            marker.pose.position.y = pt.y();
            marker.pose.position.z = pt.z();
            marker.scale.x = marker.scale.y = marker.scale.z = 2.0 * cap_radius;
            marker.color.r = 1.0; marker.color.a = 0.3;
            markers.markers.push_back(marker);
        }

        // --- Raw centroid as sphere with radius (wireframe-ish, semi-transparent cyan) ---
        {
            auto marker = MakeMarker(stamp, id++, visualization_msgs::msg::Marker::SPHERE);
            marker.pose.position.x = transformed_centroid.x();
            marker.pose.position.y = transformed_centroid.y();
            marker.pose.position.z = transformed_centroid.z();
            marker.scale.x = marker.scale.y = marker.scale.z = 2.0 * cap_radius;
            marker.color.g = 1.0; marker.color.b = 1.0; marker.color.a = 0.15;  // cyan, very transparent
            marker.ns = "centroid_sphere";
            markers.markers.push_back(marker);
        }

        // --- Raw centroid point (small solid yellow sphere) ---
        {
            auto marker = MakeMarker(stamp, id++, visualization_msgs::msg::Marker::SPHERE);
            marker.pose.position.x = transformed_centroid.x();
            marker.pose.position.y = transformed_centroid.y();
            marker.pose.position.z = transformed_centroid.z();
            marker.scale.x = marker.scale.y = marker.scale.z = 0.025;
            marker.color.r = 1.0; marker.color.g = 1.0; marker.color.a = 1.0;  // yellow
            marker.ns = "centroid_point";
            markers.markers.push_back(marker);
        }

        // --- Centroid label ---
        {
            auto marker = MakeMarker(stamp, id++, visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
            marker.pose.position.x = transformed_centroid.x();
            marker.pose.position.y = transformed_centroid.y();
            marker.pose.position.z = transformed_centroid.z() + 0.05;
            marker.scale.z = 0.025;
            char buf[64];
            std::snprintf(buf, sizeof(buf), "C: (%.2f,%.2f,%.2f) r=%.3f",
                transformed_centroid.x(), transformed_centroid.y(), transformed_centroid.z(),
                cap_radius);
            marker.text = buf;
            marker.color.r = 1.0; marker.color.g = 1.0; marker.color.a = 1.0;
            markers.markers.push_back(marker);
        }

        // --- Per-link markers ---
        for (size_t i = 0; i < link_positions.size(); ++i) {
            const auto& p = link_positions[i];
            CapsuleResult res = EvaluateCapsule(p, cap_p0, cap_p1, cap_radius);

            // Link position sphere
            {
                auto marker = MakeMarker(stamp, id++, visualization_msgs::msg::Marker::SPHERE);
                marker.pose.position.x = p.x();
                marker.pose.position.y = p.y();
                marker.pose.position.z = p.z();
                marker.scale.x = marker.scale.y = marker.scale.z = 0.03;
                if (res.g > 0.05) {
                    marker.color.g = 1.0;
                } else if (res.g > 0.0) {
                    marker.color.r = 1.0; marker.color.g = 1.0;
                } else {
                    marker.color.r = 1.0;
                }
                marker.color.a = 1.0;
                markers.markers.push_back(marker);
            }

            // Closest point (blue)
            {
                auto marker = MakeMarker(stamp, id++, visualization_msgs::msg::Marker::SPHERE);
                marker.pose.position.x = res.closest.x();
                marker.pose.position.y = res.closest.y();
                marker.pose.position.z = res.closest.z();
                marker.scale.x = marker.scale.y = marker.scale.z = 0.02;
                marker.color.b = 1.0; marker.color.a = 1.0;
                markers.markers.push_back(marker);
            }

            // Normal arrow
            {
                auto marker = MakeMarker(stamp, id++, visualization_msgs::msg::Marker::ARROW);
                geometry_msgs::msg::Point start, end;
                start.x = res.closest.x();
                start.y = res.closest.y();
                start.z = res.closest.z();
                double arrow_len = std::clamp(std::abs(res.g), 0.02, 0.3);
                Eigen::Vector3d tip = Eigen::Vector3d(res.closest.x(), res.closest.y(), res.closest.z())
                    + arrow_len * res.normal;
                end.x = tip.x();
                end.y = tip.y();
                end.z = tip.z();
                marker.points.push_back(start);
                marker.points.push_back(end);
                marker.scale.x = 0.005;
                marker.scale.y = 0.01;
                marker.scale.z = 0.0;
                if (res.g > 0.05) {
                    marker.color.g = 1.0;
                } else if (res.g > 0.0) {
                    marker.color.r = 1.0; marker.color.g = 1.0;
                } else {
                    marker.color.r = 1.0;
                }
                marker.color.a = 1.0;
                markers.markers.push_back(marker);
            }

            // Distance text
            {
                auto marker = MakeMarker(stamp, id++,
                    visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
                marker.pose.position.x = p.x();
                marker.pose.position.y = p.y();
                marker.pose.position.z = p.z() + 0.04;
                marker.scale.z = 0.02;
                char buf[32];
                std::snprintf(buf, sizeof(buf), "L%zu: %.3f", i + 1, res.g);
                marker.text = buf;
                marker.color.r = marker.color.g = marker.color.b = 1.0;
                marker.color.a = 1.0;
                markers.markers.push_back(marker);
            }
        }

        // Delete leftover markers
        if (id < prev_marker_count_) {
            for (int del_id = id; del_id < prev_marker_count_; ++del_id) {
                auto marker = MakeMarker(stamp, del_id, visualization_msgs::msg::Marker::SPHERE);
                marker.action = visualization_msgs::msg::Marker::DELETE;
                markers.markers.push_back(marker);
            }
        }
        prev_marker_count_ = id;

        marker_pub_->publish(markers);
    }

    visualization_msgs::msg::Marker MakeMarker(
            const rclcpp::Time& stamp, int id, int type) {
        visualization_msgs::msg::Marker m;
        m.header.frame_id = "panda_link0";
        m.header.stamp = stamp;
        m.ns = "constraint_viz";
        m.id = id;
        m.type = type;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.pose.orientation.w = 1.0;
        m.color.a = 1.0;
        m.lifetime = rclcpp::Duration(0, 200'000'000);
        return m;
    }

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::unique_ptr<drake::multibody::MultibodyPlant<double>> plant_;
    std::unique_ptr<drake::systems::Context<double>> context_;

    std::unique_ptr<CapsuleKalmanFilter> capsule_kf_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

    int prev_marker_count_ = 0;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ConstraintVisualizer>());
    rclcpp::shutdown();
    return 0;
}
