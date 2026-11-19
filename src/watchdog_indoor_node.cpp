#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <deque>        
#include <Eigen/Dense>
#include <sstream>
#include <cmath>
#include <limits>
#include <mutex>
#include <cstdint>
#include <stdexcept>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>

using std::placeholders::_1;
using namespace std::chrono_literals;

using std_msgs::msg::Bool;
using nav_msgs::msg::Odometry;
using geometry_msgs::msg::PoseWithCovarianceStamped;
using geometry_msgs::msg::PoseStamped;
using px4_msgs::msg::VehicleOdometry;

struct Pose {
    double t;
    Eigen::Vector3f position;
    Eigen::Quaternionf orientation;
};

class Watchdog : public rclcpp::Node {
public:
    Watchdog() : Node("watchdog_indoor_node") {       
        // Check parameters for switching between MoCap and odometry
        min_x_ = this->declare_parameter<double>("min_x", -1.0);
        max_x_ = this->declare_parameter<double>("max_x", 8.0);

        min_y_ = this->declare_parameter<double>("min_y", -2.0);
        max_y_ = this->declare_parameter<double>("max_y", 4.0);
        xy_limit_count_ = this->declare_parameter<int>("xy_limit_count", 10);

        max_z_ = this->declare_parameter<double>("max_z", 2.0);
        z_limit_count_ = this->declare_parameter<int>("z_limit_count", 10);

        max_v_ = this->declare_parameter<double>("max_v", 2.0);
        max_delta_v_ = this->declare_parameter<double>("max_delta_v", 2.0);

        timeout_sec_ = this->declare_parameter<double>("timeout_sec", 0.5);
        min_init_samples_ = this->declare_parameter<int>("min_init_samples", 10);

        max_stamp_age_sec_ = this->declare_parameter<double>("max_stamp_age_sec", 0.5);
        max_future_stamp_sec_ = this->declare_parameter<double>("max_future_stamp_sec", 0.05);

        // Asset name parameter from mocap system (e.g., "PoliceDrone")
        asset_name_ = this->declare_parameter<std::string>("asset_name", "PoliceDrone");

        // Defining topic names as parameters
        mocap_topic_ = this->declare_parameter<std::string>("mocap_topic", "/vrpn_mocap/" + asset_name_ + "/pose");
        mocap_debug_topic_ = this->declare_parameter<std::string>("mocap_debug_topic", "/vrpn_mocap/out/debug");
        odometry_topic_ = this->declare_parameter<std::string>("odometry_topic", "/vio/pose");
        odometry_debug_topic_ = this->declare_parameter<std::string>("odometry_debug_topic", "/vio/pose_debug");
        fmu_in_topic_ = this->declare_parameter<std::string>("fmu_in_topic", "/fmu/in/vehicle_visual_odometry");
        watchdog_status_topic_ = this->declare_parameter<std::string>("watchdog_status_topic", "/watchdog/status");

        // QoS for sensor data
        rclcpp::QoS qos = rclcpp::SensorDataQoS().keep_last(1);
        rclcpp::QoS status_qos = rclcpp::QoS(1).reliable().transient_local();

        // Subscribers
        mocap_sub_ = this->create_subscription<PoseStamped>(mocap_topic_, qos, std::bind(&Watchdog::mocap_callback, this, _1));
        odometry_sub_ = this->create_subscription<PoseWithCovarianceStamped>(odometry_topic_, qos, std::bind(&Watchdog::odom_callback, this, _1));

        // Publishers
        mocap_debug_pub_ = this->create_publisher<VehicleOdometry>(mocap_debug_topic_, qos);
        odometry_debug_pub_ = this->create_publisher<VehicleOdometry>(odometry_debug_topic_, qos);
        fmu_in_pub_ = this->create_publisher<VehicleOdometry>(fmu_in_topic_, qos);
        watchdog_status_pub_ = this->create_publisher<Bool>(watchdog_status_topic_, status_qos);

        // Timer 
        stop_timer_ = this->create_wall_timer(1000ms, std::bind(&Watchdog::estimate_transformation, this));
            
        // Watchdog timer
        watchdog_timer_ = this->create_wall_timer(20ms, [this]() {
            if (initialised) {
                watchdog_checks();
            } else if (init_window_started_) {
                if (!watchdog_checks()) {
                    init_window_started_ = false;
                    mocap_buffer_.clear();
                    odom_buffer_.clear();
                    RCLCPP_WARN(this->get_logger(), "Initialisation window reset due to subscriber inactivity");
                }
            }

            Bool status;
            status.data = use_mocap_;
            watchdog_status_pub_->publish(status);
        });

        RCLCPP_INFO(this->get_logger(), "Indoor Watchdog Started");
    }

private:
    void mocap_callback(const PoseStamped::SharedPtr msg) {
        Pose p;
        p.t = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
        p.position = Eigen::Vector3f(
            msg->pose.position.x,
            msg->pose.position.z,
            -msg->pose.position.y
        );
        p.orientation = Eigen::Quaternionf(
            msg->pose.orientation.w,
            msg->pose.orientation.x,
            msg->pose.orientation.z,
            -msg->pose.orientation.y
        );

        // Check timestamp and message validity
        if (!watchdog_checks(&p, true)) {
            return;
        }

        p.orientation.normalize();

        // Create MoCap message
        VehicleOdometry mocap_msg = create_vehicle_odometry(p);

        // Always publish valid MoCap data to debug topic
        mocap_debug_pub_->publish(mocap_msg);

        // Collect data during transformation initialisation
        if (!initialised) {
            std::lock_guard<std::mutex> lock(mocap_mutex_);
            mocap_buffer_.push_back(p);

            while (!mocap_buffer_.empty() &&
                p.t - mocap_buffer_.front().t > 10.0) {
                mocap_buffer_.pop_front();
            }

            return;
        }

        // Publish MoCap to PX4 if watchdog has triggered
        if (use_mocap_) {
            publish_to_px4(mocap_msg);
        }
    }

    void odom_callback(const PoseWithCovarianceStamped::SharedPtr msg) {
        Pose p;
        p.t = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
        p.position = Eigen::Vector3f(
            msg->pose.pose.position.x,
            msg->pose.pose.position.y,
            msg->pose.pose.position.z
        );
        p.orientation = Eigen::Quaternionf(
            msg->pose.pose.orientation.w,
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z
        );

        // Perform watchdog checks
        if (!watchdog_checks(&p, false)) {
            return;
        }

        p.orientation.normalize();

        if (initialised) {
            odom_update(p);
        } else {
            std::lock_guard<std::mutex> lock(odom_mutex_);
            odom_buffer_.push_back(p);

            while (!odom_buffer_.empty() &&
                p.t - odom_buffer_.front().t > 10.0) {
                odom_buffer_.pop_front();
            }
        }
    }

    void odom_update(const Pose& p) {
        Eigen::Matrix3f R_m_to_s = transform_m_to_s_.block<3,3>(0,0);
        Eigen::Vector3f t_m_to_s = transform_m_to_s_.block<3,1>(0,3);

        // Apply full SE(3) transform
        Eigen::Vector3f transformed_pos = R_m_to_s * p.position + t_m_to_s;

        // Rotate orientation
        Eigen::Matrix3f R_slam = p.orientation.toRotationMatrix();
        Eigen::Matrix3f R_rotated = R_m_to_s * R_slam;
        Eigen::Quaternionf q_rotated(R_rotated);

        // Update the pose with transformed data
        Pose odom_ned;
        odom_ned.t = p.t;
        odom_ned.position = transformed_pos;
        odom_ned.orientation = q_rotated.normalized();

        // Validate transformed pose
        if (!odom_ned.position.allFinite() ||
            !odom_ned.orientation.coeffs().allFinite()) {

            fail("Transformed odometry pose invalid");
            return;
        }

        // Create odometry message
        VehicleOdometry odom_msg = create_vehicle_odometry(odom_ned);

        // Always publish transformed odometry for debugging
        odometry_debug_pub_->publish(odom_msg);

        // Check subscriber activity before forwarding to PX4
        if (!use_mocap_) {
            watchdog_checks();
        }

        // Publish odometry to PX4 if watchdog has not triggered
        if (!use_mocap_) {
            publish_to_px4(odom_msg);
        }
    }

    bool watchdog_checks(const Pose* pose = nullptr, bool is_mocap = false) {
        const auto now = std::chrono::steady_clock::now();

        // Check 1A: Subscriber activity
        if (pose == nullptr) {
            const double mocap_age = std::chrono::duration<double>(now - last_mocap_rx_).count();
            const double odom_age = std::chrono::duration<double>(now - last_odom_rx_).count();

            if (mocap_age > timeout_sec_) {
                fail("MoCap subscriber inactive");
                return false;
            }

            if (odom_age > timeout_sec_) {
                fail("Odometry subscriber inactive");
                return false;
            }

            return true;
        }

        const Pose& p = *pose;
        double& last_timestamp = is_mocap ? last_mocap_timestamp_ : last_odom_timestamp_;

        // Check 1B: Timestamp validity
        if (!std::isfinite(p.t) || p.t <= 0.0 || p.t <= last_timestamp) {
            fail(is_mocap ? "MoCap timestamp invalid" : "Odometry timestamp invalid");
            return false;
        }

        // Check 1C: Message validity
        const float q_norm = p.orientation.norm();
        if (!p.position.allFinite() || !p.orientation.coeffs().allFinite() ||
            !std::isfinite(q_norm) || std::abs(q_norm - 1.0f) > 0.05f) {

            fail(is_mocap ? "MoCap message invalid" : "Odometry message invalid");
            return false;
        }

        // Check 1D: Measurement timestamp age
        const double ros_now = this->get_clock()->now().seconds();
        const double stamp_age = ros_now - p.t;
        if (stamp_age > max_stamp_age_sec_ ||
            stamp_age < -max_future_stamp_sec_) {

            std::ostringstream reason;
            reason << (is_mocap ? "MoCap" : "Odometry")
                << " measurement timestamp outside permitted range ("
                << stamp_age << " s)";

            fail(reason.str().c_str());
            return false;
        }

        // Update timestamp and receipt time after validation
        last_timestamp = p.t;

        if (is_mocap) {
            last_mocap_rx_ = now;
        } else {
            last_odom_rx_ = now;
        }

        // Remaining checks only apply to aligned odometry
        if (is_mocap || !initialised || use_mocap_) {
            return true;
        }

        // Transform odometry to MoCap frame for further checks
        Eigen::Vector3f position =
            transform_m_to_s_.block<3,3>(0,0) * p.position +
            transform_m_to_s_.block<3,1>(0,3);

        if (!position.allFinite()) {
            fail("Transformed odometry position invalid");
            return false;
        }

        // Check 2: Altitude
        const float altitude = -position.z();
        if (altitude > max_z_) {
            ++z_bad_count_;
        } else {
            z_bad_count_ = 0;
        }

        if (z_bad_count_ >= z_limit_count_) {
            std::ostringstream reason;
            reason << "Altitude exceeded limit ("
                << altitude << " m > "
                << max_z_ << " m) for "
                << z_bad_count_ << " consecutive messages";

            fail(reason.str().c_str());
            return true;
        }

        // Check 3: XY position
        const bool outside_xy = position.x() < min_x_ || position.x() > max_x_ || position.y() < min_y_ || position.y() > max_y_;
        if (outside_xy) {
            ++xy_bad_count_;
        } else {
            xy_bad_count_ = 0;
        }

        if (xy_bad_count_ >= xy_limit_count_) {
            std::ostringstream reason;
            reason << "XY position exceeded limits ("
                << "x=" << position.x()
                << ", y=" << position.y()
                << ") for " << xy_bad_count_
                << " consecutive messages";

            fail(reason.str().c_str());
            return true;
        }

        // Check 4: Velocity
        if (have_previous_position_) {
            const double dt = p.t - previous_odom_time_;

            if (dt > 0.0) {
                Eigen::Vector3f velocity = (position - previous_odom_position_) / static_cast<float>(dt);

                if (!velocity.allFinite()) {
                    fail("Odometry velocity invalid");
                    return true;
                }

                // Check 4A: Absolute velocity
                const float speed = velocity.norm();

                if (!std::isfinite(speed) || speed > max_v_) {
                    std::ostringstream reason;

                    reason << "Velocity exceeded limit ("
                        << speed << " m/s > "
                        << max_v_ << " m/s)"
                        << "\nTime difference: " << dt * 1000.0 << " ms"
                        << "\nPosition difference: "
                        << (position - previous_odom_position_).norm() << " m"
                        << "\nPrevious position: ["
                        << previous_odom_position_.transpose() << "]"
                        << "\nCurrent position: ["
                        << position.transpose() << "]"
                        << "\nVelocity vector: ["
                        << velocity.transpose() << "] m/s";

                    fail(reason.str().c_str());

                    return true;
                }

                // Check 4B: Change in velocity
                if (have_previous_velocity_) {
                    const float delta_velocity = (velocity - previous_odom_velocity_).norm();

                    if (!std::isfinite(delta_velocity)) {
                        fail("Odometry delta velocity invalid");
                        return true;
                    }

                    if (delta_velocity > max_delta_v_) {
                        std::ostringstream reason;
                        reason << "Delta velocity exceeded limit ("
                            << delta_velocity << " m/s > "
                            << max_delta_v_ << " m/s)";

                        fail(reason.str().c_str());
                        return true;
                    }
                }

                previous_odom_velocity_ = velocity;
                have_previous_velocity_ = true;
            }
        }

        // Store current position for the next velocity calculation
        previous_odom_position_ = position;
        previous_odom_time_ = p.t;
        have_previous_position_ = true;

        return true;
    }

    void estimate_transformation() {
        if (initialised) {
            return;
        }

        // Check that both subscribers are active    
        if (!watchdog_checks()) {
            init_window_started_ = false;
            mocap_buffer_.clear();
            odom_buffer_.clear();

            return;
        }

        // Waiting for both streams to have a sample
        if (mocap_buffer_.empty() || odom_buffer_.empty()) {
            RCLCPP_WARN(this->get_logger(), "Waiting for first MoCap + SLAM messages...");
            return;
        }

        // Start initialisaiton window
        if (!init_window_started_) {
            init_start_time_ = this->get_clock()->now();
            init_window_started_ = true;

            mocap_buffer_.clear();
            odom_buffer_.clear();

            RCLCPP_INFO(this->get_logger(),
                        "Both streams active — collecting data for %.1f seconds...",
                        init_duration_sec_);
            return;
        }

        // Check to see if enough time has passed
        double dt = (this->get_clock()->now() - init_start_time_).seconds();

        if (dt < init_duration_sec_) {
            RCLCPP_DEBUG(this->get_logger(),
                        "Collecting calibration data: %.2f / %.2f sec",
                        dt, init_duration_sec_);
            return;
        }

        // Ensure sufficient samples have been collected
        if (mocap_buffer_.size() < static_cast<size_t>(min_init_samples_) ||
            odom_buffer_.size() < static_cast<size_t>(min_init_samples_)) {

            RCLCPP_WARN(this->get_logger(),
                        "Insufficient initialisation samples: MoCap=%zu, Odometry=%zu",
                        mocap_buffer_.size(), odom_buffer_.size());

            return;
        }

        // Estimate transform
        RCLCPP_INFO(this->get_logger(), "Estimating SLAM → MoCap transform...");

        Pose mocap_avg = average_pose(mocap_buffer_);
        Pose slam_avg  = average_pose(odom_buffer_);

        Eigen::Matrix4f T_m_to_s = compute_transform(mocap_avg, slam_avg);
        Eigen::Matrix3f R = T_m_to_s.block<3,3>(0,0);
        Eigen::Vector3f t = T_m_to_s.block<3,1>(0,3);

        // Convert rotation matrix to Euler angles (ZYX order → yaw, pitch, roll)
        float yaw   = atan2(R(1,0), R(0,0));
        float pitch = asin(-R(2,0));
        float roll  = atan2(R(2,1), R(2,2));

        Eigen::Vector3f euler_deg(
            roll * 180.0f / M_PI,
            pitch * 180.0f / M_PI,
            yaw * 180.0f / M_PI
        );

        std::stringstream ss;
        ss << "\n--- Estimated Rigid Transform (SLAM -> MoCap) ---\n" << T_m_to_s;
        RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());

        RCLCPP_INFO(this->get_logger(),"Rotation (Euler angles, degrees): [Roll=%.2f°, Pitch=%.2f°, Yaw=%.2f°]",
            euler_deg.x(), euler_deg.y(), euler_deg.z()
        );
        RCLCPP_INFO(this->get_logger(),"Translation [x, y, z]: [%.4f, %.4f, %.4f]",
            t.x(), t.y(), t.z());
        
        RCLCPP_INFO(this->get_logger(), "Average MoCap position: [%.3f, %.3f, %.3f]",
                    mocap_avg.position.x(), mocap_avg.position.y(), mocap_avg.position.z());
        RCLCPP_INFO(this->get_logger(), "Average SLAM position: [%.3f, %.3f, %.3f]",
                    slam_avg.position.x(), slam_avg.position.y(), slam_avg.position.z());
    
        // Validate estimated transformation
        if (!T_m_to_s.allFinite()) {
            RCLCPP_ERROR(this->get_logger(), "Estimated transformation invalid");

            init_window_started_ = false;
            mocap_buffer_.clear();
            odom_buffer_.clear();

            return;
        }

        // Store transformation
        transform_m_to_s_ = T_m_to_s;

        // Mark system as initialised
        initialised = true;
        RCLCPP_INFO(this->get_logger(), "Transform successfully estimated — system initialised.");

        // Clear pose buffers and stop initialisation timer
        mocap_buffer_.clear();
        odom_buffer_.clear();
        stop_timer_->cancel();
    }

    // --- Averaging and Transform Estimation ---
    Pose average_pose(const std::deque<Pose>& poses) {
        Pose avg;
        avg.t = 0;
        avg.position = Eigen::Vector3f::Zero();
        avg.orientation = Eigen::Quaternionf::Identity();

        size_t n = poses.size();
        if (n == 0) return avg;

        for (const auto& p : poses) {
            avg.t += p.t;
            avg.position += p.position;
        }
        avg.t /= n;
        avg.position /= n;

        // Average quaternion (Markley method)
        Eigen::Matrix4f A = Eigen::Matrix4f::Zero();
        for (const auto& p : poses) {
            Eigen::Vector4f q(p.orientation.w(), p.orientation.x(), p.orientation.y(), p.orientation.z());
            q.normalize();
            A += q * q.transpose();
        }
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix4f> solver(A);
        Eigen::Vector4f avg_q = solver.eigenvectors().col(3);
        avg.orientation = Eigen::Quaternionf(avg_q(0), avg_q(1), avg_q(2), avg_q(3)).normalized();

        return avg;
    }

    Eigen::Matrix4f compute_transform(const Pose& mocap_avg, const Pose& slam_avg) {
        // Compute transform from SLAM → MoCap (instead of MoCap → SLAM)
        Eigen::Quaternionf q_ms = mocap_avg.orientation * slam_avg.orientation.conjugate();
        Eigen::Matrix3f R_ms = q_ms.toRotationMatrix();
        Eigen::Vector3f t_ms = mocap_avg.position - R_ms * slam_avg.position;

        Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
        T.block<3,3>(0,0) = R_ms;
        T.block<3,1>(0,3) = t_ms;
        return T;
    }

    VehicleOdometry create_vehicle_odometry(const Pose& p) {
        VehicleOdometry msg{};

        const float nan = std::numeric_limits<float>::quiet_NaN();

        // Timestamps in microseconds
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        msg.timestamp_sample = static_cast<uint64_t>(
            std::llround(p.t * 1e6)
        );

        // Coordinate frame
        msg.pose_frame = VehicleOdometry::POSE_FRAME_FRD;

        // Position
        msg.position = {
            p.position.x(),
            p.position.y(),
            p.position.z()
        };

        // Orientation
        msg.q = {
            p.orientation.w(),
            p.orientation.x(),
            p.orientation.y(),
            p.orientation.z()
        };

        // Velocity unavailable
        msg.velocity_frame = VehicleOdometry::VELOCITY_FRAME_UNKNOWN;
        msg.velocity.fill(nan);
        msg.angular_velocity.fill(nan);

        // Measurement variance unavailable
        msg.position_variance.fill(nan);
        msg.orientation_variance.fill(nan);
        msg.velocity_variance.fill(nan);

        // Additional fields
        msg.reset_counter = 0;
        msg.quality = 0;

        return msg;
    }

    void fail(const char* reason) {
        if (!initialised) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Initialisation check failed (%s)", reason);
            return;
        }

        if (!use_mocap_) {
            use_mocap_ = true;
            ++reset_counter_;

            RCLCPP_WARN(this->get_logger(), "Watchdog check failed (%s) — switching to MoCap", reason);
        }
    }

    void publish_to_px4(VehicleOdometry msg) {
        // Ensure timestamps sent to PX4 are increasing
        if (msg.timestamp_sample <= last_fmu_timestamp_sample_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "PX4 odometry message rejected: timestamp_sample is not increasing");
            return;
        }

        msg.reset_counter = reset_counter_;
        fmu_in_pub_->publish(msg);
        last_fmu_timestamp_sample_ = msg.timestamp_sample;
    }

    // Topic names
    std::string asset_name_;
    std::string mocap_topic_;
    std::string mocap_debug_topic_;
    std::string odometry_topic_;
    std::string odometry_debug_topic_;
    std::string fmu_in_topic_;
    std::string watchdog_status_topic_;

    // Subscribers
    rclcpp::Subscription<PoseStamped>::SharedPtr mocap_sub_;
    rclcpp::Subscription<PoseWithCovarianceStamped>::SharedPtr odometry_sub_;

    // Publishers
    rclcpp::Publisher<VehicleOdometry>::SharedPtr mocap_debug_pub_;
    rclcpp::Publisher<VehicleOdometry>::SharedPtr odometry_debug_pub_;
    rclcpp::Publisher<VehicleOdometry>::SharedPtr fmu_in_pub_;
    rclcpp::Publisher<Bool>::SharedPtr watchdog_status_pub_;

    // Timers
    rclcpp::TimerBase::SharedPtr stop_timer_;
    rclcpp::TimerBase::SharedPtr watchdog_timer_;

    // Transformation initialisation
    rclcpp::Time init_start_time_;
    bool init_window_started_ = false;
    bool initialised = false;
    const double init_duration_sec_ = 3.0;

    // Coordinate transformation
    Eigen::Matrix4f transform_m_to_s_ = Eigen::Matrix4f::Identity();

    // Initialisation pose buffers
    std::deque<Pose> odom_buffer_;
    std::deque<Pose> mocap_buffer_;

    mutable std::mutex odom_mutex_;
    mutable std::mutex mocap_mutex_;

    // Watchdog parameters
    double min_x_, max_x_;
    double min_y_, max_y_;
    double max_z_, max_v_, max_delta_v_;
    double timeout_sec_;
    double max_stamp_age_sec_;
    double max_future_stamp_sec_;

    int xy_limit_count_;
    int z_limit_count_;
    int min_init_samples_;

    // Consecutive limit violation counters
    int xy_bad_count_ = 0;
    int z_bad_count_ = 0;

    // Subscriber timestamp monitoring
    double last_mocap_timestamp_ = 0.0;
    double last_odom_timestamp_ = 0.0;

    // Subscriber activity monitoring
    std::chrono::steady_clock::time_point last_mocap_rx_{};
    std::chrono::steady_clock::time_point last_odom_rx_{};

    // Velocity monitoring
    bool have_previous_position_ = false;
    bool have_previous_velocity_ = false;

    double previous_odom_time_ = 0.0;

    Eigen::Vector3f previous_odom_position_ = Eigen::Vector3f::Zero();
    Eigen::Vector3f previous_odom_velocity_ = Eigen::Vector3f::Zero();

    // Localisation source selection
    bool use_mocap_ = false;
    uint8_t reset_counter_ = 0;

    // PX4 timestamp monitoring
    uint64_t last_fmu_timestamp_sample_ = 0;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Watchdog>();

    // using Multi-threaded Executor allows multiple callbacks to run in parallel using separate threads
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
