#include <chrono>
#include <memory>
#include <cmath>
#include <random>

#include "calibrate_position/calibrate_position.hpp"

#include "geometry_msgs/msg/point_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include <rclcpp/publisher.hpp>
#include <visualization_msgs/msg/marker_array.hpp>



using namespace std::chrono_literals;

//假设相机固定在云台离旋转中心camera_gimbal处，在云台旋转中心前方armor_world_处有一装甲板，云台yaw轴从-30°到30°以60°每秒的速度来回摆动
class GimbalController : public rclcpp::Node {
public:
    GimbalController() 
    : Node("gimbal_controller"), 
        camera_gimbal(0.1, 0.005, 0.005, 0.05, 0.05, 0.05), 
        armor_world_(2.0, 0.5, 0.5, 0, 0, 0),
        gen_(rd_()),
        dist_(-0.05, 0.05)  // ±5% 均匀分布噪声
    {
        armors_topic = this->declare_parameter<std::string>("armors_topic", "/detector/armors");
        gimbal_frame = this->declare_parameter<std::string>("gimbal_frame", "gimbal_link");
        world_frame = this->declare_parameter<std::string>("world_frame", "odom");
        
        // 初始化TF广播器
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        
        // 创建定时器，错开发布时间测试tf的插值功能
        armor_pub_timer_ = this->create_wall_timer(
            0.01s, std::bind(&GimbalController::publish_armor_position, this));
            
        tf_pub_timer_ = this->create_wall_timer(
            0.003s, std::bind(&GimbalController::publish_tf_transform, this));
            
        // 创建发布器
        armor_pub_ = this->create_publisher<auto_aim_interfaces::msg::Armors>(armors_topic, 10);

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // 创建TF变换
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = this->now();
        transform.header.frame_id = world_frame;
        transform.child_frame_id = "armor";
        
        transform.transform.translation.x = add_noise(std::get<0>(armor_world_));
        transform.transform.translation.y = add_noise(std::get<1>(armor_world_));
        transform.transform.translation.z = add_noise(std::get<2>(armor_world_));
        
        tf2::Quaternion q;
        q.setRPY(
            add_noise(std::get<3>(camera_gimbal)), 
            add_noise(std::get<4>(camera_gimbal)), 
            add_noise(std::get<5>(camera_gimbal))
        );  
        transform.transform.rotation.x = q.x();
        transform.transform.rotation.y = q.y();
        transform.transform.rotation.z = q.z();
        transform.transform.rotation.w = q.w();
        
        // 广播TF
        static_tf_broadcaster_->sendTransform(transform);

        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/armor_marker", 10);

        RCLCPP_INFO(this->get_logger(), "Starting publish TF transform and armor position!");
    }

private:
    // 获取当前偏航角（弧度）
    double get_current_yaw() {
        auto now = this->now().seconds();
        const double period = 2.0; //周期，60°*2/角速度
        const double max_angle = 30.0 * M_PI / 180.0;
        
        double t_mod = fmod(now, period);
        if(t_mod < 0) t_mod += period;
        
        if(t_mod <= period/2) {
            return -max_angle + (2*max_angle/(period/2)) * t_mod;
        } else {
            return max_angle - (2*max_angle/(period/2)) * (t_mod - period/2);
        }
    }

    // 添加1%噪声的辅助函数
    double add_noise(double value) {
        return value * (1.0 + dist_(gen_));
    }

    void publish_tf_transform() {
        // 计算当前旋转矩阵
        double yaw = get_current_yaw();
        
        // 创建TF变换
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = this->now();
        transform.header.frame_id = world_frame;
        transform.child_frame_id = gimbal_frame;

        
        // 设置旋转（添加角度噪声）
        tf2::Quaternion q;
        q.setRPY(0, 0, add_noise(yaw));  // 旋转角度添加1%噪声
        transform.transform.rotation.x = q.x();
        transform.transform.rotation.y = q.y();
        transform.transform.rotation.z = q.z();
        transform.transform.rotation.w = q.w();
        
        // 广播TF
        tf_broadcaster_->sendTransform(transform);
        
        transform.header.stamp = this->now();
        transform.header.frame_id = gimbal_frame;
        transform.child_frame_id = "camera_frame";
        
        // 计算并添加平移噪声
        transform.transform.translation.x = add_noise(std::get<0>(camera_gimbal));
        transform.transform.translation.y = add_noise(std::get<1>(camera_gimbal));
        transform.transform.translation.z = add_noise(std::get<2>(camera_gimbal));

        // 设置旋转（添加角度噪声）
        q.setRPY(
            add_noise(std::get<3>(camera_gimbal)), 
            add_noise(std::get<4>(camera_gimbal)), 
            add_noise(std::get<5>(camera_gimbal))
        );
        transform.transform.rotation.x = q.x();
        transform.transform.rotation.y = q.y();
        transform.transform.rotation.z = q.z();
        transform.transform.rotation.w = q.w();
        
        // 广播TF
        tf_broadcaster_->sendTransform(transform);


        // RCLCPP_INFO(this->get_logger(), "Published TF transform with noise");
    }

    void publish_armor_position() {
        
        // 获取当前时间
        rclcpp::Time current_time = this->now();
        
        // 创建10ms时间间隔
        rclcpp::Duration time_offset = 1000ms;
        
        // 计算调整后的时间
        rclcpp::Time adjusted_time = current_time - time_offset;

        geometry_msgs::msg::TransformStamped transform;
        try {
            // 尝试获取插值后的 TF 变换
            transform = tf_buffer_->lookupTransform(
                "camera_frame",   //目标坐标系
                "armor", //源坐标系
                adjusted_time,  //有时调用时间会快于tf发布时间
                std::chrono::duration<int>(1) 
            );
        } catch (const tf2::ExtrapolationException& ex) {
            // TF 数据不足，等待后续数据
            RCLCPP_WARN(this->get_logger(), "TF data not yet available: %s", ex.what());
            return;    
        } catch (const tf2::LookupException& ex) {
            // 坐标系不存在或超时
            RCLCPP_ERROR(this->get_logger(), "TF error: %s", ex.what());
            return;
        }

        auto msg = auto_aim_interfaces::msg::Armors();
        auto armor = auto_aim_interfaces::msg::Armor();
        msg.header.stamp = adjusted_time;
        msg.header.frame_id = "camera_frame";
        armor.pose.position.x = transform.transform.translation.x;
        armor.pose.position.y = transform.transform.translation.y;
        armor.pose.position.z = transform.transform.translation.z;
        armor.pose.orientation = transform.transform.rotation;
        msg.armors.push_back(armor);
        armor_pub_->publish(msg);

        visualization_msgs::msg::Marker armor_marker_;
        visualization_msgs::msg::MarkerArray marker_array_;
        armor_marker_.ns = "armors";
        armor_marker_.action = visualization_msgs::msg::Marker::ADD;
        armor_marker_.type = visualization_msgs::msg::Marker::CUBE;
        armor_marker_.scale.x = 0.05;
        armor_marker_.scale.z = 0.125;
        armor_marker_.color.a = 1.0;
        armor_marker_.color.g = 0.5;
        armor_marker_.color.b = 1.0;
        armor_marker_.lifetime = rclcpp::Duration::from_seconds(0.1);
        armor_marker_.header = msg.header;
        armor_marker_.id = 0;
        armor_marker_.id++;
        armor_marker_.scale.y = 0.135 ;
        armor_marker_.pose = armor.pose;
        marker_array_.markers.emplace_back(armor_marker_);
        marker_pub_->publish(marker_array_);
        // RCLCPP_INFO(this->get_logger(), "Published noisy armor position");
    }

    // 成员变量
    const std::tuple<double, double, double, double, double, double> camera_gimbal;
    const std::tuple<double, double, double, double, double, double> armor_world_;
    std::string armors_topic;
    std::string gimbal_frame;
    std::string world_frame;
    
    // 随机数生成器
    std::random_device rd_;
    std::mt19937 gen_;
    std::uniform_real_distribution<> dist_;
    
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;

    rclcpp::TimerBase::SharedPtr armor_pub_timer_;
    rclcpp::TimerBase::SharedPtr tf_pub_timer_;
    rclcpp::Publisher<auto_aim_interfaces::msg::Armors>::SharedPtr armor_pub_;

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GimbalController>());
    rclcpp::shutdown();
    return 0;
}


