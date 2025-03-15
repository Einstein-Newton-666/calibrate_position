#include<iostream>
#include <fstream>
#include "calibrate_position/calibrate_position.hpp"

namespace calibrate_position
{
CalibratePosition::CalibratePosition(const rclcpp::NodeOptions & options)
: Node("calibrate_position", options)
{   
    RCLCPP_INFO(this->get_logger(), "Starting calibrate position!");

    time_space = this->declare_parameter<int>("time_space", 100);
    date_length = this->declare_parameter<int>("date_length", 100);
    data_dir = this->declare_parameter<std::string>("data_dir", "../../../../src/calibrate_position/data.txt");
    armors_topic = this->declare_parameter<std::string>("armors_topic", "/detector/armors");
    gimbal_frame = this->declare_parameter<std::string>("gimbal_frame", "gimbal_link");
    world_frame = this->declare_parameter<std::string>("world_frame", "odom");

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // 订阅自定义消息
    armors_sub_ = this->create_subscription<auto_aim_interfaces::msg::Armors>(
        armors_topic, rclcpp::SensorDataQoS(),
        std::bind(&CalibratePosition::msgCallBack, this, std::placeholders::_1)
    );
}


void CalibratePosition::msgCallBack(const auto_aim_interfaces::msg::Armors& armors_msg){
    if(armors_msg.armors.size() != 1){
        RCLCPP_ERROR(this->get_logger(), "视野里有多个装甲板");
        rclcpp::sleep_for(std::chrono::duration<int>(time_space));
        return;
    }

    geometry_msgs::msg::TransformStamped transform;
    try {
        // 尝试获取插值后的 TF 变换
        transform = tf_buffer_->lookupTransform(
            gimbal_frame,   //目标坐标系
            world_frame,   //源坐标系
            armors_msg.header.stamp,
            std::chrono::duration<int>(1) 
        );
    } catch (const tf2::ExtrapolationException& ex) {
        // TF 数据不足，等待后续数据
        RCLCPP_WARN(this->get_logger(), "TF data not yet available: %s", ex.what());
        rclcpp::sleep_for(std::chrono::duration<int>(time_space));
        return;    
    } catch (const tf2::LookupException& ex) {
        // 坐标系不存在或超时
        RCLCPP_ERROR(this->get_logger(), "TF error: %s", ex.what());
        rclcpp::sleep_for(std::chrono::duration<int>(time_space));
        return;
    }

    tf2::Transform tf_transform;
    tf2::fromMsg(transform.transform, tf_transform);

    tf2::Quaternion q={
        armors_msg.armors[0].pose.orientation.x,
        armors_msg.armors[0].pose.orientation.y,
        armors_msg.armors[0].pose.orientation.z,
        armors_msg.armors[0].pose.orientation.w
    };
    tf2::Matrix3x3 tf_m;
    tf_m.setRotation(q);

    cv::Mat cv_m = cv::Mat::zeros(3, 3, CV_32FC1);
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            cv_m.at<float>(row, col) = tf_m[row][col];
        }
      }

    r_target2cam.push_back(cv_m);
    t_target2cam.emplace_back(
        armors_msg.armors[0].pose.position.x,
        armors_msg.armors[0].pose.position.y,
        armors_msg.armors[0].pose.position.z
    );


    q=tf_transform.getRotation();
    tf_m.setRotation(q);
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            cv_m.at<float>(row, col) = tf_m[row][col];
        }
      }
    
    r_gripper2base.push_back(cv_m);
    t_gripper2base.emplace_back(
        tf_transform.getOrigin()[0],
        tf_transform.getOrigin()[0],
        tf_transform.getOrigin()[0]
    );

    if(t_gripper2base.size() == date_length){
        cv::Mat t_cam2gripper, r_cam2gripper;
        calibrateHandEye(r_target2cam, t_gripper2base, r_target2cam, t_target2cam, 
            t_cam2gripper, r_cam2gripper, cv::CALIB_HAND_EYE_HORAUD);
        // CALIB_HAND_EYE_TSAI	Tsai 方法 (1989)	经典方法，分离旋转和平移，对数据质量敏感。
        // CALIB_HAND_EYE_PARK	Park 方法 (1994)	改进的闭式解，适用于噪声较小的场景。
        // CALIB_HAND_EYE_HORAUD	Horaud 方法 (1995)	基于四元数的全局优化，对噪声鲁棒性较好。
        // CALIB_HAND_EYE_ANDREFF	Andreff 方法 (1999)	线性代数方法，要求运动轨迹包含充分旋转。
        // CALIB_HAND_EYE_DANIILIDIS  Daniilidis 方法 (1999)	基于几何约束，适用于 Eye-in-Hand 和 Eye-to-Hand 两种配置。

        double pitch, yaw, roll;
        if (r_cam2gripper.at<double>(2, 0) != 1 && r_cam2gripper.at<double>(2, 0) != -1) {
            pitch = -asin(r_cam2gripper.at<double>(2, 0));
            roll  = atan2(r_cam2gripper.at<double>(2, 1), r_cam2gripper.at<double>(2, 2));
            yaw   = atan2(r_cam2gripper.at<double>(1, 0), r_cam2gripper.at<double>(0, 0));
        } else {
            //万向锁
            yaw = 0;
            if (r_cam2gripper.at<double>(2, 0) == -1) {
                pitch = CV_PI / 2;
                roll  = yaw + atan2(r_cam2gripper.at<double>(0, 1), r_cam2gripper.at<double>(0, 2));
            } else {
                pitch = -CV_PI / 2;
                roll  = -yaw + atan2(-r_cam2gripper.at<double>(0, 1), -r_cam2gripper.at<double>(0, 2));
            }
        }

        std::ofstream file(data_dir);
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open file!");
        }else{
            file << "平移向量" << t_cam2gripper << std::endl;
            file << "旋转欧拉角" << "pitch:" << pitch << "yaw:" << yaw << "roll:" << roll << std::endl;
            file.close();
        }
        RCLCPP_INFO(this->get_logger(), "标定结束");
        rclcpp::shutdown();
    }

    // r_gripper2base
    // t_gripper2base
    // r_target2cam
    // t_target2cam
    rclcpp::sleep_for(std::chrono::duration<int>(time_space));
}

}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<calibrate_position::CalibratePosition>());
    rclcpp::shutdown();
    return 0;
  }
  