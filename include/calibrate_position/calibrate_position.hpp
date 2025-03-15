#pragma once

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "auto_aim_interfaces/msg/armor.hpp"
#include "auto_aim_interfaces/msg/armors.hpp"

#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>


namespace calibrate_position
{

class CalibratePosition : public rclcpp::Node
{
public:
    explicit CalibratePosition(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

    ~CalibratePosition() = default;

private:    
    void msgCallBack(const auto_aim_interfaces::msg::Armors& armors_msg);
    
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Subscription<auto_aim_interfaces::msg::Armors>::SharedPtr armors_sub_;

    std::vector<cv::Mat>
        r_gripper2base, t_gripper2base,//云台到世界坐标系的平移向量，旋转向量
        r_target2cam, t_target2cam;    //装甲板到相机坐标系的平移向量，旋转向量

        int time_space;     //采样时间间隔
        int date_length;    //采样数据数量
        std::string data_dir, armors_topic, 
        gimbal_frame,   //云台坐标系
        world_frame;    //世界坐标系

};
}

