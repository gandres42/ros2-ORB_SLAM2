#ifndef MONOCULAR_SLAM_NODE_HPP_
#define MONOCULAR_SLAM_NODE_HPP_


#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

#include <cv_bridge/cv_bridge.hpp>

#include "System.h"


class MonocularSlamNode : public rclcpp::Node
{
public:
    MonocularSlamNode(ORB_SLAM2::System* pSLAM);

    ~MonocularSlamNode();

private:
    using ImageMsg = sensor_msgs::msg::Image;

    void GrabImage(const ImageMsg::SharedPtr msg);

    ORB_SLAM2::System* m_SLAM;

    rclcpp::Subscription<ImageMsg>::SharedPtr m_image_subscriber;
};

#endif  // MONOCULAR_SLAM_NODE_HPP_
