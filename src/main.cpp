#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/core/core.hpp>
#include <sensor_msgs/msg/image.hpp>
#include "System.h"

class MonocularSlamNode : public rclcpp::Node
{
public:
    explicit MonocularSlamNode(ORB_SLAM2::System* slam) : Node("orbslam"), slam_(slam)
    {
        image_subscriber_ = this->create_subscription<ImageMsg>("camera", 10, std::bind(&MonocularSlamNode::GrabImage, this, std::placeholders::_1));
    }

    ~MonocularSlamNode() override
    {
        if (slam_ == nullptr) {
            return;
        }

        slam_->Shutdown();
        slam_->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    }

private:
    using ImageMsg = sensor_msgs::msg::Image;

    void GrabImage(const ImageMsg::SharedPtr msg)
    {
        cv_bridge::CvImagePtr cv_image;

        try {
            cv_image = cv_bridge::toCvCopy(msg);
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        const double timestamp = rclcpp::Time(msg->header.stamp).seconds();
        slam_->TrackMonocular(cv_image->image, timestamp);
    }

    ORB_SLAM2::System* slam_;
    rclcpp::Subscription<ImageMsg>::SharedPtr image_subscriber_;
};

int main(int argc, char** argv)
{
    if (argc < 3) {
        std::cerr << "\nUsage: ros2 run ros2_orbslam mono path_to_vocabulary path_to_settings" << std::endl;
        return 1;
    }

    rclcpp::init(argc, argv);

    const bool visualization = true;
    ORB_SLAM2::System slam(argv[1], argv[2], ORB_SLAM2::System::MONOCULAR, visualization);

    auto node = std::make_shared<MonocularSlamNode>(&slam);
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}