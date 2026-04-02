#include "monocular-slam-node.hpp"

#include <opencv2/core/core.hpp>

MonocularSlamNode::MonocularSlamNode(ORB_SLAM2::System* pSLAM)
:   Node("orbslam"), 
    m_SLAM(pSLAM)
{
    m_image_subscriber = this->create_subscription<ImageMsg>(
        "camera",
        10,
        std::bind(&MonocularSlamNode::GrabImage, this, std::placeholders::_1));
}

MonocularSlamNode::~MonocularSlamNode()
{
    if (m_SLAM == nullptr) {
        return;
    }

    // Stop all threads
    m_SLAM->Shutdown();

    // Save camera trajectory
    m_SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
}

void MonocularSlamNode::GrabImage(const ImageMsg::SharedPtr msg)
{
    cv_bridge::CvImagePtr cv_image;

    // Copy the ROS image message to cv::Mat.
    try
    {
        cv_image = cv_bridge::toCvCopy(msg);
    }
    catch (const cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    const double timestamp = rclcpp::Time(msg->header.stamp).seconds();
    m_SLAM->TrackMonocular(cv_image->image, timestamp);
}
