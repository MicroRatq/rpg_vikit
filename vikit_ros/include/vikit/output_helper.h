/*
 * output_helper.h
 *
 *  Created on: Jan 20, 2013
 *  Updated for ROS2: 2024
 *      Author: chrigi
 */

#ifndef OUTPUT_HELPER_H_
#define OUTPUT_HELPER_H_

#include <string>
#include <Eigen/Core>
#include <sophus/se3.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/msg/marker.hpp>

namespace vk {
namespace output_helper {

using namespace std;
using namespace Eigen;

void
publishTfTransform      (const Sophus::SE3& T, const rclcpp::Time& stamp,
                         const string& frame_id, const string& child_frame_id,
                         tf2_ros::TransformBroadcaster& br);

void
publishPointMarker      (rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub,
                         const Vector3d& pos,
                         const string& ns,
                         const rclcpp::Time& timestamp,
                         int id,
                         int action,
                         double marker_scale,
                         const Vector3d& color,
                         rclcpp::Duration lifetime = rclcpp::Duration(0, 0));

void
publishLineMarker       (rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub,
                         const Vector3d& start,
                         const Vector3d& end,
                         const string& ns,
                         const rclcpp::Time& timestamp,
                         int id,
                         int action,
                         double marker_scale,
                         const Vector3d& color,
                         rclcpp::Duration lifetime = rclcpp::Duration(0, 0));

void
publishArrowMarker      (rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub,
                         const Vector3d& pos,
                         const Vector3d& dir,
                         double scale,
                         const string& ns,
                         const rclcpp::Time& timestamp,
                         int id,
                         int action,
                         double marker_scale,
                         const Vector3d& color);

void
publishHexacopterMarker (rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub,
                         const string& frame_id,
                         const string& ns,
                         const rclcpp::Time& timestamp,
                         int id,
                         int action,
                         double marker_scale,
                         const Vector3d& color);

void
publishCameraMarker(rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub,
                    const string& frame_id,
                    const string& ns,
                    const rclcpp::Time& timestamp,
                    int id,
                    double marker_scale,
                    const Vector3d& color);

void
publishFrameMarker     (rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub,
                        const Matrix3d& rot,
                        const Vector3d& pos,
                        const string& ns,
                        const rclcpp::Time& timestamp,
                        int id,
                        int action,
                        double marker_scale,
                        rclcpp::Duration lifetime = rclcpp::Duration(0, 0));

} // namespace output_helper
} // namespace vk

#endif // OUTPUT_HELPER_H_
