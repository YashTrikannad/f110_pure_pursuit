//
// Created by yash on 10/20/19.
//

#pragma once

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <vector>

namespace f110
{

std::vector<WayPoint> transform(const std::vector<WayPoint>& reference_way_points, const WayPoint& current_way_point,
        const tf2_ros::Buffer& tfBuffer, const tf2_ros::TransformListener& tf2_listener)
{
    geometry_msgs::TransformStamped map_to_laser;
    map_to_laser = tfBuffer.lookupTransform("laser", "map", ros::Time(0));

    std::vector<WayPoint> transformed_way_points;
    for(const auto& reference_way_point: reference_way_points)
    {
        geometry_msgs::Pose map_way_point;
        map_way_point.position.x = reference_way_point.x;
        map_way_point.position.y = reference_way_point.y;
        map_way_point.position.z = 0;
        const auto quat = tf::createQuaternionFromYaw(reference_way_point.heading);
        map_way_point.orientation.x = quat.getX();
        map_way_point.orientation.y = quat.getY();
        map_way_point.orientation.z = quat.getZ();
        map_way_point.orientation.w = quat.getW();

        tf2::doTransform(map_way_point, map_way_point, map_to_laser);

        transformed_way_points.emplace_back(map_way_point);
    }
    return transformed_way_points;
}


///
/// @param current_way_point
/// @return
WayPoint get_best_track_point(const std::vector<WayPoint>& way_point_data,
        double lookahead_distance)
{
    size_t closest_way_point_index = 0;
    double closest_distance = std::numeric_limits<double>::max();
    for(size_t i=0; i <way_point_data.size(); ++i)
    {
        if(way_point_data[i].x < 0) continue;
        double distance = way_point_data[i].x*way_point_data[i].x + way_point_data[i].y*way_point_data[i].y;
        double lookahead_diff = std::abs(distance - lookahead_distance);
        if(lookahead_diff < closest_distance)
        {
            closest_distance = lookahead_diff;
            closest_way_point_index = i;
        }
    }
    ROS_WARN("closest_way_point_index %i", static_cast<int>(closest_way_point_index));
    return way_point_data[closest_way_point_index];
}

} // namespace f110

