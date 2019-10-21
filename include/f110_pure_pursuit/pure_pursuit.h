//
// Created by yash on 10/20/19.
//

#pragma once

#include <vector>

namespace f110
{

/// Get the l2 distance between the two waypoints
/// @param way_point1
/// @param way_point2
/// @return
double waypoint_distance(const WayPoint& way_point1, const WayPoint& way_point2)
{
    const double diff_x = way_point1.x - way_point2.x;
    const double diff_y = way_point1.y - way_point2.y;
    return sqrt((diff_x*diff_x) + (diff_y*diff_y));
}

///
/// @param current_way_point
/// @return
WayPoint get_best_track_point(const WayPoint &current_way_point, const std::vector<WayPoint>& way_point_data,
        double lookahead_distance, double last_best = 0)
{
    size_t closest_way_point_index = 0;
    double closest_distance = 0.0;
    for(size_t i=0; i <way_point_data.size(); ++i)
    {
        double distance = waypoint_distance(current_way_point, way_point_data[i]);
        double lookahead_diff = std::abs(distance - lookahead_distance);
        if(lookahead_diff < closest_distance)
        {
            closest_distance = lookahead_diff;
            closest_way_point_index = i;
        }
    }
    return way_point_data[closest_way_point_index];
}

} // namespace f110

