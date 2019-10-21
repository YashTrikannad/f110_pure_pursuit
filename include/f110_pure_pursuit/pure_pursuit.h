//
// Created by yash on 10/20/19.
//

#pragma once

#include <vector>

namespace f110
{

std::vector<WayPoint> transform(const std::vector<WayPoint>& reference_way_points, const WayPoint& current_way_point)
{
    std::vector<WayPoint> transformed_way_points;
    for(const auto& reference_way_point: reference_way_points)
    {
        WayPoint transformed_way_point{};
        transformed_way_point.x = reference_way_point.x - current_way_point.x;
        transformed_way_point.y = reference_way_point.y - current_way_point.y;
        transformed_way_point.speed = reference_way_point.speed;
        transformed_way_points.emplace_back(transformed_way_point);
    }
    return transformed_way_points;
}


///
/// @param current_way_point
/// @return
WayPoint get_best_track_point(const WayPoint &current_way_point, const std::vector<WayPoint>& way_point_data,
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
    return way_point_data[closest_way_point_index];
}

} // namespace f110

