//
// Created by yash on 10/20/19.
//

#pragma once

#include <tf/tf.h>
#include <geometry_msgs/PoseStamped.h>

namespace f110
{

struct WayPoint
{
    WayPoint() = default;
    WayPoint(const geometry_msgs::PoseStamped::ConstPtr &pose_msg, double current_speed = 1.0)
    {
        x = pose_msg->pose.position.x;
        y = pose_msg->pose.position.y;
        speed = current_speed;
    }

    friend std::ostream& operator << (std::ostream& os, const WayPoint& way_point);

    std::string GetString() const
    {
        std::stringstream stream;
        stream << "x: " << x << "\ty: " << y << "\tspeed: " <<  speed << "\n";
    }

    double x;
    double y;
    double speed;
};

std::ostream& operator << (std::ostream& os, const WayPoint& way_point)
{
    return os << "x: " << way_point.x << "\ty: " << way_point.y << "\ty: " << std::endl;
}

}

