#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <visualization_msgs/Marker.h>

#include "f110_pure_pursuit/csv_reader.h"
#include "f110_pure_pursuit/pure_pursuit.h"
#include "f110_pure_pursuit/types.h"


class PurePursuit
{
public:
    PurePursuit() :
        node_handle_(ros::NodeHandle()),
        pose_sub_(node_handle_.subscribe("gt_pose", 5, &PurePursuit::pose_callback, this)),
        drive_pub_(node_handle_.advertise<ackermann_msgs::AckermannDriveStamped>("nav", 5)),
        way_point_viz_pub_(node_handle_.advertise<visualization_msgs::Marker>("waypoint_markers", 100)),
        visualized_(false)
    {
        node_handle_.getParam("/lookahead_distance", lookahead_distance_);
        f110::CSVReader reader("/home/yash/yasht_ws/src/f110_pure_pursuit/sensor_data/gtpose.csv");
        way_point_data_ = reader.getData();
        ROS_INFO("Pure Pursuit Node Initialized");
        visualize_waypoint_data();
        ROS_INFO("Way Points Published as Markers.");
    }

    void visualize_waypoint_data()
    {
        const size_t increment = way_point_data_.size()/100;
        for(size_t i=0, j=0; i<way_point_data_.size(); i=i+increment, j++)
        {
            visualization_msgs::Marker way_point_marker;
            way_point_marker.header.frame_id = "map";
            way_point_marker.header.stamp = ros::Time();
            way_point_marker.ns = "pure_pursuit";
            way_point_marker.id = j;
            way_point_marker.type = visualization_msgs::Marker::SPHERE;
            way_point_marker.action = visualization_msgs::Marker::ADD;
            way_point_marker.pose.position.x = way_point_data_[i].x;
            way_point_marker.pose.position.y = way_point_data_[i].y;
            way_point_marker.pose.position.z = 0;
            way_point_marker.pose.orientation.x = 0.0;
            way_point_marker.pose.orientation.y = 0.0;
            way_point_marker.pose.orientation.z = 0.0;
            way_point_marker.pose.orientation.w = 1.0;
            way_point_marker.scale.x = 0.1;
            way_point_marker.scale.y = 0.1;
            way_point_marker.scale.z = 0.1;
            way_point_marker.color.a = 0.5;
            way_point_marker.color.r = 0.0;
            way_point_marker.color.g = 0.0;
            way_point_marker.color.b = 1.0;
            way_point_viz_pub_.publish(way_point_marker);
        }
    }

    ///
    /// @param pose_msg - Localized Pose of the Robot
    void pose_callback(const geometry_msgs::PoseStamped::ConstPtr &pose_msg)
    {
        // Convert pose_msg to WayPoint
        const auto current_way_point = f110::WayPoint(pose_msg);
        ROS_DEBUG("current_way_point x: %f", current_way_point.x);
        ROS_DEBUG("current_way_point y: %f", current_way_point.y);
        ROS_DEBUG("current_way_point heading: %f", current_way_point.heading);
        ROS_DEBUG("current_way_point speed: %f", current_way_point.speed);

        // Find the best waypoint to track (at lookahead distance)
        const auto goal_way_point = f110::get_best_track_point(current_way_point, way_point_data_, lookahead_distance_);
        ROS_DEBUG("goal_way_point x: %f", goal_way_point.x);
        ROS_DEBUG("goal_way_point y: %f", goal_way_point.y);
        ROS_DEBUG("goal_way_point heading: %f", goal_way_point.heading);
        ROS_DEBUG("goal_way_point speed: %f", goal_way_point.speed);

        // Calculate curvature/steering angle
        const double steering_angle = lookahead_distance_/(2*(std::abs(goal_way_point.y - current_way_point.y)));

        // Publish drive message
        ackermann_msgs::AckermannDriveStamped drive_msg;
        drive_msg.header.stamp = ros::Time::now();
        drive_msg.header.frame_id = "base_link";
        drive_msg.drive.speed = 1.0;

        // Thresholding for limiting the movement of car wheels to avoid servo locking
        if(steering_angle > 0.4)
        {
            drive_msg.drive.steering_angle = 0.4;
        }
        else if(steering_angle < -0.4)
        {
            drive_msg.drive.steering_angle = -0.4;
        }
    }

private:
    ros::NodeHandle node_handle_;
    ros::Subscriber pose_sub_;
    ros::Publisher way_point_viz_pub_;
    ros::Publisher drive_pub_;

    double lookahead_distance_;

    std::vector<f110::WayPoint> way_point_data_;

    bool visualized_;
};

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "pure_pursuit_node");
    PurePursuit pp;
    ros::spin();
    return 0;
}