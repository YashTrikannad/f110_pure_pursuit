#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <ackermann_msgs/AckermannDriveStamped.h>

#include "csv_reader.h"

class PurePursuit {
public:
    PurePursuit() :
        node_handle_(ros::NodeHandle()),
        pose_sub_(node_handle_.subscribe("scan", 5, &PurePursuit::pose_callback, this)),
        drive_pub_(node_handle_.advertise<ackermann_msgs::AckermannDriveStamped>("nav", 5))
    {
        CSVReader reader("wp-2019-10-21-00-46-17.csv");
        waypoint_data_ = reader.getData();
    }

    void pose_callback(const geometry_msgs::PoseStamped::ConstPtr &pose_msg) {
        // TODO: find the current waypoint to track using methods mentioned in lecture

        // TODO: transform goal point to vehicle frame of reference

        // TODO: calculate curvature/steering angle

        // TODO: publish drive message, don't forget to limit the steering angle between -0.4189 and 0.4189 radians
    }

private:
    std::vector<std::vector<double>> waypoint_data_;
    ros::NodeHandle node_handle_;
    ros::Subscriber pose_sub_;
    ros::Subscriber true_pose_;
    ros::Publisher drive_pub_;
};

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "pure_pursuit_node");
    PurePursuit pp;
    ros::spin();
    return 0;
}