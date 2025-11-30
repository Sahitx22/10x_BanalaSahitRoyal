#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <path_smoothing_ros/Trajectory.h>
#include <tf/tf.h>
#include <cmath>

path_smoothing_ros::Trajectory traj;
bool traj_received = false;

double lookahead_min = 0.3;  // meters
double lookahead_gain = 0.8; // dynamic lookahead based on speed

ros::Publisher cmd_pub;

void trajectoryCallback(const path_smoothing_ros::Trajectory::ConstPtr& msg)
{
    traj = *msg;
    traj_received = true;
    ROS_INFO("Trajectory received with %ld points", traj.points.size());
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    if (!traj_received || traj.points.empty())
        return;

    // Robot pose
    double rx = msg->pose.pose.position.x;
    double ry = msg->pose.pose.position.y;
    double yaw = tf::getYaw(msg->pose.pose.orientation);

    // Robot speed
    double v_current = msg->twist.twist.linear.x;

    // Dynamic lookahead distance
    double Ld = std::max(lookahead_min, lookahead_gain * fabs(v_current));

    // Select tracking point
    int idx = 0;
    double best_dist = 1e9;

    for (int i = 0; i < traj.points.size(); i++)
    {
        double dx = traj.points[i].x - rx;
        double dy = traj.points[i].y - ry;
        double dist = hypot(dx, dy);

        if (dist >= Ld)
        {
            idx = i;
            break;
        }

        if (dist < best_dist)
        {
            best_dist = dist;
            idx = i;
        }
    }

    // Lookahead target
    double tx = traj.points[idx].x;
    double ty = traj.points[idx].y;

    // Geometry for Pure Pursuit
    double dx = tx - rx;
    double dy = ty - ry;

    // Convert to local robot frame
    double local_x =  cos(yaw)*dx + sin(yaw)*dy;
    double local_y = -sin(yaw)*dx + cos(yaw)*dy;

    // Avoid division issues
    if (local_x < 0.001)
        local_x = 0.001;

    // Pure pursuit curvature
    double curvature = 2 * local_y / (local_x*local_x + local_y*local_y);

    // Compute angular velocity
    double angular_z = curvature * traj.points[idx].v;

    // Longitudinal control: follow desired speed
    double v_des = traj.points[idx].v;

    geometry_msgs::Twist cmd;
    cmd.linear.x = v_des;
    cmd.angular.z = angular_z;

    cmd_pub.publish(cmd);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "trajectory_follower");
    ros::NodeHandle nh;

    ros::Subscriber traj_sub = nh.subscribe("/trajectory", 1, trajectoryCallback);
    ros::Subscriber odom_sub = nh.subscribe("/odom", 1, odomCallback);

    cmd_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    ros::spin();
    return 0;
}
