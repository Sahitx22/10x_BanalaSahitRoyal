#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <path_smoothing_ros/Trajectory.h>
#include <path_smoothing_ros/TrajectoryPoint.h>
#include <cmath>

double v_max = 1.0;
double a_lat_max = 1.5;

double computeCurvature(const geometry_msgs::Pose& p0,
                        const geometry_msgs::Pose& p1,
                        const geometry_msgs::Pose& p2)
{
    double x1 = p0.position.x, y1 = p0.position.y;
    double x2 = p1.position.x, y2 = p1.position.y;
    double x3 = p2.position.x, y3 = p2.position.y;

    double a = hypot(x2 - x1, y2 - y1);
    double b = hypot(x3 - x2, y3 - y2);
    double c = hypot(x3 - x1, y3 - y1);

    double area = fabs((x2-x1)*(y3-y1) - (y2-y1)*(x3-x1)) / 2.0;
    if (a * b * c == 0)
        return 0.0;

    return 4.0 * area / (a * b * c);
}

ros::Publisher traj_pub;

void pathCallback(const nav_msgs::Path::ConstPtr& msg)
{
    const auto& pts = msg->poses;
    if (pts.size() < 3) return;

    path_smoothing_ros::Trajectory traj_msg;
    traj_msg.header.stamp = ros::Time::now();
    traj_msg.header.frame_id = "map";

    double t = 0.0;

    for (size_t i = 1; i < pts.size() - 1; i++)
    {
        const auto& p0 = pts[i-1].pose;
        const auto& p1 = pts[i].pose;
        const auto& p2 = pts[i+1].pose;

        double dx = p1.position.x - p0.position.x;
        double dy = p1.position.y - p0.position.y;
        double ds = hypot(dx, dy);

        double kappa = computeCurvature(p0, p1, p2);

        double v_turn = sqrt(a_lat_max / (fabs(kappa) + 1e-6));
        double v = std::min(v_max, v_turn);

        t += ds / v;

        path_smoothing_ros::TrajectoryPoint tp;
        tp.x = p1.position.x;
        tp.y = p1.position.y;
        tp.t = t;
        tp.v = v;

        traj_msg.points.push_back(tp);
    }

    traj_pub.publish(traj_msg);
    ROS_INFO("Published trajectory with %lu points", traj_msg.points.size());
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "trajectory_generator");
    ros::NodeHandle nh;

    traj_pub = nh.advertise<path_smoothing_ros::Trajectory>("/trajectory", 1);

    ros::Subscriber sub = nh.subscribe("/path_smoothing_demo/smoothed_path", 1, pathCallback);

    ros::spin();
}

