#ifndef _ROS_DATA_H_
#define _ROS_DATA_H_
#include <mutex>
#include <time.h>
#include <vector>
#include <queue>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/subscriber.h>
#include "cv_bridge/cv_bridge.h"
#include "type.h"
#include <apriltag_ros/AprilTagDetectionArray.h>
#define IMUDATASIZE 10
using namespace std;
class get_ros_data
{
private:
    ros::NodeHandle nd;
    // ros::Subscriber sub_image;// 暂时不用
    // ros::Subscriber sub_imu;
    ros::Subscriber sub_tag_poses;

    // typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,sensor_msgs::Image> slamSyncPolicy;
    // message_filters::Subscriber<sensor_msgs::Image>* color_sub_ ;             // topic1 输入
    // message_filters::Subscriber<sensor_msgs::Image>* depth_sub_;   // topic2 输入
    // message_filters::Synchronizer<slamSyncPolicy>* sync_;

    ros::Publisher pub_hull_steps;
    ros::Publisher pub_order;
    ros::Publisher pub_raw_pointcloud;
    ros::Publisher pub_plane_outliner;
    std::vector<double> colors_;
public:
    get_ros_data(ros::NodeHandle & node_);
    ~get_ros_data() = default;
    // void tag_callback(const ar_track_alvar_msgs::AlvarMarkers & poses);
    void tag_callback(const apriltag_ros::AprilTagDetectionArray & poses);
    
    void publish_messages(vector<footstep> & publish_steps, priority_queue<rectPlane> & publish_planes);
    void publishOrderMarkerArray(vector<footstep> & publish_steps);
    void publishStepsAsMarkerArray(vector<footstep> & publish_steps);
    void publishPlanesMarkerArray(priority_queue<rectPlane> & publish_planes);
};

#endif