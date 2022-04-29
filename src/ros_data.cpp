#include "ros_data.h"
#include <glog/logging.h>
#include "type.h"
#include <queue>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>
#include <thread>

extern std::mutex m_cameradata;
extern vector<geometry_msgs::Pose> tag_poses;
extern bool publish_flag;
extern vector<footstep> publish_steps;
extern priority_queue<rectPlane> publish_planes;
using namespace std;
get_ros_data::get_ros_data(ros::NodeHandle & node_):nd(node_)
{
  // sub_image = nd.subscribe("/camera/depth/image_rect_raw", 1, &get_ros_data::chatterCallback, this);
  // color_sub_ = new message_filters::Subscriber<sensor_msgs::Image>(nd, "/camera/color/image_raw", 1);
  // depth_sub_  = new message_filters::Subscriber<sensor_msgs::Image>(nd, "/camera/aligned_depth_to_color/image_raw", 1);
  // sync_ = new  message_filters::Synchronizer<slamSyncPolicy>(slamSyncPolicy(10), *color_sub_, *depth_sub_);
  // sync_->registerCallback(boost::bind(&get_ros_data::combineCallback,this, _1, _2));

  // sub_imu = nd.subscribe("/imu_topic", 1, &get_ros_data::IMUmessageCallback, this);
  sub_tag_poses = nd.subscribe("/tag_detections", 1, &get_ros_data::tag_callback, this);
  pub_hull_steps = nd.advertise<visualization_msgs::MarkerArray>("hull_step_marker_array", 1);
  pub_order = nd.advertise<visualization_msgs::MarkerArray>("order_marker_array", 1);
  pub_plane_outliner = nd.advertise<visualization_msgs::MarkerArray>("plane_outliner",1);
  // pub_color_image = nd.advertise<sensor_msgs::Image>("color_image", 1);
  pub_raw_pointcloud = nd.advertise<sensor_msgs::PointCloud2>("raw_cloud", 1);
  colors_ = {
      51/255.0, 160/255.0, 44/255.0,  //0
      166/255.0, 206/255.0, 227/255.0,
      178/255.0, 223/255.0, 138/255.0,//6
      31/255.0, 120/255.0, 180/255.0,
      251/255.0, 154/255.0, 153/255.0,// 12
      227/255.0, 26/255.0, 28/255.0,
      253/255.0, 191/255.0, 111/255.0,// 18
      106/255.0, 61/255.0, 154/255.0,
      255/255.0, 127/255.0, 0/255.0, // 24
      202/255.0, 178/255.0, 214/255.0,
      1.0, 0.0, 0.0, // red // 30
      0.0, 1.0, 0.0, // green
      0.0, 0.0, 1.0, // blue// 36
      1.0, 1.0, 0.0,
      1.0, 0.0, 1.0, // 42
      0.0, 1.0, 1.0,
      0.5, 1.0, 0.0,
      1.0, 0.5, 0.0,
      0.5, 0.0, 1.0,
      1.0, 0.0, 0.5,
      0.0, 0.5, 1.0,
      0.0, 1.0, 0.5,
      1.0, 0.5, 0.5,
      0.5, 1.0, 0.5,
      0.5, 0.5, 1.0,
      0.5, 0.5, 1.0,
      0.5, 1.0, 0.5,
      0.5, 0.5, 1.0};
}

void get_ros_data::tag_callback(const apriltag_ros::AprilTagDetectionArray & poses)
{
  tag_poses.clear();
  // cout<<"tag size is "<<poses.detections.size()<<endl;
  unique_lock<mutex> g(m_cameradata, std::defer_lock);
  g.lock();
  for (size_t i = 0; i < poses.detections.size(); i++)
  {
    // LOG(INFO)<<"get pose: "<<poses.detections[i].pose.pose.pose.position.x
    //                       <<poses.detections[i].pose.pose.pose.position.y
    //                       <<poses.detections[i].pose.pose.pose.position.z
    //                       <<poses.detections[i].pose.pose.pose.orientation.w
    //                       <<poses.detections[i].pose.pose.pose.orientation.x
    //                       <<poses.detections[i].pose.pose.pose.orientation.y
    //                       <<poses.detections[i].pose.pose.pose.orientation.z<<endl;
    tag_poses.emplace_back(poses.detections[i].pose.pose.pose);
  }
  g.unlock();
  if (publish_flag)
  {
    vector<footstep> publish_steps_now;
    priority_queue<rectPlane> publish_planes_now;
    publish_steps_now.clear();
    while (!publish_planes_now.empty())
    {
      publish_planes_now.pop();
    }
    
    unique_lock<mutex> g(m_cameradata, std::defer_lock);
    g.lock();
    publish_steps_now = publish_steps;
    publish_flag = false;
    publish_planes_now = publish_planes;
    g.unlock();
    cout<<"start to publish, the steps size is "<<publish_steps_now.size()<<endl;
    cout<<"plane size is "<<publish_planes_now.size()<<endl;
    publish_messages(publish_steps_now, publish_planes_now);
  }
}

vector<Eigen::Vector3d> computeStepMarker(footstep& f)
{
  vector<Eigen::Vector3d> return_v;
  vector<Eigen::Vector3d> v;
  v.reserve(4); return_v.reserve(4);
  Eigen::Vector3d v1(0.15, 0.05, 0.0);
  Eigen::Vector3d v2(0.15,  - 0.09, 0.0);
  Eigen::Vector3d v3(- 0.09, - 0.09, 0.0);
  Eigen::Vector3d v4(-0.09, 0.05, 0.0);
  v.emplace_back(v1);
  v.emplace_back(v2);
  v.emplace_back(v3);
  v.emplace_back(v4);
  Eigen::AngleAxisd r_v(f.theta, Eigen::Vector3d(0,0,1));
  for (auto & iter : v)
  {
    return_v.emplace_back(r_v.matrix()*iter + Eigen::Vector3d(f.x, f.y, f.z));
  }
  return return_v;
}

void get_ros_data::publishStepsAsMarkerArray(vector<footstep> & publish_steps)
{
  geometry_msgs::Point point;
  std_msgs::ColorRGBA point_color;
  visualization_msgs::MarkerArray ma;
  for (size_t i = 0; i < publish_steps.size (); i++)
  {
    // cout<<"step "<<i+1<<endl;
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();
    marker.ns = "step_" + std::to_string(i);
    marker.id = i;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.03;
    marker.scale.y = 0.03;
    marker.scale.z = 0.03;
    marker.color.a = 1.0;

    // const int nColor = i % (colors_.size()/3);
    const double r = colors_[/* nColor*3 +  */0]*255.0;
    const double g = colors_[/* nColor*3 +  */1]*255.0;
    const double b = colors_[/* nColor*3 +  */2]*255.0;

    marker.points.reserve(8);
    marker.colors.reserve(8);
    // 偏离12cm
    // 脚宽12cm
    // cout<<"compute points"<<endl;
    vector<Eigen::Vector3d> step = computeStepMarker(publish_steps.at(i));
    // cout<<"load points"<<endl;
    for (size_t j = 0; j < step.size(); j++)
    {
      point.x = step[j](0);
      point.y = step[j](1);
      point.z = step[j](2);
      point_color.r = r;
      point_color.g = g;
      point_color.b = b;
      point_color.a = 1.0;
      marker.colors.push_back(point_color);
      marker.points.push_back(point);

      point.x = step[(j+1)%step.size()](0);
      point.y = step[(j+1)%step.size()](1);
      point.z = step[(j+1)%step.size()](2);
      point_color.r = r;
      point_color.g = g;
      point_color.b = b;
      point_color.a = 1.0;
      marker.colors.push_back(point_color);
      marker.points.push_back(point);
    }
    // cout<<"ok"<<endl;
    marker.frame_locked = true;
    ma.markers.push_back(marker);
  }
  pub_hull_steps.publish(ma);
}

void get_ros_data::publishOrderMarkerArray(vector<footstep> & publish_steps)
{
  visualization_msgs::MarkerArray ma;
  for (size_t i = 0; i < publish_steps.size(); i++)
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();
    marker.ns = "order" + std::to_string(i);
    marker.id = i;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = publish_steps.at(i).x;
    marker.pose.position.y = publish_steps.at(i).y;
    marker.pose.position.z = publish_steps.at(i).z;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;
    marker.color.a = 1.0;

    marker.color.b = 0;
    marker.color.g = 0;
    marker.color.r = 1;
    marker.color.a = 1;

    ostringstream str;
    str<<i+1;
    marker.text=str.str();
    ma.markers.push_back(marker);
  }
  pub_hull_steps.publish(ma);
}

void get_ros_data::publishPlanesMarkerArray(priority_queue<rectPlane> & publish_planes)
{
  geometry_msgs::Point point;
  std_msgs::ColorRGBA point_color;
  visualization_msgs::MarkerArray ma;
  LOG(INFO)<<"publish planes: "<<endl;
  int index = 0;
  while (!publish_planes.empty())
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();
    marker.ns = "hull_" + std::to_string(index);
    marker.id = index;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.03;
    marker.scale.y = 0.03;
    marker.scale.z = 0.03;
    marker.color.a = 1.0;
    const double r = colors_[3]*255.0;
    const double g = colors_[4]*255.0;
    const double b = colors_[5]*255.0;
    marker.points.reserve(8);
    marker.colors.reserve(8);
    rectPlane tmpplane = publish_planes.top();
    publish_planes.pop();
    for (size_t j = 0; j < 4; j++)
    {
      point.x = tmpplane.corners[j](0);
      point.y = tmpplane.corners[j](1);
      point.z = tmpplane.corners[j](2);
      point_color.r = r;
      point_color.g = g;
      point_color.b = b;
      point_color.a = 1.0;
      marker.colors.push_back(point_color);
      marker.points.push_back(point);
      point.x = tmpplane.corners[(j+1)%4](0);
      point.y = tmpplane.corners[(j+1)%4](1);
      point.z = tmpplane.corners[(j+1)%4](2);
      point_color.r = r;
      point_color.g = g;
      point_color.b = b;
      point_color.a = 1.0;
      marker.colors.push_back(point_color);
      marker.points.push_back(point);
      marker.frame_locked = true;
    }
    ma.markers.push_back(marker);
  }
  pub_plane_outliner.publish(ma);
}

void get_ros_data::publish_messages(vector<footstep> & publish_steps, priority_queue<rectPlane> & publish_planes)
{
  for (size_t i = 0; i < publish_steps.size(); i++)
  {
    publishPlanesMarkerArray(publish_planes);
    vector<footstep> tmp_publish_steps(publish_steps.begin(), publish_steps.begin() + (i+1));
    publishStepsAsMarkerArray(tmp_publish_steps);
    publishOrderMarkerArray(tmp_publish_steps);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }
}