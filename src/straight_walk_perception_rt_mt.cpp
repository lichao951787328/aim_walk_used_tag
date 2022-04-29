#include "ros/ros.h"
#include <fstream>
#include <iostream>             // std::cout
#include <future>               // std::async, std::future
#include <chrono>               // std::chrono::milliseconds
#include "tcpip_port.h"
#include "ros_data.h"
#include <thread>
#include <queue>
#include <glog/logging.h>
#include <geometry_msgs/Pose.h>
#include <conio.h>
#include <unistd.h>
#include <Eigen/Geometry>
// 记录相机深度数据和imu数据
std::mutex m_cameradata, m_tcpip;
vector<geometry_msgs::Pose> tag_poses;
int sock_fd,client_fd;
bool publish_flag = false;
vector<footstep> publish_steps;
priority_queue<rectPlane> publish_planes;

auto getT(const double &px, const double &py, const double &pz, const double &rx, const double &ry, const double &rz)
{
  using namespace Eigen;
  Matrix4d res;
  res.setIdentity();
  res.block<3,3>(0,0) = (AngleAxisd(rz, Vector3d::UnitZ())*AngleAxisd(ry, Vector3d::UnitY())*AngleAxisd(rx, Vector3d::UnitX())).matrix();
  res(0,3) = px;
  res(1,3) = py;
  res(2,3) = pz;
  return res;
}

auto _deg2rad(double degree)
{
  double rad = degree/57.3;
  return rad;
}

void communicate()
{
    LOG(INFO)<<"enter communicate function"<<endl;
    std::ofstream fs_communicate;
    int index = 0;
    char buff[250];
    getcwd(buff, 250);
    string current_working_directory(buff);
    LOG(INFO)<<"current directory is "<<current_working_directory<<endl;
    fs_communicate.open(current_working_directory + "/communicate.txt");
    fs_communicate.clear();
    char currentFlag = 'A';
    char lastFlag = 'A';
    clock_t start, finish;
    double duration;
    tcpip_port com;
    com.initial();
    LOG(INFO)<<"tcp ip port initial finish"<<endl;
    while (1)
    {
        com.accept_client();
        if (com.recvData() == -1)
        {
            printf("recv in server error.");
            exit(1);
        }
        LOG(INFO)<<"recev data"<<endl;
        // cout<<"have recev data"<<endl;
        com.analysisBuf(); 
        // perception.publishWindata(0.0, 0.0);
        lastFlag = currentFlag;
        currentFlag = com.getRecvFlag();
        // std::cout<<"current letter is "<<currentFlag<<std::endl;
        // com.resetRecvFlag();
        // std::cout<<"the com recv flag is change to "<<com.getRecvFlag()<<std::endl;
        if (lastFlag == 'A' && currentFlag == 'B')
        {

            LOG(INFO)<<"GOOOOOOOOOOO"<<endl;
            // sleep(10);
            // 解析 数据，给ubuntu perception
            // 解析wintime数据
#ifdef REALTIME
            time_self win_time(com.recevData.secs, com.recevData.nsecs);
#else
            time_self win_time(ros::Time::now());
#endif
            time_self camera_time(ros::Time::now());//获取相机时间
            LOG(INFO)<<"win time: "<<win_time.secs<<" "<<win_time.usecs<<endl;
            LOG(INFO)<<"camera time: "<<camera_time.secs<<" "<<camera_time.usecs<<endl;
            // 只考虑非实时情况
            std::cout<<"need to detection plane."<<std::endl;
            fs_communicate<<"perception "<<++index<<" : "<<endl;
            start = clock();

            double BaseVisionX, BaseVisionY, BaseVisionZ, base_x, base_y, base_height,BaseVisionPitchDeg;
            double base_posture[3];
            BaseVisionZ = +1.1258e+00-+7.1365e-01+0.18938;
            // 对于新的工装
            BaseVisionZ -= 0.06435;
            BaseVisionX = 0.15995;
            // 对于新的工装
            BaseVisionX -= 0.00298;
            BaseVisionY = 0.0;
            BaseVisionPitchDeg = 27.5;

            base_x = com.recevData.base_x;
            base_y = com.recevData.base_y;
            base_height = com.recevData.base_height;
            base_posture[0] = com.recevData.pose_roll;
            base_posture[1] = com.recevData.pose_pitch;
            base_posture[2] = com.recevData.pose_yaw;

            Eigen::Matrix<double,4,4> World_T_Base, Base_T_Vision, VisionL515_T_VisionD435i, VisionD435i_T_Target, World_T_Tar;
            Base_T_Vision = getT(BaseVisionX, BaseVisionY, BaseVisionZ, 0, 0, 0);
            Eigen::Matrix3d Base_R_VisionTemp;
            Base_R_VisionTemp << 0,-1,0, -1,0,0, 0,0,-1;
            // Base_R_VisionTemp << 0,-1,0, 1,0,0, 0,0,-1;
            // cout<<"Base_R_VisionTemp = "<<endl<<Base_R_VisionTemp<<endl;
            Base_T_Vision.block<3,3>(0,0) = Base_R_VisionTemp*(Eigen::AngleAxisd(_deg2rad(BaseVisionPitchDeg),Eigen::Vector3d::UnitX())).matrix();
            World_T_Base = getT(base_x, base_y, base_height, base_posture[0], base_posture[1], base_posture[2]);
            VisionL515_T_VisionD435i = getT(-0.0325, 0.01865, 0.00575, 0, 0, 0);
            geometry_msgs::Pose p;
            unique_lock<mutex> g_(m_cameradata, std::defer_lock);
            g_.lock();
            p = tag_poses.front();
            g_.unlock();
            LOG(INFO)<<"tag pose is: "<<p.position.x<<" "<<p.position.y<<" "<<p.position.z<<" "<<p.orientation.w<<" "<<p.orientation.x<<" "<<p.orientation.y<<" "<<p.orientation.z<<endl;
            Eigen::Quaterniond q(p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z);
            Eigen::Vector3d euler_angles = q.toRotationMatrix().eulerAngles(2, 1, 0);
            LOG(INFO)<<"euler angle: "<<euler_angles.transpose()<<endl;
            VisionD435i_T_Target.setIdentity();
            VisionD435i_T_Target(0, 3) = p.position.x;
            VisionD435i_T_Target(1, 3) = p.position.y;
            VisionD435i_T_Target(2, 3) = p.position.z;
            // Eigen::Quaterniond q(p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z);
            VisionD435i_T_Target.block<3,3>(0,0) = q.toRotationMatrix();
            World_T_Tar = World_T_Base*Base_T_Vision*VisionL515_T_VisionD435i*VisionD435i_T_Target;

            // 假设为正确的x方向
            Eigen::Vector4d x_d;
            x_d.head(3) = Eigen::Vector3d::UnitY();
            x_d(3) = 1.0;
            Eigen::Vector3d direct = (World_T_Tar * x_d).head(3);
            Eigen::Vector2d direct_2d = direct.head(2).normalized();
            Eigen::Vector2d goal = World_T_Tar.block<2, 1>(0, 3);
            LOG(INFO)<<"goal position: "<<goal.transpose()<<endl;
            double dis_tag = 0.1;// 此为粘贴时测量
            Eigen::Vector2d walk_goal = goal - dis_tag * direct_2d;
            // 至此，便得到了方向和目标点

            double dis = abs(walk_goal.dot(direct_2d));
            double goal_dis = dis - 0.17;//前脚长15cm + 1cm阈值
            LOG(INFO)<<"aim direct "<<direct_2d.transpose()<<endl;
            LOG(INFO)<<"goal distance : "<<goal_dis<<endl;
            double theta = acos(Eigen::Vector2d::UnitX().dot(direct_2d));
            CHECK_GE(theta, 0.0)<<"theta is > 0"<<endl;
            LOG(ERROR)<<"THETA : "<<theta<<endl;
            struct line_step
            {
            double x, y, theta;
            };
            vector<footstep> steps_result;
            if (direct_2d(1) < 0)//右转
            {
                LOG(INFO)<<"TURN RIGHT ..."<<endl;
                int num_foot_len = (int)(goal_dis/0.25 + 0.8);
                int num_foot_angle = (int)(abs(theta)/(6/57.3) + 0.8);
                int num_foot = max(num_foot_len, num_foot_angle);
                double length_step = goal_dis / num_foot;
                double theta_step = theta / num_foot;
                LOG(INFO)<<"step length "<<length_step<<endl;
                LOG(INFO)<<"step angle "<<theta_step<<endl;
                vector<line_step> line_steps;
                line_steps.reserve(num_foot);
                for (size_t i = 0; i < num_foot; i++)
                {
                    Eigen::Vector2d line_cor = length_step *(i+1) *direct_2d;
                    double tmptheta = (i+1) * theta_step;
                    line_step tmp_line_step;
                    tmp_line_step.x = line_cor(0);
                    tmp_line_step.y = line_cor(1);
                    tmp_line_step.theta = tmptheta;
                    line_steps.emplace_back(tmp_line_step);
                }
                LOG(INFO)<<"line steps :"<<endl;
                for (auto & iter_line_step : line_steps)
                {
                    LOG(INFO)<<iter_line_step.x<<" "<<iter_line_step.y<<" "<<iter_line_step.theta<<endl;
                }
                for (auto & iter_line_step : line_steps)
                {
                    Eigen::Vector3d t(iter_line_step.x, iter_line_step.y, 0.0);
                    Eigen::Vector3d left_foot_cor(0.0, 0.08, 0.0);
                    Eigen::Vector3d right_foot_cor(0.0, -0.08, 0.0);
                    Eigen::AngleAxisd rotate_vector( - iter_line_step.theta, Eigen::Vector3d::UnitZ());
                    Eigen::Vector3d left_foot_cor_ro = rotate_vector.toRotationMatrix() * left_foot_cor + t;
                    Eigen::Vector3d right_foot_cor_ro = rotate_vector.toRotationMatrix() * right_foot_cor + t;
                    footstep tmpstep1;
                    tmpstep1.is_left = false;
                    tmpstep1.x = right_foot_cor_ro(0);
                    tmpstep1.y = right_foot_cor_ro(1);
                    tmpstep1.z = right_foot_cor_ro(2);
                    tmpstep1.theta = - iter_line_step.theta;
                    steps_result.emplace_back(tmpstep1);
                    footstep tmpstep2;
                    tmpstep2.is_left = true;
                    tmpstep2.x = left_foot_cor_ro(0);
                    tmpstep2.y = left_foot_cor_ro(1);
                    tmpstep2.z = left_foot_cor_ro(2);
                    tmpstep2.theta = - iter_line_step.theta;
                    steps_result.emplace_back(tmpstep2);
                }
                LOG(INFO)<<"foot step:"<<endl;
                for (auto & iter_footstep : steps_result)
                {
                    LOG(INFO)<<iter_footstep.is_left<<" "<<iter_footstep.x<<" "<<iter_footstep.y<<" "<<iter_footstep.z<<" "<<iter_footstep.theta<<endl;
                }
            }
            else//左转
            {
                LOG(INFO)<<"TURN LEFT ..."<<endl;
                int num_foot_len = (int)(goal_dis/0.25 + 0.8);
                int num_foot_angle = (int)(abs(theta)/(6/57.3) + 0.8);
                int num_foot = max(num_foot_len, num_foot_angle);
                double length_step = goal_dis / num_foot;
                double theta_step = theta / num_foot;
                LOG(INFO)<<"step length "<<length_step<<endl;
                LOG(INFO)<<"step angle "<<theta_step<<endl;
                vector<line_step> line_steps;
                line_steps.reserve(num_foot);
                steps_result.reserve(num_foot * 2);
                for (size_t i = 0; i < num_foot; i++)
                {
                    Eigen::Vector2d line_cor = length_step *(i+1) *direct_2d;
                    double tmptheta = (i+1) * theta_step;
                    line_step tmp_line_step;
                    tmp_line_step.x = line_cor(0);
                    tmp_line_step.y = line_cor(1);
                    tmp_line_step.theta = tmptheta;
                    line_steps.emplace_back(tmp_line_step);
                }
                for (auto & iter_line_step : line_steps)
                {
                    LOG(INFO)<<iter_line_step.x<<" "<<iter_line_step.y<<" "<<iter_line_step.theta<<endl;
                }
                for (auto & iter_line_step : line_steps)
                {
                    Eigen::Vector3d t(iter_line_step.x, iter_line_step.y, 0.0);
                    Eigen::Vector3d left_foot_cor(0.0, 0.08, 0.0);
                    Eigen::Vector3d right_foot_cor(0.0, -0.08, 0.0);
                    Eigen::AngleAxisd rotate_vector(iter_line_step.theta, Eigen::Vector3d::UnitZ());
                    Eigen::Vector3d left_foot_cor_ro = rotate_vector.toRotationMatrix() * left_foot_cor + t;
                    Eigen::Vector3d right_foot_cor_ro = rotate_vector.toRotationMatrix() * right_foot_cor + t;
                    footstep tmpstep1;
                    tmpstep1.is_left = true;
                    tmpstep1.x = left_foot_cor_ro(0);
                    tmpstep1.y = left_foot_cor_ro(1);
                    tmpstep1.z = left_foot_cor_ro(2);
                    tmpstep1.theta = iter_line_step.theta;
                    steps_result.emplace_back(tmpstep1);
                    footstep tmpstep2;
                    tmpstep2.is_left = false;
                    tmpstep2.x = right_foot_cor_ro(0);
                    tmpstep2.y = right_foot_cor_ro(1);
                    tmpstep2.z = right_foot_cor_ro(2);
                    tmpstep2.theta = iter_line_step.theta;
                    steps_result.emplace_back(tmpstep2);
                }
                LOG(INFO)<<"foot step:"<<endl;
                for (auto & iter_footstep : steps_result)
                {
                    LOG(INFO)<<iter_footstep.is_left<<" "<<iter_footstep.x<<" "<<iter_footstep.y<<" "<<iter_footstep.z<<" "<<iter_footstep.theta<<endl;
                }
            }
            bool return_flag = steps_result.size() > 0;
            finish = clock();
            duration = (double)(finish - start) / CLOCKS_PER_SEC;
            cout<<"plane detection and step planning cost "<<duration<<endl;

            if (!return_flag)
            {
                fs_communicate<<"perception wrong, send an empty data to win. "<<endl;
                int buf_size = sizeof(int);
                char* senddata = new char[buf_size];
                int steps_num = 0;
                memcpy((void*)senddata, (void*)(&steps_num), sizeof(int));
                if (com.sendSteps(senddata, buf_size) == -1)
                {
                    perror("send error");
                    exit(1);
                }
                cout<<"has send 0 to win, you can ansys it use type int at buffer"<<endl;
            }
            else
            {
                vector<footstep> send_steps = steps_result;
                // send_steps.emplace_back(perception.steps_result.at(2));
                // data to file
                if (send_steps.size() <= 2)
                {
                    fs_communicate<<"some logic error occured in map or stepplanning, please check..."<<endl;
                }
                
                // for (size_t i = 0; i < perception.steps_result.size(); i++)
                // {
                //     fs_communicate<<"step "<<i+1<<": ";
                //     fs_communicate<<perception.steps_result.at(i).is_left<<" "<<perception.steps_result.at(i).x<<" "<<perception.steps_result.at(i).z<<endl;
                // }
                
                //  = perception.steps_result;
                LOG(INFO)<<"STEP PLANNING RESULTS:"<<endl;
                for (auto & iter_step : send_steps)
                {
                    LOG(INFO)<<iter_step.is_left<<" "<<iter_step.x<<" "<<iter_step.y<<" "<<iter_step.z<<" "<<iter_step.theta<<endl;
                }
                // 改了最大步数，16步
                struct sendddata
                {
                    int n;
                    footstep steps[16];
                };

                sendddata SENDdata;
                SENDdata.n = send_steps.size();
                for (size_t i = 0; i < send_steps.size(); i++)
                {
                    SENDdata.steps[i].is_left = send_steps.at(i).is_left;
                    SENDdata.steps[i].x = send_steps.at(i).x;
                    SENDdata.steps[i].y = send_steps.at(i).y;
                    SENDdata.steps[i].z = send_steps.at(i).z;
                    SENDdata.steps[i].theta = send_steps.at(i).theta;

                }
                cout<<"size of sendddata "<<sizeof(sendddata)<<endl;
                if (com.sendSteps((char*)(&SENDdata), sizeof(sendddata)) == -1)
                {
                    perror("send error");
                    exit(1);
                }
                unique_lock<mutex> g(m_cameradata, std::defer_lock);
                g.lock();
                publish_flag = true;
                publish_steps = send_steps;
                // publish_planes = perception.planes;
                g.unlock();
            }
            lastFlag = 'A';
            currentFlag = 'A';
        }
        else
        {
            fs_communicate<<"do not need to perception, send an empty data to win. "<<endl;
            int buf_size = sizeof(int);
            char* senddata = new char[buf_size];
            int steps_num = 0;
            memcpy((void*)senddata, (void*)(&steps_num), sizeof(int));
            if (com.sendSteps(senddata, buf_size) == -1)
            {
                perror("send error");
                exit(1);
            }
            // cout<<"has send 0 to win"<<endl;
        }
        LOG(INFO)<<"last flag is "<<lastFlag<<endl;
        LOG(INFO)<<"current flag is "<<currentFlag<<endl;
        com.close_client();
        LOG(INFO)<<"CLOSE CLIENT FINISH"<<endl;
    }
}

void exitall()
{
    LOG(INFO)<<"enter exit all function"<<endl;
    int ch;
    while (1){
		if (_kbhit())
        {//如果有按键按下，则_kbhit()函数返回真
            LOG(INFO)<<"PRESS KEY"<<endl;
			ch = getch();//使用_getch()获取按下的键值
            LOG(INFO)<<"GET CHAR "<<ch<<endl;
			cout << ch;
			if (ch == 27)
            { 
                close(sock_fd);
                close(client_fd);
                LOG(INFO)<<"CLOSE TCPIP PORT..."<<endl;
                break; 
                //当按下ESC时退出循环，ESC键的键值是27.
            }
        }
	}
}

int main(int argc, char** argv)
{
    // google::ParseCommandLineFlags(&argc, &argv, true); 
    google::InitGoogleLogging(argv[0]); 
    FLAGS_colorlogtostderr = true;
    FLAGS_log_dir = "./log"; 
    FLAGS_alsologtostderr = true;
    LOG(INFO)<<"initial glog finish"<<endl;
    LOG(WARNING)<<"WARNING"<<endl;

    thread th1(exitall);
    thread th2(communicate);
    ros::init(argc, argv,"plane_detection");
    ros::NodeHandle nh;
    // std::unique_ptr<ubuntuPlaneDetection> perception = std::make_unique<ubuntuPlaneDetection>(nh);
    get_ros_data get_data_node(nh);
    ros::spin();
    sleep(10);
    th1.join();
    th2.join();
    google::ShutdownGoogleLogging();
    return 0;
}