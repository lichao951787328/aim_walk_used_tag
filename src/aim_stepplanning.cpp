#include "aim_stepplanning.h"
aim_stepplanning::aim_stepplanning(Eigen::Vector2d goal_, Eigen::Vector2d direct_, double fit_length_)
{
    goal = goal_; direct = direct_; fit_length = fit_length_;
    theta = acos(Eigen::Vector2d::UnitX().dot(direct));
    cout<<"theta  = "<<theta;
    if (direct(1) < 0)
    {
        theta = - theta;
        cout<<"theta  = "<<theta;
    }
    go_length = goal.norm();
}

aim_stepplanning::~aim_stepplanning()
{
    steps.clear();
}

footstep computeStep(double theta, double x, bool is_left)
{
    double y = is_left ? 0.08 : -0.08;
    cout<<is_left<<" "<<x<<" "<<y<<" "<<theta<<endl;
    return footstep(is_left, x, y, theta);
}

void aim_stepplanning::go(/* std::ofstream & fs */)
{
    cout<<"enter go function"<<endl;

    int num_foot_len = (int)(go_length/fit_length + 0.8);
    int num_foot_angle = (int)(abs(theta)/(6/57.3) + 0.8);
    int num_foot = max(num_foot_len, num_foot_angle);
    double length_step = go_length / num_foot;
    double theta_step = theta / num_foot;
    // cout<<"r is "<<r<<endl;
    cout<<"num foot is "<<num_foot<<endl<<"length "<<length_step<<endl<<"theta step "<<theta_step<<endl;
    // fs<<"r is "<<r<<endl;
    // fs<<"num foot is "<<num_foot<<endl<<"length "<<length<<endl<<"theta step "<<theta_step<<endl;
    
    steps.reserve(num_foot*2);
    if (theta < 0 )//证明需要右转，先迈右脚
    {
        ROS_INFO("turn right");
        // cout<<"turn right"<<endl;
        for (size_t i = 0; i < num_foot; i++)
        {
            steps.emplace_back(computeStep(theta_step * (i+1), length_step * (i+1), false));
            // cout<<"last step : "<<steps.back().is_left<<" "<<steps.back().x<<" "<<steps.back().y<<" "<<steps.back().z<<" "<<steps.back().theta<<endl;
            steps.emplace_back(computeStep(theta_step * (i+1), length_step * (i+1), true)); 
            // cout<<"last step : "<<steps.back().is_left<<" "<<steps.back().x<<" "<<steps.back().y<<" "<<steps.back().z<<" "<<steps.back().theta<<endl;
        }
    }
    else//左转
    {
        // cout<<"turn left"<<endl;
        ROS_INFO("turn left");
        for (size_t i = 0; i < num_foot; i++)
        {
            steps.emplace_back(computeStep(theta_step * (i+1), length_step * (i+1), true));
            // cout<<"last step : "<<steps.back().is_left<<" "<<steps.back().x<<" "<<steps.back().y<<" "<<steps.back().z<<" "<<steps.back().theta<<endl;
            steps.emplace_back(computeStep(theta_step * (i+1), length_step * (i+1), false));
            // cout<<"last step : "<<steps.back().is_left<<" "<<steps.back().x<<" "<<steps.back().y<<" "<<steps.back().z<<" "<<steps.back().theta<<endl;
        }
    }
    cout<<"go end"<<endl;
}

