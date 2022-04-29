#include"straight_walk_show.h"
#include <iostream>
using namespace std;
straight_walk::straight_walk(std::vector<taijie> & map_, foot_data & footpara_)
{
    map = map_; footpara = footpara_;
}

void straight_walk::last_plan_stepplanning()
{
    // testcout<<"plane last falt plane."<<endl;
    // cout<<"plane last falt plane."<<endl;
    bool start_flag = !steps.back().is_left;
    double start = steps.back().x;
    taijie last_plan = map.back();
    double need_walk_length = last_plan.end - start - footpara.foot_front_length - footpara.threld;
    // testcout<<"we walk on flat plane now, the walk distance is "<<need_walk_length<<endl;
    // cout<<"we walk on flat plane now, the walk distance is "<<need_walk_length<<endl;
    if (need_walk_length > footpara.fit_length)
    {
        // cout<<"the walk distance is longer than fit length, we need to walk multi steps."<<endl;
        // 有长有短
        int num = int(need_walk_length/footpara.fit_length + 0.8);
        double tmp_step_length = (double)need_walk_length/(double)(num);
        // testcout<<"we need to walk "<<num<<" steps. and the walk length each step is "<<tmp_step_length<<endl;
        // cout<<"we need to walk "<<num<<" steps. and the walk length each step is "<<tmp_step_length<<endl;
        for (size_t i = 0; i < num; i++)
        {
            footstep tmpstep;
            tmpstep.is_left =  start_flag;
            double last_x = 0.0;
            if (!steps.empty())
            {
                last_x = steps.back().x;
            }
            tmpstep.x = last_x + tmp_step_length;
            tmpstep.z = last_plan.height;
            tmpstep.y = 0.0;
            tmpstep.theta = 0.0;
            steps.emplace_back(tmpstep);
            start_flag = !start_flag;
        }
    }
    else//计算下一个台阶开始的位置
    {
        // cout<<"the walk distance is less than fit length, we need to walk one step."<<endl;
        // 动与不动
        if (need_walk_length > footpara.min_length)
        {
            footstep tmpstep;
            tmpstep.is_left =  start_flag;
            double last_x = 0.0;
            if (!steps.empty())
            {
                last_x = steps.back().x;
            }
            tmpstep.x = last_x + need_walk_length;
        }
    }
}

void straight_walk::walk_on_flat(taijie flat)
{
    // 检查脚是否落在了该平面s
    bool start_flag = true;
    double start = 0.0;
    if (!steps.empty())
    {
        start_flag = !steps.back().is_left;
        start = steps.back().x;
        if (steps.back().x < flat.start || steps.back().x > flat.end)
        {
            return;
        }
    }
    cout<<"start : "<<start<<endl;
    double need_walk_length = flat.end - start - footpara.foot_front_length - footpara.threld;
    if (need_walk_length > footpara.fit_length)
    {
        cout<<"the walk distance is longer than fit length, we need to walk multi steps."<<endl;
        // fs<<"the walk distance is longer than fit length, we need to walk multi steps."<<endl;
        // fs<<"fit length is "<<step_long.fit_length<<endl;
        int num = int(need_walk_length/footpara.fit_length + 0.8);
        double tmp_step_length = (double)need_walk_length/(double)(num);
        cout<<"we need to walk "<<num<<" steps. and the walk length each step is "<<tmp_step_length<<endl;
        // fs<<"we need to walk "<<num<<" steps. and the walk length each step is "<<tmp_step_length<<endl;
        for (size_t i = 0; i < num; i++)
        {
            footstep tmpstep;
            cout<<start_flag<<endl;
            tmpstep.is_left = start_flag;
            start_flag = !start_flag;
            double last_x = 0.0;
            tmpstep.x = start + tmp_step_length * (i + 1);
            tmpstep.z = flat.height;
            tmpstep.y = tmpstep.is_left ? 0.08 : -0.08;
            tmpstep.theta = 0.0;
            cout<<"emplace step: "<<endl;
            cout<<tmpstep.is_left<<" "<<tmpstep.x<<" "<<tmpstep.y<<" "<<tmpstep.z<<" "<<tmpstep.theta<<endl;
            steps.emplace_back(tmpstep);
        }
    }
    else
    {
        cout<<"the walk distance is less than fit length, we need to walk one step."<<endl;
        if (need_walk_length > footpara.min_length)
        {
            footstep tmpstep;
            tmpstep.is_left =  start_flag;
            tmpstep.x = start + need_walk_length;
            tmpstep.z = flat.height;
            tmpstep.y = tmpstep.is_left ? 0.08 : -0.08;
            tmpstep.theta = 0.0;
            steps.emplace_back(tmpstep);
        }
    }
}
void straight_walk::go()
{
    std::cout<<"map : "<<endl;
    for (auto & iter_taijie : map)
    {
        std::cout<<iter_taijie.start<<" "<<iter_taijie.end<<" "<<iter_taijie.height<<endl;
    }
    
    while (!map.empty())
    {
        // footstep step;
        // step.is_left = false;
        // step.x = 0.0;
        // step.y = - 0.08;
        // step.z = 0.0;
        // if (!steps.empty())
        // {
        //     step = steps.back();
        // }
        cout<<"current taiijie "<<endl;
        cout<<map.front().start<<" "<<map.front().end<<" "<<map.front().height<<endl;
        walk_on_flat(map.front());
        map.erase(map.begin());
        while(!map.empty())
        {
            if (map.front().end - map.front().start < 0.4)
            {
                cout<<"walk on the mid"<<endl;
                footstep step;
                step.is_left = !steps.back().is_left;
                step.x = (map.front().end + map.front().start )/2.0 - 0.03;
                step.y = step.is_left ? 0.08 : -0.08;
                step.z = map.front().height;
                step.theta = 0.0;
                cout<<"emplace step: "<<endl;
                cout<<step.is_left<<" "<<step.x<<" "<<step.y<<" "<<step.z<<" "<<step.theta<<endl;
                steps.emplace_back(step);
                // 垫步
                footstep dianstep;
                dianstep.is_left = !steps.back().is_left;
                dianstep.x = steps.back().x;
                dianstep.y = dianstep.is_left ? 0.08 : -0.08;
                dianstep.z = steps.back().z;
                dianstep.theta = 0.0;
                cout<<"emplace step: "<<endl;
                cout<<dianstep.is_left<<" "<<dianstep.x<<" "<<dianstep.y<<" "<<dianstep.z<<" "<<dianstep.theta<<endl;
                steps.emplace_back(dianstep);
                map.erase(map.begin());
            }
            else
            {
                cout<<"walk on step"<<endl;
                footstep step;
                step.is_left = !steps.back().is_left;
                step.x = map.front().start + footpara.foot_end_length + footpara.threld;
                step.y = step.is_left ? 0.08 : -0.08;
                step.z = map.front().height;
                cout<<"emplace step: "<<endl;
                cout<<step.is_left<<" "<<step.x<<" "<<step.y<<" "<<step.z<<" "<<step.theta<<endl;
                steps.emplace_back(step);
                break;
            }
        }
    }
    if (steps.size() >= 2)
    {
        if (steps.at(steps.size() - 2).x != steps.at(steps.size() - 1).x)
        {
            footstep step;
            step.is_left = !steps.back().is_left;
            step.x = steps.back().x;
            step.y = step.is_left ? 0.08 : -0.08;
            step.z = steps.back().z;
            steps.emplace_back(step);
        }
    }
    if (steps.size() == 1)
    {
        footstep step;
        step.is_left = !steps.back().is_left;
        step.x = steps.back().x;
        step.y = step.is_left ? 0.08 : -0.08;
        step.z = steps.back().z;
        steps.emplace_back(step);
    }
}

straight_walk::~straight_walk()
{
    map.clear();
    steps.clear();
}