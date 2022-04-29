
#include "stepplanning.h"
#define USE_ABOVE
stepplaning_obj::stepplaning_obj(steplong step_info, vector<taijie> map_, double foot_l_f, double foot_l_e, double t)
{
    step_long = step_info;
    map = map_;
    foot_length_front = foot_l_f;
    foot_length_end = foot_l_e;
    th1 = t;
}

// 考虑机器人当前状态来进行落脚点规划

vector<step> stepplaning_obj::go(std::ofstream& fs)
{
    cout<<"enter stepplanning function"<<endl;
    vector<step> steps;
    vector<taijie> changed_map;// 会不会map只有一个， 初始map可能是落脚点的延续 
    // 处理除第一个之后的台阶
    if (map.size() > 1)
    {
        vector<taijie>::const_iterator iter_taijie;
        for (iter_taijie = map.begin() + 1; iter_taijie != map.end(); iter_taijie++)
        {
            if (iter_taijie->end - iter_taijie->start < foot_length_front + foot_length_end + th1)
            {
                // cout<<"erase: "<<iter_taijie->start<<" "<<iter_taijie->end<<" "<<iter_taijie->height<<endl;
                continue;
            }
            changed_map.emplace_back(*iter_taijie);
        }
    }
    // if (changed_map.size() > 1)
    // {
    //     vector<taijie>::const_iterator iter_taijie;
    //     for (iter_taijie = map.begin() + 1; iter_taijie != map.end(); iter_taijie++)
    //     {
    //         if (iter_taijie->end - iter_taijie->start < foot_length_front + foot_length_end + th1)
    //         {
    //             cout<<"erase: "<<iter_taijie->start<<" "<<iter_taijie->end<<" "<<iter_taijie->height<<endl;
    //             continue;
    //         }
    //         changed_map.emplace_back(*iter_taijie);
    //     }
    // }
    if (!map.empty())
    {
        if (abs(current_front_foot.z - map.front().height) < 0.02)//前脚点与地图的第一个台阶接上
        {
            if (map.front().end - map.front().start > 0.02)//第一个台阶的长度较大，需要加入到地图中
            {
                // changed_map.front().start = current_front_foot.x - foot_length_end;
                taijie first_taijie;
                first_taijie.start = current_front_foot.x - foot_length_end;
                first_taijie.end = map.front().end;
                first_taijie.height = map.front().height;
                changed_map.insert(changed_map.begin(), first_taijie);
            }
        }// 前脚点与第一个台阶没有接上，且长度较长，加入地图中
        else if (changed_map.front().end - changed_map.front().start > foot_length_front + foot_length_end + th1)
        {
            changed_map.insert(changed_map.begin(), map.front());
            // cout<<"erase ..."<<endl;
            // // vector<taijie>::iterator erase_taijie =  changed_map.begin();
            // changed_map.erase(changed_map.begin());
        }
    }
    else
        return steps;
    
    
    
    // 如果是初始状态 发送0 0
    // 抬起脚是current_end_foot，非抬起脚是current_front_foot，下一个状态对应的是抬起脚，也就是后脚
    // cout<<" ************ important information in stepplanning obj+**************"<<endl;
    fs<<" ************ important information in stepplanning obj+**************"<<endl;
    fs<<"map in stepplanning object"<<endl;
    for (size_t i = 0; i < changed_map.size(); i++)
    {
        fs<<"taijie start: "<<changed_map.at(i).start<<" end: "<<changed_map.at(i).end<<" height: "<<changed_map.at(i).height<<endl;
    }
    fs<<" ************* important information in stepplanning obj***************"<<endl;
    // step和map都是在世界坐标系下
    fs<<"current front foot : "<<current_front_foot.is_left<<" "<<current_front_foot.x<<" "<<current_front_foot.z<<endl;
    fs<<"next foot : "<<next_foot.is_left<<" "<<next_foot.x<<" "<<next_foot.z<<endl;
    cout<<"current front foot : "<<current_front_foot.is_left<<" "<<current_front_foot.x<<" "<<current_front_foot.z<<endl;
    cout<<"next foot : "<<next_foot.is_left<<" "<<next_foot.x<<" "<<next_foot.z<<endl;
    // cout<<"next foot : "<<next_foot.is_left<<" "<<next_foot.x<<" "<<next_foot.z<<endl;
    steps.emplace_back(current_front_foot);
    steps.emplace_back(next_foot);

    // 判断机器人当前是不是位于一个在上一个在下 
    // 在做平面检测时，需要排除next_foot.x之前的点
    if (abs(current_front_foot.z - next_foot.z) > 0.02)//一个在上 一个在下
    {
        // 按照平路规划
        // 地图确认current_front_foot 应该落在地图的第一个台阶上
        // 并一步
        fs<<"the next state of robot is one step on, one step down."<<endl;
        // cout<<"the next state of robot is one step on, one step down."<<endl;
        step dian_step;
        dian_step.is_left = !steps.back().is_left;
        dian_step.x = steps.back().x;
        dian_step.z = steps.back().z;
        steps.emplace_back(dian_step);
        // ？？？？ 检查第一个台阶，实际上应该不需要检查
        while (!changed_map.empty())
        {
            fs<<"the map size is not empty."<<endl;
            // cout<<"the map size is not empty."<<endl;
            if (!(changed_map.front().end > steps.back().x + foot_length_front))
            {
                fs<<"the erase taijie is "<<changed_map.front().start<<changed_map.front().end<<changed_map.front().height<<endl;
                fs<<"the last step info "<<steps.back().is_left<<" "<<steps.back().x<<" "<<steps.back().z<<endl;
                changed_map.erase(changed_map.begin());
            }
            else
                break;
        }
        if (!changed_map.empty())
        {
            fs<<"the map is not empty."<<endl;
            fs<<"the taijie in map is : "<<endl;
            // cout<<"the map is not empty."<<endl;
            // cout<<"the taijie in map is : "<<endl;
            for (size_t i = 0; i < changed_map.size(); i++)
            {
                fs<<i+1<<": "<<changed_map.at(i).start<<" "<<changed_map.at(i).end<<" "<<changed_map.at(i).height<<endl;
            }
            fs<<"the steps is : "<<endl;
            // cout<<"the steps is : "<<endl;
            for (size_t i = 0; i < steps.size(); i++)
            {
                fs<<i+1<<": "<<steps.at(i).is_left<<" "<<steps.at(i).x<<" "<<steps.at(i).z<<endl;
            }
            flat_stepplanning(steps, changed_map, fs);
        }
        
        // if (changed_map.size() > 0)
        // {
        //     cout<<"map size is more than 0"<<endl;
        //     // 
        //     while (!(changed_map.front().start + 0.01 > steps.back().x + foot_length_front))
        //     {
        //         cout<<"the erase taijie is "<<changed_map.front().start<<changed_map.front().end<<changed_map.front().height<<endl;
        //         changed_map.erase(changed_map.begin());
        //     }
        //     flat_stepplanning(steps, changed_map);
        // }
    }
    else//当前处于同一平面
    {
        fs<<"the next state of robot is on the same plane"<<endl;
        // cout<<"the next state of robot is on the same plane"<<endl;
        if (abs(current_front_foot.x - next_foot.x) < 0.02)//下一个状态机器人处于并步 并步可能上台阶准备，可能下台阶准备
        {
            fs<<"next state is bingbu"<<endl;
            // cout<<"next state is bingbu"<<endl;
            // 检查迈步，并迈步
            // 当前状态和下个状态不可能同时并步
            // 弹出不需要的前序平面
            if (changed_map.size() > 0)
            {
                fs<<"taijie in map is not zero"<<endl;
                // cout<<"taijie in map is not zero"<<endl;
                while (!(changed_map.front().end > steps.back().x + foot_length_front))
                {
                    changed_map.erase(changed_map.begin());
                }
                // 没有考虑初始状态，不上也不下
                if (abs(steps.back().z - changed_map.front().height) < 0.01)
                {
                    // 虽然下以状态为并步，但是不是上台阶也不是下台阶
                    fs<<"next the robot is bingbu, but not step on or step down."<<endl;
                    // cout<<"next the robot is bingbu, but not step on or step down."<<endl;
                    flat_stepplanning(steps, changed_map, fs);
                }
                else if (steps.back().z > changed_map.front().height)// 下台阶
                {
                    fs<<"need to step down"<<endl;
                    // cout<<"need to step down"<<endl;
                    step step_on_1;
                    step_on_1.is_left = !steps.back().is_left;
                    step_on_1.x = changed_map.front().start + foot_length_end + 0.015;
                    step_on_1.z = changed_map.front().height;
                    steps.emplace_back(step_on_1);
                    step step_on_2;
                    step_on_2.is_left = !steps.back().is_left;
                    step_on_2.x = changed_map.front().start + foot_length_end + 0.015;
                    step_on_2.z = changed_map.front().height;
                    steps.emplace_back(step_on_2);
                    // 平路规划
                    flat_stepplanning(steps, changed_map,fs);
                }
                else// 上台阶
                {
                    fs<<"need to step on"<<endl;
                    // cout<<"need to step on"<<endl;
                    step step_on_1;
                    step_on_1.is_left = !steps.back().is_left;
                    step_on_1.x = changed_map.front().start + foot_length_end;
                    step_on_1.z = changed_map.front().height;
                    step step_on_2;
                    step_on_2.is_left = !steps.back().is_left;
                    step_on_2.x = changed_map.front().start + foot_length_end;
                    step_on_2.z = changed_map.front().height;

                    // 平路规划
                    flat_stepplanning(steps, changed_map, fs);
                }
            }    
        }
        else//下一个状态不是并步，进行平路规划
        {
            fs<<"next state is not bingbu"<<endl;
            // cout<<"next state is not bingbu"<<endl;
            if (changed_map.size() > 0)
            {
                flat_stepplanning(steps, changed_map, fs);
            }
            // // 这个判断条件
            // if (changed_map.front().start - next_foot.x < foot_length_front + 0.05)
            // {
            //     // 判断需要上还是下 下一个台阶需要的动作
            //     if (changed_map.front().height > next_foot.z + 0.02)//上
            //     {
            //         step dian_step;
            //         dian_step.is_left = !steps.back().is_left;
            //         dian_step.x = steps.back().x;
            //         dian_step.z = steps.back().z;
            //         steps.emplace_back(dian_step);
            //         fs<<"need to step on"<<endl;
            //         step step_on_1;
            //         step_on_1.is_left = !steps.back().is_left;
            //         step_on_1.x = changed_map.front().start + foot_length_end;
            //         step_on_1.z = changed_map.front().height;
            //         steps.emplace_back(step_on_1);
            //         step step_on_2;
            //         step_on_2.is_left = !steps.back().is_left;
            //         step_on_2.x = changed_map.front().start + foot_length_end;
            //         step_on_2.z = changed_map.front().height;
            //         steps.emplace_back(step_on_2);
            //         // 平路规划
            //         flat_stepplanning(steps, changed_map, fs);
            //     }
            //     else if (changed_map.front().height + 0.02 < next_foot.z)// 下
            //     {
            //         step dian_step;
            //         dian_step.is_left = !steps.back().is_left;
            //         dian_step.x = steps.back().x;
            //         dian_step.z = steps.back().z;
            //         steps.emplace_back(dian_step);
            //         fs<<"need to step down"<<endl;
            //         step step_on_1;
            //         step_on_1.is_left = !steps.back().is_left;
            //         step_on_1.x = changed_map.front().start + foot_length_end + 0.015;
            //         step_on_1.z = changed_map.front().height;
            //         steps.emplace_back(step_on_1);
            //         step step_on_2;
            //         step_on_2.is_left = !steps.back().is_left;
            //         step_on_2.x = changed_map.front().start + foot_length_end + 0.015;
            //         step_on_2.z = changed_map.front().height;
            //         steps.emplace_back(step_on_2);   
            //         // 平路规划
            //         flat_stepplanning(steps, changed_map,fs);
            //     }
            //     else
            //     {
            //         // 平路规划
            //         fs<<"on falt plan stepplanning "<<endl;
            //         flat_stepplanning(steps, changed_map,fs);
            //     }
            // }
            // else
            // {
            //     if (changed_map.size() > 0)
            //     {
            //         flat_stepplanning(steps, changed_map, fs);
            //     }
            // }
        }
    }
    fs<<"compute the last step."<<endl;
    // cout<<"compute the last step."<<endl;
    if (steps.at(steps.size()-2).x != steps.at(steps.size() - 1).x)
    {
        step tmpstep;
        tmpstep.is_left = !steps.back().is_left;
        tmpstep.x = steps.back().x;
        tmpstep.z = steps.back().z;
        steps.emplace_back(tmpstep);
    }
    return steps;
}

void stepplaning_obj::flat_stepplanning(vector<step> & current_steps, vector<taijie> & flat_map, std::ofstream& fs)
{
    // 检查台阶
    if (flat_map.size() > 0)
    {
        if (!(current_steps.back().x - foot_length_end + 0.02 > flat_map.front().start))
        {
            fs<<"the first taijie is not ok, please check."<<endl;
            // cout<<"the first taijie is not ok, please check."<<endl;
            flat_map.front().start = current_steps.back().x - foot_length_end;
        }
    }
    while (flat_map.size() > 1)
    {
        taijie tmp_taijie = flat_map.front();
        flat_map.erase(flat_map.begin());
        fs<<"changed map size is "<<flat_map.size()<<endl;
        // cout<<"changed map size is "<<flat_map.size()<<endl;
        double thre = tmp_taijie.height < flat_map.front().height ? th1:0;// 上台阶有阈值， 下台阶没有
        fs<<"current step size is "<<current_steps.size()<<endl;
        // cout<<"current step size is "<<current_steps.size()<<endl;
        bool start_flag = !current_steps.back().is_left;
        double start = current_steps.back().x;
        double need_walk_length = tmp_taijie.end - start - foot_length_front - thre;
        if (need_walk_length > step_long.fit_length)
        {
            fs<<"the walk distance is longer than fit length, we need to walk multi steps."<<endl;
            fs<<"fit length is "<<step_long.fit_length<<endl;
            int num = int(need_walk_length/step_long.fit_length + 0.8);
            double tmp_step_length = (double)need_walk_length/(double)(num);
            fs<<"we need to walk "<<num<<" steps. and the walk length each step is "<<tmp_step_length<<endl;
            for (size_t i = 0; i < num; i++)
            {
                step tmpstep;
                tmpstep.is_left =  start_flag;
                double last_x = 0.0;
                if (!current_steps.empty())
                {
                    last_x = current_steps.back().x;
                }
                tmpstep.x = last_x + tmp_step_length;
                tmpstep.z = tmp_taijie.height;
                current_steps.emplace_back(tmpstep);
                start_flag = !start_flag;
            }
        }
        else//计算下一个台阶开始的位置
        {
            fs<<"the walk distance is less than fit length, we need to walk one step."<<endl;
            // 动与不动
            if (need_walk_length > step_long.min_length)
            {
                step tmpstep;
                tmpstep.is_left =  start_flag;
                double last_x = 0.0;
                if (!current_steps.empty())
                {
                    last_x = current_steps.back().x;
                }
                tmpstep.x = last_x + need_walk_length;
                tmpstep.z = tmp_taijie.height;
                current_steps.emplace_back(tmpstep);
            }
        }

        fs<<"current steps size is "<<current_steps.size()<<endl;
        if (current_steps.size() >= 2)
        {
            if (current_steps.at(current_steps.size()-2).x != current_steps.at(current_steps.size() - 1).x)
            {
                step tmpstep;
                tmpstep.is_left = !current_steps.back().is_left;
                tmpstep.x = current_steps.back().x;
                tmpstep.z = current_steps.back().z;
                current_steps.emplace_back(tmpstep);
            }
        }
        if (current_steps.size() == 1)
        {
            step tmpstep;
            tmpstep.is_left = !current_steps.back().is_left;
            tmpstep.x = current_steps.back().x;
            tmpstep.z = current_steps.back().z;
            current_steps.emplace_back(tmpstep);
        }
        if (current_steps.size() == 0 )
        {
            // cout<<"the step is wrong"<<endl;
        }
        
        fs<<"plan to step on next step."<<endl;
        // 走上下一个台阶
        // 计算下一个台阶的step
        double height_changed = flat_map.front().height - tmp_taijie.height;
        
        step tmpstep;
        tmpstep.is_left =  !current_steps.back().is_left;
        tmpstep.x = flat_map.front().start + foot_length_end;
        if (tmpstep.x - current_steps.back().x > step_long.max_length)
        {
            fs<<"the step is too far, the robot can not step on."<<endl;
            tmpstep.x = current_steps.back().x;
            tmpstep.z = current_steps.back().z;
            current_steps.emplace_back(tmpstep);
        }
        tmpstep.z = flat_map.front().height;
        current_steps.emplace_back(tmpstep);
        
        // 上下台阶，之后垫一步
        step dian_step;
        dian_step.x = current_steps.back().x;
        dian_step.z = current_steps.back().z;
        dian_step.is_left = !current_steps.back().is_left;
        current_steps.emplace_back(dian_step);
    }
    // 如果仅剩一个台阶平面
    if (flat_map.size() == 1)
    {
        last_plan_stepplanning(current_steps, flat_map.back());
    }
}

void stepplaning_obj::last_plan_stepplanning(vector<step> & current_steps, taijie & last_plan)
{
    // testcout<<"plane last falt plane."<<endl;
    // cout<<"plane last falt plane."<<endl;
    bool start_flag = !current_steps.back().is_left;
    double start = current_steps.back().x;
    double thre = 0;
    double need_walk_length = last_plan.end - start - foot_length_front - thre;
    // testcout<<"we walk on flat plane now, the walk distance is "<<need_walk_length<<endl;
    // cout<<"we walk on flat plane now, the walk distance is "<<need_walk_length<<endl;
    if (need_walk_length > step_long.fit_length)
    {
        // cout<<"the walk distance is longer than fit length, we need to walk multi steps."<<endl;
        // 有长有短
        int num = int(need_walk_length/step_long.fit_length + 0.8);
        double tmp_step_length = (double)need_walk_length/(double)(num);
        // testcout<<"we need to walk "<<num<<" steps. and the walk length each step is "<<tmp_step_length<<endl;
        // cout<<"we need to walk "<<num<<" steps. and the walk length each step is "<<tmp_step_length<<endl;
        for (size_t i = 0; i < num; i++)
        {
            step tmpstep;
            tmpstep.is_left =  start_flag;
            double last_x = 0.0;
            if (!current_steps.empty())
            {
                last_x = current_steps.back().x;
            }
            tmpstep.x = last_x + tmp_step_length;
            tmpstep.z = last_plan.height;
            current_steps.emplace_back(tmpstep);
            start_flag = !start_flag;
        }
    }
    else//计算下一个台阶开始的位置
    {
        // cout<<"the walk distance is less than fit length, we need to walk one step."<<endl;
        // 动与不动
        if (need_walk_length > step_long.min_length)
        {
            step tmpstep;
            tmpstep.is_left =  start_flag;
            double last_x = 0.0;
            if (!current_steps.empty())
            {
                last_x = current_steps.back().x;
            }
            tmpstep.x = last_x + need_walk_length;
        }
    }
}