#ifndef _STEPPLANNING_H_
#define _STEPPLANNING_H_
#include <iostream>
#include <vector>
#include <fstream>
using namespace std;
// fstream testcout("test_txt.txt");

// 步长的定义，左脚踝到右脚踝
struct steplong
{
    double min_length;
    double max_length;
    double fit_length;
    double min_step_height;
    double max_step_height;
    
    steplong(double min_l = 0.01, double max_l = 0.4, double fit_l = 0.2, double min_h = -0.04, double max_h = 0.04)
    {
        min_length = min_l; max_length = max_l; fit_length = fit_l; min_step_height = min_h; max_step_height = max_h;
    }
    steplong& operator = (const steplong& other)
    {
        min_length = other.min_length;
        max_length = other.max_length;
        fit_length = other.fit_length;
        min_step_height = other.min_step_height;
        max_step_height = other.max_step_height;
        return *this;
    }

    // 当跨越台阶时，计算一个最远范围值供检查使用
    double compute_fit_length(double height)
    {
        if (height > - 0.01 && height < -0.01)
        {
            return fit_length;
        }
        return 0;
    }
};
// 绝对坐标
struct taijie
{
    double start;
    double end;
    double height;
};
// 绝对坐标
struct step
{
    bool is_left;
    double x;
    // double y;
    double z;
    // double roll;
    // double pitch; 
    // double yaw;
};

class stepplaning_obj
{
private:
    steplong step_long;
    // 排好序的台阶输入
    vector<taijie> map;
    double foot_length_front;
    double foot_length_end;
    double th1;//上台阶的一个准备阈值，需要离台阶有一定的距离

    // 当前状态
    step current_front_foot;
    step current_end_foot;
    // 下一个状态
    step next_foot;

public:
    stepplaning_obj(steplong step_info, vector<taijie> map_, double foot_l_f, double foot_l_e, double t);
    void inline set_current_front_foot(bool isleft, double x, double z)
    {
        current_front_foot.is_left = isleft;
        current_front_foot.x = x;
        current_front_foot.z = z;
    }
    void inline set_current_end_foot(bool isleft, double x, double z)
    {
        current_end_foot.is_left = isleft;
        current_end_foot.x = x;
        current_end_foot.z = z;

    }
    void inline set_next_foot(bool isleft, double x, double z)
    {
        next_foot.is_left = isleft;
        next_foot.x = x;
        next_foot.z = z;
    }
    // 规划 先抬左脚
    vector<step> go(std::ofstream& fs);
    void flat_stepplanning(vector<step> & current_steps, vector<taijie> & flat_map, std::ofstream& fs);
    void last_plan_stepplanning(vector<step> & current_steps, taijie & last_plan);
};

#endif