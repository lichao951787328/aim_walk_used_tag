#ifndef _STRAIGHT_WALK_H_
#define _STRAIGHT_WALK_H_
#include <vector>
#include "type.h"
class straight_walk
{
private:
    std::vector<taijie> map;
    foot_data footpara;
    std::vector<footstep> steps;
public:
    straight_walk(std::vector<taijie> & map_, foot_data & footpara_);
    void walk_on_flat(taijie flat);
    void go();
    std::vector<footstep> inline getSteps()
    {
        return steps;
    }
    void last_plan_stepplanning();
    ~straight_walk();
};
#endif
