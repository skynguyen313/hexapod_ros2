#ifndef IK_HPP_
#define IK_HPP_

#include <cmath>
#include "hexapod_msgs/msg/pose.hpp"
#include "hexapod_msgs/msg/feet_positions.hpp"
//=============================================================================
// Define structs and classes for gait system
//=============================================================================

struct Trig
{
    double sine;
    double cosine;
};

class Ik
{
public:
    Ik();
    
private:
    
};



#endif // IK_HPP_