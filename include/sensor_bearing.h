#ifndef SENSOR_BEARING_H
#define SENSOR_BEARING_H

#include <string>
#include <envir.h>
#include <robot.h>
#include <sensor.h>
#include <algorithm>
#include <cmath>

namespace arpro{
class BearingSensor : public Sensor{
public:
    BearingSensor(Robot &_robot, double _x, double _y, double _theta):
        Sensor(_robot, _x, _y, _theta) // Call the sensor constructor
    {} // The RangeSensor constructor does nothing more


    void update(const Pose &_p) override{

        //look for first other robot
        for(auto other: envir_->robots_)
            if(other != robot_){
                //compute angle between sensor and detected robot

                Pose robot_pose = other->pose();

                s_ = std::atan2(robot_pose.y-_p.y,robot_pose.x-_p.x) - _p.theta;
                //s_ = atan2(_p.y-robot_pose.y,_p.x-robot_pose.x)-robot_pose.theta;
                //s_history_.push_back(s_);

//                while(s_ > M_PI)
//                    s_ -= M_PI;

//                while(s_ < -M_PI)
//                    s_ += M_PI;

                //std::cout << "\n Sensored angle : " << s_ / M_PI  << "\n";

//                //set angle back to [-pi,pi]
//            if(s_ > M_PI){

//                while(s_ > M_PI){
//                    s_ -= M_PI;}
//            }

//             if(s_ < M_PI){

//                 while(s_ < -M_PI){
//                    s_ += M_PI;}
//            }

                break;

            }




}

    void correctTwist(Twist &_v) override{

        auto gain = 0.5;

        _v.w = _v.w - s_*gain;
        std::cout<< "twist : " << _v <<std::endl;

    }
};

};

#endif // SENSOR_BEARING_H
