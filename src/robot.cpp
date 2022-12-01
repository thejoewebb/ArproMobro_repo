

#include <iostream>
#include <math.h>
#include <robot.h>
#include <sensor.h>

using namespace arpro;
using namespace std;

Environment* Sensor::envir_ = nullptr;

/*Q2 Robot::Robot signature:
"Robot::" refers to the Robot class in robot.h
"::Robot" refers to and enables editing of the Robot constructor function within the Robot class.
Arguments must be consistent with args in constructor function.

From now on, whenever a Robot object is declared using the constructor, a new set of variables are created inside
the Robot class (pose_.x, pose_.y, pose_.theta, name_, x_history_, y_history_) */


Robot::Robot(string _name, double _x, double _y, double _theta)
{    
    pose_.x = _x;
    pose_.y = _y;
    pose_.theta = _theta;

    name_ = _name;

    // init position history
    x_history_.push_back(_x);
    y_history_.push_back(_y);
}


void Robot::moveXYT(double _vx, double _vy, double _omega)
{
    // update position
    pose_.x += _vx*dt_;
    pose_.y += _vy*dt_;
    pose_.theta += _omega*dt_;

    // store position history
    x_history_.push_back(pose_.x);
    y_history_.push_back(pose_.y);
}

void Robot::initWheel(double _r, double _b, double _vlim)
{
    wheel_r_ = _r;
    wheel_b_ = _b;
    wheels_init_ = true;
    wheels_vlim_ = _vlim;
}

void Robot::rotateWheels(double _left, double _right)
{
    // to fill up after defining an initWheel method

    //check that wheels are initialised
    if(wheels_init_ == false)
        cout << "Wheels are not initialised. please initialise wheels before passing velocity instructions.";
    else{
        //check that vlim is respected.
        if(_left>wheels_vlim_ || _right>wheels_vlim_)
            cout << "Wheel velocity setpoint out of bounds, inputs have been scaled automatically";


        auto scale = max(abs(_left)/wheels_vlim_,abs(_right)/wheels_vlim_);
        if(scale<1)
            scale = 1;

        _left = _left/scale;
        _right = _right/scale;

        double v_ = wheel_r_ * (_left + _right) / 2;
        double omega_ = wheel_r_ * (_left - _right) / (2 * wheel_b_);

        double vx_ = v_*cos(pose_.theta);
        double vy_ = v_*sin(pose_.theta);

        moveXYT(vx_,vy_,omega_);
        }

}


// move robot with linear and angular velocities
void Robot::moveVW(double _v, double _omega)
{
    // convert _v and _omega to _vx, _vy and _omega

    auto left_ = (_v + wheel_b_*_omega)/wheel_r_;
    auto right_ = (_v - wheel_b_*_omega)/wheel_r_;

    rotateWheels(left_,right_);
}


// try to go to a given x-y position
void Robot::goTo(const Pose &_p)
{
    // error in robot frame
    Pose error = _p.transformInverse(pose_);

    // try to do a straight line with sensor constraints
    moveWithSensor(Twist(error.x, error.y, 0));
}


void Robot::moveWithSensor(Twist _twist)
{
    // to fill up, sensor measurement and twist checking

    for(auto & sensor: sensors_){
        sensor->updateFromRobotPose(pose_);
        sensor->correctTwist(_twist);
    }


    // uses X-Y motion (perfect but impossible in practice)
    //moveXYT(_twist.vx, _twist.vy,_twist.w);

    // to fill up, use V-W motion when defined
    auto alpha = 20;

    double v_ = _twist.vx;
    double omega_ = alpha*_twist.vy + _twist.w;

    moveVW(v_,omega_);
}


void Robot::printPosition()
{
    cout << "Current position: " << pose_.x << ", " << pose_.y << endl;
}

