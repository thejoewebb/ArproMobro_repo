#ifndef SENSOR_RANGE_H
#define SENSOR_RANGE_H

#include <string>
#include <envir.h>
#include <robot.h>
#include <sensor.h>
#include <algorithm>


namespace arpro{

class RangeSensor : public Sensor{
public:
    RangeSensor(Robot &_robot, double _x, double _y, double _theta):
        Sensor(_robot, _x, _y, _theta) // Call the sensor constructor
    {} // The RangeSensor constructor does nothing more

    void update(const Pose &_p) override{

        Pose p1, p2;
        std::vector<double> distances;

        for(int i=0;i<envir_->walls.size();++i)
        {
        p1 = envir_->walls[i];
        p2 = envir_->walls[(i+1)%envir_->walls.size()];
        // do whatever you want to do with points p1 and p2
        // TO DO: to allow computation when denominator is null

        auto x1 = p1.x;
        auto y1 = p1.y;
        auto x2 = p2.x;
        auto y2 = p2.y;

        auto x = _p.x;
        auto y = _p.y;

        auto d = (x1*y2 - x1*y - x2*y1 + x2*y + x*y1 - x*y2)/
                (x1*sin(_p.theta) - x2*sin(_p.theta) - y1*cos(_p.theta) + y2*cos(_p.theta));

        if(d>0)
            distances.push_back(d);

        }

        s_ = *min_element(distances.begin(), distances.end());
        //cout<<s_;
    }

    void correctTwist(Twist &_v) override{

        auto smax = 0.1;
        auto gain = 1;

        if(_v.vx > gain*(s_-smax))
            _v.vx = gain*(s_-smax);

    }
};

}

#endif // SENSOR_RANGE_H
