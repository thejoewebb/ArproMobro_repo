#include <iostream>
#include <math.h>
#include <cmath>

#include <robot.h>
#include <envir.h>
#include <sensor.h>
#include <sensor_range.h>
#include <sensor_bearing.h>

using namespace std;
using namespace arpro;

int main(int argc, char **argv)
{

  // default environment with moving target
  Environment envir;
  // sensors gets measurements from this environment
  Sensor::setEnvironment(envir);

  // init robot at (0,0,0)
  Robot robot("R2D2", 0,0,0);
  envir.addRobot(robot);
  Robot robot2("C3P0", 0,0.5,0);
  envir.addRobot(robot2);

  //initiate sensors
  RangeSensor Sensor1 = RangeSensor(robot, 0.1, 0, 0);
  //RangeSensor Sensor2 = RangeSensor(robot, -0.1, 0, 0);
  //RangeSensor Sensor3 = RangeSensor(robot, 1, -1, 1);
  BearingSensor Sensor4 = BearingSensor(robot2,0.1,0,0);

  // init wheels (r = 0.07m, b = 0.3m)
  robot.initWheel(0.07,0.3,10);
  robot2.initWheel(0.05,0.3,10);

  // simulate 100 sec
  while(envir.time() < 100)
  {
    cout << "---------------------" << endl;

    // update target position
    envir.updateTarget();

    // try to follow target
    robot.goTo(envir.target());
    robot2.moveWithSensor(Twist(0.4,0,0));

    robot2.printPosition();

  }

  // plot trajectory
  envir.plot();

}
