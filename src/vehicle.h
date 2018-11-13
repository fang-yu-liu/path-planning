#ifndef VEHICLE_H_
#define VEHICLE_H_
#include "json.hpp"
#include "helper_functions.h"

using namespace std;
using json = nlohmann::json;

class Vehicle {
public:

  int lane_;
  //json real-time data
  double car_x_;
  double car_y_;
  double car_s_;
  double car_d_;
  double car_yaw_;
  double car_speed_;

  /**
  * Constructor
  */
  Vehicle();
  Vehicle(int lane);

  /**
  * Destructor
  */
  virtual ~Vehicle();

  void update(json &j);
};

#endif
