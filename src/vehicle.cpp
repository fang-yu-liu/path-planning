#include "vehicle.h"

/**
 * Initializes Vehicle
 */

Vehicle::Vehicle(){}

Vehicle::Vehicle(int lane) {
  this->lane_ = lane;
}

void Vehicle::update(json &j){
  // j[1] is the data JSON object
  // Main car's localization Data
  this->car_x_ = j[1]["x"];
  this->car_y_ = j[1]["y"];
  this->car_s_ = j[1]["s"];
  this->car_d_ = j[1]["d"];
  this->car_yaw_ = j[1]["yaw"];

  double car_speed_mph = j[1]["speed"];
  this->car_speed_ = mph2mps(car_speed_mph);

  nlohmann::basic_json<> previous_path_x_ = j[1]["previous_path_x"];
  double end_path_s_ = j[1]["end_path_s"];
  int prev_size = previous_path_x_.size();
  if (prev_size > 0) {
    this->car_s_ = end_path_s_;
  }
}

Vehicle::~Vehicle() {}
