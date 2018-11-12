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

  this->previous_path_x_ = j[1]["previous_path_x"];
  this->previous_path_y_ = j[1]["previous_path_y"];
  this->end_path_s_ = j[1]["end_path_s"];
  this->end_path_d_ = j[1]["end_path_d"];

  int prev_size = this->previous_path_x_.size();

  if (prev_size > 0) {
    this->car_s_ = this->end_path_s_;
  }
}

Vehicle::~Vehicle() {}
