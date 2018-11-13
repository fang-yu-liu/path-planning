#ifndef PATH_PLANNER_H_
#define PATH_PLANNER_H_
#include <string>
#include <map>
#include <vector>
#include <fstream>
#include <utility>
#include "vehicle.h"
#include "json.hpp"
#include "helper_functions.h"
#include "spline/spline.h"

using namespace std;

class Path_Planner {
public:

  map<string, int> lane_direction_ = {{"LCL", -1}, {"LCR", 1}};

  string current_state_ = "KL";
  int current_lane_;

  int total_num_lanes_;
  double desired_speed_; //m/s
  double ref_speed_; //m/s
  double max_acceleration_;
  double max_s_;

  nlohmann::basic_json<> previous_path_x_;
  nlohmann::basic_json<> previous_path_y_;
  double end_path_s_;
  double end_path_d_;
  nlohmann::basic_json<> sensor_fusion_;
  map<string, vector<double>> map_;

  Vehicle vehicle_;

  /**
  * Constructor
  */
  Path_Planner();
  Path_Planner(int total_num_lanes, double desired_speed, double ref_speed, double max_acceleration);

  /**
  * Destructor
  */
  virtual ~Path_Planner();

  void configure(string map_file);
  void setVehicle(Vehicle &vehicle);
  void update(json &j);
  pair<map<int, Vehicle>, map<int, Vehicle>> get_vehicles_information();
  string choose_next_state(vector<string> possible_successor_states, map<int, Vehicle> ahead_vehicles, map<int, Vehicle> behind_vehicles);
  vector<string> successor_states(string state);
  float calculate_cost(int new_lane, Vehicle ahead_vehicle, Vehicle behind_vehicle);
  vector<vector<double>> path_planning();
  vector<vector<double>> generate_spline_trajectory(string state);

};
#endif
