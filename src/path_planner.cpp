#include "path_planner.h"

/**
 * Initializes Path Planner
 */
Path_Planner::Path_Planner(){}

Path_Planner::Path_Planner(int total_num_lanes, double desired_speed, double ref_speed, double max_acceleration) {
    this->total_num_lanes_ = total_num_lanes;
    this->desired_speed_ = desired_speed;
    this->ref_speed_ = ref_speed;
    this->max_acceleration_ = max_acceleration;
}

void Path_Planner::configure(string map_file) {

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // The max s value before wrapping around the track back to 0
  this->max_s_ = 6945.554;

  ifstream in_map_(map_file.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
  	double x;
  	double y;
  	float s;
  	float d_x;
  	float d_y;
  	iss >> x;
  	iss >> y;
  	iss >> s;
  	iss >> d_x;
  	iss >> d_y;
  	map_waypoints_x.push_back(x);
  	map_waypoints_y.push_back(y);
  	map_waypoints_s.push_back(s);
  	map_waypoints_dx.push_back(d_x);
  	map_waypoints_dy.push_back(d_y);
  }

  map<string, vector<double>> map;
  map["map_waypoints_x"] = map_waypoints_x;
  map["map_waypoints_y"] = map_waypoints_y;
  map["map_waypoints_s"] = map_waypoints_s;
  map["map_waypoints_dx"] = map_waypoints_dx;
  map["map_waypoints_dy"] = map_waypoints_dy;

  this->map_ = map;
}

void Path_Planner::setVehicle(Vehicle &vehicle) {
  this->vehicle_ = vehicle;
  this->current_lane_ = vehicle.lane_;
}

void Path_Planner::update(json &j) {
  // update vehicle localization data
  this->vehicle_.update(j);
  /**
  Sensor Fusion Data, a list of all other cars on the same side of the road.
  (1) car's unique ID
  (2) car's x position in map coordinates
  (3) car's y position in map coordinates
  (4) car's x velocity in m/s
  (5) car's y velocity in m/s
  (6) car's s position in frenet coordinates
  (7) car's d position in frenet coordinates
  */
  this->sensor_fusion_ = j[1]["sensor_fusion"];

  this->previous_path_x_ = j[1]["previous_path_x"];
  this->previous_path_y_ = j[1]["previous_path_y"];
  this->end_path_s_ = j[1]["end_path_s"];
  this->end_path_d_ = j[1]["end_path_d"];
}

vector<vector<double>> Path_Planner::path_planning() {
  vector<string> possible_successor_states = successor_states(this->current_state_);

  pair<map<int, Vehicle>, map<int, Vehicle>> vehicles_information = get_vehicles_information();
  map<int, Vehicle> ahead_vehicles = vehicles_information.first;
  map<int, Vehicle> behind_vehicles = vehicles_information.second;

  string next_state = choose_next_state(possible_successor_states, ahead_vehicles, behind_vehicles);
  return generate_spline_trajectory(next_state);
}

pair<map<int, Vehicle>, map<int, Vehicle>> Path_Planner::get_vehicles_information() {
  double check_distance = 50;
  map<int, Vehicle> ahead_vehicles; //map of lane id -> closet vehicles in front within the check_distance
  map<int, Vehicle> behind_vehicles; //map of lane id -> closet vehicles in behind within the check_distance
  for (int i = 0; i < total_num_lanes_; i++) {
    Vehicle ahead_vehicle;
    ahead_vehicle.car_s_ = this->max_s_;
    ahead_vehicle.car_speed_ = this->desired_speed_;
    ahead_vehicles[i] = ahead_vehicle;

    Vehicle behind_vehicle;
    behind_vehicle.car_s_ = -1;
    behind_vehicle.car_speed_ = -1;
    behind_vehicles[i] = behind_vehicle;
  }

  for(int i = 0; i< sensor_fusion_.size(); i++) {
    double check_car_vx = sensor_fusion_[i][3];
    double check_car_vy = sensor_fusion_[i][4];
    double check_car_s = sensor_fusion_[i][5];
    double check_car_d = sensor_fusion_[i][6];
    double check_car_v = sqrt(check_car_vx*check_car_vx+check_car_vy*check_car_vy); //m/s
    int lane = (int)(check_car_d/4.0);

    if(check_car_s > vehicle_.car_s_ && check_car_s < (vehicle_.car_s_ + check_distance)) {
      if (check_car_s < ahead_vehicles[lane].car_s_) {
        ahead_vehicles[lane].car_s_ = check_car_s;
        ahead_vehicles[lane].car_speed_ = check_car_v;
      }
    } else if (check_car_s <= vehicle_.car_s_ && check_car_s > (vehicle_.car_s_ - check_distance)){
      if (check_car_s > behind_vehicles[lane].car_s_) {
        behind_vehicles[lane].car_s_ = check_car_s;
        behind_vehicles[lane].car_speed_ = check_car_v;
      }
    }
  }
  return make_pair(ahead_vehicles, behind_vehicles);
}

string Path_Planner::choose_next_state(vector<string> possible_successor_states, map<int, Vehicle> ahead_vehicles, map<int, Vehicle> behind_vehicles) {

  map<string, float> costs; //map of state -> cost

  for(auto const& state : possible_successor_states) {
    int new_lane = this->current_lane_ + this->lane_direction_[state];
    float cost_for_state = calculate_cost(new_lane, ahead_vehicles[new_lane], behind_vehicles[new_lane]);
    costs[state] = cost_for_state;
  }

  string preferred_state = "KL";
  float min_cost = costs[preferred_state];

  for(auto const& state : possible_successor_states) {
    if (costs[state] < min_cost) {
      min_cost = costs[state];
      preferred_state = state;
    }
  }

  return preferred_state;
}

vector<string> Path_Planner::successor_states(string state) {
  /*
  Provides the possible next states given the current state for the FSM
  discussed in the course, with the exception that lane changes happen
  instantaneously, so LCL and LCR can only transition back to KL.
  */
  vector<string> states;
  states.push_back("KL");
  if (state.compare("KL") == 0) {
    states.push_back("LCL"); //lane change left
    states.push_back("LCR"); //lane change right
  }
  //If state is "LCL" or "LCR", then just return "KL"
  return states;
}

float Path_Planner::calculate_cost(int new_lane, Vehicle ahead_vehicle, Vehicle behind_vehicle) {

  Vehicle vehicle = this->vehicle_;
  map<string, vector<double>> map = this->map_;
  nlohmann::basic_json<> sensor_fusion = this->sensor_fusion_;
  double ahead_safe_distance = 40;
  double behind_safe_distance = 30;

  //if the state "LCL" or "LCR" is not doable -> cost = 99
  //if the state is doable -> calculate the cost
  if (this->current_lane_ != new_lane) {
    //LCL or LCR
    bool has_car_behind = (vehicle.car_s_ - behind_safe_distance) < behind_vehicle.car_s_;
    bool has_car_ahead = (vehicle.car_s_ + ahead_safe_distance) > ahead_vehicle.car_s_;
    bool out_of_boundary = new_lane < 0 || new_lane > this->total_num_lanes_-1;
    if (has_car_behind || has_car_ahead || out_of_boundary) {
      return 99;
    }
  }
  return pow(this->desired_speed_ - ahead_vehicle.car_speed_, 2)/pow(this->desired_speed_, 2) + 0.01*pow(new_lane-this->current_lane_, 2);
}

vector<vector<double>> Path_Planner::generate_spline_trajectory(string next_state) {

  Vehicle vehicle = this->vehicle_;
  int new_lane = this->current_lane_ + this->lane_direction_[next_state];
  map<string, vector<double>> map = this->map_;

  int prev_size = this->previous_path_x_.size();

  if (prev_size > 0) {
    vehicle.car_s_ = this->end_path_s_;
  }

  vector<double> ptsx;
  vector<double> ptsy;

  double ref_x = vehicle.car_x_;
  double ref_y = vehicle.car_y_;
  double ref_yaw = deg2rad(vehicle.car_yaw_);

  if (prev_size < 2) {
    // If start from scratch, use the current car position and
    // create one previous point that is tangential to the car_yaw
    double prev_car_x = vehicle.car_x_ - cos(vehicle.car_yaw_);
    double prev_car_y = vehicle.car_y_ - sin(vehicle.car_yaw_);

    ptsx.push_back(prev_car_x);
    ptsx.push_back(vehicle.car_x_);

    ptsy.push_back(prev_car_y);
    ptsy.push_back(vehicle.car_y_);
  } else {
    ref_x = this->previous_path_x_[prev_size-1];
    ref_y = this->previous_path_y_[prev_size-1];

    double prev_ref_x = this->previous_path_x_[prev_size-2];
    double prev_ref_y = this->previous_path_y_[prev_size-2];
    ref_yaw = atan2(ref_y-prev_ref_y, ref_x-prev_ref_x);

    ptsx.push_back(prev_ref_x);
    ptsx.push_back(ref_x);

    ptsy.push_back(prev_ref_y);
    ptsy.push_back(ref_y);
  }

  double next_d = 2+4*new_lane;
  vector<double> next_wp0 = getXY(vehicle.car_s_+30, next_d, map["map_waypoints_s"], map["map_waypoints_x"], map["map_waypoints_y"]);
  vector<double> next_wp1 = getXY(vehicle.car_s_+60, next_d, map["map_waypoints_s"], map["map_waypoints_x"], map["map_waypoints_y"]);
  vector<double> next_wp2 = getXY(vehicle.car_s_+90, next_d, map["map_waypoints_s"], map["map_waypoints_x"], map["map_waypoints_y"]);

  ptsx.push_back(next_wp0[0]);
  ptsx.push_back(next_wp1[0]);
  ptsx.push_back(next_wp2[0]);

  ptsy.push_back(next_wp0[1]);
  ptsy.push_back(next_wp1[1]);
  ptsy.push_back(next_wp2[1]);

  for (int i = 0; i < ptsx.size(); i++) {
    double ptx_map = ptsx[i];
    double pty_map = ptsy[i];

    double ptx_vehicle = (ptx_map - ref_x) * cos(-ref_yaw) - (pty_map - ref_y) * sin(-ref_yaw);
    double pty_vehicle = (ptx_map - ref_x) * sin(-ref_yaw) + (pty_map - ref_y) * cos(-ref_yaw);
    ptsx[i] = ptx_vehicle;
    ptsy[i] = pty_vehicle;
  }

  double min_collision_s = this->max_s_;
  double safe_s = 30;
  double safe_speed = desired_speed_;
  double ref_speed = this->ref_speed_;
  nlohmann::basic_json<> sensor_fusion = this->sensor_fusion_;

  for (int i = 0; i< sensor_fusion.size();i++) {
    double check_car_d = sensor_fusion[i][6];

    if (check_car_d < (2+4*new_lane+2) && check_car_d > (2+4*new_lane-2)) {
      double check_car_vx = sensor_fusion[i][3];
      double check_car_vy = sensor_fusion[i][4];
      double check_car_v = sqrt(check_car_vx*check_car_vx+check_car_vy*check_car_vy); //m/s
      double check_car_s = sensor_fusion[i][5];

      if(check_car_s > vehicle.car_s_) {
        double collision_s = check_car_s - vehicle.car_s_;
        if (collision_s < min_collision_s) {
          min_collision_s = collision_s;
          safe_speed = check_car_v;
        }
      }
    }
  }

  if (min_collision_s < safe_s) {
    if (ref_speed > safe_speed) {
      ref_speed = max(ref_speed - this->max_acceleration_, safe_speed);
    } else {
      ref_speed = min(ref_speed + this->max_acceleration_, safe_speed);
    }
  } else if (ref_speed < this->desired_speed_) {
    ref_speed = min(ref_speed + this->max_acceleration_, this->desired_speed_);
  }

  this->ref_speed_ = ref_speed;

  tk::spline s;
  s.set_points(ptsx, ptsy);

  vector<double> next_x_vals;
  vector<double> next_y_vals;

  for (int i = 0; i < prev_size; i++) {
    next_x_vals.push_back(this->previous_path_x_[i]);
    next_y_vals.push_back(this->previous_path_y_[i]);
  }

  double target_x = 30.0;
  double target_y = s(target_x);
  double target_dist = sqrt(target_x*target_x + target_y*target_y);

  double x_add_on = 0;

  for (int i = 1; i <= 50-prev_size; i++) {
    double N = target_dist/(0.02*ref_speed);
    double x_point = x_add_on + (target_x/N);
    double y_point = s(x_point);
    x_add_on = x_point;

    double x_point_map = (x_point - 0) * cos(ref_yaw) - (y_point - 0) * sin(ref_yaw);
    double y_point_map = (x_point - 0) * sin(ref_yaw) + (y_point - 0) * cos(ref_yaw);

    x_point_map += ref_x;
    y_point_map += ref_y;

    next_x_vals.push_back(x_point_map);
    next_y_vals.push_back(y_point_map);
  }

  vector<vector<double>> traj;
  traj.push_back(next_x_vals);
  traj.push_back(next_y_vals);

  this->current_lane_ = new_lane;
  this->current_state_ = next_state;

  return traj;
}

Path_Planner::~Path_Planner() {}
