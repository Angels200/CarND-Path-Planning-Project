//some design of code are inspired from https://github.com/mithi/highway-path-planning/blob/master/src/main.cpp
#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"

#include "Vehicle.h"
#include "Utils.h"
#include "BehaviorPlanner.h"
#include "PathTransform.h"
#include "Trajectory.h"
#include "JMT.h"
#include "Lane.h"

using namespace std;

// for convenience
using json = nlohmann::json;
bool start = false;
// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2( (map_y-y),(map_x-x) );

	double angle = abs(theta-heading);

	if(angle > pi()/4)
	{
		closestWaypoint++;
	}

	return closestWaypoint;

}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = maps_x.size()-1;
	}

	double n_x = maps_x[next_wp]-maps_x[prev_wp];
	double n_y = maps_y[next_wp]-maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();

	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};

}

SimPoints startcar(Vehicle& car, PathTransform& pathTransform){

  const int n = 225;
  const double t = n * DELTA_T;
  const double target_speed = 2.0;
  const double target_s = car.sstate.p + 40.0;

  const State s_sstate = {car.sstate.p, car.sstate.v, 0.0};
  const State s_dstate = {car.dstate.p, 0.0, 0.0};

  const State e_sstate = {target_s, target_speed, 0.0};
  const State e_dstate = {car.dstate.p, 0.0, 0.0};

  JMT jmt_s(s_sstate, e_sstate, t);
  JMT jmt_d(s_dstate, e_dstate, t);

  car.update(e_sstate, e_dstate);

  return pathTransform.compute(jmt_s, jmt_d, DELTA_T, n);
}

int main() {
	uWS::Hub h;
	Tools tools = Tools();
	//cout << "Loading map..." << endl;
	tools.log("Loading map...");
    PathTransform pathTransform = PathTransform("../data/highway_map.csv", HIGHAWAY_TRACK_DISTANCE);
    tools.log("Map loaded...");

    tools.log("Egocar creation...");
	Vehicle egocar(1000);
	egocar.print();

    h.onMessage([&pathTransform,&tools,&start,&egocar](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
            // j[1] is the data JSON object
        	tools.log("Main car localization : State recovering...");
        	// Main car's localization Data
        	double prev_speed ;
        	//if((j[1]["speed"]> LEGAL_SPEED_LIMIT))
        	//	prev_speed = LEGAL_SPEED_LIMIT - 5.0;
        	//else
        		prev_speed = j[1]["speed"];
        		prev_speed *= 0.44704; //convert speed from mph to m/s

        	State sstate {j[1]["s"],prev_speed,0};
			State dstate {j[1]["d"],0,0};

			// Main car's localization Data
			tools.log("Main car localization : X-coordinates recovering...");
			double car_x = j[1]["x"];
			double car_y = j[1]["y"];

          	// Previous path data given to the Planner
			tools.log("Previous path data given to the planner : path_x_y recovering...");
          	auto previous_path_x = j[1]["previous_path_x"];
          	auto previous_path_y = j[1]["previous_path_y"];
          	// Previous path's end s and d values 
          	//double end_path_s = j[1]["end_path_s"];
          	//double end_path_d = j[1]["end_path_d"];

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	tools.log("Sensor fusion recovering...");
          	auto sensor_fusion = j[1]["sensor_fusion"];

          	//Update car object's s,d state components and lane object propreties (current lane, left lane, right lane)
          	tools.log("Egocar update states...");
          	egocar.update(sstate,dstate);
          	//egocar.check_speed();
          	egocar.print();
		  //*********************************
		  //* Generate the XY_points which will be sent to the simulator
		  //*********************************

          	// Our default is to just give back our previous plan
          	int n = previous_path_x.size();
          	SimPoints XY_points = {previous_path_x, previous_path_y, n};

            tools.log_bool(start);
            tools.log_number(n,"Previous_paht");

          	if (!start) {

					//Our car hasn't moved yet. Let's move it!
					tools.log("Starting engine...");
					XY_points = startcar(egocar, pathTransform);
					start = true;
					tools.log("Engine started...");

          	} else if (n < CTRL_REMAINING_PATH_SIZE || egocar.check_speed()) {

					// Our previous plan is about to run out, so append to it
					// Make a list of all relevant information about other cars
					tools.log("Neighbors cars construction...");
					vector<Vehicle> otherCars;

					for (int i = 0; i < sensor_fusion.size(); i++) {
						  int id = sensor_fusion[i][0];
						  double s = sensor_fusion[i][5];
						  double d = sensor_fusion[i][6];
						  double vx = sensor_fusion[i][3];
						  double vy = sensor_fusion[i][4];

						  Vehicle car(id);

						  State sstate {s,sqrt(vx * vx + vy * vy),0};
						  State dstate {d,0,0};

						  car.update(sstate,dstate);
						  otherCars.push_back(car);
					}


					// Decide whether to turn left, turn right or keeplane based on data at hand
					// NOTE: BehaviorPlanner updates our car's current leading/front vehicle's speed and gap
					tools.log("Egocar environment update...");
					egocar.update_environment(otherCars);
					egocar.print();

					tools.log("Planner update...");
					BehaviorPlanner bplanner = BehaviorPlanner();
					bplanner.update(egocar);

					/* This trajectory generates target states updates the egocar states and lane for next time
					// generates a jerk minimized trajectory function
					 given the impending state and suggested behavior*/
					Trajectory trajectory = Trajectory();
					trajectory.update(egocar,bplanner.type);


					/* convert this trajectory in the s-d frame to to discrete XY points
					// the simulator can understand*/
					SimPoints NextXY_points = pathTransform.compute(trajectory.jmtPair[0], trajectory.jmtPair[1], DELTA_T, NUMBER_OF_POINTS);

					NextXY_points.size = NUMBER_OF_POINTS;

					// Append these generated points to the old points
					XY_points.x.insert(XY_points.x.end(), NextXY_points.x.begin(), NextXY_points.x.end());

					XY_points.y.insert(XY_points.y.end(), NextXY_points.y.begin(), NextXY_points.y.end());

					XY_points.size = XY_points.x.size();
          	}
          	json msgJson;
          	//TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
          	msgJson["next_x"] = XY_points.x;
          	msgJson["next_y"] = XY_points.y;
			auto msg = "42[\"control\","+ msgJson.dump()+"]";
			//this_thread::sleep_for(chrono::milliseconds(1000));
			ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      }
      else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
