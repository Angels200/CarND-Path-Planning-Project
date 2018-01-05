/*
 * Utils.h
 *
 *  Created on: Nov 5, 2017
 *      Author: naaman
 */

#ifndef UTILS_H_
#define UTILS_H_
#include <vector>
#include <iostream>
#include <sstream>
using namespace std;

// Here's the duration period for each path plan we send to the controller
const double DELTA_T = 0.02;
const double HORIZON = 2.0;
const int NUMBER_OF_POINTS = int(HORIZON / DELTA_T);

const double INFINITE = 1000000.0;

// the d value of each lane's center in the freenet coordiantes : FC
const double FC_LEFT_D_CENTER = 2;//2.2;
const double FC_MID_D_CENTER = 6.0;
const double FC_RIGHT__D_CENTER = 10; //9.8;

// how much points left for the controller to perform
// before we start planning again
const int  CTRL_REMAINING_PATH_SIZE = 10;

// used as parameter in BehaviorPlanner::get_gap()
const double VEHICLE_FROM_FRONT = 1.0;
const double VEHICLE_FROM_BACK = -1.0;

// Total road distance of the the highway loop
const double HIGHAWAY_TRACK_DISTANCE = 6945.554;

// boundaries of acceptable speed of our vehicle
const double LEGAL_HARD_SPEED_LIMIT = 22.352; // 50mph in m/s
const double LEGAL_SPEED_LIMIT = 20.75;
const double SAFETY_MIN_SPEED = 18; //15.0;

// if the gap is less than this we consider it unsafe to turn
const double SAFETY_FRONT_GAP_THRESH = 20.0;
const double SAFETY_BACK_GAP_THRESH = 5.0;

// This is the buffers we want against the leading front vehicle
// for safety so we don't collide with the vehicle right in front of us
const double SAFETY_FRONT_GAP_BUFFER = SAFETY_FRONT_GAP_THRESH + 10.0;
const double SAFETY_DISTANCE_BUFFER = 5.0;
const double SAFETY_SPEED_BUFFER =  3.0;
const double SAFETY_MAX_ACC = .224;

// Parameters than can be tweaked which affects the cost of each behavior
const double FEASABILITY_MIDLANE_REWARD_FACTOR = 1.4; //0.35; //must be 0 < x < 1
const double FEASABILITY_BACK_GAP_FACTOR = 20.0; // must be more than FRONT_GAP_FACTOR
const double FEASABILITY_FRONT_GAP_FACTOR = 10.0;
const double FEASABILITY_TURN_PENALTY_FACTOR = 0.35; //1.4; // must be x > 1

/*
 * State - stores three variables p: position, v:velocity, and a:acceleration components in the s, or d axis
 */


struct State{
	double p;
	double v;
	double a;
};


/* SimPoints stores two vectors x, y which are the map coordinates to be passed to
 * the simulator. Also holds an the size which is the number of (x, y) pairs
 */
struct SimPoints {
  vector<double> x;
  vector<double> y;
  int size;
};

/*
 * Gap stores the front and back gaps from the ego and front vehicle and back vehicle of the ego's lane
 */
struct Gap{
	double front;
	double back;
};

struct Cost{
	double left;
	double right;
	double current;
};

enum class LaneType {
  LEFT=0, MID=1, RIGHT=2, UNKNOWN=3, NONE=4
};



enum class BehaviorType {
  KEEPLANE, TURNRIGHT, TURNLEFT
};

class Tools{
public:
	Tools(){};
	virtual ~Tools(){};
	void log(const std::string& message){
			cout<< message << endl;
	}
	std::string cast_lane_type(const LaneType& lane_type){
		switch(lane_type){
		case LaneType::LEFT:
			return "LEFT";
			break;
		case LaneType::MID:
			return "MID";
			break;
		case LaneType::RIGHT:
			return "RIGHT";
			break;
		case LaneType::UNKNOWN:
			return "UNKNOWN";
			break;
		case LaneType::NONE:
			return "NONE";
			break;
		}
	}
	std::string cast_behavior_type(const BehaviorType& behavior_type){
			switch(behavior_type){
			case BehaviorType::KEEPLANE:
				return "KEEPLANE";
				break;
			case BehaviorType::TURNLEFT:
				return "TURNLEFT";
				break;
			case BehaviorType::TURNRIGHT:
				return "TURNRIGHT";
				break;
			}
		}

	void log_enum(const LaneType& lane_type){
	  std::string s;
	  (s.append("LaneType : ")).append(cast_lane_type(lane_type));
	  log(s);
	}

	void log_enum(const BehaviorType& behavior_type){
	  std::string s;
	  (s.append("BehaviorType : ")).append(cast_behavior_type(behavior_type));
	  log(s);
	}

	void log_number(int value, string key){
		std:string s;
		(s.append(key)).append(to_string(value));
		log(s);
	}

	void log_state(const State& value, string key){
		std:string s;
		(s.append(key)).append("position :").append(to_string(value.p));
		log(s);
		s="";
		(s.append(key)).append("speed :").append(to_string(value.v));
		log(s);
	}

	void log_bool(const bool& value){
		if(value)
			log("true");
		else
			log("false");
	}

	std::string to_string(int number){
		stringstream  convert;   // stream used for the conversion
		string s;
		convert << number;
		s = convert.str();
		return s;
	}
};



#endif /* UTILS_H_ */
