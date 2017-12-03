/*
 * Vehicle.h
 *
 *  Created on: Nov 5, 2017
 *      Author: naaman
 */

//#include "Utils.h"
#include "Lane.h"

#ifndef VEHICLE_H_
#define VEHICLE_H_

using namespace std;

class Vehicle {
public:

	int identifier;
	//double s, d,v,front_gap,front_v,front_s;
	bool initialized = false;
	Tools tools = Tools();
	State sstate,dstate,fv_sstate;
	Gap current_gap,right_gap,left_gap; //Current gap, Right gap, left gap
	Lane current_lane=Lane();//, right_lane=Lane(8.0),left_lane=Lane(2.0);
	//Lane current_lane=Lane(); //, right_lane=Lane(),left_lane=Lane();

	Vehicle(const int identifier);
	virtual ~Vehicle();

	void print();
	void update(const State& sstate, const State& dstate);
	void update_environment(const std::vector<Vehicle>& otherVehicles);
	bool check_speed();
private:

	double compute_gap(const Vehicle& otherVehicle, const double direction);
};

#endif /* VEHICLE_H_ */
