/*
 * Vehicle.h
 *
 *  Created on: Jan 2, 2018
 *      Author: naaman
 */
#include <vector>
#include "Utils.h"
using namespace std;

#ifndef VEHICLE_H_
#define VEHICLE_H_

class Vehicle {
public:
	bool close = false;
	Tools tools;
	std::vector<double> front_gap; //vector containing the distance between the ego car and the nearest vehicle ahead for each lane
	std::vector<double> back_gap; //vector containing the distance between the ego car and the nearest vehicle behind for each lane
	std::vector<double> front_speed; //speed of the cars ahead of us
	State sstate, dstate;
	int lane;
	Vehicle(int lane);
	void update(const double& s, const double& d, const double& speed);
	void set_lane(const double& d);
	void update_environment(const Vehicle& otherVehicle);

	virtual ~Vehicle();
};


#endif /* VEHICLE_H_ */
