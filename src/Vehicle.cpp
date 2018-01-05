/*
 * Vehicle.cpp
 *
 *  Created on: Jan 2, 2018
 *      Author: naaman
 */

#include "Vehicle.h"

Vehicle::Vehicle(int lane) {
	// TODO Auto-generated constructor stub
	this->lane = lane;
	tools.log("Initializating Vehicle...");
	for (int i = 0; i < 3; i++)
	{
		this->front_gap.push_back(99999999.00);
		this->back_gap.push_back(99999999.00);
		this->front_speed.push_back(99999999.00);
		tools.log_number(this->front_gap[i], "this->front_gap[i]: ");
	}

}

void Vehicle::update(const double& s, const double& d, const double& speed){
	tools.log("Updating Vehicle...");

	this->sstate.p = s;
	this->sstate.v = speed;
	this->sstate.a = 0.0;

	this->dstate.p = d;
	this->dstate.v = 0.0;
	this->dstate.a = 0.0;
	tools.log_number(this->dstate.p, "d: ");
}

void Vehicle::set_lane(const double& d){
	tools.log("Setting  Vehicle lane...");
	if (d < 4.0 && d > 0.0){
		tools.log("d < 4.0 && d > 0.0");
		this->lane = 0;
	}
	else if (d < 8.0 && d >= 4.0){
		tools.log("d < 8.0 && d >= 4.0");
		this->lane = 1;
	}
	else{
		tools.log(">= 8.0");
		this->lane = 2;
	}
}


//This method is used only by the egocar : This - otherVehicle : neighbor car provided by the sensor fusion data
void Vehicle::update_environment(const Vehicle& otherVehicle){
	tools.log("Updating Environment...");

	if (otherVehicle.sstate.p > this->sstate.p){
		if ((otherVehicle.sstate.p - this->sstate.p) < this->front_gap[otherVehicle.lane]){

			this->front_gap[otherVehicle.lane] = otherVehicle.sstate.p - this->sstate.p;
			this->front_speed[otherVehicle.lane] = otherVehicle.sstate.v;
		}
	}
	else{

		if ((this->sstate.p - otherVehicle.sstate.p) < this->back_gap[otherVehicle.lane]){
			this->back_gap[otherVehicle.lane] = this->sstate.p - otherVehicle.sstate.p;
		}
	}

	if(otherVehicle.lane == this->lane){
		double gap = otherVehicle.sstate.p - this->sstate.p;
		//check s values greater than mine and s gap
		if((otherVehicle.sstate.p > this->sstate.p) && ((gap) < 30)){
				this->close = true;
		}
	}
}



Vehicle::~Vehicle() {
	// TODO Auto-generated destructor stub
}

