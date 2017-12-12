/*
 * Vehicle.cpp
 *
 *  Created on: Nov 5, 2017
 *      Author: naaman
 */

#include "Vehicle.h"

Vehicle::Vehicle(const int identifier){
	this->identifier = identifier;
	this->current_gap.front = INFINITE;
	this->current_gap.back = INFINITE;
	this->left_gap.front = INFINITE;
	this->left_gap.back = INFINITE;
	this->right_gap.front = INFINITE;
	this->right_gap.back = INFINITE;
}

void Vehicle::update(const State& sstate, const State& dstate){
	this->sstate = sstate;
	this->dstate = dstate;
	/*if(!initialized){
		this->current_lane = Lane(this->dstate.p);
		initialized = true;
	}
	else{
		this->current_lane.set(dstate.p);
	}*/
	this->current_lane.set(dstate.p);
	
	//tools.log_number(this->identifier,"Car#:");


}

bool Vehicle::check_speed(){
	if(this->current_gap.front < SAFETY_FRONT_GAP_THRESH){
		//this->sstate.v -= this->sstate.v * 0.2;
		return true;
	}
	return false;
}

void Vehicle::update_environment(const std::vector<Vehicle>& otherVehicles){

	for(const Vehicle& otherVehicle: otherVehicles){
		double gap = 0.0;
		gap = Vehicle::compute_gap(otherVehicle,VEHICLE_FROM_FRONT);
		bool front = false;
		if(gap >= 0.0){
			front=true;
		}
		else{
			gap *= -1.0;
		}

		//OtherVehicle and egocar are on the same lane
		if(otherVehicle.current_lane.type==this->current_lane.type){
			if(front && gap < this->current_gap.front){
				tools.log_number(gap, "this->current_gap.front: ");
				this->current_gap.front=gap;
				this->fv_sstate = otherVehicle.sstate;
			}
			if(!front && gap < this->current_gap.back){
				this->current_gap.back = gap;
			}
		}
		else if(otherVehicle.current_lane.type==this->current_lane.left_type){   //otherVehicle is on the left of the egocar
			if(front && gap < this->left_gap.front){
				tools.log_number(gap, "this->current_gap.front: ");
				this->left_gap.front=gap;
			}
			if(!front && gap < this->left_gap.back){
				this->left_gap.back = gap;
			}
		}else if(otherVehicle.current_lane.type==this->current_lane.right_type){ //otherVehicle is on the right of the egocar
			if(front && gap < this->right_gap.front){
				tools.log_number(gap, "this->current_gap.front: ");
				this->right_gap.front=gap;
			}
			if(!front && gap < this->right_gap.back){
				this->right_gap.back = gap;
			}
		}
	}
}
/*
void Vehicle::update_environment(const std::vector<Vehicle>& otherVehicles){

	for(const Vehicle& otherVehicle: otherVehicles){
		double fgap = 0.0;
		fgap = Vehicle::compute_gap(otherVehicle,VEHICLE_FROM_FRONT);
		double bgap = 0.0;
		bgap = Vehicle::compute_gap(otherVehicle,VEHICLE_FROM_BACK);

		//OtherVehicle and egocar are on the same lane
		if(otherVehicle.current_lane.type==this->current_lane.type){
			if(fgap < this->current_gap.front){
				this->current_gap.front=fgap;
				this->fv_sstate = otherVehicle.sstate;
			}
			if(bgap < this->current_gap.back){
				this->current_gap.back = bgap;
			}
		}
		else if(otherVehicle.current_lane.type==this->current_lane.left_type){   //otherVehicle is on the left of the egocar
			if(fgap < this->left_gap.front){
				this->left_gap.front=fgap;
			}
			if(bgap < this->left_gap.back){
				this->left_gap.back = bgap;
			}
		}else if(otherVehicle.current_lane.type==this->current_lane.right_type){ //otherVehicle is on the right of the egocar
			if(fgap < this->right_gap.front){
				this->right_gap.front=fgap;
			}
			if(bgap < this->right_gap.back){
				this->right_gap.back = bgap;
			}
		}
	}
}
*/

double Vehicle::compute_gap(const Vehicle& otherVehicle, const double direction){
	double gap = 0.0;
	gap = (otherVehicle.sstate.p - this->sstate.p); //* direction;
	return gap;
	/*if (gap > 0.0 && gap < INFINITE) {
		  return gap;
	}
	return INFINITE;*/
}

void Vehicle::print(){
	tools.log("---------Car-Report-Start----------");
	tools.log_number(this->identifier,"Id :");
	tools.log("Straight Lane :");
	tools.log_enum(this->current_lane.type);
	tools.log("Left Lane :");
	tools.log_enum(this->current_lane.left_type);
	tools.log("Right Lane :");
	tools.log_enum(this->current_lane.right_type);
	tools.log_state(this->sstate,"Start ");
	tools.log_state(this->dstate, "Start ");
	tools.log_number(this->current_gap.front, "Current Front Gap ");
	tools.log_number(this->current_gap.back, "Current Back Gap ");
	tools.log_number(this->left_gap.front, "Left Front Gap ");
	tools.log_number(this->left_gap.back, "Left Back Gap ");
	tools.log_number(this->right_gap.front, "Right Front Gap ");
	tools.log_number(this->right_gap.back, "Right Back Gap ");
	//tools.log_number(this->fv_sstate.p-this->sstate.p, "Current Front Gap From calculation: ");
	tools.log("---------Car-Report-End----------");
}

Vehicle::~Vehicle() {
	// TODO Auto-generated destructor stub
}

