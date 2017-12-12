/*
 * Trajectory.cpp
 *
 *  Created on: Nov 6, 2017
 *      Author: naaman
 */

#include "Trajectory.h"

Trajectory::Trajectory() {
}

void Trajectory::update(Vehicle& egocar, const BehaviorType behavior){
	// TODO Auto-generated constructor stub
	double target_s = egocar.sstate.p + HORIZON * egocar.sstate.v;
	double target_v = egocar.sstate.v;

	// If the car in front is going fast or we are very far from it anyway, go as fast as we can
	// Else let's go a notch slower than the car in front
	double fgap = egocar.sstate.p - egocar.fv_sstate.p;
	if(egocar.current_gap.front <= SAFETY_FRONT_GAP_BUFFER){
		target_v = (egocar.fv_sstate.v- SAFETY_SPEED_BUFFER);
		tools.log_number(fgap, "Front Gap: ");
		tools.log_number(egocar.current_gap.front, "egocar.current_gap.front: ");
		tools.log("Reduce Speed");
	}
	else{//(behavior==BehaviorType::KEEPLANE && egocar.current_gap.front >= SAFETY_FRONT_GAP_BUFFER){
		target_v = LEGAL_SPEED_LIMIT ; //(egocar.fv_sstate.v);
		tools.log("LEGAL_SPEED_LIMIT");
	}
	/*else{
		bool safe = (egocar.fv_sstate.v >= LEGAL_SPEED_LIMIT) || (egocar.current_gap.front >= SAFETY_FRONT_GAP_BUFFER);
		target_v =  safe ? LEGAL_SPEED_LIMIT : (egocar.fv_sstate.v - SAFETY_SPEED_BUFFER);
		// But if the car in front is too slow, let's go a little faster
		target_v = target_v > SAFETY_MIN_SPEED ? target_v : SAFETY_MIN_SPEED;
	}*/
	// Estimate a safe target distance based on our selected speed
	target_s =  egocar.sstate.p + HORIZON * 0.5 * (egocar.sstate.v + target_v); // HORIZON * 0.5 * (egocar.sstate.v + target_v);

	// target position and velocity a
	this->tsstate = {target_s, target_v, 0.0};

	// get target d component state based on behavior
	// target speed and acceleration sideways of the road are both zero
	this->tdstate = {egocar.current_lane.target_lane(behavior), 0.0, 0.0};

	print(egocar);

	// generate JMTs
	JMT jmt_s(egocar.sstate, this->tsstate, HORIZON);
	JMT jmt_d(egocar.dstate, this->tdstate, HORIZON);
	this->jmtPair.push_back(jmt_s);
	this->jmtPair.push_back(jmt_d);

	//update the egocar s state and d state and lane
	egocar.update(this->tsstate,this->tdstate);
}

void Trajectory::print(const Vehicle& egocar){

	tools.log("---------Trajectory-Report-Start----------");
	tools.log("Build Trajectory...");
	tools.log_number(egocar.fv_sstate.v , "Front vehicle  velocity v :");
	tools.log_state(egocar.sstate, "egocar start ");
	tools.log_state(egocar.dstate, "egocar start d ");
	tools.log_state(this->tsstate, "egocar target ");
	tools.log_state(this->tdstate, "egocar target d ");
	tools.log("---------Trajectory-Report-End----------");
}

Trajectory::~Trajectory() {
	// TODO Auto-generated destructor stub
}

