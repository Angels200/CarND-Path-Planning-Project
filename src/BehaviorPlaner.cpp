/*
 * BehaviorPlaner.cpp
 *
 *  Created on: Jan 3, 2018
 *      Author: naaman
 */

#include "BehaviorPlaner.h"

BehaviorPlaner::BehaviorPlaner() {
	// TODO Auto-generated constructor stub

}

void BehaviorPlaner::update(Vehicle& egocar, double& ref_vel){
	//if ego car is too close to car ahead, then consider switching lanes
	tools.log("Updating Behavior...");
	if(!egocar.close){
		if (ref_vel < 49.5){
			ref_vel += .224;
		}
		return;
	}

	ref_vel -= .224;
	double mingap_ahead = 10*egocar.sstate.v/49.5;//minimum distance required to change lane safely
	double mingap_behind = 20 * 49.5 / egocar.sstate.v;;
	//left lane
	if (egocar.lane == 0)
	{
		if (egocar.back_gap[1] > mingap_behind && egocar.front_gap[1] > mingap_ahead)
		{
			if (egocar.front_speed[1] > egocar.front_speed[egocar.lane] || egocar.front_speed[1]==0)
			{
				egocar.lane = egocar.lane + 1;
			}
		}

	}
	//middle lane
	else if (egocar.lane == 1)
	{
		if (egocar.back_gap[0] > mingap_behind && egocar.front_gap[0] > mingap_ahead && egocar.front_speed[0] == 0)
		{
			egocar.lane = egocar.lane - 1;
		}
		else if (egocar.back_gap[2] > mingap_behind && egocar.front_gap[2] > mingap_ahead && egocar.front_speed[2] == 0)
		{
			egocar.lane = egocar.lane + 1;
		}
		else if (egocar.front_gap[0] > egocar.front_gap[2] && egocar.back_gap[0] > mingap_behind && egocar.front_gap[0] > mingap_ahead)
		{
			if (egocar.front_speed[0] > egocar.front_speed[egocar.lane] || egocar.front_gap[0] > 60)
			{
				egocar.lane = egocar.lane - 1;
			}
		}
		else if (egocar.front_gap[2] > egocar.front_gap[0] && egocar.back_gap[2] > mingap_behind && egocar.front_gap[2] > mingap_ahead)
		{
			if (egocar.front_speed[2] > egocar.front_speed[egocar.lane] || egocar.front_gap[2]>60)
			{
				egocar.lane = egocar.lane + 1;
			}
		}
	}
	//right lane
	else
	{
		if (egocar.back_gap[1] > mingap_behind && egocar.front_gap[1] > mingap_ahead)
		{
			if (egocar.front_speed[1] > egocar.front_speed[egocar.lane] || egocar.front_speed[1] == 0)
			{
				egocar.lane = egocar.lane - 1;
			}
		}
	}

}

BehaviorPlaner::~BehaviorPlaner() {
	// TODO Auto-generated destructor stub
}

