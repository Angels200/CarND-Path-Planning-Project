/*
 * BehaviorPlanner.h
 *
 *  Created on: Nov 5, 2017
 *      Author: naaman
 */
#include <iostream>
#include "Utils.h"
#include "Vehicle.h"

#ifndef BEHAVIORPLANNER_H_
#define BEHAVIORPLANNER_H_

class BehaviorPlanner {
public:
	Cost cost;
	BehaviorType type;
	Tools tools=Tools();
	BehaviorPlanner();
	virtual ~BehaviorPlanner();
	void update(const Vehicle& vehicle);
	void print();
private:
	double compute_cost(const double front_gap, const double back_gap, const LaneType& lane_type, string tcost);

};

#endif /* BEHAVIORPLANNER_H_ */
