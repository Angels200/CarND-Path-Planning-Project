/*
 * BehaviorPlaner.h
 *
 *  Created on: Jan 3, 2018
 *      Author: naaman
 */
#include "Utils.h"
#include "Vehicle.h"
using namespace std;

#ifndef BEHAVIORPLANER_H_
#define BEHAVIORPLANER_H_

class BehaviorPlaner {
public:
	Tools tools;
	BehaviorPlaner();

	void update(Vehicle& egocar, double& ref_vel);
	virtual ~BehaviorPlaner();
};

#endif /* BEHAVIORPLANER_H_ */
