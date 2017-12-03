/*
 * Lane.h
 *
 *  Created on: Nov 5, 2017
 *      Author: naaman
 */
#ifndef LANE_H_
#define LANE_H_

#include "Utils.h"

using namespace std;

class Lane {
public:
	std::vector<double> lane_bounds;
	double d;
	LaneType type,left_type,right_type;
	Lane(const double& d);
	Lane();
	Tools tools=Tools();
	virtual ~Lane();
	void set(const double d);
	double target_lane(const BehaviorType b);

private:
	void update();
};

#endif /* LANE_H_ */
