/*
 * Trajectory.h
 *
 *  Created on: Nov 6, 2017
 *      Author: naaman
 */

#ifndef TRAJECTORY_H_
#define TRAJECTORY_H_
#include "Utils.h"
#include "JMT.h"
#include "Vehicle.h"
using namespace std;

class Trajectory {
public:

    Trajectory();
	virtual ~Trajectory();
	//Vehicle car;
	Tools tools = Tools();
	BehaviorType btype;
    State tsstate, tdstate;
    void update(Vehicle& car, const BehaviorType behavior);
    JMT get_jmt_s() const ;
    JMT get_jmt_d() const ;
    void print(const Vehicle& egocar);

    std::vector<JMT> jmtPair;
};

#endif /* TRAJECTORY_H_ */
