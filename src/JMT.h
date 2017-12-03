/*
 * JMT.h
 *
 *  Created on: Nov 7, 2017
 *      Author: naaman
 */

#ifndef JMT_H_
#define JMT_H_

#include <iostream>
#include "../src/Eigen-3.3/Eigen/Core"
#include "../src/Eigen-3.3/Eigen/QR"
#include "../src/Eigen-3.3/Eigen/Dense"
#include "Utils.h"

class JMT {

  public:
    Eigen::VectorXd c;
    JMT(const State& start, const State& end, const double t);
    virtual ~JMT();
    double get(const double t) const;
};


#endif /* JMT_H_ */
