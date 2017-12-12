/*
 * BehaviorPlanner.cpp
 *
 *  Created on: Nov 5, 2017
 *      Author: naaman
 */

#include "BehaviorPlanner.h"
#include <algorithm> // std::min_element
#include <iterator>  // std::begin, std::end

BehaviorPlanner::BehaviorPlanner() {
	// TODO Auto-generated constructor stub

}

BehaviorPlanner::~BehaviorPlanner() {
	// TODO Auto-generated destructor stub
}

void BehaviorPlanner::update(const Vehicle& vehicle){
	  this->cost.left = compute_cost(vehicle.left_gap.front,vehicle.left_gap.back,vehicle.current_lane.left_type, "LEFT_COST------");
	  this->cost.right = compute_cost(vehicle.right_gap.front, vehicle.right_gap.back, vehicle.current_lane.right_type, "RIGHT_COST---------");
	  this->cost.current = compute_cost(vehicle.current_gap.front,vehicle.current_gap.back, vehicle.current_lane.type, "STRAIGHT_COST------");

	  this->type = BehaviorType::KEEPLANE;


	  std::vector<double> costs = {this->cost.left,this->cost.right, this->cost.current};
	  auto result = std::min_element(costs.begin(),costs.end());

	  if(*result==this->cost.left){
		  this->type = BehaviorType::TURNLEFT;
	  }
	  else if(*result==this->cost.right){
		  this->type = BehaviorType::TURNRIGHT;
	  }

	  /*
	  switch(vehicle.current_lane.type){
	  case LaneType::RIGHT:
		  if(this->cost.current > this->cost.left){
			  this->type = BehaviorType::TURNLEFT;
		  }
		  break;
	  case LaneType::MID:
		  if(this->cost.current > this->cost.right && this->cost.current > this->cost.left &&
				  this->cost.right >= this->cost.left){
			  this->type = BehaviorType::TURNLEFT;
		  }
		  else if(this->cost.current > this->cost.right && this->cost.current > this->cost.left &&
				  this->cost.left >= this->cost.right){
			  this->type = BehaviorType::TURNRIGHT;
		  }
		  break;
	  case LaneType::LEFT:
		  if(this->cost.current > this->cost.right){
		  			  this->type = BehaviorType::TURNRIGHT;
		  }
		  break;
	  }*/

	  /*if(this->cost.left < this->cost.current && this->cost.left < this->cost.right
			  	  	  && vehicle.current_lane.left_type != LaneType::NONE){
		  this->type = BehaviorType::TURNLEFT;
	  }
	  else if(this->cost.right < this->cost.current && this->cost.right < this->cost.left
			  	  	  && vehicle.current_lane.right_type != LaneType::NONE){
	  		  this->type = BehaviorType::TURNRIGHT;
	  }
	  else if(this->cost.current < this->cost.left && this->cost.current < this->cost.right){
	  		  this->type = BehaviorType::KEEPLANE;
	  }*/

	  this->print();

}

double BehaviorPlanner::compute_cost(const double front_gap, const double back_gap, const LaneType& lane_type, string tcost){

	  const double fcost=front_gap,bcost = back_gap;
	  double cost= fcost + bcost;

	  tools.log_number(fcost,"Front Cost: ");
	  tools.log_number(bcost,"Back Cost: ");
	  tools.log_number(fcost + bcost,"Sum of cost: ");

	  cout << "|... GAP - front: " << front_gap << " back: " << back_gap << endl;
	  tools.log_enum(lane_type);

	  if (lane_type == LaneType::NONE) {

			tools.log("|... No lane. \n |");
			return INFINITE;
	  }

	  if (front_gap < SAFETY_FRONT_GAP_THRESH || back_gap < SAFETY_BACK_GAP_THRESH) {

		tools.log("|... Insufficient space to turn!");
		cout << "|... GAP - front: " << front_gap << " back: " << back_gap << endl;

		return INFINITE;

	  } else {
		cout << "| \n" << "|" << endl;
	  }

	  // We don't want to turn at every opportunity back and forth
	  cost = cost * FEASABILITY_TURN_PENALTY_FACTOR;

	  if (lane_type == LaneType::MID) {
		// we want to reward going in the middle
		cost = cost * FEASABILITY_MIDLANE_REWARD_FACTOR;
	  }

	  tools.log_number(cost,tcost);
	  return 1.0 - cost;
}

void BehaviorPlanner::print(){
	tools.log("---------BehviorPlanner-Report-Start----------");
	tools.log_enum(this->type);
	tools.log_number(this->cost.left,"Left Cost :");
	tools.log_number(this->cost.current,"Current Cost :");
	tools.log_number(this->cost.right,"Right Cost :");
	tools.log_enum(this->type);
	tools.log("---------BehviorPlanner-Report-End----------");
}
