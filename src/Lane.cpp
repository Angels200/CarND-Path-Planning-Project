/*
 * Lane.cpp
 *
 *  Created on: Nov 5, 2017
 *      Author: naaman
 */
#include <iostream>
#include "Lane.h"



Lane::Lane(const double& d) {
	this->d=d;
	this->type=LaneType::UNKNOWN;
	this->left_type=LaneType::UNKNOWN;
	this->right_type=LaneType::UNKNOWN;
	this->update();

}

Lane::Lane(){
	this->type=LaneType::UNKNOWN;
	this->left_type=LaneType::UNKNOWN;
	this->right_type=LaneType::UNKNOWN;
}
void Lane::update(){
	  if (this->d > 0.0 && this->d <= 4.0) {
		  //correct the d component to the middle of the lane
		  this->d =2;
		  this->type = LaneType::LEFT;
		  this->left_type = LaneType::NONE;
		  this->right_type = LaneType::MID;
	  } else if (this->d > 4.0 && this->d <= 8.0) {
		  this->d =6;
		  this->type = LaneType::MID;
		  this->left_type = LaneType::LEFT;
		  this->right_type = LaneType::RIGHT;
	  } else if (this->d > 8.0 && this->d < 13.0) {
		  this->d =10;
		  this->type = LaneType::RIGHT;
		  this->left_type = LaneType::MID;
		  this->right_type = LaneType::NONE;
	  }
	  else{
		  this->type=LaneType::UNKNOWN;
		  this->left_type = LaneType::UNKNOWN;
		  this->right_type = LaneType::UNKNOWN;
	  }

	  //tools.log("Lane::update");
	  //tools.log_number(this->d,"d: ");
	  //tools.log_enum(this->type);
}

void Lane::set(const double d){
	if(d>=0 && d<13){
		this->d = d;
		this->update();
		return;
	}
	tools.log("d is out the boundaries 0--12");

}

double Lane::target_lane(const BehaviorType b){
	switch(b){
	case BehaviorType::KEEPLANE :
		return this->d;
		break;
	case BehaviorType::TURNLEFT :
		if(this->left_type==LaneType::NONE){ //egoCar is on the left lane
			return this->d;
		}
		else if(this->left_type==LaneType::LEFT){ //egocar is on the mid lane
			return FC_LEFT_D_CENTER;
		}
		else if(this->left_type==LaneType::MID){ //egocar is on the right lane
			return FC_MID_D_CENTER;
		}
		break;
	case BehaviorType::TURNRIGHT :
		if(this->right_type==LaneType::MID){  //egoCar is on the left lane
			return FC_MID_D_CENTER;
		}
		else if(this->right_type==LaneType::RIGHT){//egocar is on the mid lane
			return FC_RIGHT__D_CENTER;
		}
		else if(this->right_type==LaneType::NONE){//egocar is on the right lane
			return this->d;
		}
		break;
	}
}
Lane::~Lane() {
	// TODO Auto-generated destructor stub
}

