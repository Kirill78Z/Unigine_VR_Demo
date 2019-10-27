#include "Vehicle.h"



Vehicle::Vehicle(Ñarriageway* carriageway,
	TrafficLane* trafficLane, Unigine::NodePtr node, float velocity)
{
	this->carriageway = carriageway;
	this->trafficLane = trafficLane;
	this->node = node;
	this->velocity = velocity;

	//TODO: get static envelop of node
	//place node into start position (in start of first spline segment with respect to ground position)
}


Vehicle::~Vehicle()
{
}

void Vehicle::update() {
	//if currently changing lane in process continue change line

	//check if there is obstacle at the front in dynamic envelope in current lane
	//if there is obstacle make desision about changing lane
	//start changing lane if possible
	//otherwise slow down

	//if there is no obstacle accelerate to the speed limit

	//set acceleration and velocity...
	//update dynamic envelope
	//update position

	//keep track vehicle position on all traffic lanes of current carrageway
}
