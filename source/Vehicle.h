#pragma once

#include "Ñarriageway.h";
#include "TrafficLane.h";
class Vehicle
{
public:
	Vehicle(Ñarriageway* carriageway, 
		TrafficLane* trafficLane, Unigine::NodeDummyPtr node);
	~Vehicle();

	void update();

private:
	Ñarriageway* carriageway;
	TrafficLane* trafficLane;
	Unigine::NodeDummyPtr node;
};

