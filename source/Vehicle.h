#pragma once

#include "�arriageway.h";
#include "TrafficLane.h";
class Vehicle
{
public:
	Vehicle(�arriageway* carriageway, 
		TrafficLane* trafficLane, Unigine::NodeDummyPtr node);
	~Vehicle();

	void update();

private:
	�arriageway* carriageway;
	TrafficLane* trafficLane;
	Unigine::NodeDummyPtr node;
};

