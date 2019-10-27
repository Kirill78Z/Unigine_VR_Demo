#pragma once
#include <UnigineNodes.h>
#include "TrafficSimulation.h"

class TrafficLane;

class �arriageway
{
public:
	�arriageway(TrafficSimulation* trafficSim, Unigine::NodeDummyPtr node);
	~�arriageway();

	void update();


private:
	Unigine::NodeDummyPtr _node;
	TrafficSimulation* _trafficSim;

	Unigine::Vector<TrafficLane*> trafficLanes;
};

