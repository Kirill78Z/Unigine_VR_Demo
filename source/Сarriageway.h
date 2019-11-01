#pragma once
#include <UnigineNodes.h>
#include "TrafficSimulation.h"

class MainLane;

class Ñarriageway
{
public:
	Ñarriageway(TrafficSimulation* trafficSim, Unigine::NodeDummyPtr node);
	~Ñarriageway();

	void update();

private:
	Unigine::NodeDummyPtr _node;
	TrafficSimulation* _trafficSim;

	Unigine::Vector<MainLane*> trafficLanes;//TODO: sorting of Unigine::Vector
};

