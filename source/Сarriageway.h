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


	MainLane* getTrafficLane(int num) {
		return trafficLanes[num];
	}

private:
	Unigine::NodeDummyPtr _node;
	TrafficSimulation* _trafficSim;

	Unigine::Vector<MainLane*> trafficLanes;//TODO: sorting of Unigine::Vector
};

