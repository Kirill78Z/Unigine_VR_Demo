#pragma once
#include <UnigineNodes.h>
#include "TrafficSimulation.h"

class �arriageway
{
public:
	�arriageway(TrafficSimulation* trafficSim, Unigine::NodeDummyPtr node);
	~�arriageway();

	void update();


private:
	Unigine::NodeDummyPtr _node;
	TrafficSimulation* _trafficSim;
};

