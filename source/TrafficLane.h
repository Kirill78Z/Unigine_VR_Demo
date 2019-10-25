#pragma once
#include <UnigineWorlds.h>
#include "TrafficSimulation.h"

class TrafficLane
{
public:
	TrafficLane(TrafficSimulation* trafficSim, Unigine::WorldSplineGraphPtr node);
	~TrafficLane();

	void update();

	Unigine::WorldSplineGraphPtr getWorldSplineGraph() {
		return worldSplineGraph;
	}

private:
	Unigine::WorldSplineGraphPtr worldSplineGraph;
	TrafficSimulation* trafficSim;
};

