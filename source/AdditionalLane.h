#pragma once
#include "TrafficLane.h"

class AdditionalLane : public TrafficLane
{
public:
	AdditionalLane(TrafficSimulation* trafficSim, �arriageway* carriageway, Unigine::WorldSplineGraphPtr node);
	~AdditionalLane();
};

