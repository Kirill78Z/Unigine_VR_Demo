#pragma once
#include "TrafficLane.h"

class AdditionalLane : public TrafficLane
{
public:
	AdditionalLane(TrafficSimulation* trafficSim, Ñarriageway* carriageway, Unigine::WorldSplineGraphPtr node);
	~AdditionalLane();
};

