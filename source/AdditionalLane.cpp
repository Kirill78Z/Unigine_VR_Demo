#include "AdditionalLane.h"



AdditionalLane::AdditionalLane(TrafficSimulation* trafficSim, �arriageway* carriageway, Unigine::WorldSplineGraphPtr node)
	: TrafficLane(trafficSim, carriageway, node)
{
	laneType = LaneType::AdditionalLane_;
}


AdditionalLane::~AdditionalLane()
{
}
