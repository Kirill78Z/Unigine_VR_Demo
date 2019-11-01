#include "AdditionalLane.h"



AdditionalLane::AdditionalLane(TrafficSimulation* trafficSim, Ñarriageway* carriageway, Unigine::WorldSplineGraphPtr node)
	: TrafficLane(trafficSim, carriageway, node)
{
	laneType = LaneType::AdditionalLane_;
}


AdditionalLane::~AdditionalLane()
{
}
