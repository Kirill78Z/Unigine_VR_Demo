#include "TrafficLane.h"



TrafficLane::TrafficLane(TrafficSimulation* trafficSim, Unigine::WorldSplineGraphPtr node)
{
	worldSplineGraph = node;
	this.trafficSim = trafficSim;
}


TrafficLane::~TrafficLane()
{
}


void TrafficLane::update() {
	//update all vehicles on lane

	//remove vehicle if destination reached
	//add new vehicle if needed
}