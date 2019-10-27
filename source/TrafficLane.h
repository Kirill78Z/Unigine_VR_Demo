#pragma once
#include <UnigineWorlds.h>
#include "TrafficSimulation.h"
#include "Ñarriageway.h"
#include <list>

class Vehicle;

class TrafficLane
{
public:
	TrafficLane(TrafficSimulation* trafficSim, Ñarriageway* carriageway, Unigine::WorldSplineGraphPtr node);
	~TrafficLane();

	void update();

	Unigine::WorldSplineGraphPtr getWorldSplineGraph() {
		return worldSplineGraph;
	}

private:
	Unigine::WorldSplineGraphPtr worldSplineGraph;
	TrafficSimulation* trafficSim;
	Ñarriageway* carriageway;

	//vehicles on this lane
	std::list<Vehicle*> vehicles;

	float timeToAddNewVehicle;
	float timeSpanBetweenAddingVehicles = 5.0f;
};

