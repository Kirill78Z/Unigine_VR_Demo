#pragma once
#include <UnigineWorlds.h>
#include "TrafficSimulation.h"
#include "�arriageway.h"
#include <list>

class Vehicle;

class TrafficLane
{
public:
	TrafficLane(TrafficSimulation* trafficSim, �arriageway* carriageway, Unigine::WorldSplineGraphPtr node);
	~TrafficLane();

	void update();

	Unigine::WorldSplineGraphPtr getWorldSplineGraph() {
		return worldSplineGraph;
	}

private:
	Unigine::WorldSplineGraphPtr worldSplineGraph;
	TrafficSimulation* trafficSim;
	�arriageway* carriageway;

	//vehicles on this lane
	std::list<Vehicle*> vehicles;

	float timeToAddNewVehicle;
	float timeSpanBetweenAddingVehicles = 5.0f;
};

