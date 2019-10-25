#pragma once
#include <UnigineNodes.h>
#include <UnigineHashMap.h>
#include "Ñarriageway.h"


class TrafficSimulation
{
public:
	TrafficSimulation();
	~TrafficSimulation();

	void update();

	Unigine::NodeDummyPtr createCar();

private:
	Unigine::Vector<Ñarriageway*> carriageways;


	Unigine::HashMap<Unigine::String, Unigine::NodeDummyPtr> vehicles;
};

