#pragma once
#include <UnigineNodes.h>
#include <UnigineHashMap.h>

class Ñarriageway;

class TrafficSimulation
{
public:
	TrafficSimulation();
	~TrafficSimulation();

	void update();

	Unigine::NodeDummyPtr createCar();

	Unigine::NodeDummyPtr getVehicles() {
		return  vehicles;
	}

private:
	Unigine::Vector<Ñarriageway*> carriageways;


	//Unigine::HashMap<Unigine::String, Unigine::NodeDummyPtr> vehicles;

	Unigine::NodeDummyPtr vehicles;
};

