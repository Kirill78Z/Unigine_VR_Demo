#pragma once
#include <UnigineNodes.h>
#include <UnigineHashMap.h>

class �arriageway;

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
	Unigine::Vector<�arriageway*> carriageways;


	//Unigine::HashMap<Unigine::String, Unigine::NodeDummyPtr> vehicles;

	Unigine::NodeDummyPtr vehicles;
};

