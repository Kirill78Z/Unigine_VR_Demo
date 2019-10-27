#include "TrafficLane.h"
#include "Vehicle.h"
#include <UnigineGame.h>


TrafficLane::TrafficLane(TrafficSimulation* trafficSim, Ñarriageway* carriageway, Unigine::WorldSplineGraphPtr node)
{
	worldSplineGraph = node;
	this->trafficSim = trafficSim;
	this->carriageway = carriageway;
	//TODO: validate WorldSplineGraph: all points must be connected one by one from first to last
	//prepare start position and direction for sorting from left to right
	//get intensity for all vehicle types
	//set timeToAddNewVehicle corresponding to summary intensity
}


TrafficLane::~TrafficLane()
{
}


void TrafficLane::update() {
	//update all vehicles on lane
	std::list<Vehicle*>::iterator it;
	for (it = vehicles.begin(); it != vehicles.end(); ++it) {
		(*it)->update();
	}

	//remove vehicle if destination reached
	if (vehicles.end() != vehicles.begin()) {
		Unigine::Vector<std::list<Vehicle*>::iterator> toErase;
		for (it = vehicles.end(); it != vehicles.begin(); ++it) {
			if (it == vehicles.end()) continue;
			if ((*it)->reachedEndOfRoad()) {
				toErase.append(it);
			}
			else break;
		}
		//remove from list end delete
		for (int i = 0; i < toErase.size(); i++) {

			vehicles.erase(toErase[i]);
		}
	}
	
	

	//add new vehicle if needed
	//just for testing: add every 5 sec box-shaped vehicle
	//TODO: add vehicles corresponding to specified intensities
	if (timeToAddNewVehicle >= 0.0f)
	{
		timeToAddNewVehicle -= Unigine::Game::get()->getIFps();
		if (timeToAddNewVehicle < 0.0f)
		{
			//it is time to add new vehicle
			//TODO: need to take into account is there enough space to add new vehicle
			Unigine::MeshPtr mesh = Unigine::Mesh::create();
			mesh->addBoxSurface("dummy", Unigine::Math::vec3(1, 2, 1));
			Unigine::ObjectMeshStaticPtr testVehicleNode = Unigine::ObjectMeshStatic::create(mesh);

			Vehicle* vehicle = new Vehicle(carriageway, this, testVehicleNode->getNode());
			vehicles.push_front(vehicle);

			timeToAddNewVehicle = timeSpanBetweenAddingVehicles;
		}

	}
}