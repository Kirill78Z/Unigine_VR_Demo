#include "Сarriageway.h"
#include "TrafficLane.h"
#include <time.h>
#include "Vehicle.h"


Сarriageway::Сarriageway(TrafficSimulation* trafficSim, Unigine::NodeDummyPtr node)
{
	_trafficSim = trafficSim;
	_node = node;

	//find traffic lanes


	int n = node->findChild("traffic_lanes");
	if (n == -1) return;
	bool changeLaneTracksTransfInited = false;
	Unigine::NodePtr tlsnode = node->getChild(n);
	for (int c = 0; c < tlsnode->getNumChildren(); c++) {
		Unigine::NodePtr tlnode = tlsnode->getChild(c);
		if (tlnode->getType() != Unigine::Node::WORLD_SPLINE_GRAPH) continue;

		assert(tlnode->getWorldRotation() == Unigine::Math::quat::IDENTITY
			&& tlnode->getWorldScale() == Unigine::Math::vec3::ONE);

		int n = tlnode->findProperty("traffic_lane_main");
		if (n == -1) continue;

		if (!changeLaneTracksTransfInited) {
			changeLaneTracksTransf = tlnode->getWorldTransform();
			changeLaneTracksITransf = tlnode->getIWorldTransform();
		}


		TrafficLane* tl = new TrafficLane(trafficSim, this, Unigine::WorldSplineGraph::cast(tlnode));
		trafficLanes.append(tl);

		int num = tl->getNumFromLeftToRight();
		trafficLanesByNum[num].append(tl);

	}

	//init random numbers for vehicle types
	long ltime = time(NULL);
	unsigned int stime = (unsigned int)ltime / 2;
	srand(stime);

	//for each lane calc linear spans of each neighboring lane
	for (int tl = 0; tl < trafficLanes.size(); tl++) {
		trafficLanes[tl]->calcNeighborLanesLinearSpans();
	}

	int a = 0;
}


Сarriageway::~Сarriageway()
{
}

void Сarriageway::update() {

	//update all vehicles
	std::list<Vehicle*>::iterator it;
	for (it = vehicles.begin(); it != vehicles.end(); ++it) {
		if (!(*it)->isReachedEndOfRoad()) {
			(*it)->update();
		}
	}

	//update all traffic lanes
	for (int tl = 0; tl < trafficLanes.size(); tl++) {
		trafficLanes[tl]->update();
	}

	//garbage collection
	if (deletedTempChangeLaneIts.size() > 100) {
		long ltime = time(NULL);
		std::list<Vehicle*>::iterator it = deletedTempChangeLaneIts.begin();
		while (it != deletedTempChangeLaneIts.end())
		{
			if ((*it)->isReachedEndOfRoad()) {
				double t = difftime(ltime, (*it)->reachedEndOfRoadTimeStamp());
				if (t > 60) {
					//удалить итератор только тогда, когда машина, на которую он указывал уже уехала
					//но не удалять саму машину здесь
					deletedTempChangeLaneIts.erase(++it);
					continue;
				}
			}

			//перейти к следующему
			it++;
		}
	}


	if (deletedVehicles.size() > 100) {
		long ltime = time(NULL);
		std::list<Vehicle*>::iterator it = deletedVehicles.begin();
		while (it != deletedVehicles.end())
		{
			double t = difftime(ltime, (*it)->reachedEndOfRoadTimeStamp());
			if (t > 60) {
				//удалить окончательно
				delete (*it);
				deletedVehicles.erase(++it);
			}
			else
			{
				//перейти к следующему
				it++;
			}
		}
	}


}
