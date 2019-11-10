#pragma once
#include <UnigineNodes.h>
#include "TrafficSimulation.h"
#include <list>
#include <UnigineHashSet.h>


class TrafficLane;
class Vehicle;

class Ñarriageway
{
public:
	Ñarriageway(TrafficSimulation* trafficSim, Unigine::NodeDummyPtr node);
	~Ñarriageway();

	void update();
	void addLane(TrafficLane* lane) {
		trafficLanes.append(lane);
	}

	Unigine::Vector<TrafficLane*> getLanesToLeft(int num) {
		Unigine::Vector<TrafficLane*> result;
		int searchNum = num - 1;
		if (trafficLanesByNum.contains(searchNum)) {
			result.append(trafficLanesByNum[searchNum]);
		}
		return result;
	}


	Unigine::Vector<TrafficLane*> getLanesToRight(int num) {
		Unigine::Vector<TrafficLane*> result;
		int searchNum = num + 1;
		if (trafficLanesByNum.contains(searchNum)) {
			result.append(trafficLanesByNum[searchNum]);
		}
		return result;
	}

#ifdef DEBUG
	int vehn = 0;

	//updateNeighborVehicles
	int updateNeighborVehiclesCalls = 0;
	int shiftedForwardCount = 0;
	int shiftedBackwardCount = 0;
	int succcessfulUpdatedByOldValues = 0;
	int noInitialNeighborVehicles = 0;
	int neighborsChangedLanes = 0;
	int linearSearchOnEmptyLane = 0;
#endif
	//îáùèé ñïèñîê ìàøèí äëÿ çàïóñêà update
	std::list<Vehicle*> vehicles;

	std::list<Vehicle*> deletedVehicles;
	std::list<Vehicle*> deletedTempChangeLaneIts;

	Unigine::Math::dmat4 changeLaneTracksTransf;
	Unigine::Math::dmat4 changeLaneTracksITransf;

private:
	Unigine::NodeDummyPtr _node;
	TrafficSimulation* _trafficSim;

	Unigine::Vector<TrafficLane*> trafficLanes;
	Unigine::HashMap<int, Unigine::Vector<TrafficLane*>> trafficLanesByNum;
};

