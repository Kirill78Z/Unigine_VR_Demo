#pragma once
#include <UnigineNodes.h>
#include "TrafficSimulation.h"

class TrafficLane;

class �arriageway
{
public:
	�arriageway(TrafficSimulation* trafficSim, Unigine::NodeDummyPtr node);
	~�arriageway();

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

private:
	Unigine::NodeDummyPtr _node;
	TrafficSimulation* _trafficSim;

	Unigine::Vector<TrafficLane*> trafficLanes;
	Unigine::HashMap<int, Unigine::Vector<TrafficLane*>> trafficLanesByNum;
};

