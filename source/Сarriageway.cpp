#include "Ñarriageway.h"
#include "TrafficLane.h"
#include <time.h>


Ñarriageway::Ñarriageway(TrafficSimulation* trafficSim, Unigine::NodeDummyPtr node)
{
	_trafficSim = trafficSim;
	_node = node;

	//find traffic lanes
	

	int n = node->findChild("traffic_lanes");
	if (n == -1) return;
	Unigine::NodePtr tlsnode = node->getChild(n);
	for (int c = 0; c < tlsnode->getNumChildren(); c++) {
		Unigine::NodePtr tlnode = tlsnode->getChild(c);
		if (tlnode->getType() != Unigine::Node::WORLD_SPLINE_GRAPH) continue;

		int n = tlnode->findProperty("traffic_lane_main");
		if (n == -1) continue;

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


Ñarriageway::~Ñarriageway()
{
}

void Ñarriageway::update() {
	//update all traffic lanes
	for (int tl = 0; tl < trafficLanes.size(); tl++) {
		trafficLanes[tl]->update();
	}
}