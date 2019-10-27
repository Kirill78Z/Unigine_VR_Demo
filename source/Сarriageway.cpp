#include "Ñarriageway.h"
#include "TrafficLane.h"



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

		TrafficLane* tl = new TrafficLane(trafficSim, this, Unigine::WorldSplineGraph::cast(tlnode));
		trafficLanes.append(tl);
	}

	//TODO: sort trafficLanes from left to right
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