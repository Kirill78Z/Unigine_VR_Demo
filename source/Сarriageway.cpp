#include "Ñarriageway.h"
#include "MainLane.h"
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

		MainLane* tl = new MainLane(trafficSim, this, Unigine::WorldSplineGraph::cast(tlnode));
		trafficLanes.append(tl);
	}

	//init random numbers for vehicle types
	long ltime = time(NULL);
	unsigned int stime = (unsigned int)ltime / 2;
	srand(stime);

	//TODO: sort main traffic lanes from left to right (or use parameters for lanes numbering)
	//And set to left and to right pointers for each main traffic lane

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