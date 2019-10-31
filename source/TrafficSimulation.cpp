#include "TrafficSimulation.h"
#include "�arriageway.h"
#include <UnigineEditor.h>


TrafficSimulation::TrafficSimulation()
{
	//find vehicle parking
	{
		vehicles = Unigine::NodeDummy::cast(Unigine::Editor::get()->getNodeByName("vehicles"));
		assert(vehicles);
	}

	//find all carriageways
	{
		Unigine::NodePtr carriagewaysNode = Unigine::Editor::get()->getNodeByName("carriageways");
		for (int n = 0; n < carriagewaysNode->getNumChildren(); n++) {
			Unigine::NodePtr node = carriagewaysNode->getChild(n);
			if (node->getType() != Unigine::Node::NODE_DUMMY) continue;
			�arriageway* cw = new �arriageway(this, Unigine::NodeDummy::cast(node));
			carriageways.append(cw);
		}
	}

}



TrafficSimulation::~TrafficSimulation()
{
}

void TrafficSimulation::update() {
	//update all carriageways
	for (int cw = 0; cw < carriageways.size(); cw++) {
		carriageways[cw]->update();
	}
}


Unigine::NodeDummyPtr TrafficSimulation::createCar() {
	//create random car
	return Unigine::NodeDummy::create();
}