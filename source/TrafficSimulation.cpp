#include "TrafficSimulation.h"
#include <UnigineEditor.h>


TrafficSimulation::TrafficSimulation()
{
	//find all carriageways
	{
		Unigine::NodePtr carriagewaysNode = Unigine::Editor::get()->getNodeByName("carriageways");
		for (int n = 0; n < carriagewaysNode->getNumChildren(); n++) {
			Unigine::NodePtr node = carriagewaysNode->getChild(n);
			if (node->getType() != Unigine::Node::NODE_DUMMY) continue;
			Ñarriageway* cw = new Ñarriageway(this, Unigine::NodeDummy::cast(node));
			carriageways.append(cw);
		}
	}


	//find vehicle parking
	{
		Unigine::NodePtr vehiclesNode = Unigine::Editor::get()->getNodeByName("vehicles");
		for (int n = 0; n < vehiclesNode->getNumChildren(); n++) {
			Unigine::NodePtr node = vehiclesNode->getChild(n);
			if (node->getType() != Unigine::Node::NODE_DUMMY) continue;
			vehicles.append(node->getName(), Unigine::NodeDummy::cast(node));
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