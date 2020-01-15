#pragma once
#include <UnigineNode.h>
#include "LinearPosition.h"
class MovingObject
{
public:
	MovingObject(Unigine::NodePtr node);
	~MovingObject();

	void update();

	LinearPosition currLinearPos = LinearPosition::Null();

	float velocity;

	void updatePos() {
		Position3D pos = currLinearPos.getPos3D();
		node->setWorldPosition(pos.absPos);
		node->setWorldDirection(pos.tangent, pos.up, Unigine::Math::AXIS_Y);
	}


private:
	Unigine::NodePtr node;
};

