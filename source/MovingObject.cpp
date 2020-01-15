#include "MovingObject.h"
#include <UnigineGame.h>


MovingObject::MovingObject(Unigine::NodePtr node)
{
	this->node = node;
}


MovingObject::~MovingObject()
{

}

void MovingObject::update()
{
	float time = Unigine::Game::get()->getIFps();

	float s = velocity * time;

	if (s > 0) {
		currLinearPos.increaseLinearPos(s);
		updatePos();
	}

	//TODO: Если достиг конца то нужно удалить этот объект!
}
