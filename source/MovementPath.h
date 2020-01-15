#pragma once
#include <UnigineWorlds.h>
#include "MovingObject.h"

class MovementPath
{
public:
	MovementPath(Unigine::WorldSplineGraphPtr node);
	~MovementPath();

	void update();


private:
	Unigine::WorldSplineGraphPtr worldSplineGraph;

	Unigine::Vector<MovingObject*> movingObjects;

	Unigine::SplineSegmentPtr startSegment;

	LinearPosition getStartOfPath() {
		//generate new struct
		LinearPosition lp = LinearPosition(startSegment, 0, 0);
		return lp;
	}
};

