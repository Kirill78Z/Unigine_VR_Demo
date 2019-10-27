#pragma once

#include "Ñarriageway.h";
#include "TrafficLane.h";
class Vehicle
{
public:
	Vehicle(Ñarriageway* carriageway, 
		TrafficLane* trafficLane, Unigine::NodePtr node, float velocity);
	~Vehicle();

	void update();

	void setIterator(std::list<Vehicle*>::iterator it) {
		_it = it;
	}

	bool reachedEndOfRoad() {
		return _reachedEndOfRoad;
	}

private:
	Ñarriageway* carriageway;
	TrafficLane* trafficLane;
	Unigine::NodePtr node;

	//iterator pointing position in traffic lane list
	/*Adding, removing and moving the elements within the list or across several lists does not invalidate the iterators or references.
	An iterator is invalidated only when the corresponding element is deleted.*/
	//https://en.cppreference.com/w/cpp/container/list
	//use splice to move across several lists -- https://stackoverflow.com/a/48330737/8020304
	std::list<Vehicle*>::iterator _it;

	//set true if last point of spline graph reached
	bool _reachedEndOfRoad;

	//TODO: generalize for all lanes of carriageway (can store iterator for trafficLane)
	Unigine::SplineSegmentPtr currentSegment;
	float currentSegmentParam;

	float velocity;
};

