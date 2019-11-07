#pragma once

#include "Сarriageway.h";
#include "TrafficLane.h";
#include <UnigineObjects.h>


enum VehicleActivity
{
	StraightMove,
	Wait,
	ChangeLine
};


class Vehicle
{
public:
	Vehicle(Сarriageway* carriageway,
		TrafficLane* trafficLane, Unigine::NodeDummyPtr node,
		float speedLimit, LinearPosition startLinPos);
	~Vehicle();

	void update();

	double getClearDist(std::list<Vehicle*>::iterator nextIt,
		LinearPosition linPos, ObstacleType &obstacleType, LinearPosition &obstacleLP);

	void setIterator(std::list<Vehicle*>::iterator it) {
		vehicleIterator = it;
	}

	bool isReachedEndOfRoad() {
		return _reachedEndOfRoad;
	}

	LinearPosition getCurrPosOnLane() {
		return currLinearPosOnLane;
	}

	float getVelocityToFitIntoSpan(double span);

	void setVelocity(float v) {
		velocity = v;
	}

	void setEnabled(int e) {
		node->setEnabled(e);
	}

	float getSpeedLimit() {
		return speedLimit;
	}

private:
	const float reserveDistBetweenCars = 2.0f;

	//measurement unit - meter/(sec^2)
	const float standartAcceleration = 7.0f;
	const float standartDamping = 7.0f;//TODO?: 5.5 for tracks by Babkov's textbook...

	const float timeToWaitOnPaymentCollectionPoint = 0.5;

	Сarriageway* carriageway;
	TrafficLane* trafficLane;


	Unigine::NodeDummyPtr node;

	void moveOnPos(Position3D pos) {
		node->setWorldPosition(pos.absPos);
		node->setWorldDirection(pos.tangent, pos.up, Unigine::Math::AXIS_Y);
	}

	//iterator pointing position in traffic lane list
	/*Adding, removing and moving the elements within the list or across several lists does not invalidate the iterators or references.
	An iterator is invalidated only when the corresponding element is deleted.*/
	//https://en.cppreference.com/w/cpp/container/list
	//use splice to move across several lists -- https://stackoverflow.com/a/48330737/8020304
	std::list<Vehicle*>::iterator vehicleIterator;


	//car length
	float length = 0;

	//dynamic envelop of vehicle depending of current velocity
	float getDynamicEnvelop() {
		return length * 2 + Unigine::Math::pow(velocity, 2) / (2 * standartDamping);
	}

	LinearPosition currLinearPosOnLane = LinearPosition::Null();


	//set true if last point of spline graph reached
	bool _reachedEndOfRoad = false;

	float velocity;
	float speedLimit;


	VehicleActivity currentActivity = VehicleActivity::StraightMove;

	float timeToWait = 0;
	LinearPosition movingThroughObstacle = LinearPosition::Null();


	//current position for neighbor lanes
	bool justChangedLane = true;


	float distFromLastNeighborLanesPositionUpdate = 0.0f;

	std::list<LinearSpan*>::iterator laneToTheLeft;
	std::list<LinearSpan*>::iterator laneToTheRight;
	std::list<LinearSpan*>::iterator updateNeighborLane(
		std::list<LinearSpan*>::iterator i,
		std::list<LinearSpan*>::iterator end)
	{
		//если текущее положение больше конца отрезка - перейти к след
		for (i; i != end; i++)
		{
			if (currLinearPosOnLane.absLinearPos <= (*i)->end.absLinearPos) {
				return i;
			}

		}

		return end;
	}


	LinearPosition posOnLaneToTheLeft = LinearPosition::Null();
	LinearPosition posOnLaneToTheRight = LinearPosition::Null();

#ifdef DEBUG 
	Unigine::ObjectMeshStaticPtr testMarkerLeft;
	Unigine::ObjectMeshStaticPtr testMarkerRight;
#endif

	void updatePosOnNeighborLane(LinearPosition* lp, LinearSpan* ls) {
		//если не попадаем в промежуток, то lp пустой
		//TODO: Здесь можно учитывать отгоны ширины полосы
		assert(currLinearPosOnLane.absLinearPos <= ls->end.absLinearPos);

		if (currLinearPosOnLane.absLinearPos < ls->start.absLinearPos) {
			lp->splSegment = Unigine::SplineSegmentPtr::Ptr();
			lp->copyFrom(LinearPosition::Null());
			return;
		}

		//если попадаем, то линейный поиск 
		// - от начала полосы если lp пустой
		// - от lp если он не пустой
		if (lp->isEmpty()) {
			lp->copyFrom(ls->data->startOfLaneLinear());
		}

		Position3D pt = currLinearPosOnLane.getPos3D();

#ifdef DEBUG
		int n = 0;
#endif
		while (pt.isParallelLineInFrontOf(lp) == 0)
		{
#ifdef DEBUG
			n++;
#endif
			if (lp->moveToNextSegment()) {
				//spline graph закончился
				lp->copyFrom(LinearPosition::Null());
				break;
			}

		}

	}

	//возвращает линейное положение на соседней полосе, на которое можно перестроиться
	LinearPosition canChangeLane(LinearSpan* neighborlaneLS, LinearPosition posOnNeighborLane,
		ObstacleType obstacleType)
	{
		//если полоса начинается дальше, то проверить начался ли отгон ее ширины
				//если текущая полоса заканчивается, то выполнять перестроение с учетом ее отгона

		double transitionDist = 0;//перестроение заканчивать не раньше чем заканчивается отгон соседней полосы
		double distToChangeLineMax;

		LinearPosition startNeighborLaneLP = neighborlaneLS->start;
		TrafficLane* neighborLane = neighborlaneLS->data;

		LinearPosition neighborLaneLP = LinearPosition::Null();

		if (currLinearPosOnLane.absLinearPos< startNeighborLaneLP.absLinearPos)
			//полоса еще не началась
		{
			//начался ли ее отгон?
			double transitionLen = neighborLane->getTransitionLengthStart();

			if (currLinearPosOnLane.absLinearPos
				< startNeighborLaneLP.absLinearPos- transitionLen) {
				return LinearPosition::Null();
			}
			else
			{
				//закончить перестроение не раньше чем в конце отгона
				//(в начале соседней полосы)
				transitionDist = startNeighborLaneLP.absLinearPos 
					- currLinearPosOnLane.absLinearPos;
				posOnNeighborLane = neighborLane->startOfLaneLinear();
			}
		}


		assert(!posOnNeighborLane.isEmpty());
		//есть соседняя полоса
		//проверить расстояние до препятствий на этой полосе
		/*LinearPosition obstacleLp = neighborLane->getNextObstacle(posOnNeighborLane, false);
		std::list<Vehicle*>::iterator* neighborVehicles = neighborLane
			->getNextAndPrevVehicles(posOnNeighborLane);
		LinearPosition prevVehLp = LinearPosition::Null();
		if (neighborVehicles[0] != neighborLane->getQueueEnd()) {
			prevVehLp = (*neighborVehicles[0])->currLinearPosOnLane;
		}
		LinearPosition nextVehLp = LinearPosition::Null();
		if (neighborVehicles[1] != neighborLane->getQueueEnd()) {
			nextVehLp = (*neighborVehicles[1])->currLinearPosOnLane;
		}*/



	}

};

