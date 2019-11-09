#pragma once

#include "Сarriageway.h";
#include "TrafficLane.h";
#include <UnigineObjects.h>
#include <UnigineVisualizer.h>
#include "UnigineEditor.h"
#include <time.h>


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
		return _reachedEndOfRoad > 0;
	}

	long reachedEndOfRoadTimeStamp() {
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

	void highlight() {
		if (node)
			Unigine::Visualizer::get()->renderNodeBoundBox(node->getNode(),
				Unigine::Math::vec4(0, 0, 1, 1));
	}

	void deleteNode() {
		if (node) {
			Unigine::Editor::get()->removeNode(node->getNode(), 1);
			node.clear();

#ifdef DEBUG 
			Unigine::Editor::get()->removeNode(testMarkerLeft->getNode(), 1);
			testMarkerLeft.clear();
			Unigine::Editor::get()->removeNode(testMarkerRight->getNode(), 1);
			testMarkerRight.clear();
#endif
		}
	}

private:
	const float reserveDistBetweenCars = 2.0f;

	//measurement unit - meter/(sec^2)
	const float standartAcceleration = 7.0f;
	const float standartDamping = 7.0f;//TODO?: 5.5 for tracks by Babkov's textbook...

	const float timeToWaitOnPaymentCollectionPoint = 0.5;

	const double changeLineMinDistance = 10;

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
	long _reachedEndOfRoad = -1;
	void reachedEndOfRoad() {
		_reachedEndOfRoad = time(NULL);
	}

	float velocity;
	float speedLimit;


	VehicleActivity currentActivity = VehicleActivity::StraightMove;

	float timeToWait = 0;
	LinearPosition movingThroughObstacle = LinearPosition::Null();


	//current position for neighbor lanes
	bool justChangedLane = true;


	float distFromLastNeighborLanesScanning = 0.0f;

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


	std::list<Vehicle*>::iterator* neighborVehiclesLeft = nullptr;
	std::list<Vehicle*>::iterator* neighborVehiclesRight = nullptr;

#ifdef DEBUG 
	Unigine::ObjectMeshStaticPtr testMarkerLeft;
	Unigine::ObjectMeshStaticPtr testMarkerRight;

	bool testigVeh = false;

#endif

	void updatePosOnNeighborLane(LinearPosition* lp, LinearSpan* ls) {
		//если не попадаем в промежуток, то lp пустой
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



	void updateNeighborVehicles(std::list<Vehicle*>::iterator* neighborVehicles,
		TrafficLane* neighborLane, LinearPosition posOnNeighborLane) {
#ifdef DEBUG
		carriageway->updateNeighborVehiclesCalls++;
#endif
		if (!updateNeighborVehiclesByOldValues(
			neighborVehicles, neighborLane, posOnNeighborLane)) {
#ifdef DEBUG
			if (neighborLane->getQueueStart() == neighborLane->getQueueEnd())
				carriageway->linearSearchOnEmptyLane++;
#endif
			neighborLane->getNextAndPrevVehicles(posOnNeighborLane, neighborVehicles);
		}
	}



	bool updateNeighborVehiclesByOldValues(
		std::list<Vehicle*>::iterator* neighborVehiclesOld,
		TrafficLane* neighborLane, LinearPosition posOnNeighborLane) {

		assert(neighborVehiclesOld);


		std::list<Vehicle*>::iterator prev = neighborVehiclesOld[0];
		std::list<Vehicle*>::iterator next = neighborVehiclesOld[1];
		std::list<Vehicle*>::iterator i_null = neighborLane->getQueueEnd();

		//проверить что машины не уехали с дороги
		if (prev != i_null && (*prev)->isReachedEndOfRoad())
			prev = i_null;
		if (next != i_null && (*next)->isReachedEndOfRoad())
			next = i_null;


		if (prev == i_null && next == i_null) {
#ifdef DEBUG
			carriageway->noInitialNeighborVehicles++;
#endif
			return false;
		}


		//проверить, что машины не перестроились на другую полосу
		bool prevChangedLane = false;
		if (prev != i_null &&
			(*prev)->trafficLane != neighborLane) {
			prev = i_null;
			prevChangedLane = true;
		}
		bool nextChangedLane = false;
		if (next != i_null &&
			(*next)->trafficLane != neighborLane) {
			next = i_null;
			nextChangedLane = true;
		}

		if (prev == i_null && next == i_null)
		{
#ifdef DEBUG
			carriageway->neighborsChangedLanes++;
#endif
			return false;
		}
		//TODO: Заложить специальную логику если машина начала перестроение 


		if (next == i_null && prev != i_null) {
			next = std::next(prev);//принять next - следующий просле prev
		}

		if (prev == i_null && next != i_null
			&& next != neighborLane->getQueueStart()) {
			prev = std::prev(next);
		}


#ifdef DEBUG 
		bool shiftedForward = false;
		bool shiftedBackward = false;
#endif
		if (next != i_null) {
			//проверить, что перед next нет машин, находящихся перед текущей
			while (next != i_null &&
				posOnNeighborLane.absLinearPos > (*next)->getCurrPosOnLane().absLinearPos)//обогнали следующий автомобиль
			{
				prev = next;
				next = std::next(next);
#ifdef DEBUG
				shiftedForward = true;
#endif
			}
#ifdef DEBUG
			if (shiftedForward) {
				carriageway->shiftedForwardCount++;
			}
#endif
		}

		if (prev != i_null)
		{
			//проверить, что после prev нет машин, находящихся после текущей

			while (prev != i_null &&
				posOnNeighborLane.absLinearPos < (*prev)->getCurrPosOnLane().absLinearPos)// автомобиль сзади обогнал нас
			{
				next = prev;
				prev = prev != neighborLane->getQueueStart() ?
					std::prev(prev) : i_null;
#ifdef DEBUG
				shiftedBackward = true;
#endif
			}

#ifdef DEBUG
			if (shiftedBackward) {
				carriageway->shiftedBackwardCount++;
			}
#endif
		}

		assert(prev != i_null || next != i_null);

		if (prev != i_null) {
			assert((*prev)->getCurrPosOnLane().absLinearPos < posOnNeighborLane.absLinearPos);
		}

		if (next != i_null) {
			assert((*next)->getCurrPosOnLane().absLinearPos > posOnNeighborLane.absLinearPos);
		}

		neighborVehiclesOld[0] = prev;
		neighborVehiclesOld[1] = next;

#ifdef DEBUG
		carriageway->succcessfulUpdatedByOldValues++;
#endif

		return true;

	}


	//возвращает линейное положение на соседней полосе, на которое можно перестроиться
	/*
	LinearPosition canChangeLane(LinearSpan* neighborlaneLS,
		LinearPosition posOnNeighborLane,
		ObstacleType obstacleType, std::list<Vehicle*>::iterator* neighborVehiclesOld)
	{
		//если полоса начинается дальше, то проверить начался ли отгон ее ширины
				//если текущая полоса заканчивается, то выполнять перестроение с учетом ее отгона

		double transitionDist = 0;//перестроение заканчивать не раньше чем заканчивается отгон соседней полосы
		double distToChangeLineMax;

		LinearPosition startNeighborLaneLP = neighborlaneLS->start;
		TrafficLane* neighborLane = neighborlaneLS->data;

		LinearPosition neighborLaneLP = LinearPosition::Null();

		assert(currLinearPosOnLane.absLinearPos <= neighborlaneLS->end.absLinearPos);

		//до конца полосы должно быть достаточно места
		//10 метров
		if (neighborlaneLS->end.absLinearPos - currLinearPosOnLane.absLinearPos < 10) {
			return LinearPosition::Null();
		}


		if (currLinearPosOnLane.absLinearPos < startNeighborLaneLP.absLinearPos)
			//полоса еще не началась
		{
			//начался ли ее отгон?
			double transitionLen = neighborLane->getTransitionLengthStart();

			if (currLinearPosOnLane.absLinearPos
				< startNeighborLaneLP.absLinearPos - transitionLen) {
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

		if (posOnNeighborLane.isEmpty()) return LinearPosition::Null();
		assert(!posOnNeighborLane.isEmpty());//TODO: Как такое может быть?!!!
		//есть соседняя полоса
		//проверить расстояние до препятствий на этой полосе
		LinearPosition obstacleLp = neighborLane->getNextObstacle(posOnNeighborLane, false);


		//проверить валидность neighborVehicles
		std::list<Vehicle*>::iterator* neighborVehicles
			= updateNeighborVehicles(neighborVehiclesOld,
				neighborLane, posOnNeighborLane);


		if (!neighborVehicles) {
			std::list<Vehicle*>::iterator nv[] =
			{ neighborLane->getQueueEnd() , neighborLane->getQueueEnd() };
			neighborLane->getNextAndPrevVehicles(posOnNeighborLane, nv);
			neighborVehicles = nv;
		}



		LinearPosition prevVehLp = LinearPosition::Null();
		if (neighborVehicles[0] != neighborLane->getQueueEnd()) {
			prevVehLp = (*neighborVehicles[0])->currLinearPosOnLane;
		}
		LinearPosition nextVehLp = LinearPosition::Null();
		if (neighborVehicles[1] != neighborLane->getQueueEnd()) {
			nextVehLp = (*neighborVehicles[1])->currLinearPosOnLane;
		}


#ifdef DEBUG
		if (testigVeh) {
			if (!prevVehLp.isEmpty()) {
				Position3D pos = prevVehLp.getPos3D();
				(*neighborVehicles[0])->highlight();
			}

			if (!nextVehLp.isEmpty()) {
				Position3D pos = nextVehLp.getPos3D();
				(*neighborVehicles[1])->highlight();
			}
		}

		if (obstacleType == ObstacleType::None) return LinearPosition::Null();
#endif






		return LinearPosition::Null();

	}
	*/
};

