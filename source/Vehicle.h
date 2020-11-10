#pragma once

#include "�arriageway.h";
#include "TrafficLane.h";
#include <UnigineObjects.h>
#include <UnigineVisualizer.h>
#include "UnigineEditor.h"
#include <time.h>


enum VehicleActivity
{
	StraightMove,
	Wait,
	ChangeLane
};

enum ChangeLaneBehabior
{
	Standard,
	Persistent,
	Aggressive
};


class Vehicle
{
public:
	Vehicle(�arriageway* carriageway,
		TrafficLane* trafficLane, Unigine::NodeDummyPtr node,
		float speedLimit, LinearPosition startLinPos);
	~Vehicle();

	void update();

	static double getClearDist(std::list<Vehicle*>::iterator nextIt,
		LinearPosition linPos, ObstacleType& obstacleType,
		LinearPosition& obstacleLP, TrafficLane* lane, bool ignoreFirstObstacle);

	//������������ ������ ��� ������ ��������
	void setLaneIterator(std::list<Vehicle*>::iterator it) {
		vehicleIterator = it;
	}

	void setMainIterator(std::list<Vehicle*>::iterator it) {
		mainIterator = it;
	}

	std::list<Vehicle*>::iterator getMainIterator() {
		return mainIterator;
	}

	bool isReachedEndOfRoad() {
		return _reachedEndOfRoad > 0;
	}

	//������������ ��� ������ � �������� ������ ��������
	long reachedEndOfRoadTimeStamp() {
		return _reachedEndOfRoad;
	}


	LinearPosition getCurrPosOnLane(TrafficLane* lane)
	{
		if (currentActivity != VehicleActivity::ChangeLane)
		{
			assert(lane == trafficLane);
			return currLinearPosOnLane;
		}
		else
		{
			assert(trafficLaneChangeTo != nullptr);
			//����� ������ � ��������� ������������ ��� �������� ����� 2 ������
			if (lane == trafficLane) return currLinearPosOnLane;
			else if (lane == trafficLaneChangeTo) return tempLPosOnLaneChangeTo;
			else assert(false);

		}
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
			node = Unigine::NodeDummyPtr::Ptr();
		}

		currLinearPosOnLane = LinearPosition::Null();
		movingThroughObstacle = LinearPosition::Null();
		posOnLaneAfterChange = LinearPosition::Null();
		tempLPosOnLaneChangeTo = LinearPosition::Null();
		posOnLaneToTheLeft = LinearPosition::Null();
		posOnLaneToTheRight = LinearPosition::Null();

		changeLaneTrack = Unigine::WorldSplineGraphPtr::Ptr();
		changeLaneTrackSeg = Unigine::SplineSegmentPtr::Ptr();

		trafficLaneChangeTo = nullptr;
		neighborVehiclesLeft = nullptr;
		neighborVehiclesRight = nullptr;


		laneToTheLeft = trafficLane->lanesToTheLeftBegin();
		laneToTheRight = trafficLane->lanesToTheRightBegin();

		if (neighborVehiclesLeft)
			delete[] neighborVehiclesLeft;
		if (neighborVehiclesRight)
			delete[] neighborVehiclesRight;



		vehicleIterator = trafficLane->getQueueEnd();
		tempChangeLaneIt = trafficLane->getQueueEnd();
		trafficLane = nullptr;
		mainIterator = carriageway->vehicles.end();
		carriageway = nullptr;

	}

private:
	const float reserveDistBetweenCars = 2.0f;

	//measurement unit - meter/(sec^2)
	const float standartAcceleration = 7.0f;
	const float fastAcceleration = 21.0f;
	const float urgentAcceleration = 35.0f;

	const float standartDamping = 7.0f;//TODO?: 5.5 for tracks by Babkov's textbook...
	const float fastDamping = 21.0f;
	const float urgentDamping = 35.0f;

	const float timeToWaitOnPaymentCollectionPoint = 0.5;

	const double changeLaneMinDistance = 10;

	//const double denseTrafficVelocity = 11.111;

	const double trafficJamVelocity = 2.78;

	//const double changeLineMinSpaceToPrevVehicle = 30;//TODO: ����������� �� ��������?

	const double needToChangeLaneIfItEndsAfter = 150;//TODO: ����������� �� ��������?
	const double noChangeLaneIfPCPCloser = 50;

	�arriageway* carriageway = nullptr;
	TrafficLane* trafficLane = nullptr;


	Unigine::NodeDummyPtr node;

	void moveOnPos(Position3D pos) {
		node->setWorldPosition(pos.absPos);
		node->setWorldDirection(pos.tangent, pos.up, Unigine::Math::AXIS_Y);
		//TODO: UP direction!

	}

	//iterator pointing position in traffic lane list
	/*Adding, removing and moving the elements within the list or across several lists does not invalidate the iterators or references.
	An iterator is invalidated only when the corresponding element is deleted.*/
	//https://en.cppreference.com/w/cpp/container/list
	//use splice to move across several lists -- https://stackoverflow.com/a/48330737/8020304
	std::list<Vehicle*>::iterator vehicleIterator;


	std::list<Vehicle*>::iterator mainIterator;

	//car length
	float length = 0;

	//dynamic envelop of vehicle depending of current velocity
	float getDynamicEnvelop() {
		return length * 2 + Unigine::Math::pow(velocity, 2) / (2 * standartDamping);
	}

	float getBoldDynamicEnvelop() {
		return length * 2 + Unigine::Math::pow(velocity, 2) / (2 * fastDamping);
	}

	float getAggressiveDynamicEnvelop() {
		return length * 2 + Unigine::Math::pow(velocity, 2) / (2 * urgentDamping);
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
	//bool justChangedLane = true;

#pragma region ������������ �������� �����
	float distFromLastNeighborLanesScanning = 0.0f;

	std::list<LinearSpan*>::iterator laneToTheLeft;
	std::list<LinearSpan*>::iterator laneToTheRight;
	std::list<LinearSpan*>::iterator updateNeighborLane(
		std::list<LinearSpan*>::iterator i,
		std::list<LinearSpan*>::iterator end)
	{
		//���� ������� ��������� ������ ����� ������� - ������� � ����
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
	bool testigVeh = false;
#endif

	void updatePosOnNeighborLane(LinearPosition* lp, LinearSpan* ls) {
		//���� �� �������� � ����������, �� lp ������
		assert(currLinearPosOnLane.absLinearPos <= ls->end.absLinearPos);

		if (currLinearPosOnLane.absLinearPos < ls->start.absLinearPos) {
			lp->splSegment = Unigine::SplineSegmentPtr::Ptr();
			lp->copyFrom(LinearPosition::Null());
			return;
		}

		//���� ��������, �� �������� ����� 
		// - �� ������ ������ ���� lp ������
		// - �� lp ���� �� �� ������
		if (lp->isEmpty()) {
			lp->copyFrom(ls->data->startOfLaneLinear());
		}

		Position3D pt = currLinearPosOnLane.getPos3D();

#ifdef DEBUG
		int n = 0;
#endif
		while (pt.isInFrontOf(lp) == 0)
		{
#ifdef DEBUG
			n++;
#endif
			if (lp->moveToNextSegment()) {
				//spline graph ����������
				lp->copyFrom(LinearPosition::Null());
				break;
			}

		}

	}

#ifdef DEBUG
	int _updateNeighborVehicles = 0;
#endif

	void updateNeighborVehicles(std::list<Vehicle*>::iterator* neighborVehicles,
		TrafficLane* neighborLane, LinearPosition posOnNeighborLane) {
#ifdef DEBUG
		carriageway->updateNeighborVehiclesCalls++;
		_updateNeighborVehicles = 1;
#endif
		/*
		if (!updateNeighborVehiclesByOldValues(
			neighborVehicles, neighborLane, posOnNeighborLane)) {
#ifdef DEBUG
			_updateNeighborVehicles == 2;
			if (neighborLane->getQueueStart() == neighborLane->getQueueEnd()) {
				carriageway->linearSearchOnEmptyLane++;
				neighborVehicles[0] = neighborLane->getQueueEnd();
				neighborVehicles[1] = neighborLane->getQueueEnd();
			}

#endif
*/
		neighborLane->getNextAndPrevVehicles(posOnNeighborLane, neighborVehicles);
		//}
	}

#ifdef DEBUG

#endif

	bool updateNeighborVehiclesByOldValues(
		std::list<Vehicle*>::iterator* neighborVehiclesOld,
		TrafficLane* neighborLane, LinearPosition posOnNeighborLane) {

		assert(neighborVehiclesOld);


		std::list<Vehicle*>::iterator prev = neighborVehiclesOld[0];
		std::list<Vehicle*>::iterator next = neighborVehiclesOld[1];
		std::list<Vehicle*>::iterator i_null = neighborLane->getQueueEnd();

		if ((*prev) == nullptr)
			prev = i_null;
		if ((*next) == nullptr)
			next = i_null;


		//��������� ��� ������ �� ������ � ������
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


		//��� ������������ �� ���� ������ ������� ���������� 2 ���������, �����������
		//�� ���� ������.
		//�� ��� ������ ������������ ������������� ��������� �������� ����������� � ������� �� ��������
		//������� ���� ������� ��������� ������ �� ChangeLane
		//�������� prev � next, ����� ��� ���� ����� ���� vehicleIterator ������ ������� Vehicle
		//��� �� ��������?
		//��������� �������� ����� ���� ������ �� ���������
		if (prev != i_null
			&& ((*prev)->currentActivity != VehicleActivity::ChangeLane
				|| (*prev)->vehicleIteratorReplaced)
			&& prev != (*prev)->vehicleIterator) {
			prev = (*prev)->vehicleIterator;
		}
		if (next != i_null
			&& ((*next)->currentActivity != VehicleActivity::ChangeLane
				|| (*next)->vehicleIteratorReplaced)
			&& next != (*next)->vehicleIterator) {
			next = (*next)->vehicleIterator;
		}


		if (prev != i_null && (*prev) == this) {
			prev = i_null;
		}

		if (next != i_null && (*next) == this) {
			next = i_null;
		}

		if (prev == next)
			next = i_null;







		//���������, ��� ������ �� ������������� �� ������ ������
		//������� �� ������������ ������, ������� �������� ������
		if (prev != i_null
			&& (*prev)->currentActivity != VehicleActivity::ChangeLane
			&& (*prev)->trafficLane != neighborLane) {
			prev = i_null;
		}
		if (next != i_null
			&& (*next)->currentActivity != VehicleActivity::ChangeLane
			&& (*next)->trafficLane != neighborLane) {
			next = i_null;
		}

		if (prev != i_null && (*prev)->currentActivity == VehicleActivity::ChangeLane
			&& (*prev)->trafficLane != neighborLane
			&& (*prev)->trafficLaneChangeTo != neighborLane)
			prev = i_null;//������� ���� �� ��������

		if (next != i_null && (*next)->currentActivity == VehicleActivity::ChangeLane
			&& (*next)->trafficLane != neighborLane
			&& (*next)->trafficLaneChangeTo != neighborLane)
			next = i_null;//������� ���� �� ��������


		if (prev == i_null && next == i_null)
		{
#ifdef DEBUG
			carriageway->neighborsChangedLanes++;
#endif
			return false;
		}



		if (next != i_null && prev != i_null && prev != std::prev(next)) {
			next = i_null;
		}


		//#ifdef DEBUG 
		//		bool shrink = false;
		//#endif
		//		if (next != i_null && prev != i_null) {
		//			
		//
		//			assert(prev != next);
		//
		//			std::list<Vehicle*>::iterator nextnext = std::next(next);
		//			assert(prev != nextnext);
		//			if (nextnext != i_null) {
		//				std::list<Vehicle*>::iterator nextnextnext = std::next(std::next(next));
		//				assert(prev != nextnextnext);
		//			}
		//			
		//#ifdef DEBUG 
		//			int shrinkn = 0;
		//#endif
		//			//���������, ��� ����� ���� ��� ������ ����
		//			while (prev != std::prev(next))//TODO: ������������� � ����������� ���� => ���� ��������� � ������ ��������
		//			{
		//				if ((*prev)->currentActivity != VehicleActivity::ChangeLane)
		//					assert((*prev)->trafficLane == neighborLane);
		//				else
		//					assert((*prev)->trafficLane == neighborLane
		//						|| (*prev)->trafficLaneChangeTo == neighborLane);
		//				if (next != i_null) {
		//					if ((*next)->currentActivity != VehicleActivity::ChangeLane)
		//						assert((*next)->trafficLane == neighborLane);
		//					else
		//						assert((*next)->trafficLane == neighborLane
		//							|| (*next)->trafficLaneChangeTo == neighborLane);
		//				}
		//				
		//
		//
		//				next = std::prev(next);
		//				shrink = true;
		//				shrinkn++;
		//				assert(next != carriageway->deletedTempChangeLaneIts.begin());
		//			}
		//		}
		//		else

#ifdef DEBUG 
		bool next_nextprev = false;
		bool prev_prevnext = false;
#endif

		if (next == i_null && prev != i_null) {
			next = std::next(prev);//������� next - ��������� ������ prev
#ifdef DEBUG 
			bool next_nextprev = true;
#endif
		}
		else if (prev == i_null && next != i_null
			&& next != neighborLane->getQueueStart()) {
			prev = std::prev(next);
#ifdef DEBUG 
			bool prev_prevnext = true;
#endif
		}


		//��� �� �����?
		if (prev != i_null
			&& (*prev)->currentActivity != VehicleActivity::ChangeLane
			&& (*prev)->trafficLane != neighborLane) {
			prev = i_null;
		}
		if (next != i_null
			&& (*next)->currentActivity != VehicleActivity::ChangeLane
			&& (*next)->trafficLane != neighborLane) {
			next = i_null;
		}


		if (prev != i_null && (*prev)->currentActivity == VehicleActivity::ChangeLane
			&& (*prev)->trafficLane != neighborLane
			&& (*prev)->trafficLaneChangeTo != neighborLane)
			prev = i_null;

		if (next != i_null && (*next)->currentActivity == VehicleActivity::ChangeLane
			&& (*next)->trafficLane != neighborLane
			&& (*next)->trafficLaneChangeTo != neighborLane)
			next = i_null;
		//��� �� �����?


#ifdef DEBUG 
		bool shiftedForward = false;
		bool shiftedBackward = false;
#endif
		if (next != i_null) {
			//���������, ��� ����� next ��� �����, ����������� ����� �������
			while (next != i_null &&
				posOnNeighborLane.absLinearPos > (*next)->getCurrPosOnLane(neighborLane).absLinearPos)//�������� ��������� ����������
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
			//���������, ��� ����� prev ��� �����, ����������� ����� �������

			while (prev != i_null &&
				posOnNeighborLane.absLinearPos < (*prev)->getCurrPosOnLane(neighborLane).absLinearPos)// ���������� ����� ������� ���
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
			assert((*prev)->getCurrPosOnLane(neighborLane).absLinearPos <= posOnNeighborLane.absLinearPos);
		}

		if (next != i_null) {
			assert((*next)->getCurrPosOnLane(neighborLane).absLinearPos >= posOnNeighborLane.absLinearPos);
		}

		if (prev != i_null && next != i_null) {
			assert(prev == std::prev(next));
			assert(next == std::next(prev));
		}


		neighborVehiclesOld[0] = prev;
		neighborVehiclesOld[1] = next;

#ifdef DEBUG
		carriageway->succcessfulUpdatedByOldValues++;
		_updateNeighborVehicles == 3;
#endif

		return true;

	}
#pragma endregion


#pragma region ���������� � ������������
	//���������� �������� ��������� �� �������� ������, �� ������� ����� �������������
	LinearPosition changeLaneDecision(
		LinearSpan* neighborlaneLS, LinearPosition posOnNeighborLane,
		ObstacleType obstacleType, std::list<Vehicle*>::iterator* neighborVehicles,
		float currDynEnv, bool closeToImmovableObstacle, ChangeLaneBehabior& changeLaneBehabior, double& changeLaneDist)
	{
		bool trafficJam = velocity < trafficJamVelocity;
		changeLaneBehabior = ChangeLaneBehabior::Standard;
		if (closeToImmovableObstacle || trafficJam)
			changeLaneBehabior = ChangeLaneBehabior::Persistent;

		//if (aggressiveChange)
		//	currDynEnv = getBoldDynamicEnvelop();

		if (!movingThroughObstacle.isEmpty())
			return LinearPosition::Null();

		//���� ������ ���������� ������, �� ��������� ������� �� ����� �� ������
		//���� ������� ������ �������������, �� ��������� ������������ � ������ �� ������

		double transitionDist = 0;//������������ ����������� �� ������ 
		//��� ������������� ����� �������� ������ ���� ��� ��� �� ��������

		LinearPosition startNeighborLaneLP = neighborlaneLS->start;
		TrafficLane* neighborLane = neighborlaneLS->data;

		assert(currLinearPosOnLane.absLinearPos <= neighborlaneLS->end.absLinearPos);


		//����������, ������� ���������� ��� ������������ ��� ������� ��������
		changeLaneDist = changeLaneMinDistance + velocity * 1.1;//����������� �� �������� ���������������

		//���� ����� ������ �������� � ���������� ������������, ��
		//������ ���� ������ ������ ���� ���������� 
		//(�� ��� ���� �� ����������� changeLaneMinDistance)
		if (!trafficLane->getLeadToEndOfRoad())
		{
			double toEndOfThisLane = trafficLane->endOfLaneLinear().absLinearPos - currLinearPosOnLane.absLinearPos;
			if (toEndOfThisLane < changeLaneDist - changeLaneMinDistance) {
				double transitionLenEnd = trafficLane->getTransitionLengthEnd();
				if (toEndOfThisLane + transitionLenEnd < changeLaneDist)
					return LinearPosition::Null();
			}
		}


		if (currLinearPosOnLane.absLinearPos < startNeighborLaneLP.absLinearPos)
			//������ ��� �� ��������
		{
			//������� �� �� �����?
			double transitionLen = neighborLane->getTransitionLengthStart();

			if (currLinearPosOnLane.absLinearPos
				< startNeighborLaneLP.absLinearPos - transitionLen) {
				return LinearPosition::Null();
			}
			else
			{
				//��������� ������������ �� ������ ��� � ����� ������
				//(� ������ �������� ������)
				transitionDist = startNeighborLaneLP.absLinearPos
					- currLinearPosOnLane.absLinearPos;
				posOnNeighborLane = neighborLane->startOfLaneLinear();
			}
		}



		if (posOnNeighborLane.isEmpty()) return LinearPosition::Null();//����� ������ � ����� ������ ��������

		//���� �������� ������

		//����������� �������
		LinearPosition obstacleLp = neighborLane->getNextObstacle(posOnNeighborLane, false);
		assert(neighborVehicles);

		//TODO?: ���� ���������� ���������� �� ������� ������ � �� ������
		//�� ������� ������ ���� ����� ��������� ����������

		//������ �������
		std::list<Vehicle*>::iterator nextIt = neighborVehicles[1];




		//��������� ������������ �������
		ObstacleType obstacleTypeNeighborLane = ObstacleType::None;
		LinearPosition obstacleLPNeighborLane = LinearPosition::Null();


#ifdef DEBUG
		//std::list<Vehicle*>::iterator test1 = std::next(nextIt);
		//std::list<Vehicle*>::iterator test2 = std::prev(nextIt);
		//std::list<Vehicle*>::iterator test3 = std::prev(std::prev(nextIt));
#endif // DEBUG


		double clearDist = getClearDist(nextIt, posOnNeighborLane,
			obstacleTypeNeighborLane, obstacleLPNeighborLane, neighborLane, false);

		if (obstacleTypeNeighborLane == ObstacleType::MovingVehicle
			&& (*nextIt)->velocity > velocity)
			changeLaneBehabior = ChangeLaneBehabior::Persistent;


		if (obstacleTypeNeighborLane == ObstacleType::MovingVehicle && trafficJam
			&& (*nextIt)->velocity > trafficJamVelocity * 1.2)
			changeLaneBehabior = ChangeLaneBehabior::Aggressive;//�� �����, � �������� ������ ����

		//�������� ��� ������������ ������� � ������� ������ �����
		//������ �� ������ ���������
		std::list<Vehicle*>::iterator prevIt = neighborVehicles[0];
		float prevDynEnv = 0;
		switch (changeLaneBehabior)
		{
		case ChangeLaneBehabior::Persistent:
			//currDynEnv = getBoldDynamicEnvelop();
			if (prevIt != neighborLane->getQueueEnd())
				prevDynEnv = (*prevIt)->getBoldDynamicEnvelop();
			break;
		case ChangeLaneBehabior::Aggressive:
			//currDynEnv = getAggressiveDynamicEnvelop();
			if (prevIt != neighborLane->getQueueEnd())
				prevDynEnv = (*prevIt)->getAggressiveDynamicEnvelop();
			break;
		default:
			if (prevIt != neighborLane->getQueueEnd())
				prevDynEnv = (*prevIt)->getDynamicEnvelop();
			break;
		}


		if (changeLaneBehabior == ChangeLaneBehabior::Standard) {
			//�� ��������������� ���� ������ ������� ���������������
			if (nextIt != neighborLane->getQueueEnd() && (*nextIt)->currentActivity == VehicleActivity::ChangeLane)
				return LinearPosition::Null();
		}





		if (clearDist < reserveDistBetweenCars)
			return LinearPosition::Null();//������������ ����� (�������� ���-�� ���� �� ������ ������)

		if (clearDist != DBL_MAX)
			clearDist += transitionDist;//������ ����� � ���������� ������� 



		if (changeLaneBehabior == ChangeLaneBehabior::Aggressive) {
			//�������� ��� ����� ������� ��������
			//�� ������ ���� ������ ������� �� ��������������� �� ���� ������
			if (changeLaneDist + reserveDistBetweenCars + length * 2 >= clearDist
				|| (currDynEnv > clearDist && obstacleType == ObstacleType::MovingVehicle
					&& (*nextIt)->currentActivity == VehicleActivity::ChangeLane
					&& (*nextIt)->trafficLaneChangeTo == trafficLane))
				return LinearPosition::Null();
		}
		else
		{
			//���� ��������� �����������, �� ���������, ��� �� ����� ������ 
			//����� ����� ���������� �������� � ������� ���������
			//����� �� ���������������
			if (changeLaneDist + reserveDistBetweenCars + currDynEnv > clearDist) {
				return LinearPosition::Null();//����� ������� ������������
			}

		}


		//������ �����
		if (prevIt != neighborLane->getQueueEnd()) {

			//����� ���� ������. �� �� ������ �������� � �� ������������ �������
			double spaceToPrevCar = posOnNeighborLane.absLinearPos - (*prevIt)->getCurrPosOnLane(neighborLane).absLinearPos;

			if (prevDynEnv > spaceToPrevCar)
			{
				return LinearPosition::Null();//����� ����� ������������, ����� �������������
			}
		}


		//�� ������ ���������������
		//����� ��������� �� �������� ������ � ����� ������������ � ������� ���

		//posOnNeighborLane.absLinearPos
		LinearPosition result = LinearPosition(posOnNeighborLane);
		double forwardShift = changeLaneDist - transitionDist;
		if (forwardShift > 0) {
			bool reachedEnd = result.increaseLinearPos(forwardShift);
			if (reachedEnd) return LinearPosition::Null();//������ ��� ������������� - �� ���������������
		}




		return result;

	}
#pragma endregion


#pragma region ���������������� ������������
	Unigine::WorldSplineGraphPtr changeLaneTrack;
	Unigine::SplineSegmentPtr changeLaneTrackSeg;

	void createChangeLaneTrack(Position3D start, Position3D end)
	{
		changeLaneTrack->clear();
		using namespace Unigine::Math;
		dvec3 startPos = carriageway->changeLaneTracksITransf * start.absPos;
		dvec3 endPos = carriageway->changeLaneTracksITransf * end.absPos;
		Unigine::SplinePointPtr pt0 = changeLaneTrack->createSplinePoint(startPos);
		Unigine::SplinePointPtr pt1 = changeLaneTrack->createSplinePoint(endPos);

		double tanMult = (startPos - endPos).length() / 4;

		vec3 tan0 = normalize(start.tangent) * tanMult;
		vec3 tan1 = normalize(-end.tangent) * tanMult;


		changeLaneTrackSeg = changeLaneTrack->createSplineSegment(pt0, tan0, start.up,
			pt1, tan1, end.up);
	}


	LinearPosition posOnLaneAfterChange = LinearPosition::Null();
	TrafficLane* trafficLaneChangeTo = nullptr;
	LinearPosition tempLPosOnLaneChangeTo = LinearPosition::Null();
	std::list<Vehicle*>::iterator tempChangeLaneIt;
	float currPosOnChangeLaneTrack = 0.0f;

	float distFromLastChangeLanesScanning = 0.0f;

	double changeLaneDist = 0;

	ChangeLaneBehabior changeLaneBehabior = ChangeLaneBehabior::Standard;

	bool vehicleIteratorReplaced = false;


	void copyNeighborLanesDataTo(Vehicle* veh) {
		veh->laneToTheLeft = laneToTheLeft;
		veh->laneToTheRight = laneToTheRight;
		veh->posOnLaneToTheLeft.copyFrom(posOnLaneToTheLeft);
		veh->posOnLaneToTheRight.copyFrom(posOnLaneToTheRight);
		if (neighborVehiclesLeft != nullptr) {
			veh->neighborVehiclesLeft = new std::list<Vehicle*>::iterator[2]
			{ neighborVehiclesLeft[0], neighborVehiclesLeft[1] };
		}
		if (neighborVehiclesRight != nullptr) {
			veh->neighborVehiclesRight = new std::list<Vehicle*>::iterator[2]
			{ neighborVehiclesRight[0], neighborVehiclesRight[1] };
		}

	}

	bool endChangeLanePrevFrame = false;
#ifdef DEBUG
	int endChangeLaneCount = 0;
#endif

#pragma endregion



};

