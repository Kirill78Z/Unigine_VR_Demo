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
	ChangeLane
};


class Vehicle
{
public:
	Vehicle(Сarriageway* carriageway,
		TrafficLane* trafficLane, Unigine::NodeDummyPtr node,
		float speedLimit, LinearPosition startLinPos);
	~Vehicle();

	void update();

	static double getClearDist(std::list<Vehicle*>::iterator nextIt,
		LinearPosition linPos, ObstacleType &obstacleType,
		LinearPosition &obstacleLP, TrafficLane * lane, bool ignoreFirstObstacle);

	//используется только при начале движения
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

	//используется для выбора и удаления старых объектов
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
			//когда машина в состоянии перестроения она занимает сразу 2 полосы
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

	const double trafficJamVelocity = 2.78;

	const double changeLineMinSpaceToPrevVehicle = 30;

	Сarriageway* carriageway;
	TrafficLane* trafficLane;


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

#pragma region сканирование соседних полос
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
		while (pt.isInFrontOf(lp) == 0)
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


		//при перестроении на двух разных полосах существует 2 итератора, указывающих
		//на одну машину.
		//Но как только перестроение заканчивается временный итератор переносится в очередь на удаление
		//Поэтому если текущее состояние машины не ChangeLane
		//уточнить prev и next, чтобы они были равны полю vehicleIterator самого объекта Vehicle
		if (prev != i_null
			&& (*prev)->currentActivity != VehicleActivity::ChangeLane
			&& prev != (*prev)->vehicleIterator) {
			prev = (*prev)->vehicleIterator;
		}
		if (next != i_null
			&& (*next)->currentActivity != VehicleActivity::ChangeLane
			&& next != (*next)->vehicleIterator) {
			next = (*next)->vehicleIterator;
		}


		//проверить, что машины не перестроились на другую полосу
		{
			//убирать из рассмотрения машины, которые поменяли полосы
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
		}


		if (prev == i_null && next == i_null)
		{
#ifdef DEBUG
			carriageway->neighborsChangedLanes++;
#endif
			return false;
		}


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
				posOnNeighborLane.absLinearPos > (*next)->getCurrPosOnLane(neighborLane).absLinearPos)//обогнали следующий автомобиль
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
				posOnNeighborLane.absLinearPos < (*prev)->getCurrPosOnLane(neighborLane).absLinearPos)// автомобиль сзади обогнал нас
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
			assert((*prev)->getCurrPosOnLane(neighborLane).absLinearPos < posOnNeighborLane.absLinearPos);
		}

		if (next != i_null) {
			assert((*next)->getCurrPosOnLane(neighborLane).absLinearPos > posOnNeighborLane.absLinearPos);
		}

		neighborVehiclesOld[0] = prev;
		neighborVehiclesOld[1] = next;

#ifdef DEBUG
		carriageway->succcessfulUpdatedByOldValues++;
#endif

		return true;

	}
#pragma endregion


#pragma region подготовка к перестроению
	//возвращает линейное положение на соседней полосе, на которое можно перестроиться
	LinearPosition canChangeLane(
		LinearSpan* neighborlaneLS, LinearPosition posOnNeighborLane,
		ObstacleType obstacleType, std::list<Vehicle*>::iterator* neighborVehicles,
		float currDynEnv)
	{
		if (!movingThroughObstacle.isEmpty())
			return LinearPosition::Null();

		//если полоса начинается дальше, то проверить начался ли отгон ее ширины
		//если текущая полоса заканчивается, то выполнять перестроение с учетом ее отгона

		double transitionDist = 0;//перестроение заканчивать не раньше 
		//чем заканчивается отгон соседней полосы если она еще не началась
		double distToChangeLineMax;

		LinearPosition startNeighborLaneLP = neighborlaneLS->start;
		TrafficLane* neighborLane = neighborlaneLS->data;

		assert(currLinearPosOnLane.absLinearPos <= neighborlaneLS->end.absLinearPos);


		//расстояние, которое необходимо для перестроении при текущей скорости
		double changeLaneDist = changeLineMinDistance;
		//TODO: сделать зависимость от текущей скорости

		


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



		if (posOnNeighborLane.isEmpty()) return LinearPosition::Null();//такое бывает в самом начале движения

		//есть соседняя полоса

		//препятствие спереди
		LinearPosition obstacleLp = neighborLane->getNextObstacle(posOnNeighborLane, false);
		assert(neighborVehicles);


		//машина спереди
		std::list<Vehicle*>::iterator nextIt = neighborVehicles[1];
		//свободное пространство спереди
		ObstacleType obstacleTypeNeighborLane = ObstacleType::None;
		LinearPosition obstacleLPNeighborLane = LinearPosition::Null();
		double clearDist = getClearDist(nextIt, posOnNeighborLane,
			obstacleTypeNeighborLane, obstacleLPNeighborLane, neighborLane, false);
		if (clearDist != DBL_MAX)
			clearDist += transitionDist;//учесть также и переходный участок 


		if (currDynEnv > clearDist || changeLaneDist + reserveDistBetweenCars >= clearDist) {
			return LinearPosition::Null();//места спереди недостаточно
		}

		//машина сзади
		std::list<Vehicle*>::iterator prevIt = neighborVehicles[0];
		if (prevIt != neighborLane->getQueueEnd()) {

			//сзади есть машина. Мы не должны попадать в ее динамический габарит
			double spaceToPrevCar = posOnNeighborLane.absLinearPos - (*prevIt)->getCurrPosOnLane(neighborLane).absLinearPos;

			if ((*prevIt)->getDynamicEnvelop() > spaceToPrevCar)
			{
				//если мы встряли в пробку, то машина сзади должна нас выпустить если есть небольшой зазор
				if (velocity <= trafficJamVelocity
					&& spaceToPrevCar > changeLineMinSpaceToPrevVehicle) {

				}
				else
				{
					return LinearPosition::Null();
					//места сзади недостаточно, чтобы перестроиться не создавая помех
				}
			}
		}


		//мы готовы перестраиваться
		//найти положение на соседней полосе в конце перестроения и вернуть его



		/*if (obstacleType == ObstacleType::EndOfLane)
			//TODO: Учет доступного расстояния на текущей полосе и ее отгона
		{

		}*/

		//posOnNeighborLane.absLinearPos
		LinearPosition result = LinearPosition(posOnNeighborLane);
		double forwardShift = changeLaneDist - transitionDist;
		if (forwardShift > 0) {
			bool reachedEnd = result.increaseLinearPos(forwardShift);
			assert(!reachedEnd);
		}

		return result;

	}
#pragma endregion


#pragma region непосредственное перестроение
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

#ifdef DEBUG
	//VehicleActivity prevFrameActivity;
	bool endChangeLanePrevFrame = false;
	int endChangeLaneCount = 0;
#endif

#pragma endregion



};

