#include "TrafficLane.h"
#include "Vehicle.h"
#include "AdditionalLane.h"
#include <UnigineEditor.h>
#include <UnigineGame.h>
#include <algorithm>

TrafficLane::TrafficLane(TrafficSimulation* trafficSim, Сarriageway* carriageway,
	Unigine::WorldSplineGraphPtr node)
{
	worldSplineGraph = node;
	this->trafficSim = trafficSim;
	this->carriageway = carriageway;

	//validate WorldSplineGraph: all points must be connected one by one from first to last with gap where there obstacle
	//prepare list of spline segments from start to end
	//create splines where there are obstacles and save obstacle position
	Unigine::Vector<Unigine::SplinePointPtr> points;
	worldSplineGraph->getSplinePoints(points);

	Unigine::Vector<Unigine::SplineSegmentPtr> segmentsTemp;
	worldSplineGraph->getSplineSegments(segmentsTemp);
	segments.append(segmentsTemp);


	int prevSegEndIndex = 1;
	Unigine::SplineSegmentPtr prevSeg = segments[0];
	if (prevSeg->getEndPoint() != points[1]) {
		assert(false);//something wrong
	}
	LinearPosition startLp = LinearPosition(segments[0], 0, 0);
	segmentPositions.append(startLp);
	double currLinearPos = prevSeg->getLength();
	int startSegCount = segments.size();
	for (int s = 1; s < startSegCount; s++) {
		Unigine::SplineSegmentPtr seg = segments[s];

		float len = seg->getLength();
		double lenPts = (seg->getStartPoint()->getPosition() - seg->getEndPoint()->getPosition()).length();

		if (seg->getStartPoint() != points[prevSegEndIndex])
		{
			//there is gap
			if (seg->getStartPoint() == points[++prevSegEndIndex]) {
				//gap in one segment - obstacle
				LinearPosition obstacleLp = LinearPosition(
					segments[s - 1], currLinearPos, currLinearPos);
				obstacles.append(obstacleLp);

				//add segment to close this gap
				Unigine::SplineSegmentPtr closingGapSeg
					= worldSplineGraph->createSplineSegment(
						points[prevSegEndIndex - 1], -prevSeg->getEndTangent(), prevSeg->getEndUp(),
						seg->getStartPoint(), -seg->getStartTangent(), seg->getStartUp());
				segments.append(closingGapSeg);

				//segmentPositions
				LinearPosition additionalLp =
					LinearPosition(segments[segments.size() - 1],
						currLinearPos, currLinearPos);
				segmentPositions.append(additionalLp);

				currLinearPos += closingGapSeg->getLength();
			}
			else
			{
				assert(false);//something wrong
			}
		}

		//segmentPositions
		LinearPosition lp = LinearPosition(segments[s], currLinearPos, currLinearPos);
		segmentPositions.append(lp);

		currLinearPos += seg->getLength();
		prevSegEndIndex++;
		prevSeg = seg;
	}

	overalLength = currLinearPos;

	std::list< TrafficLane*> neighborLanes;


	barriers.allocate(obstacles.size());


	//get start intensity for all vehicle types

	int n = worldSplineGraph->findProperty("traffic_lane_main");
	if (n != -1) {
		Unigine::PropertyPtr prop = worldSplineGraph->getProperty(n);

		int sumVehPerHour = 0;
		Unigine::HashMap<int, int> absIntencity;

		for (int p = 0; p < prop->getNumParameters(); p++) {
			Unigine::String pn = prop->getParameterName(p);
			if (pn == "leads_to_end_of_road") {
				leadToEndOfRoad = prop->getParameterInt(p) > 0;
			}
			else if (pn == "lane_num_from_left_to_right")
			{
				numFromLeftToRight = prop->getParameterInt(p);
			}
			else if (pn == "transition_length_start")
			{
				transitionLengthStart = prop->getParameterDouble(p);
			}
			else if (pn == "transition_length_end")
			{
				transitionLengthEnd = prop->getParameterDouble(p);
			}
			else if(pn == "barriers")
			{
				/*prop->getpa
				Unigine::NodePtr* barriers =  prop->getParameterNode(p);*/
			}
			//TODO: Also add speed limit for each vehicle type
			else
			{
				//Find vehicle type container node
				int vehTypeContainerI = trafficSim->getVehicles()->findChild(pn);
				assert(vehTypeContainerI >= 0);
				if (vehTypeContainerI < 0) continue;

				int vehPerHour = prop->getParameterInt(p);

				if (vehPerHour > 0) {
					sumVehPerHour += vehPerHour;
					absIntencity.append(vehTypeContainerI, vehPerHour);
				}

			}
		}

		if (absIntencity.size() > 0) {
			//calculate timeSpanBetweenAddingVehicles by sumVehPerHour
			timeSpanBetweenAddingVehicles = 1 / ((float)sumVehPerHour / 3600);

			//calculate probability of vehicle each type
			float prevProbDivider = 0;
			for (auto it = absIntencity.begin(); it != absIntencity.end(); it++) {
				int nodeId = it->key;
				int vehPerHour = it->data;

				float ratio = (float)vehPerHour / sumVehPerHour;
				assert(ratio <= 1.0f);
				float probDivider = prevProbDivider + ratio;
				vehProbability.append(nodeId, probDivider);
				prevProbDivider = probDivider;

				if (std::next(it) == absIntencity.end()) {
					assert(probDivider == 1.0f);
				}

			}
		}

		if (sumVehPerHour == 0) {

		}


	}


}

/*void TrafficLane::scanNeighboringLanes(std::list<TrafficLane *> &neighboringLanes)
{
	LinearSpan* currentLSLeft = nullptr;
	LinearSpan* currentLSLeftInverse = nullptr;


	LinearSpan* currentLSRight = nullptr;
	LinearSpan* currentLSRightInverse = nullptr;


	for (int s = 0; s < segmentPositions.size(); s++) {

		LinearPosition lp = LinearPosition(segmentPositions[s]);


		if (currentLSLeft == nullptr || currentLSRight == nullptr) {
			//searching for start points of lanes
			std::list<TrafficLane*>::iterator i = neighboringLanes.begin();

			while (i != neighboringLanes.end()) {//https://stackoverflow.com/a/596180/8020304
				Position3D pos = (*i)->startOfLane();
				int checkResult = pos.isInFrontOf(&lp);
				if (checkResult != 0) {
					//start of lane founded
					LinearSpan* ls = nullptr;

					ls = new LinearSpan;
					ls->start = lp;
					ls->dataType = DataType::AdditionalLane_;
					ls->data = (*i);


					LinearSpan lsPointingThisLane;
					lsPointingThisLane.start = (*i)->startOfLaneLinear();
					lsPointingThisLane.end = (*i)->endOfLaneLinear();//this can be changed if this line ends earler
					lsPointingThisLane.dataType = this->laneType;
					lsPointingThisLane.data = this;

					if (checkResult == -1) {
						assert(currentLSLeft == nullptr);
						currentLSLeft = ls;
						currentLSLeftInverse = &lsPointingThisLane;
						(*i)->lanesToTheRight.append(lsPointingThisLane);
					}
					else {
						assert(currentLSRight == nullptr);
						currentLSRight = ls;
						currentLSRightInverse = &lsPointingThisLane;
						(*i)->lanesToTheLeft.append(lsPointingThisLane);
					}


					neighboringLanes.erase(i++);
				}
				else
				{
					++i;
				}
			}
		}

		if (currentLSLeft != nullptr || currentLSRight != nullptr) {
			//searching for end points of lanes
			Unigine::Vector< LinearSpan*> lanesToSearchEnd;
			if (currentLSLeft != nullptr)
				lanesToSearchEnd.append(currentLSLeft);
			if (currentLSRight != nullptr)
				lanesToSearchEnd.append(currentLSRight);

			for (int l = 0; l < lanesToSearchEnd.size(); l++) {
				LinearSpan* ls = lanesToSearchEnd[l];
				Position3D endPos = ((MainLane*)ls->data)->endOfLane();

				//Position3D test = lp.getPos3D();

				int checkResult = endPos.isInFrontOf(&lp);
				if (checkResult != 0) {
					//end of lane founded
					ls->end = lp;

					if (checkResult == -1) {
						assert(currentLSLeft == ls);
						lanesToTheLeft.append(*currentLSLeft);
						currentLSLeft = nullptr;
						currentLSLeftInverse = nullptr;
					}
					else
					{
						assert(currentLSRight == ls);
						lanesToTheRight.append(*currentLSRight);
						currentLSRight = nullptr;
						currentLSRightInverse = nullptr;
					}



				}

			}


		}

	}


	//if we still have currentLSLeft or currentLSRight
	//then this lane ends earler than neighbor lane
	if (currentLSLeft) {
		currentLSLeft->end = endOfLaneLinear();
		currentLSLeftInverse->end = ;//чтобы найти это значение нужно ???
		lanesToTheLeft.append(*currentLSLeft);
	}


}
*/

TrafficLane::~TrafficLane()
{
}


void TrafficLane::update() {
	//update all vehicles on lane
	Unigine::Vector<std::list<Vehicle*>::iterator> toErase;
	std::list<Vehicle*>::iterator it;
	for (it = vehicles.begin(); it != vehicles.end(); ++it) {
		if ((*it)->isReachedEndOfRoad()) {
			toErase.append(it);
		}
	}

	//remove from list end delete
	for (int i = 0; i < toErase.size(); i++) {
		//удалить из очеренди на update
		carriageway->vehicles.erase((*toErase[i])->getMainIterator());

		//нужно пока сохранить объекты, так как на них могут быть ссылки
		(*toErase[i])->deleteNode();
		carriageway->deletedVehicles.splice(carriageway->deletedVehicles.end(), vehicles, toErase[i]);
	}


	//add new vehicle if needed
	if (vehProbability.size() == 0) return;

	if (waitingVehicle == nullptr) {
		if (timeToAddNewVehicle >= 0.0f)
		{
			timeToAddNewVehicle -= Unigine::Game::get()->getIFps();
			if (timeToAddNewVehicle < 0.0f)
			{
				//it is time to add new vehicle

				int rn = rand();
				float randomRatio = (float)rn / RAND_MAX;
				int vehContainerId = -1;
				for (auto it = vehProbability.begin(); it != vehProbability.end(); it++) {
					float probDivider = it->data;
					if (randomRatio <= probDivider) {
						vehContainerId = it->key;
						break;
					}

				}

				assert(vehContainerId != -1);

				Unigine::NodePtr car;
				Unigine::NodePtr vehContainer = trafficSim->getVehicles()->getChild(vehContainerId);
				if ((!vehContainer) || vehContainer->getNumChildren() == 0) {
					//dummy car
					int dcn = trafficSim->getVehicles()->findChild("DummyCar");
					assert(dcn >= 0);

					Unigine::NodePtr dummyCar = trafficSim->getVehicles()->getChild(dcn);

					car = dummyCar->clone();
				}
				else
				{
					//get random car from container
					int cn = rand() % vehContainer->getNumChildren();

					Unigine::NodePtr carToClone = vehContainer->getChild(cn);

					car = carToClone->clone();
				}


				car->release();
				Unigine::Editor::get()->addNode(car, 1);
				car->setWorldParent(Unigine::NodePtr::Ptr() /*worldSplineGraph->getNode()*/);



				float speedLimit = 25.0f;//TODO: speed limit
				Unigine::String vehTypeName = vehContainer->getName();
				if (vehTypeName.contains("track", 0) || vehTypeName.contains("bus", 0)) {
					speedLimit = 19.4444;
				}



				Vehicle* vehicle = new Vehicle(carriageway, this,
					Unigine::NodeDummy::cast(car), speedLimit, startOfLaneLinear());

				float velocity = 0;

				getNewVehicleVelocity(vehicle, velocity, speedLimit);

				//take into account is there enough space to add new vehicle
				if (velocity > 0) {
					startNewVehicle(vehicle, velocity);
				}
				else
				{
					waitingVehicle = vehicle;
					waitingVehicle->setEnabled(0);
				}



			}

		}
	}
	else
	{
		float velocity = 0;
		getNewVehicleVelocity(waitingVehicle, velocity, waitingVehicle->getSpeedLimit());

		if (velocity > 0) {
			startNewVehicle(waitingVehicle, velocity);
			waitingVehicle->setEnabled(1);
			waitingVehicle = nullptr;
		}
	}
}


void TrafficLane::getNewVehicleVelocity(Vehicle * vehicle, float &velocity, float speedLimit)
{
	ObstacleType obstacleType;
	LinearPosition obstacleLP = LinearPosition::Null();
	double clearDist = Vehicle::getClearDist(vehicles.begin(),
		startOfLaneLinear(), obstacleType, obstacleLP, this, false);
	if (clearDist == DBL_MAX)
	{
		velocity = speedLimit;
	}
	else
	{
		velocity = vehicle->getVelocityToFitIntoSpan(clearDist);
		if (velocity > speedLimit)
			velocity = speedLimit;
	}
}

void TrafficLane::startNewVehicle(Vehicle * &vehicle, float velocity)
{
	vehicles.push_front(vehicle);
	std::list<Vehicle*>::iterator it = vehicles.begin();
	vehicle->setLaneIterator(it);
	vehicle->setVelocity(velocity);

	carriageway->vehicles.push_front(vehicle);
	vehicle->setMainIterator(carriageway->vehicles.begin());

	timeToAddNewVehicle = timeSpanBetweenAddingVehicles;
}



//получает только неподвижные препятствия
LinearPosition TrafficLane::getNextObstacle(LinearPosition pos, bool ignoreFirst) {
	LinearPosition empty = LinearPosition::Null();

	if (obstacles.size() == 0)
	{

		return empty;
	}

	//binary search for nearest obstacle ahead
	int lpIndex = searchNearestLinearPosAhead(obstacles, 0, obstacles.size() - 1, pos);

	if (lpIndex >= 0) {
		return !ignoreFirst ? obstacles[lpIndex] :
			lpIndex < obstacles.size() - 1 ? obstacles[lpIndex + 1] : empty;
	}
	else
	{

		return empty;
	}

}



void TrafficLane::calcNeighborLanesLinearSpansOneSide(
	Unigine::Vector<TrafficLane*> lanes, bool right) {

	//проход по всем сегментам текущего графа
	//сначала - поиск начала или конца любой из полос
	//если найден конец раньше чем начало - это полоса, которая начинается раньше этой
	//после того как найдено начало полосы, далее искать только ее конец
	//после того как найден конец - опять искать только начало любой полосы

	std::list< TrafficLane*> notFoundedLanes;
	for (int i = 0; i < lanes.size(); i++) {
		notFoundedLanes.push_back(lanes[i]);
	}
	bool firstNeigborFounded = false;
	LinearSpan* currNeighborLaneLS = nullptr;

	for (int s = 0; s < segmentPositions.size(); s++) {

		//поиск начала или конца любой из полос
		if (currNeighborLaneLS == nullptr) {
			std::list<TrafficLane*>::iterator i = notFoundedLanes.begin();
			while (i != notFoundedLanes.end()) {//https://stackoverflow.com/a/596180/8020304

				//поиск начала
				{
					Position3D pos = (*i)->startOfLane();
					LinearPosition lp = LinearPosition(segmentPositions[s]);

#ifdef DEBUG
					double testLen = (lp.getPos3D().absPos - pos.absPos).length();
#endif



					int checkResult = pos.isInFrontOf(&lp);
					if (checkResult != 0) {
						//TODO: можно сверить лево-право

						//найдено начало полосы
						firstNeigborFounded = true;
						currNeighborLaneLS = new LinearSpan();
						currNeighborLaneLS->data = (*i);
						currNeighborLaneLS->start = lp;
						addNeighborLaneLinearSpan(currNeighborLaneLS, right);



						notFoundedLanes.erase(i++);
						continue;//ASSERTION: НЕ УЧТЕН СЛУЧАЙ ПРИ КОТОРОМ 
								//БОЛЕЕ ЧЕМ ОДНА СОСЕДНЯЯ ПОЛОСА НАХОДИТСЯ 
								//НА ПРОМЕЖУТКЕ ОДНОГО СЕГМЕНТА ТЕКУЩЕЙ
					}
				}

				//поиск конца полосы
				if (!firstNeigborFounded) //только если еще не найдено ни одной полосы
				{

					Position3D pos = (*i)->endOfLane();
					LinearPosition lp = LinearPosition(segmentPositions[s]);
					int checkResult = pos.isInFrontOf(&lp);
					if (checkResult != 0) {
						//TODO: можно сверить лево-право

						//найден конец полосы
						firstNeigborFounded = true;

						//добавить промежуток от начала
						LinearSpan* ls = new LinearSpan;
						ls->data = (*i);
						ls->start = this->startOfLaneLinear();
						ls->end = lp;
						addNeighborLaneLinearSpan(ls, right);


						notFoundedLanes.erase(i++);
						continue;
					}
				}

				++i;

			}


		}

		//поиск конца полосы для которой найдено начало
		if (currNeighborLaneLS) {
			TrafficLane*tl = currNeighborLaneLS->data;

			Position3D pos = tl->endOfLane();
			LinearPosition lp = LinearPosition(segmentPositions[s]);
			int checkResult = pos.isInFrontOf(&lp);
			if (checkResult != 0) {
				//TODO: можно сверить лево-право

				//найден конец полосы. Полоса полностью находится в пределах текущей
				currNeighborLaneLS->end = lp;

				//для таких полос необходимо добавить линейный промежуток от начала до конца,
				//указывающий на текущую полосу
				LinearSpan* ls = new LinearSpan;
				ls->data = this;
				ls->start = tl->startOfLaneLinear();
				ls->end = tl->endOfLaneLinear();
				tl->addNeighborLaneLinearSpan(ls, !right);


				currNeighborLaneLS = nullptr;
			}

		}




	}


	if (currNeighborLaneLS) {
		//значит эта полоса заканчивается после текущей
		currNeighborLaneLS->end = this->endOfLaneLinear();
	}


}


void TrafficLane::calcNeighborLanesLinearSpans() {
	//const char* test = worldSplineGraph->getName();


	Unigine::Vector<TrafficLane*> lanesToLeft
		= carriageway->getLanesToLeft(numFromLeftToRight);

	Unigine::Vector<TrafficLane*> lanesToRight
		= carriageway->getLanesToRight(numFromLeftToRight);

	if (lanesToLeft.size() > 0 && lanesToTheLeft.size() == 0)
		calcNeighborLanesLinearSpansOneSide(lanesToLeft, false);
	if (lanesToRight.size() > 0 && lanesToTheRight.size() == 0)
		calcNeighborLanesLinearSpansOneSide(lanesToRight, true);

}

//линейный поиск соседних машин по всей очереди машин на полосе
//запуск линейного поиска должен быть редкой процедурой
void TrafficLane::getNextAndPrevVehicles(
	LinearPosition lp, std::list<Vehicle*>::iterator* result) {

	result[0] = vehicles.end();
	result[1] = vehicles.end();

	bool found = false;

	std::list<Vehicle*>::iterator founded = std::find_if(vehicles.begin(), vehicles.end(),
		[lp, this](Vehicle* v)
	{
		return v->getCurrPosOnLane(this).absLinearPos > lp.absLinearPos;
	});

	if (founded != vehicles.end()) {
		result[1] = founded;
		if (founded != vehicles.begin()) {
			std::list<Vehicle*>::iterator prev = std::prev(founded);
			result[0] = prev;
		}
	}
	else if (founded == vehicles.end() && vehicles.size() > 0) {
		std::list<Vehicle*>::iterator last = std::prev(vehicles.end());
		result[0] = last;
	}
}





