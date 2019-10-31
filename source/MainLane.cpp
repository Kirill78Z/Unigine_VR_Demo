#include "MainLane.h"
#include "Vehicle.h"
#include <UnigineGame.h>
#include <UnigineEditor.h>



MainLane::MainLane(TrafficSimulation* trafficSim,
	Ñarriageway* carriageway, Unigine::WorldSplineGraphPtr graphNode)
{
	worldSplineGraph = graphNode;
	this->trafficSim = trafficSim;
	this->carriageway = carriageway;

	//TODO: get additional lanes


	//validate WorldSplineGraph: all points must be connected one by one from first to last with gap where there obstacle
	//prepare list of spline segments from start to end
	//create splines where there are obstacles and save obstacle position

	Unigine::Vector<Unigine::SplinePointPtr> points;
	graphNode->getSplinePoints(points);
	graphNode->getSplineSegments(segments);

	int prevSegEndIndex = 1;
	Unigine::SplineSegmentPtr prevSeg = segments[0];
	if (prevSeg->getEndPoint() != points[1]) {
		throw std::exception("Invalid spline graph");//something wrong
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
					= graphNode->createSplineSegment(
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
				throw std::exception("Invalid spline graph");//something wrong
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

	//TODO: get intensity for all vehicle types?
	//set timeToAddNewVehicle corresponding to summary intensity


	int n = graphNode->findProperty("traffic_lane_main");
	if (n != -1) {
		Unigine::PropertyPtr prop = graphNode->getProperty(n);

		int sumVehPerHour = 0;
		Unigine::HashMap<int, int> absIntencity;

		for (int p = 0; p < prop->getNumParameters(); p++) {
			Unigine::String pn = prop->getParameterName(p);
			if (pn == "leads_to_end_of_road") {
				leadToEndOfRoad = prop->getParameterInt(p) > 0;
			}//TODO: Also add speed limit for each vehicle type
			else
			{
				//TODO: name of vehicle type
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




	}

}


MainLane::~MainLane()
{

}


void MainLane::update() {
	//update all vehicles on lane
	Unigine::Vector<std::list<Vehicle*>::iterator> toErase;
	std::list<Vehicle*>::iterator it;
	for (it = vehicles.begin(); it != vehicles.end(); ++it) {
		if (!(*it)->isReachedEndOfRoad()) {
			(*it)->update();
		}
		else
		{
			toErase.append(it);
		}

	}

	//remove from list end delete
	for (int i = 0; i < toErase.size(); i++) {
		delete (*toErase[i]);
		vehicles.erase(toErase[i]);
	}


	//add new vehicle if needed
	//just for testing: add every 5 sec box-shaped vehicle
	//TODO: add vehicles corresponding to specified intensities
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
				car->setWorldParent(worldSplineGraph->getNode());



				float speedLimit = 30.5556f;//TODO: speed limit

				Vehicle* vehicle = new Vehicle(carriageway, _num,
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

void MainLane::getNewVehicleVelocity(Vehicle * vehicle, float &velocity, float speedLimit)
{
	ObstacleType obstacleType;
	LinearPosition obstacleLP = LinearPosition::Null();
	double clearDist = vehicle->getClearDist(vehicles.begin(),
		startOfLaneLinear(), obstacleType, obstacleLP);
	if (clearDist == DBL_MAX)
	{
		velocity = speedLimit;
	}
	else
	{
		velocity = vehicle->getVelocityToFitIntoSpan(clearDist);
	}
}

void MainLane::startNewVehicle(Vehicle * &vehicle, float velocity)
{
	vehicles.push_front(vehicle);
	std::list<Vehicle*>::iterator it = vehicles.begin();
	vehicle->setIterator(it);
	vehicle->setVelocity(velocity);


	timeToAddNewVehicle = timeSpanBetweenAddingVehicles;
}



Position3D MainLane::pointByAbsPosOnLane(LinearPosition linearPos) {

	//get spline segment (binary search)
	int segPosI = searchNearestLinearPosÂehind(segmentPositions, 0, segmentPositions.size() - 1, linearPos);

	assert(segPosI >= 0);

	LinearPosition segPos = segmentPositions[segPosI];

	assert(!segPos.isEmpty());


	//get vehicle position on this spline
	float distOnSplineSeg = (float)(linearPos.absLinearPos - segPos.absLinearPos);

	assert(distOnSplineSeg > -UNIGINE_EPSILON);


	if (Unigine::Math::abs(distOnSplineSeg) < UNIGINE_EPSILON)
		distOnSplineSeg = 0;

	Unigine::SplineSegmentPtr seg = /*segments[*/segPos.splSegment/*]*/;

	float segLen = seg->getLength();
	float linearParam = distOnSplineSeg / segLen;

	float nonLinearParam = seg->linearToParametric(linearParam);
	Unigine::Math::dvec3 pt = seg->calcPoint(nonLinearParam);
	Unigine::Math::vec3 tan = seg->calcTangent(nonLinearParam);
	Unigine::Math::vec3 up = seg->calcUpVector(nonLinearParam);

	Position3D vp;
	vp.splSegment = seg;
	vp.absPos = pt;
	vp.tangent = tan;
	vp.up = up;
	return vp;
}


LinearPosition MainLane::getNextObstacle(LinearPosition pos, bool ignoreFirst) {
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




LinearPosition MainLane::linearPosByPoint(Unigine::Math::dvec3 pt, int startSearchSegment) {
	//approximately consider segment as linear:
	//search spline segment which is in front of point starting from startSearchSegment and nearest to pt

	LinearPosition lp = LinearPosition::Null();
	return lp;
}