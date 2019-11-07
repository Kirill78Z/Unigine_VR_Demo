#include "Vehicle.h"
#include <UnigineGame.h>
#include "UnigineEditor.h"



Vehicle::Vehicle(Сarriageway* carriageway,
	TrafficLane* trafficLane, Unigine::NodeDummyPtr node,
	float speedLimit, LinearPosition startLinPos)
{
	this->carriageway = carriageway;
	this->trafficLane = trafficLane;
	this->node = node;
	this->speedLimit = speedLimit;

	//get length of node
	Unigine::WorldBoundBox bb = node->getWorldBoundBox();
	length = bb.getMax().y - bb.getMin().y;

	//insertion base point is center of back bottom edge of bounding box
	double xDisplacement = node->getPosition().x - (bb.getMin().x + bb.getMax().x) / 2;
	double yDisplacement = node->getPosition().y - bb.getMin().y;
	double zDisplacement = node->getPosition().z - bb.getMin().z;
	if (xDisplacement != 0 || yDisplacement != 0 || zDisplacement != 0) {
		for (int c = 0; c < node->getNumChildren(); c++) {
			Unigine::NodePtr child = node->getChild(c);
			child->setWorldPosition(child->getWorldPosition() + Unigine::Math::dvec3(xDisplacement, yDisplacement, zDisplacement));
		}
	}



	//place node into start position
	TrafficLane* lane = trafficLane;
	Position3D vp = lane->startOfLane();
	moveOnPos(vp);
	currLinearPosOnLane = startLinPos;



#ifdef DEBUG 
	Unigine::MeshPtr sphere = Unigine::Mesh::create();
	sphere->addSphereSurface("testMarker", 0.5f, 8, 8);

	testMarkerLeft = Unigine::ObjectMeshStatic::create(sphere);
	testMarkerLeft->release();
	Unigine::Editor::get()->addNode(testMarkerLeft->getNode());
	testMarkerLeft->setMaterialParameter("albedo_color", Unigine::Math::vec4(1, 0, 0, 1), 0);
	testMarkerLeft->setEnabled(0);

	sphere = Unigine::Mesh::create();
	sphere->addSphereSurface("testMarker", 0.5f, 8, 8);

	testMarkerRight = Unigine::ObjectMeshStatic::create(sphere);
	testMarkerRight->release();
	Unigine::Editor::get()->addNode(testMarkerRight->getNode());
	testMarkerRight->setMaterialParameter("albedo_color", Unigine::Math::vec4(1, 0, 0, 1), 0);
	testMarkerRight->setEnabled(0);

#endif
}


Vehicle::~Vehicle()
{
	Unigine::Editor::get()->removeNode(node->getNode(), 1);
	node.clear();

#ifdef DEBUG 
	Unigine::Editor::get()->removeNode(testMarkerLeft->getNode(), 1);
	testMarkerLeft.clear();
	Unigine::Editor::get()->removeNode(testMarkerRight->getNode(), 1);
	testMarkerRight.clear();
#endif
}

void Vehicle::update() {
	//if currently changing lane in process continue change line

	//check if there is obstacle at the front in dynamic envelope in current lane
	//if there is obstacle make desision about changing lane
	//start changing lane if possible
	//otherwise slow down

	//if there is no obstacle accelerate to the speed limit

	//set acceleration and velocity...
	//update dynamic envelope
	//update position

	//keep track vehicle position on all traffic lanes of current carrageway

	TrafficLane* lane = trafficLane;

	if (justChangedLane) {
		laneToTheLeft = trafficLane->lanesToTheLeftBegin();
		laneToTheRight = trafficLane->lanesToTheRightBegin();
		justChangedLane = false;
	}

	if (distFromLastNeighborLanesPositionUpdate > 1e-3f) 
		//to avoid errors update neighbor positions only if moved significaly
	{
		//update current laneToTheLeft and laneToTheRight
		laneToTheLeft = updateNeighborLane(laneToTheLeft, trafficLane->lanesToTheLeftEnd());
		laneToTheRight = updateNeighborLane(laneToTheRight, trafficLane->lanesToTheRightEnd());

		if (laneToTheLeft != trafficLane->lanesToTheLeftEnd())
			updatePosOnNeighborLane(&posOnLaneToTheLeft, *laneToTheLeft);
		else posOnLaneToTheLeft = LinearPosition::Null();//больше доп полос с этой стороны нет
		if (laneToTheRight != trafficLane->lanesToTheRightEnd())
			updatePosOnNeighborLane(&posOnLaneToTheRight, *laneToTheRight);
		else posOnLaneToTheRight = LinearPosition::Null();

#ifdef DEBUG 
		if (!posOnLaneToTheLeft.isEmpty()) {
			testMarkerLeft->setEnabled(1);
			Position3D pos = posOnLaneToTheLeft.getPos3D();
			testMarkerLeft->setWorldPosition(pos.absPos);
			testMarkerLeft->setWorldDirection(pos.tangent, pos.up, Unigine::Math::AXIS_Y);
		}
		else
		{
			testMarkerLeft->setEnabled(0);
		}

		if (!posOnLaneToTheRight.isEmpty()) {
			testMarkerRight->setEnabled(1);
			Position3D pos = posOnLaneToTheRight.getPos3D();
			testMarkerRight->setWorldPosition(pos.absPos);
			testMarkerRight->setWorldDirection(pos.tangent, pos.up, Unigine::Math::AXIS_Y);
		}
		else
		{
			testMarkerRight->setEnabled(0);
		}
#endif


		distFromLastNeighborLanesPositionUpdate = 0;//increase if moved
	}

	

	switch (currentActivity)
	{
	case StraightMove:
	{
		float currDynEnv = getDynamicEnvelop();

		std::list<Vehicle*>::iterator nextIt = std::next(vehicleIterator);
		ObstacleType obstacleType;
		LinearPosition obstacleLP = LinearPosition::Null();
		double clearDist = getClearDist(nextIt, currLinearPosOnLane, obstacleType, obstacleLP);

		//get next vehicle in this line


		//here the decision on current acceleration has to be made. 
		//if the path is free, then we accelerate with standard acceleration. 
		//if something interferes, then current acceleration is such that, with an equally slow motion, stop at a free site
		float currAcceleration;
		if (clearDist >= currDynEnv) {
			//way is clear -> accelerate to speed limit
			if (velocity < speedLimit) {
				currAcceleration = standartAcceleration;
			}
			else
			{
				currAcceleration = 0;//equally slow motion
				velocity = speedLimit;
			}

		}
		else
		{
			//TODO: start to change lane if possible
			/*if (laneToTheLeft != trafficLane->lanesToTheLeftEnd()) {

				LinearPosition lp = canChangeLane(*laneToTheLeft, posOnLaneToTheLeft, obstacleType);


				


				
				
			}


			if (laneToTheRight != trafficLane->lanesToTheRightEnd()) {

			}*/




			//slow down to a complete stop if can't change lane
			double distToStop = clearDist - reserveDistBetweenCars - length;
			if (distToStop < 0) {
				currAcceleration = 0;//we must stop already
				velocity = 0;
			}
			else
			{
				currAcceleration = -Unigine::Math::pow(velocity, 2) / (2 * distToStop);
			}

		}


		float time = Unigine::Game::get()->getIFps();

		//movement
		//calculate new velocity with current acceleration
		//move car


		if (currAcceleration != 0) {
			velocity = velocity + currAcceleration * time;
		}

		if (velocity < UNIGINE_EPSILON) velocity = 0;

		if (velocity != 0) {
			float s = velocity * time;

			//Unigine::SplineSegmentPtr oldSeg = currLinearPosOnLane.splSegment;

			if (currLinearPosOnLane.increaseLinearPos(s)) {
				_reachedEndOfRoad = lane->getLeadToEndOfRoad();
			}
			else
			{
				Position3D vp = currLinearPosOnLane.getPos3D();
				moveOnPos(vp);
				distFromLastNeighborLanesPositionUpdate += s;

				if (!movingThroughObstacle.isEmpty()
					&& currLinearPosOnLane.absLinearPos >= movingThroughObstacle.getSegEndLinearPos()) {
					//we path through obstacle!
					movingThroughObstacle = LinearPosition::Null();
				}
			}
		}


		if (obstacleType == ObstacleType::PaymentCollectionPoint && velocity == 0) {
			//we stopped on Payment Collection Point
			//wait some time
			currentActivity = VehicleActivity::Wait;
			timeToWait = timeToWaitOnPaymentCollectionPoint;
			movingThroughObstacle = obstacleLP;
		}
	}
	break;
	case Wait:
		if (timeToWait >= 0.0f)
		{
			timeToWait -= Unigine::Game::get()->getIFps();
			if (timeToWait < 0.0f)
			{
				currentActivity = VehicleActivity::StraightMove;
				timeToWait = 0;
			}
		}
		break;
	case ChangeLine:
		break;
	default:
		break;
	}






}

double Vehicle::getClearDist(std::list<Vehicle*>::iterator nextIt,
	LinearPosition linPos, ObstacleType &obstacleType, LinearPosition &obstacleLP)
{
	TrafficLane * lane = trafficLane;

	double clearDist = DBL_MAX;
	obstacleType = ObstacleType::None;
	obstacleLP = LinearPosition::Null();

	if (!lane->getLeadToEndOfRoad()) {
		clearDist = lane->getOveralLength() - linPos.absLinearPos;
		obstacleType = ObstacleType::EndOfLane;
		//TODO: obstacleLP?
	}

	//get next obstacle
	LinearPosition obstacleLp = lane->getNextObstacle(linPos, !movingThroughObstacle.isEmpty());

	if (nextIt != lane->getQueueEnd() && !obstacleLp.isEmpty()) {
		//what is closer?
		double distToNextCar = (*nextIt)->getCurrPosOnLane().absLinearPos - linPos.absLinearPos;
		double distToObstacle = obstacleLp.absLinearPos - linPos.absLinearPos;
		if (distToNextCar < distToObstacle) {
			clearDist = distToNextCar;
			obstacleType = ObstacleType::MovingVehicle;
			obstacleLP = (*nextIt)->getCurrPosOnLane();
		}
		else
		{
			clearDist = distToObstacle;
			obstacleType = ObstacleType::PaymentCollectionPoint;
			obstacleLP = obstacleLp;
		}
	}
	else if (nextIt != lane->getQueueEnd())
	{
		//there is some vehicle ahead
		clearDist = (*nextIt)->getCurrPosOnLane().absLinearPos - linPos.absLinearPos;
		obstacleType = ObstacleType::MovingVehicle;
		obstacleLP = (*nextIt)->getCurrPosOnLane();
	}
	else if (!obstacleLp.isEmpty())
	{
		//there is some obstacle ahead
		clearDist = obstacleLp.absLinearPos - linPos.absLinearPos;
		obstacleType = ObstacleType::PaymentCollectionPoint;
		obstacleLP = obstacleLp;
	}

	return clearDist;
}



float Vehicle::getVelocityToFitIntoSpan(double span) {
	double stoppingDist = span - length - reserveDistBetweenCars;

	if (stoppingDist < UNIGINE_EPSILON) {
		return 0;
	}

	double calcVelocity = Unigine::Math::dsqrt(2 * stoppingDist * standartDamping);
	return calcVelocity > UNIGINE_EPSILON ? calcVelocity : 0;
}
