#include "MovementPath.h"
#include <UnigineEditor.h>
#include "ComponentSystem\ComponentSystem.h"



MovementPath::MovementPath(Unigine::WorldSplineGraphPtr node)
{
	worldSplineGraph = node;

	//Валидация сплайна: В каждой вершине сочленяются два сегмента (в начале и в конце может быть один сегмент)
	Unigine::Vector<Unigine::SplinePointPtr> points;
	worldSplineGraph->getSplinePoints(points);

	Unigine::Vector<Unigine::SplineSegmentPtr> segments;
	worldSplineGraph->getSplineSegments(segments);
	startSegment = segments[0];

	//TODO: эта валидация нормально работает в релизе??
	for (int p = 0; p < points.size(); p++) {
		Unigine::SplinePointPtr vert = points[p];
		int numOfSeg = vert->getNumSegments();
		if (numOfSeg == 0) continue;
		assert(numOfSeg == 2 ||
			((p == 0 || p == points.size() - 1)
				&& numOfSeg == 1));
	}



	//TODO: Сделать перечисление движущихся объектов в свойстве объекта
	int n = worldSplineGraph->findProperty("simple_movement_path");
	if (n != -1) {
		Unigine::PropertyPtr prop = worldSplineGraph->getProperty(n);
		//скорость
		Unigine::PropertyParameterPtr velocityParam = prop->getParameterPtr("velocity");
		if (velocityParam && velocityParam->getType() == Unigine::Property::PARAMETER_DOUBLE) {
			double velocity = velocityParam->getValueDouble();


			//объекты
			Unigine::PropertyParameterPtr movingObjsParam = prop->getParameterPtr("movingObjects");
			Unigine::PropertyParameterPtr objStartPosParam = prop->getParameterPtr("objectStartPositions");
			if (movingObjsParam && objStartPosParam
				&& movingObjsParam->getArrayType() == Unigine::Property::PARAMETER_NODE
				&& objStartPosParam->getArrayType() == Unigine::Property::PARAMETER_DOUBLE) {
				int moveObjsCount = movingObjsParam->getArraySize();
				int startPosCount = objStartPosParam->getArraySize();

				for (int c = 0; c < moveObjsCount; c++) {
					Unigine::PropertyParameterPtr objParam = movingObjsParam->getChild(c);
					Unigine::NodePtr obj = objParam->getValueNode();

					if (obj) {
						MovingObject* mObj = new MovingObject(obj);
						mObj->currLinearPos = getStartOfPath();
						if (c < startPosCount) {
							double s = objStartPosParam->getChild(c)->getValueDouble();
							if (s > 0)
								mObj->currLinearPos.increaseLinearPos(100);
						}
						mObj->updatePos();
						mObj->velocity = (float)velocity;
						movingObjects.append(mObj);
					}
				}
			}
		}





	}
	//worldSplineGraph->getPar




	/*Unigine::NodePtr test_obj = Unigine::Editor::get()->getNodeByName("supply_ship");
	MovingObject* testObj = new MovingObject(test_obj);
	testObj->currLinearPos = getStartOfPath();
	testObj->currLinearPos.increaseLinearPos(100);
	testObj->updatePos();
	testObj->velocity = 11.1111;
	movingObjects.append(testObj);*/
}


MovementPath::~MovementPath()
{
}

void MovementPath::update()
{
	for (MovingObject* obj : movingObjects) {
		obj->update();
	}
}
