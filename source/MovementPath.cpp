#include "MovementPath.h"
#include <UnigineEditor.h>



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
		assert(numOfSeg == 2 || 
			((p == 0 || p == points.size()-1) 
				&& numOfSeg == 1));
	}



	//TODO: Сделать перечисление движущихся объектов в свойстве объекта

	Unigine::NodePtr test_obj = Unigine::Editor::get()->getNodeByName("supply_ship");
	MovingObject* testObj = new MovingObject(test_obj);
	testObj->currLinearPos = getStartOfPath();
	testObj->currLinearPos.increaseLinearPos(100);
	testObj->updatePos();
	testObj->velocity = 11.1111;
	movingObjects.append(testObj);
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
