#include "Position3D.h"
#include "LinearPosition.h"


Position3D::Position3D(Unigine::SplineSegmentPtr splSegment,
	Unigine::Math::dvec3 pos, Unigine::Math::vec3 tangent, Unigine::Math::vec3 up)
{
	this->splSegment = splSegment;
	this->absPos = splSegment->getParent()->getWorldTransform()*pos;
	this->tangent = tangent;//TODO?: also use spline grafh world transform
	this->up = up;
}


Position3D::~Position3D()
{
}

//returns 0 if point is not in front of segment
	//returns -1 if point is to the left of segment
	//returns 1 if point is to the right of segment
	//http://geomalgorithms.com/a02-_lines.html
int Position3D::isParallelLineInFrontOf(LinearPosition* lp) {
	using namespace Unigine::Math;
	//CHECK 2 NEIGHBORING SEGMENTS!!!
	Unigine::SplineSegmentPtr seg0 = lp->splSegment;
	Unigine::SplineSegmentPtr seg1 = lp->getNextSegment();
	dmat4 globalTransf = seg0->getParent()->getWorldTransform();

	float startDistOnSeg = lp->distOnSplineSeg();


	float len = seg0->getLength();
	if (!seg1 && startDistOnSeg == len) return 0;//it can be if lp is the end of lane!

	assert(startDistOnSeg < len);
	dvec3 p0 = dvec3::INF;

	//if (startDistOnSeg == 0) {
	p0 = globalTransf * seg0->getStartPoint()->getPosition();
	/*}
	else {
		float param = seg0->linearToParametric(startDistOnSeg / seg0->getLength());
		p0 = globalTransf * seg0->calcPoint(param);
	}*/

	dvec3 p1 = globalTransf * seg0->getEndPoint()->getPosition();


	dvec3 p2 = dvec3::INF;
	if (seg1)
		p2 = globalTransf * seg1->getEndPoint()->getPosition();

	bool afterSeg0 = false;
	bool inFrontOfSeg0 = false;
	dvec3 v0 = p1 - p0;
	dvec3 w0 = absPos - p0;
	double c1_0 = 0;
	double c2_0 = 0;
	{
		dvec2 v2d = dvec2(v0.x, v0.y);
		dvec2 w2d = dvec2(w0.x, w0.y);

		c1_0 = dot(w2d, v2d);
		if (c1_0 < -0.0001) return 0;
		c2_0 = dot(v2d, v2d);
		if (c2_0 < c1_0) afterSeg0 = true;

		else inFrontOfSeg0 = true;

	}

	bool beforeSeg1 = false;
	bool inFrontOfSeg1 = false;
	dvec3 v1 = p2 - p1;
	dvec3 w1 = absPos - p1;
	double c1_1 = 0;
	double c2_1 = 0;
	if (seg1) {

		{
			dvec2 v2d = dvec2(v1.x, v1.y);
			dvec2 w2d = dvec2(w1.x, w1.y);

			c1_1 = dot(w2d, v2d);

			c2_1 = dot(v2d, v2d);
			if (c2_1 < c1_1) return 0;

			if (c1_1 < -0.0001) beforeSeg1 = true;
			else inFrontOfSeg1 = true;
		}
	}
	else if (!inFrontOfSeg0)
	{
		return 0;
	}

	if (seg1)
		assert((afterSeg0 && beforeSeg1) || inFrontOfSeg0 || inFrontOfSeg1);
	else assert(inFrontOfSeg0);

	//double increaseLP = 0;

	LinearPosition newLp = LinearPosition::Null();

	if (afterSeg0 && beforeSeg1) {
		assert(!inFrontOfSeg0 && !inFrontOfSeg1);
		//point is from the outside of convex vertex
		//increaseLP = seg0->getLength() - startDistOnSeg;
		newLp.copyFrom(*lp);
		//newLp.moveToNextSegment();
		newLp.absLinearPos = newLp.segStartLinearPos + newLp.splSegment->getLength() - 0.001;//must stay on current segment
	}
	else
	{
		assert(inFrontOfSeg0 || inFrontOfSeg1);

		dvec3 pi0 = dvec3::INF;
		//double increaseLP0 = -DBL_MAX;
		LinearPosition newLp0 = LinearPosition::Null();
		bool codirectional0 = false;
		if (inFrontOfSeg0) {
			double b = c1_0 / c2_0;
			pi0 = p0 + v0 * b;
			//increaseLP0 = (pi0 - p0).length();
			newLp0.copyFrom(*lp);
			//сбросить текущую длину
			newLp0.absLinearPos = newLp0.segStartLinearPos;
			newLp0.increaseLinearPos((pi0 - p0).length());
			float d = dot(normalize(tangent), vec3(normalize(v0)));
			codirectional0 = /*seg1? d > 0.9f : */Unigine::Math::abs(d) > 0.9f;//на последней точке графа бывает наоборот
		}

		dvec3 pi1 = dvec3::INF;
		//double increaseLP1 = -DBL_MAX;
		LinearPosition newLp1 = LinearPosition::Null();
		bool codirectional1 = false;
		if (seg1 && inFrontOfSeg1) {
			double b = c1_1 / c2_1;
			pi1 = p1 + v1 * b;
			//increaseLP1 = seg0->getLength() - startDistOnSeg + (pi1 - p1).length();
			newLp1.copyFrom(*lp);
			newLp1.moveToNextSegment();
			newLp1.increaseLinearPos((pi1 - p1).length());
			float d = dot(normalize(tangent), vec3(normalize(v1)));
			codirectional1 = Unigine::Math::abs(d) > 0.9f;
		}


		if (inFrontOfSeg0 && inFrontOfSeg1 && codirectional0 && codirectional1)
		{
			double distToSeg0 = (absPos - pi0).length2();
			double distToSeg1 = (absPos - pi1).length2();

			if (distToSeg0 < distToSeg1) {
				//increaseLP = increaseLP0;
				newLp = newLp0;
			}
			else
			{
				//increaseLP = increaseLP1;
				newLp = newLp1;
			}
		}
		else if (inFrontOfSeg0 && codirectional0)
		{
			//increaseLP = increaseLP0;
			newLp = newLp0;
		}
		else if (codirectional1)
		{
			assert(inFrontOfSeg1);
			//increaseLP = increaseLP1;
			newLp = newLp1;
		}
	}

	/*if (seg1)
		assert(increaseLP <= seg0->getLength() + seg1->getLength());
	else
		assert(increaseLP <= seg0->getLength());*/


	//assert(increaseLP >= 0);

	//if (increaseLP == 0) return 0;
	if (newLp.isEmpty()) return 0;

	//lp->increaseLinearPos(increaseLP);
	lp->copyFrom(newLp);

	return 1;
	/*double leftOrRight = v0.x*w0.y - v0.y*w0.x;
	return leftOrRight > 0 ? -1 : 1;*/

}
