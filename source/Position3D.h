#pragma once
#include <UnigineMathLib.h>
#include <UnigineWorlds.h>

struct LinearPosition;

struct Position3D
{

	Position3D(Unigine::SplineSegmentPtr splSegment,
		Unigine::Math::dvec3 pos, Unigine::Math::vec3 tangent, Unigine::Math::vec3 up);
	~Position3D();

	Unigine::SplineSegmentPtr splSegment;

	Unigine::Math::dvec3 absPos = Unigine::Math::dvec3::ZERO;
	Unigine::Math::vec3 tangent = Unigine::Math::vec3::ZERO;
	Unigine::Math::vec3 up = Unigine::Math::vec3::ZERO;


	int isParallelLineInFrontOf(LinearPosition* lp);
};

