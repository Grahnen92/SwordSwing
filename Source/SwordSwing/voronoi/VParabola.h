// Fill out your copyright notice in the Description page of Project Settings.

#pragma once
#include "VSite.h"
#include "VHalfEdge.h"

class VEvent;

/**
 * 
 */
class SWORDSWING_API VParabola
{
public:
	VParabola() : parent{ nullptr }, left_child{ nullptr }, right_child{ nullptr }, is_leaf{ false } 
	{

	}
	VParabola(VSite* _s, VParabola* _parent);
	VParabola(VHalfEdge* _e, VParabola* _parent);
	~VParabola();

	float getYAt(float x, float directrix_y);

	float getXOfIntersection(float current_directrix_y);

	bool is_leaf;

	VParabola* parent;
	VParabola* left_child;
	VParabola* right_child;

	VSite* s;

	VHalfEdge* e;

	VParabola* getClosestLeftLeave();
	VParabola* getClosestRightLeave();



private:

};
