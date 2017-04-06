// Fill out your copyright notice in the Description page of Project Settings.

#pragma once
#include "VSite.h"
#include "VEdge.h"

class VEvent;

/**
 * 
 */
class SWORDSWING_API VParabolic
{
public:
	VParabolic();
	VParabolic(VSite* s);
	~VParabolic();

	bool isLeaf;
	VSite* site;
	VEdge* edge;
	VEvent*	cEvent;
	VParabolic* parent;

	void SetLeft(VParabolic * p) { _left = p; p->parent = this; }
	void SetRight(VParabolic * p) { _right = p; p->parent = this; }
	VParabolic *	Left() { return _left; }
	VParabolic * Right() { return _right; }

	static VParabolic * GetLeft(VParabolic * p);
	static VParabolic * GetRight(VParabolic * p);
	static VParabolic * GetLeftParent(VParabolic * p);
	static VParabolic * GetRightParent(VParabolic * p);
	static VParabolic * GetLeftChild(VParabolic * p);
	static VParabolic * GetRightChild(VParabolic * p);

private:

	VParabolic * _left;
	VParabolic * _right;
};
