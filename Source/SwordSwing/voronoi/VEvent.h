// Fill out your copyright notice in the Description page of Project Settings.

#pragma once
#include "VParabola.h"
#include "VSite.h"

/**
 * 
 */
class SWORDSWING_API VEvent
{
public:
	VEvent();
	VEvent(VSite* _s);
	VEvent(VSite* _r, VSite* _m, VSite* _l, FVector2D _ccm);
	~VEvent();

	bool operator <(const VEvent& rhs)
	{
		return pos.Y < rhs.pos.Y;
	}

	bool operator >(const VEvent& rhs)
	{
		return rhs.pos.Y < pos.Y;
	}
	
	FVector2D pos;

	bool point_event;
	VSite* s;

	VSite* sr;
	VSite* sm;
	VSite* sl;

	struct CompareEvent : public std::binary_function<VEvent*, VEvent*, bool>
	{
		bool operator()(const VEvent* l, const VEvent* r) const { return (l->pos.Y < r->pos.Y); }
	};
	
};
