// Fill out your copyright notice in the Description page of Project Settings.

#pragma once
#include "VParabolic.h"
#include "VSite.h"

/**
 * 
 */
class SWORDSWING_API VEvent
{
public:
	VEvent();
	~VEvent();

	FVector2D point;
	VSite* site;
	bool		pe;
	double		y;
	VParabolic* arch;

	VEvent(VSite* s, bool pev)
	{
		if (pev)
		{
			site = s;
			point = s->pos;
		}
			
		else
		{
			point = s->pos;
			site = nullptr;
		}
			
		pe = pev;
		y = s->pos.Y;
	}

	VEvent(FVector2D&& s, bool pev)
	{
		site = nullptr;
		point = s;
		pe = pev;
		y = s.Y;
	}

	struct CompareEvent : public std::binary_function<VEvent*, VEvent*, bool>
	{
		bool operator()(const VEvent* l, const VEvent* r) const { return (l->y < r->y); }
	};
};
