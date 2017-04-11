// Fill out your copyright notice in the Description page of Project Settings.

#include "SwordSwing.h"
#include "VEvent.h"

VEvent::VEvent()
{
}

VEvent::VEvent(VSite* _s)
{
	s = _s;
	pos = _s->pos;
	point_event = true;
}

VEvent::VEvent(VSite* _r, VSite* _m, VSite* _l, FVector2D _ccm)
{
	sr = _r;
	sm = _m;
	sl = _l;
	pos = _ccm;
	point_event = false;
}

VEvent::~VEvent()
{
}
