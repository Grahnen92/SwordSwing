// Fill out your copyright notice in the Description page of Project Settings.

#include "SwordSwing.h"
#include "VHalfEdge.h"

VHalfEdge::VHalfEdge()
{
}

VHalfEdge::VHalfEdge(FVector2D _start)
{
	start = _start;
}
VHalfEdge::VHalfEdge(FVector2D _start, VHalfEdge* _twin)
{
	start = _start;
	twin = _twin;
}

VHalfEdge::~VHalfEdge()
{
}


