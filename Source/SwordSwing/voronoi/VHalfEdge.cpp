// Fill out your copyright notice in the Description page of Project Settings.

#include "SwordSwing.h"
#include "VSite.h"
#include "VHalfEdge.h"
#include <iostream>

VHalfEdge::VHalfEdge()
{
}

VHalfEdge::VHalfEdge(FVector2D _start)
{
	start = _start;
}

VHalfEdge::VHalfEdge(FVector2D _start, VSite* _left, VSite* _right)
{
	start = _start;
	left = _left;
	right = _right;

	FVector2D perpendicular_dir = (_left->pos - _right->pos).GetSafeNormal();
	direction = FVector2D(-perpendicular_dir.Y, perpendicular_dir.X);

	if (perpendicular_dir.IsNearlyZero())
	{
		std::cout << "left and right site are identical" << std::endl;
		UE_LOG(LogTemp, Warning, TEXT("left and right site are identical"));

	}
}
VHalfEdge::VHalfEdge(FVector2D _start, VHalfEdge* _twin)
{
	start = _start;
	twin = _twin;
}

VHalfEdge::~VHalfEdge()
{
}


