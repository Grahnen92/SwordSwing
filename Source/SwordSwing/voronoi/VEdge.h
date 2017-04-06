// Fill out your copyright notice in the Description page of Project Settings.

#pragma once
#include "VSite.h"
/**
 * 
 */
class SWORDSWING_API VEdge
{
public:
	VEdge();
	VEdge(FVector2D& s, VSite* a, VSite* b)
	{
		start = s;
		left = a;
		right = b;
		neighbour = nullptr;
		end = FVector2D::ZeroVector;

		f = (b->pos.X - a->pos.X) / (a->pos.Y - b->pos.Y);
		g = s.Y - f * s.X;
		direction =  FVector2D(b->pos.Y - a->pos.Y, -(b->pos.X - a->pos.X));
	}
	~VEdge();

	FVector2D start;
	FVector2D end;
	FVector2D direction;
	VSite* left;
	VSite* right;

	double f;
	double g;

	VEdge* neighbour;
};
