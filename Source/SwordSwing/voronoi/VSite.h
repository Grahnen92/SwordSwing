// Fill out your copyright notice in the Description page of Project Settings.

#pragma once
#include "VHalfEdge.h"
#include <list>

/**
 * 
 */
class SWORDSWING_API VSite
{
public:
	VSite();
	VSite(FVector2D& _coords);
	~VSite();

	FVector2D pos;

private:
	
	std::list<VHalfEdge*> edges;
};
