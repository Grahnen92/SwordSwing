// Fill out your copyright notice in the Description page of Project Settings.

#pragma once
#include "VSite.h"
#include "VHalfEdge.h"
#include "VParabola.h"
#include "VEvent.h"

#include <queue>
#include <set>
#include <list>
#include <vector>

/**
 * 
 */
class SWORDSWING_API Voronoi
{
public:
	Voronoi();
	~Voronoi();

	void CalculateDiagram(TArray<FVector2D>* _sites);

	void setDims(float x, float y) 
	{
		dims.X = x; dims.Y = y;
	}
private:

	void addParabola(VSite * _s);
	VParabola* findParabolaAtX(float _x);

	std::vector<VSite> sites;
	std::vector<FVector2D> vertices;
	std::vector<VHalfEdge> edges;

	VParabola* root;

	std::priority_queue<VEvent*, std::vector<VEvent*>, VEvent::CompareEvent > event_queue;

	float directrix_y;

	FVector2D dims;
};
