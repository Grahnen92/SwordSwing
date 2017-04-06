// Fill out your copyright notice in the Description page of Project Settings.

#pragma once
#include "VSite.h"
#include "VEdge.h"
#include "VParabolic.h"
#include "VEvent.h"

#include <queue>
#include <set>
#include <list>

/**
 * 
 */
class SWORDSWING_API Voronoi
{
public:
	Voronoi();
	~Voronoi();
	
	std::list<VEdge>* GetEdges(std::list<VSite>* s, int w, int h);

private:

	std::list<VEdge> edges;
	std::list<VSite>* sites;

	double w, h;
	VParabolic* root;
	double line_y;

	std::set<VEvent *> deleted;
	std::list<FVector2D> vertices;
	std::priority_queue<VEvent *, std::vector<VEvent *>, VEvent::CompareEvent> queue;

	void		InsertParabola(VSite * p);
	void		RemoveParabola(VEvent * e);
	void		FinishEdge(VParabolic * n);
	double		GetXOfEdge(VParabolic * par, double y);
	VParabolic * GetParabolaByX(double xx);
	double		GetY(VSite * p, double x);
	void		CheckCircle(VParabolic * b);
	FVector2D* 	GetEdgeIntersection(VEdge * a, VEdge * b);
};
