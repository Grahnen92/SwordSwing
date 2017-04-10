// Fill out your copyright notice in the Description page of Project Settings.

#include "SwordSwing.h"
#include "Voronoi.h"

#include <algorithm>
#include <iostream>

Voronoi::Voronoi()
{
}

Voronoi::~Voronoi()
{
}

void Voronoi::CalculateDiagram(TArray<FVector2D>* _sites)
{
	sites.reserve(_sites->Num());
	
	for (int i = 0; i < _sites->Num(); i++)
	{
		sites.push_back(VSite((*_sites)[i]));
		event_queue.push(new VEvent(&sites[i]));
	}
	
	root = new VParabola(event_queue.top()->s, nullptr);
	directrix_y = event_queue.top()->pos.Y;
	event_queue.pop();


	while (!event_queue.empty())
	{
		VEvent* e = event_queue.top();
		directrix_y = e->pos.Y;
		if (e->point_event)
		{
			addParabola(e->s);

		}
		else
		{

		}

		event_queue.pop();
	}
}

void Voronoi::addParabola(VSite * _s)
{
	VParabola* par_to_split;

	par_to_split = findParabolaAtX(_s->pos.X);

	//Create the edge that results from this split
	VHalfEdge* new_edge = new VHalfEdge(FVector2D(_s->pos.X, par_to_split->getYAt(_s->pos.X, directrix_y)));

	//create and assign children of the parabolaNode that is being split
	VParabola* left_split = new VParabola(par_to_split->s, par_to_split);
	par_to_split->left_child = left_split;
		//this is an edgeNode that holds the created edge and is parent to the new parabola and the right split of parent parabola
	VParabola* edge_node = new VParabola(new_edge, par_to_split);
	par_to_split->right_child = edge_node;
	par_to_split->is_leaf = false;

	//create and assign children of the edgeNode
	VParabola* new_par = new VParabola(_s, edge_node);
	edge_node->left_child = new_par;
	VParabola* right_split = new VParabola(par_to_split->s, edge_node);
	edge_node->right_child = right_split;
	
}

VParabola* Voronoi::findParabolaAtX(float _x)
{
	VParabola * par_it = root;
	float x = 0.0;

	while (!par_it->is_leaf) // projdu stromem dokud nenarazím na vhodný list
	{
		x = par_it->getXOfIntersection( directrix_y);
		if (x <_x) 
			par_it = par_it->right_child;
		else 
			par_it = par_it->left_child;
	}
	return par_it;
}