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

std::list<VEdge>* Voronoi::GetEdges(std::list<VSite>* s, int _w, int _h)
{
	sites = s;
	w = _w;
	h = _h;
	root = nullptr;

	if (edges.size() == 0)
	{

	}
	else
	{
		edges.clear();
	}

	for (std::list<VSite>::iterator i = sites->begin(); i != sites->end(); ++i)
	{
		queue.push(new VEvent(&(*i), true));
	}

	VEvent * e;
	while (!queue.empty())
	{
		e = queue.top();
		queue.pop();
		line_y = e->point.Y;
		if (deleted.find(e) != deleted.end()) 
		{ 
			delete(e); 
			deleted.erase(e); 
			continue; 
		}
		if (e->pe) 
		{
			//HÄR VAR DU
			InsertParabola(e->site);
		}
		else
		{
			RemoveParabola(e);
		}
			

		delete(e);
	}

	FinishEdge(root);

	for (std::list<VEdge>::iterator i = edges.begin(); i != edges.end(); ++i)
	{
		if ((i)->neighbour)
		{
			(i)->start = (i)->neighbour->end;
			delete (i)->neighbour;
		}
	}

	return &edges;
	return nullptr;
}

void Voronoi::InsertParabola(VSite * p)
{
	if (!root) 
	{ 
		root = new VParabolic(p);
		return; 
	}

	if (root->isLeaf && root->site->pos.Y - p->pos.Y < 1) // degenerate EVENT - Both the lower space at the same height
	{
		VSite * fs = root->site;
		root->isLeaf = false;
		root->SetLeft(new VParabolic(fs));
		root->SetRight(new VParabolic(p));
		FVector2D n_vert = FVector2D((p->pos.X + fs->pos.X) / 2, h); // zaèátek hrany uprostøed míst
		vertices.push_back(n_vert);
		if (p->pos.X > fs->pos.X) 
			root->edge = new VEdge(n_vert, fs, p); // rozhodnu, který vlevo, který vpravo
		else 
			root->edge = new VEdge(n_vert, p, fs);

		edges.push_back(*root->edge);
		return;
	}

	VParabolic * par = GetParabolaByX(p->pos.X);

	if (par->cEvent)
	{
		deleted.insert(par->cEvent);
		par->cEvent = 0;
	}

	FVector2D start = FVector2D(p->pos.X, GetY(par->site, p->pos.X));
	vertices.push_back(start);

	VEdge * el = new VEdge(start, par->site, p);
	VEdge * er = new VEdge(start, p, par->site);

	el->neighbour = er;
	edges.push_back(*el);

	//// pøestavuju strom .. vkládám novou parabolu
	par->edge = er;
	par->isLeaf = false;

	VParabolic * p0 = new VParabolic(par->site);
	VParabolic * p1 = new VParabolic(p);
	VParabolic * p2 = new VParabolic(par->site);

	par->SetRight(p2);
	par->SetLeft(new VParabolic());
	par->Left()->edge = el;

	par->Left()->SetLeft(p0);
	par->Left()->SetRight(p1);

	CheckCircle(p0);
	CheckCircle(p2);
}

void Voronoi::RemoveParabola(VEvent * e)
{
	VParabolic * p1 = e->arch;

	VParabolic * xl = VParabolic::GetLeftParent(p1);
	VParabolic * xr = VParabolic::GetRightParent(p1);

	VParabolic * p0 = VParabolic::GetLeftChild(xl);
	VParabolic * p2 = VParabolic::GetRightChild(xr);

	if (p0 == p2) 
		std::cout << "chyba - pravá a levá parabola má stejné ohnisko!\n";

	if (p0->cEvent) { deleted.insert(p0->cEvent); p0->cEvent = 0; }
	if (p2->cEvent) { deleted.insert(p2->cEvent); p2->cEvent = 0; }

	FVector2D p = FVector2D(e->point.X, GetY(p1->site, e->point.X));
	vertices.push_back(p);

	xl->edge->end = p;
	xr->edge->end = p;

	VParabolic * higher = nullptr;
	VParabolic * par = p1;
	while (par != root)
	{
		par = par->parent;
		if (par == xl) higher = xl;
		if (par == xr) higher = xr;
	}
	higher->edge = new VEdge(p, p0->site, p2->site);
	edges.push_back(*higher->edge);

	VParabolic * gparent = p1->parent->parent;
	if (p1->parent->Left() == p1)
	{
		if (gparent->Left() == p1->parent) gparent->SetLeft(p1->parent->Right());
		if (gparent->Right() == p1->parent) gparent->SetRight(p1->parent->Right());
	}
	else
	{
		if (gparent->Left() == p1->parent) gparent->SetLeft(p1->parent->Left());
		if (gparent->Right() == p1->parent) gparent->SetRight(p1->parent->Left());
	}

	delete p1->parent;
	delete p1;

	CheckCircle(p0);
	CheckCircle(p2);
}

void Voronoi::FinishEdge(VParabolic * n)
{
	if (n->isLeaf) 
	{
		delete n;
		return; 
	}
	double mx;
	if (n->edge->direction.X > 0.0)
		mx = std::max(w, (double)n->edge->start.X + 10);
	else
		mx = std::min(0.0, (double)n->edge->start.X - 10);

	FVector2D end = FVector2D(mx, mx * n->edge->f + n->edge->g);
	n->edge->end = end;
	vertices.push_back(end);

	FinishEdge(n->Left());
	FinishEdge(n->Right());
	delete n;
}

double	Voronoi::GetXOfEdge(VParabolic * par, double y)
{
	VParabolic * left = VParabolic::GetLeftChild(par);
	VParabolic * right = VParabolic::GetRightChild(par);

	VSite * p = left->site;
	VSite * r = right->site;

	double dp = 2.0 * (p->pos.Y - y);
	double a1 = 1.0 / dp;
	double b1 = -2.0 * p->pos.X / dp;
	double c1 = y + dp / 4 + p->pos.X * p->pos.X / dp;

	dp = 2.0 * (r->pos.Y - y);
	double a2 = 1.0 / dp;
	double b2 = -2.0 * r->pos.X / dp;
	double c2 = line_y + dp / 4 + r->pos.X * r->pos.X / dp;

	double a = a1 - a2;
	double b = b1 - b2;
	double c = c1 - c2;

	double disc = b*b - 4 * a * c;
	double x1 = (-b + std::sqrt(disc)) / (2 * a);
	double x2 = (-b - std::sqrt(disc)) / (2 * a);

	double ry;
	if (p->pos.Y < r->pos.Y) ry = std::max(x1, x2);
	else ry = std::min(x1, x2);

	return ry;
}

VParabolic * Voronoi::GetParabolaByX(double xx)
{
	VParabolic * par = root;
	double x = 0.0;

	while (!par->isLeaf) // projdu stromem dokud nenarazím na vhodný list
	{
		x = GetXOfEdge(par, line_y);
		if (x>xx) par = par->Left();
		else par = par->Right();
	}
	return par;
}

double Voronoi::GetY(VSite * p, double x) // focus, x - coordinates of
{
	double dp = 2 * (p->pos.Y - line_y);
	double a1 = 1 / dp;
	double b1 = -2 * p->pos.X / dp;
	double c1 = line_y + dp / 4 + p->pos.X * p->pos.X / dp;

	return(a1*x*x + b1*x + c1);
}

void Voronoi::CheckCircle(VParabolic* b)
{
	VParabolic * lp = VParabolic::GetLeftParent(b);
	VParabolic * rp = VParabolic::GetRightParent(b);

	VParabolic * a = VParabolic::GetLeftChild(lp);
	VParabolic * c = VParabolic::GetRightChild(rp);

	if (!a || !c || a->site == c->site) return;

	FVector2D* s = nullptr;
	s = GetEdgeIntersection(lp->edge, rp->edge);
	if (s == nullptr) return;

	double dx = a->site->pos.X - s->X;
	double dy = a->site->pos.Y - s->Y;

	double d = std::sqrt((dx * dx) + (dy * dy));

	if (s->Y - d >= line_y) { return; }

	VEvent * e = new VEvent(FVector2D(s->X, s->Y - d), false);
	vertices.push_back(e->point);
	b->cEvent = e;
	e->arch = b;
	queue.push(e);
}

FVector2D* Voronoi::GetEdgeIntersection(VEdge * a, VEdge * b)
{
	double x = (b->g - a->g) / (a->f - b->f);
	double y = a->f * x + a->g;

	if ((x - a->start.X) / a->direction.X < 0) return nullptr;
	if ((y - a->start.Y) / a->direction.Y < 0) return nullptr;

	if ((x - b->start.X) / b->direction.X < 0) return nullptr;
	if ((y - b->start.Y) / b->direction.Y < 0) return nullptr;

	FVector2D*  p = new FVector2D(x, y);
	vertices.push_back(*p);
	return p;
}