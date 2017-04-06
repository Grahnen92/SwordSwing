// Fill out your copyright notice in the Description page of Project Settings.

#include "SwordSwing.h"
#include "VParabolic.h"

VParabolic::VParabolic()
{
	site = nullptr;
	isLeaf = false;
	cEvent = nullptr;
	edge = nullptr;
	parent = nullptr;
}

VParabolic::VParabolic(VSite* s)
{
	site = s;
	isLeaf = true;
	cEvent = nullptr;
	edge = nullptr;
	parent = nullptr;
}

VParabolic::~VParabolic()
{
}

VParabolic * VParabolic::GetLeft(VParabolic * p)
{
	return GetLeftChild(GetLeftParent(p));
}


VParabolic * VParabolic::GetRight(VParabolic * p)
{
	return GetRightChild(GetRightParent(p));
}

VParabolic * VParabolic::GetLeftParent(VParabolic * p)
{
	VParabolic * par = p->parent;
	VParabolic * pLast = p;
	while (par->Left() == pLast)
	{
		if (!par->parent) return 0;
		pLast = par;
		par = par->parent;
	}
	return par;
}

VParabolic * VParabolic::GetRightParent(VParabolic * p)
{
	VParabolic * par = p->parent;
	VParabolic * pLast = p;
	while (par->Right() == pLast)
	{
		if (!par->parent) return 0;
		pLast = par; par = par->parent;
	}
	return par;
}

VParabolic * VParabolic::GetLeftChild(VParabolic * p)
{
	if (!p) return 0;
	VParabolic * par = p->Left();
	while (!par->isLeaf) par = par->Right();
	return par;
}

VParabolic * VParabolic::GetRightChild(VParabolic * p)
{
	if (!p) return 0;
	VParabolic * par = p->Right();
	while (!par->isLeaf) par = par->Left();
	return par;
}