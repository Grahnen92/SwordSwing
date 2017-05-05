// Fill out your copyright notice in the Description page of Project Settings.

#pragma once
/**
 * 
 */
class VSite;

class SWORDSWING_API VHalfEdge
{
public:
	VHalfEdge();
	VHalfEdge(FVector2D _start);
	VHalfEdge(FVector2D _start, VSite* _left, VSite* _right);
	VHalfEdge(FVector2D _start, VHalfEdge* _twin);
	~VHalfEdge();

	FVector2D start;
	FVector2D end;

	VSite* left;
	VSite* right;

	FVector2D direction;

	VHalfEdge* twin;

};
