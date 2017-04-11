// Fill out your copyright notice in the Description page of Project Settings.

#pragma once
/**
 * 
 */
class SWORDSWING_API VHalfEdge
{
public:
	VHalfEdge();
	VHalfEdge(FVector2D _start);
	VHalfEdge(FVector2D _start, VHalfEdge* _twin);
	~VHalfEdge();

	FVector2D start;
	FVector2D end;

	FVector2D direction;

	VHalfEdge* twin;

};
