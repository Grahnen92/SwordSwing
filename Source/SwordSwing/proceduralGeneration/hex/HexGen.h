// Fill out your copyright notice in the Description page of Project Settings.

#pragma once
#include "HexInstancer.h"

/**
 * 
 */
class SWORDSWING_API HexGen
{
public:
	HexGen();
	HexGen(UInstancedStaticMeshComponent* _instance, float hex_radius, FVector position);

	void setPos(const FVector& position);
	FVector getPos();
	~HexGen();

	virtual void generate() = 0;

	AHexInstancer* instancer;

protected:

	float hrad;
	FVector pos;
	

	UInstancedStaticMeshComponent* instance;
	int instance_offset;
};
