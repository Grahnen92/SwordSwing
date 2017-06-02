// Fill out your copyright notice in the Description page of Project Settings.

#pragma once
#include "HexGen.h"

/**
 * 
 */
class SWORDSWING_API HexCircleGen : public HexGen
{
public:
	HexCircleGen();
	HexCircleGen(UInstancedStaticMeshComponent* _instance, float hex_radius, int circle_radius,  FVector position);
	~HexCircleGen();

	void generate();
	
	static void CGenerate(UInstancedStaticMeshComponent* instance, float hex_radius, int circle_radius, FVector position);

private:
	int crad;
};
