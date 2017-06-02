// Fill out your copyright notice in the Description page of Project Settings.

#pragma once
#include "HexGen.h"
/**
 * 
 */
class SWORDSWING_API HexGridGen : public HexGen
{
public:
	HexGridGen();
	HexGridGen(UInstancedStaticMeshComponent* _instance, float hex_radius, int x_halfdim, int y_halfdim, FVector position);
	~HexGridGen();

	void generate();
	static void GGenerate(UInstancedStaticMeshComponent* instance, float hex_radius, int x_halfdim, int y_halfdim, FVector position);

	void animateX(float anim_speed, bool neg2pos, float target_height, float DeltaTime);

private:
	int xhd, yhd;
};
