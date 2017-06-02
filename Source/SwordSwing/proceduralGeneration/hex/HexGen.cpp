// Fill out your copyright notice in the Description page of Project Settings.

#include "SwordSwing.h"
#include "HexGen.h"

HexGen::HexGen()
{
}

HexGen::HexGen(UInstancedStaticMeshComponent* _instance, float hex_radius, FVector position ) : hrad(hex_radius), pos(position), instance(_instance)
{
}

HexGen::~HexGen()
{
}

void HexGen::setPos(const FVector& position)
{
	pos = position;
}
FVector HexGen::getPos()
{
	return pos;
}

