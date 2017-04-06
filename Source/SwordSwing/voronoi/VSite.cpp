// Fill out your copyright notice in the Description page of Project Settings.

#include "SwordSwing.h"
#include "VSite.h"

VSite::VSite()
{
	pos = FVector2D::ZeroVector;
}

VSite::VSite(float x, float y)
{
	pos = FVector2D(x, y);
}
VSite::VSite(FVector2D& _pos)
{
	pos = _pos;
}

VSite::~VSite()
{
}
