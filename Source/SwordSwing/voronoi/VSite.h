// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

/**
 * 
 */
class SWORDSWING_API VSite
{
public:
	VSite();
	VSite(float x, float y);
	VSite(FVector2D& pos);
	~VSite();

	FVector2D pos;
};
