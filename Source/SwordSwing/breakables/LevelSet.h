// Fill out your copyright notice in the Description page of Project Settings.

#pragma once
#include "ScalarField.h"

/**
 * 
 */
class SWORDSWING_API LevelSet
{
public:
	LevelSet();
	~LevelSet();

private:
	ScalarField<float> sf;
};
