// Fill out your copyright notice in the Description page of Project Settings.

#include "SwordSwing.h"
#include "GameModeTest.h"


AGameModeTest::AGameModeTest()
{
}


// Called when the game starts or when spawned
void AGameModeTest::BeginPlay()
{
	Super::BeginPlay();
}


// Called every frame
void AGameModeTest::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
}

void AGameModeTest::SpawnPlayer() {
	//this->GetInstigatorController
}