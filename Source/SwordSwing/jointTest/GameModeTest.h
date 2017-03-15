// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "GameFramework/GameModeBase.h"
#include "GameModeTest.generated.h"

/**
 * 
 */
UCLASS()
class SWORDSWING_API AGameModeTest : public AGameModeBase
{
	GENERATED_BODY()
	
	
public:
	// Sets default values for this actor's properties
	AGameModeTest();

	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

	virtual void Tick(float DeltaSeconds) override;

	void SpawnPlayer();

};
