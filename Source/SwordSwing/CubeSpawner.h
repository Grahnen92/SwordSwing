// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "breakables/ScalarField.h"
#include <vector>

#include "GameFramework/Actor.h"
#include "CubeSpawner.generated.h"


UCLASS()
class SWORDSWING_API ACubeSpawner : public AActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	ACubeSpawner();
	

	// Called when the game starts or when spawned
	virtual void BeginPlay() override;
	
	// Called every frame
	virtual void Tick( float DeltaSeconds ) override;

private:
	std::vector<AActor*> cubes;

	UPROPERTY(EditAnywhere, Category = "Config")
	TSubclassOf<AActor> CubeClass;


	
	
};
