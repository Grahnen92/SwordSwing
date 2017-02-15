// Fill out your copyright notice in the Description page of Project Settings.

#include "SwordSwing.h"
#include "CubeSpawner.h"


// Sets default values
ACubeSpawner::ACubeSpawner()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

	

}

// Called when the game starts or when spawned
void ACubeSpawner::BeginPlay()
{
	Super::BeginPlay();
	
	ScalarField<float> sf(10, 1000);

	cubes.resize(sf.getRes().X*sf.getRes().X*sf.getRes().X);

	UWorld* World = GetWorld();
	if (!World)
	{
		return;
	}

	for (int i = 0; i < sf.getRes().X; i++) {
		for (int j = 0; j < sf.getRes().Y; j++) {
			for (int k = 0; k < sf.getRes().Z; k++) {
				if (sf.getValue(i, j, k) > sf.getIsoValue()) {
					const FVector CubeLocation = FVector(i*100, j * 100, k * 100);
					const FRotator CubeRotation = FRotator::ZeroRotator;

					cubes[i*sf.getRes().Y*sf.getRes().Z + j*sf.getRes().Z] =
						World->SpawnActor<AActor>(CubeClass, CubeLocation, CubeRotation);

					if (cubes[i*sf.getRes().Y*sf.getRes().Z + j*sf.getRes().Z] != nullptr) {
						UE_LOG(LogTemp, Warning, TEXT("cube spawned"));
					}
				}
			}
		}
	}

	//sf.generateMesh();

}

// Called every frame
void ACubeSpawner::Tick( float DeltaTime )
{
	Super::Tick( DeltaTime );

}

