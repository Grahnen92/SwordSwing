// Fill out your copyright notice in the Description page of Project Settings.


#include "SwordSwing.h"
#include "ScalarField.h"
#include "MeshGenerator.h"
#include "Triangulation.h"



// Sets default values
AMeshGenerator::AMeshGenerator()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
	mesh = CreateDefaultSubobject<UProceduralMeshComponent>(TEXT("Terrain Mesh"));
}

// Called when the game starts or when spawned
void AMeshGenerator::BeginPlay()
{
	Super::BeginPlay();
	
	ScalarField<float> sf(100, 1000);
	sf.setHalfOfAllValues(255.0f);
	//sf.generateMesh(mesh);

	triangulation::test();
}

// Called every frame
void AMeshGenerator::Tick( float DeltaTime )
{
	Super::Tick( DeltaTime );
	
}

