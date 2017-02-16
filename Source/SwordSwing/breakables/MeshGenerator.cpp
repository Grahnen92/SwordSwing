// Fill out your copyright notice in the Description page of Project Settings.


#include "SwordSwing.h"
#include "ScalarField.h"
#include "MeshGenerator.h"
#include "Triangulation.h"
#include "RawMesh.h"
#include "Components/StaticMeshComponent.h"



// Sets default values
AMeshGenerator::AMeshGenerator()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
	mesh = CreateDefaultSubobject<UProceduralMeshComponent>(TEXT("Procedural Mesh"));
	baseModel = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("Base Model"));
}

// Called when the game starts or when spawned
void AMeshGenerator::BeginPlay()
{
	Super::BeginPlay();
	
	FStaticMeshSourceModel* sourceM = &baseModel->StaticMesh->SourceModels[0];
	FRawMesh rawMesh;
	sourceM->RawMeshBulkData->LoadRawMesh(rawMesh);

	ScalarField<float> sf(100, baseModel->Bounds.GetBox().GetExtent());
	sf.meshToLeveSet(&rawMesh);
	//sf.setHalfOfAllValues(255.0f);


	triangulation::marchingCubes(mesh, &sf);
}

// Called every frame
void AMeshGenerator::Tick( float DeltaTime )
{
	Super::Tick( DeltaTime );
	
}

