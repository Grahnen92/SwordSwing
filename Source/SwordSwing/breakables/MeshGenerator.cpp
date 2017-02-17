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
	
	float max_dim = 2.0f*FMath::Max(FMath::Max(baseModel->Bounds.BoxExtent.X, baseModel->Bounds.BoxExtent.Y), baseModel->Bounds.BoxExtent.Z);
	FVector min_dims;
	FVector max_dims;
	triangulation::findMaxMinExtent(rawMesh.VertexPositions, min_dims, max_dims);
	FVector extent = max_dims - min_dims;

	float k = std::ceil(std::cbrt((extent.Z*extent.Z) / (extent.X*extent.Y)));
	float j = std::ceil((extent.Y / extent.Z)*k);
	float i = std::ceil((extent.X / extent.Z)*k);
	FVector res( resolution*i, resolution*j , resolution*k);

	//ScalarField<float> sf(resolution + 2, max_dim + 2.0f*max_dim/resolution);
	ScalarField<float> sf(res, FVector(extent.X + (extent.X/res.X)*2, extent.Y + (extent.Y / res.Y) * 2, extent.Z + (extent.Z / res.Z) * 2));
	sf.setIsoValue(0.0f);
	sf.meshToLeveSet(&rawMesh);
	//sf.setHalfOfAllValues(255.0f);


	triangulation::marchingCubes(mesh, &sf);
}

// Called every frame
void AMeshGenerator::Tick( float DeltaTime )
{
	Super::Tick( DeltaTime );
	
}

