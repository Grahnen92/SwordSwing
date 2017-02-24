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
	//mesh->bUseComplexAsSimpleCollision = true;
	RootComponent = mesh;
	mesh->SetSimulatePhysics(true);

	baseModel = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("Base Model"));
	baseModel->SetupAttachment(NULL);
}

// Called when the game starts or when spawned
void AMeshGenerator::BeginPlay()
{
	Super::BeginPlay();
	
	//Create signed distance function and level set from a model ===============================================================================
	FStaticMeshSourceModel* sourceM = &baseModel->GetStaticMesh()->SourceModels[0];
	FRawMesh rawMesh;
	sourceM->RawMeshBulkData->LoadRawMesh(rawMesh);
	
	float max_dim = 2.0f*FMath::Max(FMath::Max(baseModel->Bounds.BoxExtent.X, baseModel->Bounds.BoxExtent.Y), baseModel->Bounds.BoxExtent.Z);
	FVector min_dims;
	FVector max_dims;
	triangulation::findMaxMinExtent(rawMesh.VertexPositions, min_dims, max_dims);
	FVector extent = max_dims - min_dims;
	
	//<-- Used to create a scalar field that always have the same amount of scalar values but distributed 
	// at cubic intervals along boxes of different dimensions
	float k = std::cbrt((extent.Z*extent.Z) / (extent.X*extent.Y));
	float j = (extent.Y / extent.Z)*k;
	float i = (extent.X / extent.Z)*k;
	FVector res( std::ceilf(resolution*i), std::ceilf(resolution*j) , std::ceilf(resolution*k));
	FVector point_interval = extent / res;
	//FVector increased_extent(extent.X + (extent.X / res.X) * 2, extent.Y + (extent.Y / res.Y) * 2, extent.Z + (extent.Z / res.Z) * 2);
	FVector increased_extent = extent + 2 * point_interval;
	FVector mid_point = (min_dims)+(extent / 2.0f);

	//<-- create the scalar field with the determined resolution and with a somewhat increased dimension so that scalar values exist outside and around the original model
	ScalarField<float> sf(res, increased_extent);
	sf.setIsoValue(0.0f);
	//sf.meshToLeveSet(&rawMesh, mid_point);

	//triangulation::marchingCubes(mesh, &sf, mid_point);
	baseModel->SetRelativeLocation(FVector(0.f, 0.f, 100.f));
	baseModel->UnregisterComponent();
	baseModel->DestroyComponent();
	//triangulation::marchingCubes(mesh, &sf, mid_point);

	//Create signed distance function and level set for fragments ===============================================================================
	ScalarField<float> sf2(resolution, increased_extent.Z/2.0f);
	sf2.setIsoValue(0.0f);
	sf2.sphereSignedDistance(FVector::ZeroVector);
	

	//mid_point = FVector(0.0f, 0.0f, 0.0f);
	triangulation::marchingCubes(mesh, &sf2, mid_point);

	FMatrix test_mat;
	ScalarField<float> sf3(resolution, increased_extent.Z / 2.0f);
	//ScalarField<float>::mergeLevelSets(&sf, &sf2,FMatrix::Identity.ConcatTranslation(mid_point) , &sf3);
	//triangulation::marchingCubes(mesh, &sf3, mid_point);
	if(mesh->IsCollisionEnabled())
		UE_LOG(LogTemp, Warning, TEXT("physics is enabled..."));
	if(mesh->IsPhysicsCollisionEnabled())
		UE_LOG(LogTemp, Warning, TEXT("physicscollision is enabled..."));
	if(mesh->ContainsPhysicsTriMeshData(false))
		UE_LOG(LogTemp, Warning, TEXT("ContainsPhysicsTriMeshData..."));
	if(mesh->IsSimulatingPhysics())
		UE_LOG(LogTemp, Warning, TEXT("is simulating physics..."));

	
}

// Called every frame
void AMeshGenerator::Tick( float DeltaTime )
{
	Super::Tick( DeltaTime );
	mesh->AddForce(FVector(0.0f, 0.0f, 1000.0f));
}

