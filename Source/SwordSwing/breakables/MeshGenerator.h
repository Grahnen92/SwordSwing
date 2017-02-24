// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "GameFramework/Actor.h"
#include "ProceduralMeshComponent.h"
#include "MeshGenerator.generated.h"

UCLASS()
class SWORDSWING_API AMeshGenerator : public AActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	AMeshGenerator();

	// Called when the game starts or when spawned
	virtual void BeginPlay() override;
	
	UPROPERTY(Category = "Mesh", BlueprintReadWrite, VisibleAnywhere)
		class UProceduralMeshComponent * mesh;

	UPROPERTY(Category = "Mesh", BlueprintReadWrite, VisibleAnywhere)
		UStaticMeshComponent* baseModel;

	UPROPERTY(Category = "Mesh", BlueprintReadWrite, EditAnywhere)
		int32 resolution = 4;
	// Called every frame
	virtual void Tick( float DeltaSeconds ) override;

private:
	
	
	
};
