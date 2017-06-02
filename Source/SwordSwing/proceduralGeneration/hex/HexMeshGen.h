// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "GameFramework/Actor.h"
#include "ProceduralMeshComponent.h"
#include "HexMeshGen.generated.h"

UCLASS()
class SWORDSWING_API AHexMeshGen : public AActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	AHexMeshGen();

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

	UPROPERTY(Category = "ProcMesh", BlueprintReadWrite, VisibleAnywhere)
	UProceduralMeshComponent* hexscape;

	UPROPERTY(Category = "Root", BlueprintReadWrite, VisibleAnywhere)
	USceneComponent * root;


	UMaterialInterface* hexscape_material;
	FTexture2DMipMap* height_map;

	void createHexGrid(FVector2D res, float hex_radius);

	void createHexBox(float radius, FVector pos, float min_z);
	void createHexBoxWCollision(float radius, FVector pos, float min_z);

public:	
	// Called every frame
	virtual void Tick(float DeltaTime) override;

private:
	const static float c3;
	const static float c3m1d2;

	TArray<FVector> verts;
	TArray<FVector> vertexArray;
	const static TArray<FVector> normals;
	TArray<FVector> normalArray;
	const static TArray<FProcMeshTangent> tangents;
	TArray<FProcMeshTangent> tangentArray;
	const static TArray<FVector2D> uvs;
	TArray<FVector2D> uvArray;
	const static TArray<FLinearColor> colors;
	TArray<FLinearColor> colorArray;

	const static  TArray<int32> tris;
	
};
