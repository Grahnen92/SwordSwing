// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "GameFramework/Actor.h"

#include "proceduralGeneration/hex/HexGenComponent.h"
#include "proceduralGeneration/hex/HexCircleGenComponent.h"
#include "proceduralGeneration/hex/HexGridGenComponent.h"

#include <queue>

#include "HexInstancer.generated.h"

UCLASS()
class SWORDSWING_API AHexInstancer : public AActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	AHexInstancer();

	int addInstance();
	int addInstance(UMaterialInterface* mat);

	void setInstanceMat(int index, UMaterialInterface* mat);

	UHexCircleGenComponent* addCircle(int instance_index, float hex_rad, int circle_radius, FVector position);
	UHexGridGenComponent* addGrid(int instance_index, float hex_rad, int x_halfdim, int y_halfdim, FVector position);
	int addGen(UHexGenComponent* gen);
	void generateAll();

	void pushAnim(UHexGenComponent* a_gen);
	void popAnim();

	void animTick(float DeltaTime);
protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

	UPROPERTY(Category = "InstancedMesh", BlueprintReadWrite, VisibleAnywhere)
	TArray<UInstancedStaticMeshComponent*> instances;

	UPROPERTY(Category = "InstancedMesh", BlueprintReadWrite, VisibleAnywhere)
	TArray<UHexGenComponent*> generators;
	
	std::queue <UHexGenComponent*> animating_generators;

	UPROPERTY(Category = "Root", BlueprintReadWrite, VisibleAnywhere)
	USceneComponent * root;

public:	
	// Called every frame
	virtual void Tick(float DeltaTime) override;

	
	
};
