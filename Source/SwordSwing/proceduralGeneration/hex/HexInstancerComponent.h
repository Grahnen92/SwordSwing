// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "Components/SceneComponent.h"
#include "proceduralGeneration/hex/HexGen.h"
#include "HexInstancerComponent.generated.h"


UCLASS( ClassGroup=(Custom), meta=(BlueprintSpawnableComponent) )
class SWORDSWING_API UHexInstancerComponent : public USceneComponent
{
	GENERATED_BODY()

public:	
	// Sets default values for this component's properties
	UHexInstancerComponent();

	int addInstance();
	int addInstance(UMaterialInterface* mat);

	void setInstanceMat(int index, UMaterialInterface* mat);

	int addCircle(int instance_index, float hex_rad, int circle_radius, FVector position);

	void generateAll();

protected:
	// Called when the game starts
	virtual void BeginPlay() override;

	UPROPERTY(Category = "InstancedMesh", BlueprintReadWrite, VisibleAnywhere)
	TArray<UInstancedStaticMeshComponent*> instances;

	TArray<HexGen*> generators;

public:	
	// Called every frame
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

		
	
};
