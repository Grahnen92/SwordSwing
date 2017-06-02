// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "Components/SceneComponent.h"
#include "HexGenComponent.generated.h"


UCLASS( ClassGroup=(Custom), meta=(BlueprintSpawnableComponent) )
class SWORDSWING_API UHexGenComponent : public USceneComponent
{
	GENERATED_BODY()

public:	
	// Sets default values for this component's properties
	UHexGenComponent();
	UHexGenComponent(UInstancedStaticMeshComponent* _instance, float hex_radius, FVector position);

	virtual void generate() { UE_LOG(LogTemp, Warning, TEXT(" You should never create an instance of UHexGenComponent, instead create instances of its subclasses")) }

protected:
	// Called when the game starts
	virtual void BeginPlay() override;
	int instance_offset;

public:	
	// Called every frame
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

	UPROPERTY(VisibleAnywhere)
	float hrad;
	UPROPERTY(VisibleAnywhere)
	UInstancedStaticMeshComponent* instance;

	class AHexInstancer* instancer;

	bool animating = false;
	bool p_animating = false;

private:

	
	
};
