// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "GameFramework/Actor.h"
#include "SwordLock.generated.h"

UCLASS()
class SWORDSWING_API ASwordLock : public AActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	ASwordLock();

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

	UPROPERTY(Category = "LockComponents", VisibleAnywhere)
	USceneComponent* root;

	UPROPERTY(Category = "LockComponents", VisibleAnywhere)
	UStaticMeshComponent* lock_center;

	UPROPERTY(Category = "LockComponents", VisibleAnywhere)
	USphereComponent* target_point;

	UPROPERTY(Category = "LockComponents", VisibleAnywhere)
	UStaticMeshComponent* left_wall;

	UPROPERTY(Category = "LockComponents", VisibleAnywhere)
	UStaticMeshComponent* right_wall;

	UPROPERTY(Category = "LockComponents", VisibleAnywhere)
	UPointLightComponent* light;


public:	
	// Called every frame
	virtual void Tick(float DeltaTime) override;

	UStaticMeshComponent* getLockCenter() { return lock_center; }
	
	
};
