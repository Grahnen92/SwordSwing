// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "GameFramework/Pawn.h"
#include "InitPawn.generated.h"

UCLASS()
class SWORDSWING_API AInitPawn : public APawn
{
	GENERATED_BODY()

public:
	// Sets default values for this pawn's properties
	AInitPawn();

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

	//camera ------------------------------------------------------------------------
	UPROPERTY(VisibleAnywhere)
	USpringArmComponent* camera_spring_arm;
	UPROPERTY(VisibleAnywhere)
	UCameraComponent* camera;
	UPROPERTY(VisibleAnywhere)
	USceneComponent* camera_axis;

public:	
	// Called every frame
	virtual void Tick(float DeltaTime) override;

	// Called to bind functionality to input
	virtual void SetupPlayerInputComponent(class UInputComponent* PlayerInputComponent) override;

	void cameraCalculations(float DeltaTime);
	FVector2D camera_input;

	void pitchCamera(float AxisValue);
	void yawCamera(float AxisValue);

	void initCamera();
	void setFOV(int _fov);
};
