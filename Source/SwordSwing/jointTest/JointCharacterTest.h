// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "GameFramework/Pawn.h"
#include "JointCharacterTest.generated.h"

UCLASS()
class SWORDSWING_API AJointCharacterTest : public APawn
{
	GENERATED_BODY()

public:
	// Sets default values for this pawn's properties
	AJointCharacterTest();

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;
	//UPROPERTY(EditAnywhere)
	UPROPERTY(VisibleAnywhere)
	USpringArmComponent* camera_spring_arm;
	UPROPERTY(VisibleAnywhere)
	UCameraComponent* camera;
	UPROPERTY(VisibleAnywhere)
	USceneComponent* camera_axis;

	UPROPERTY(VisibleAnywhere)
	UStaticMeshComponent* rolling_body;

	FVector2D movement_input;
	FVector2D camera_input;



	void cameraCalculations(float DeltaTime);
	void movementCalculations(float DeltaTime);
	void hover(float DeltaTime);

	void moveForward(float AxisValue);
	void moveRight(float AxisValue);
	//cm/s
	float target_speed = 1000;
	float time_to_target_speed = 0.05f;
	
	void jump();
	bool jumping;
	UPROPERTY(EditAnywhere)
	float jump_height = 100;
	float jump_force;
	const float jump_force_time = 0.1f;
	float curr_jump_time = 0.f;
	//int jump_frame_count;
	//const int jump_frame_nr = 3;

	void pitchCamera(float AxisValue);
	void yawCamera(float AxisValue);
	
	void zoomIn();
	void zoomOut();
	float zoom_factor;
	bool zooming;

public:	
	// Called every frame
	virtual void Tick(float DeltaTime) override;

	// Called to bind functionality to input
	virtual void SetupPlayerInputComponent(class UInputComponent* PlayerInputComponent) override;

	
private:
	struct PIDData
	{
		
		float wanted;
		float error = 0.0f;
		float prev_err = 0.0f;
		float integral = 0.0f;
		float derivative;
		float adjustment;
		float max_adjustment;
	};

	UPROPERTY(EditAnywhere)
	float wanted_hover_height = 0.0f;

	PIDData hover_height;
};
