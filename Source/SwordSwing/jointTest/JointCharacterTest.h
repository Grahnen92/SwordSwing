// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "PhysicsEngine/PhysicsConstraintComponent.h"

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

	UPROPERTY(VisibleAnywhere)
	UStaticMeshComponent* weapon_axis;

	UPROPERTY(VisibleAnywhere)
	UPhysicsConstraintComponent* weapon_axis_attachment;

	UPROPERTY(VisibleAnywhere)
	UStaticMeshComponent* weapon_motor;

	UPROPERTY(VisibleAnywhere)
	UPhysicsConstraintComponent* weapon_motor_attachment;

	UPROPERTY(VisibleAnywhere)
	UStaticMeshComponent* weapon;

	UPROPERTY(VisibleAnywhere)
	UPhysicsConstraintComponent* weapon_attachment;

	FVector2D movement_input;
	FVector2D camera_input;



	void cameraCalculations(float DeltaTime);
	void movementCalculations(float DeltaTime);
	void hover(float DeltaTime);

	void moveForward(float AxisValue);
	void moveRight(float AxisValue);
	//cm/s
	
	
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

	void fightModeOn();
	void fightModeOff(); 
	FRotator axis_target_rot;
	FRotator arm_target_rot;
	FRotator axis_rot_before_fight;
	FRotator arm_rot_before_fight;
	float arm_length_before_fight;
	float target_arm_length;
	
	bool fight_mode;
	int fight_mode_state;
	float fight_t;
	UPROPERTY(EditAnywhere)
	float fight_mode_engage_time = 0.25;
	// 0 disengaged
	// 1 engaging
	// 2 engaged
	// 3 disengaging
public:	
	// Called every frame
	virtual void Tick(float DeltaTime) override;

	// Called to bind functionality to input
	virtual void SetupPlayerInputComponent(class UInputComponent* PlayerInputComponent) override;

	
private:

	void SetupJoints();

	struct PIDData
	{
		
		float target;
		float error = 0.0f;
		float prev_err = 0.0f;
		float integral = 0.0f;
		float derivative;
		float adjustment;
		float max_adjustment;

		float P;
		float I;
		float D;
	};

	struct PIDData2D
	{

		FVector2D target;
		FVector2D error;
		FVector2D prev_err;
		FVector2D integral;
		FVector2D derivative;
		FVector2D adjustment;
		FVector2D max_adjustment;

		FVector2D P;
		FVector2D I;
		FVector2D D;
	};

	struct PIDData3D
	{

		FVector target;
		FVector error;
		FVector prev_err;
		FVector integral;
		FVector derivative;
		FVector adjustment;
		FVector max_adjustment;

		FVector P;
		FVector I;
		FVector D;
	};

	//different PID data
	UPROPERTY(EditAnywhere)
	float target_hover_height = 0.0f;
	PIDData hover_height;

	PIDData3D wd;
	FVector target_wep_dir;

	UPROPERTY(EditAnywhere)
	float target_speed = 1000;
	UPROPERTY(EditAnywhere)
	float time_to_target_speed = 0.05f;
	PIDData2D movement_velocity;
};
