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

	//camera ------------------------------------------------------------------------
	UPROPERTY(VisibleAnywhere)
	USpringArmComponent* camera_spring_arm;
	UPROPERTY(VisibleAnywhere)
	UCameraComponent* camera;
	UPROPERTY(VisibleAnywhere)
	USceneComponent* camera_axis;

	//Upper body ------------------------------------------------------------------------

	UPROPERTY(VisibleAnywhere)
	UBoxComponent* torso;
	UPROPERTY(VisibleAnywhere)
	UStaticMeshComponent* torso_vis;
	FBodyInstance* torso_bi;

	UPROPERTY(VisibleAnywhere)
	UCapsuleComponent* spine;
	UPROPERTY(VisibleAnywhere)
	UStaticMeshComponent* spine_vis;
	FBodyInstance* spine_bi;
	UPROPERTY(Category = "UpperBody", VisibleAnywhere)
	UPhysicsConstraintComponent* spine_attachment;

	//weapon ------------------------------------------------------------------------

	UPROPERTY(Category = "Weapon", VisibleAnywhere)
	USphereComponent* weapon_axis;
	UPROPERTY(Category = "Weapon", VisibleAnywhere)
	UStaticMeshComponent* weapon_axis_vis;
	FBodyInstance* weapon_axis_bi;
	UPROPERTY(Category = "Weapon", VisibleAnywhere)
	UPhysicsConstraintComponent* weapon_axis_attachment;

	UPROPERTY(Category = "Weapon", VisibleAnywhere)
	USphereComponent* weapon_handle_1;
	UPROPERTY(Category = "Weapon", VisibleAnywhere)
	UStaticMeshComponent* weapon_handle_1_vis;
	FBodyInstance* weapon_handle_1_bi;
	UPROPERTY(Category = "Weapon", VisibleAnywhere)
	UPhysicsConstraintComponent* weapon_handle_1_attachment;

	UPROPERTY(Category = "Weapon", VisibleAnywhere)
	USphereComponent* weapon_handle_2;
	UPROPERTY(Category = "Weapon", VisibleAnywhere)
	UStaticMeshComponent* weapon_handle_2_vis;
	FBodyInstance* weapon_handle_2_bi;
	UPROPERTY(Category = "Weapon", VisibleAnywhere)
	UPhysicsConstraintComponent* weapon_handle_2_attachment;

	UPROPERTY(Category = "Weapon", VisibleAnywhere)
	UCapsuleComponent* weapon;
	UPROPERTY(Category = "Weapon", VisibleAnywhere)
	UStaticMeshComponent* weapon_vis;
	FBodyInstance* weapon_bi;

	UPROPERTY(Category = "Weapon", VisibleAnywhere)
	UPhysicsConstraintComponent* weapon_attachment;

	FVector offset_wep_inertia;

	void fightModeOn();
	void fightModeOff();
	void releaseWeapon();
	bool holding_weapon = true;

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

	//legs -----------------------------------------------------------------------
	
	UPROPERTY(Category = "Legs", VisibleAnywhere)
	UCapsuleComponent* pelvis;
	UPROPERTY(Category = "Legs", VisibleAnywhere)
	UStaticMeshComponent* pelvis_visu;
	FBodyInstance* pelvis_bi;
	UPROPERTY(Category = "Legs", VisibleAnywhere)
	UPhysicsConstraintComponent* pelvis_attachment;

	//right leg ------------------------------------------------------------------------
	UPROPERTY(Category = "Legs", VisibleAnywhere)
	USceneComponent* right_leg_axis;
	
	UPROPERTY(Category = "Legs", VisibleAnywhere)
	USphereComponent* r_hip_motor;
	UPROPERTY(Category = "Legs", VisibleAnywhere)
	UStaticMeshComponent* r_hip_motor_vis;
	FBodyInstance* r_hip_motor_bi;
	UPROPERTY(Category = "Legs", VisibleAnywhere)
	UPhysicsConstraintComponent* r_hip_attachment;

	UPROPERTY(Category = "Legs", VisibleAnywhere)
	UCapsuleComponent* r_thigh;
	UPROPERTY(Category = "Legs", VisibleAnywhere)
	UStaticMeshComponent* r_thigh_vis;
	FBodyInstance* r_thigh_bi;
	UPROPERTY(Category = "Legs", VisibleAnywhere)
	UPhysicsConstraintComponent* r_thigh_attachment;

	UPROPERTY(Category = "Legs", VisibleAnywhere)
	USphereComponent* r_knee_motor;
	UPROPERTY(Category = "Legs", VisibleAnywhere)
	UStaticMeshComponent* r_knee_motor_vis;
	FBodyInstance* r_knee_motor_bi;
	UPROPERTY(Category = "Legs", VisibleAnywhere)
	UPhysicsConstraintComponent* r_knee_attachment;

	UPROPERTY(Category = "Legs", VisibleAnywhere)
	UCapsuleComponent* r_shin;
	UPROPERTY(Category = "Legs", VisibleAnywhere)
	UStaticMeshComponent* r_shin_vis;
	FBodyInstance* r_shin_bi;
	UPROPERTY(Category = "Legs", VisibleAnywhere)
	UPhysicsConstraintComponent* r_shin_attachment;

	UPROPERTY(Category = "Legs", VisibleAnywhere)
	UCapsuleComponent* r_toe;
	FBodyInstance* r_toe_bi;
	UPROPERTY(Category = "Legs", VisibleAnywhere)
	UStaticMeshComponent* r_toe_vis;
	//UPROPERTY(Category = "Legs", VisibleAnywhere)
	//UPhysicsConstraintComponent* r_toe_attachment;
	//left leg ------------------------------------------------------------------------
	UPROPERTY(Category = "Legs", VisibleAnywhere)
	USceneComponent* left_leg_axis;

	UPROPERTY(Category = "Legs", VisibleAnywhere)
	USphereComponent* l_hip_motor;
	UPROPERTY(Category = "Legs", VisibleAnywhere)
	UStaticMeshComponent* l_hip_motor_vis;
	FBodyInstance* l_hip_motor_bi;
	UPROPERTY(Category = "Legs", VisibleAnywhere)
	UPhysicsConstraintComponent* l_hip_attachment;

	UPROPERTY(Category = "Legs", VisibleAnywhere)
	UCapsuleComponent* l_thigh;
	UPROPERTY(Category = "Legs", VisibleAnywhere)
	UStaticMeshComponent* l_thigh_vis;
	FBodyInstance* l_thigh_bi;
	UPROPERTY(Category = "Legs", VisibleAnywhere)
	UPhysicsConstraintComponent* l_thigh_attachment;

	UPROPERTY(Category = "Legs", VisibleAnywhere)
	USphereComponent* l_knee_motor;
	UPROPERTY(Category = "Legs", VisibleAnywhere)
	UStaticMeshComponent* l_knee_motor_vis;
	FBodyInstance* l_knee_motor_bi;
	UPROPERTY(Category = "Legs", VisibleAnywhere)
	UPhysicsConstraintComponent* l_knee_attachment;

	UPROPERTY(Category = "Legs", VisibleAnywhere)
	UCapsuleComponent* l_shin;
	UPROPERTY(Category = "Legs", VisibleAnywhere)
	UStaticMeshComponent* l_shin_vis;
	FBodyInstance* l_shin_bi;
	UPROPERTY(Category = "Legs", VisibleAnywhere)
	UPhysicsConstraintComponent* l_shin_attachment;

	UPROPERTY(Category = "Legs", VisibleAnywhere)
	UCapsuleComponent* l_toe;
	FBodyInstance* l_toe_bi;
	UPROPERTY(Category = "Legs", VisibleAnywhere)
	UStaticMeshComponent* l_toe_vis;
	//UPROPERTY(Category = "Legs", VisibleAnywhere)
	//UPhysicsConstraintComponent* l_toe_attachment;

	
	

	void cameraCalculations(float DeltaTime);
	FVector2D camera_input;
	void movementCalculations(float DeltaTime);
	FVector2D movement_input;
	FVector target_direction;

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


public:	

	/** called when projectile hits something */
	UFUNCTION()
	void OnHit(UPrimitiveComponent* HitComp, AActor* OtherActor, UPrimitiveComponent* OtherComp, FVector NormalImpulse, const FHitResult& Hit);

	// Called every frame
	virtual void Tick(float DeltaTime) override;


	// Called to bind functionality to input
	virtual void SetupPlayerInputComponent(class UInputComponent* PlayerInputComponent) override;
	
private:

	FCalculateCustomPhysics OnCalculateCustomHoverPhysics;
	void customHoverPhysics(float DeltaTime, FBodyInstance* BodyInstance);
	FCalculateCustomPhysics OnCalculateCustomStabilizerPhysics;
	void customStabilizerPhysics(float DeltaTime, FBodyInstance* BodyInstance);
	FCalculateCustomPhysics OnCalculateCustomWeaponPhysics;
	void customWeaponPhysics(float DeltaTime, FBodyInstance* BodyInstance);
	FCalculateCustomPhysics OnCalculateCustomWalkingPhysics;
	void customWalkingPhysics(float DeltaTime, FBodyInstance* BodyInstance);

	
	void initCamera();

	void initUpperBody();
	void initUpperBodyJoints();

	void initWeapon();
	void initWeaponJoints();

	void initLegs();
	void initLegJoints();

	void initPIDs();

	void initCustomPhysics();

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

	UPROPERTY(EditAnywhere)
	float target_hover_height = 200;
	PIDData hover_height;

	PIDData3D sword_motor_pos;
	PIDData sword_rotation;
	PIDData sword_rotation_speed;
	PIDData sword_incline;
	PIDData sword_twist;
	FVector sword_twist_solder;
	FVector sword_twist_target;
	FVector target_wep_dir;
	FVector prev_target_wep_dir;
	FVector prev_target_wep_dir_xy;
	bool was_standing_still = true;
	bool wep_extended = false;

	//torso_twist_controller
	PIDData3D ttc;
	//torso_position_controller
	PIDData2D pose_controller;

	//spine_swing_controller
	PIDData3D ssc;
	//spine_twist_controller
	PIDData3D stc;

	//pelvis_twist_controller
	PIDData3D ptc;

	//r_thigh_direction_controller
	PIDData3D rtdc;
	//r_knee_extension_controller
	PIDData rkec;
	PIDData3D r_toe_position_controller;
	//l_thigh_direction_controller
	PIDData3D ltdc;
	//l_knee_extension_controller
	PIDData lkec;
	PIDData3D l_toe_position_controller;

	UPROPERTY(EditAnywhere)
	float target_speed = 500;
	UPROPERTY(EditAnywhere)
	float time_to_target_speed = 0.05f;
	PIDData2D movement_velocity;
};
