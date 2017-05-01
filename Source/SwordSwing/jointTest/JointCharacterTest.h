// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "PhysicsEngine/PhysicsConstraintComponent.h"

//#include "ForceFeedbackComponent.h"

#include "Weapon.h"
#include "GameFramework/Pawn.h"
#include "JointCharacterTest.generated.h"


UCLASS()
class SWORDSWING_API AJointCharacterTest : public APawn
{
	GENERATED_BODY()

public:
	// Sets default values for this pawn's properties
	AJointCharacterTest();

	bool isAlive();

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;
	
	bool alive = true;

	bool can_move = true;
	bool can_swing = true;


	//camera ------------------------------------------------------------------------
	UPROPERTY(VisibleAnywhere)
	USpringArmComponent* camera_spring_arm;
	UPROPERTY(VisibleAnywhere)
	UCameraComponent* camera;
	UPROPERTY(VisibleAnywhere)
	USceneComponent* camera_axis;

	//targeting
	TArray<USceneComponent* >lock_on_targets;
	USceneComponent* locked_target;
	UPROPERTY(Category = "Weapon", VisibleAnywhere)
	USphereComponent* targeting_sphere;
	
	UFUNCTION()
	void addPotentialTarget(UPrimitiveComponent* OverlappedComp, AActor* OtherActor, UPrimitiveComponent* OtherComp, int32 OtherBodyIndex, bool bFromSweep, const FHitResult& SweepResult);
	UFUNCTION()
	void removePotentialTarget(UPrimitiveComponent* OverlappedComp, AActor* OtherActor, UPrimitiveComponent* OtherComp, int32 OtherBodyIndex);
	
	void aquireTarget();

	//weapon ------------------------------------------------------------------------
	UPROPERTY(Category = "Weapon", VisibleAnywhere)
	USphereComponent* grip_axis;
	UPROPERTY(Category = "Weapon", VisibleAnywhere)
	UStaticMeshComponent* grip_axis_vis;
	FBodyInstance* grip_axis_bi;
	UPROPERTY(Category = "Weapon", VisibleAnywhere)
	UPhysicsConstraintComponent* grip_axis_attachment;

	UPROPERTY(Category = "Weapon", VisibleAnywhere)
	USphereComponent* grip;
	UPROPERTY(Category = "Weapon", VisibleAnywhere)
	UStaticMeshComponent* grip_vis;
	FBodyInstance* grip_bi;
	UPROPERTY(Category = "Weapon", VisibleAnywhere)
	UPhysicsConstraintComponent* grip_attachment;
	UPROPERTY(Category = "Weapon", VisibleAnywhere)
	UPhysicsConstraintComponent* wep_attachment;

	FVector offset_wep_inertia;

	void fightModeOn();
	void fightModeOff();
	void release();
	void grab();
	void abortGrab();
	void attachWeapon(AWeapon* _wep);

	void guard();
	void abortGuard();
	void lockGuard();
	void unlockGuard();
	bool guarding = false;
	bool guard_locked = false;
	
	void calculateWepInertia();
	bool grabbing_weapon = false;
	bool holding_weapon = false;
	AWeapon* held_weapon;
	bool holding_object = false;
	UObject* held_object;

	FRotator axis_target_rot;
	FRotator arm_target_rot;
	FRotator axis_rot_before_fight;
	FRotator arm_rot_before_fight;
	float arm_length_before_fight;
	float target_arm_length;

	bool fight_mode;

	// 0 disengaged
	// 1 engaging
	// 2 engaged
	// 3 disengaging
	int fight_mode_state;
	float fight_t;
	UPROPERTY(EditAnywhere)
	float fight_mode_engage_time = 0.25;


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
	float scaled_inverted_cam_input_size = 0.f;
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

	UFUNCTION()
	void OnBodyHit(UPrimitiveComponent* HitComp, AActor* OtherActor, UPrimitiveComponent* OtherComp, FVector NormalImpulse, const FHitResult& Hit);

	// Called every frame
	virtual void Tick(float DeltaTime) override;

	// Called to bind functionality to input
	virtual void SetupPlayerInputComponent(class UInputComponent* PlayerInputComponent) override;
	
	void setPlayerSpecificMaterial(UMaterial* mat);

	void setCanMove(bool new_state);
	void setCanSwing(bool new_state);

private:

	void initInputVars();

	FCalculateCustomPhysics OnCalculateCustomHoverPhysics; //HOVER
	void customHoverPhysics(float DeltaTime, FBodyInstance* BodyInstance);
	FCalculateCustomPhysics OnCalculateCustomStabilizerPhysics;//MOVEMENT
	void customStabilizerPhysics(float DeltaTime, FBodyInstance* BodyInstance);
	FCalculateCustomPhysics OnCalculateControlGripPhysics; //GRIP
	void ControlGripPhysics(float DeltaTime, FBodyInstance* BodyInstance);
	FCalculateCustomPhysics OnCalculateControlArmDirectionPhysics; //GRIP POSITION
	void ControlArmDirectionPhysics(float DeltaTime, FBodyInstance* BodyInstance);
	FCalculateCustomPhysics OnCalculateControlGripDirectionPhysics; //GRIP DIRECTION
	void ControlGripDirectionPhysics(float DeltaTime, FBodyInstance* BodyInstance);
	FCalculateCustomPhysics OnCalculateControlArmTwistPhysics; // WEAPON TWist
	void ControlArmTwistPhysics(float DeltaTime, FBodyInstance* BodyInstance);
	FCalculateCustomPhysics OnCalculateControlGripInclinePhysics; //GRIP INCLINE
	void ControlGripInclinePhysics(float DeltaTime, FBodyInstance* BodyInstance);
	FCalculateCustomPhysics OnCalculateControlWeaponTwistPhysics; // WEAPON TWist
	void ControlWeaponTwistPhysics(float DeltaTime, FBodyInstance* BodyInstance);
	FCalculateCustomPhysics OnCalculateWeaponGrabControl; // WEAPON Grabbing
	void weaponGrabControl(float DeltaTime, FBodyInstance* BodyInstance);
	FCalculateCustomPhysics OnCalculateCustomInitGripPhysics;
	void customInitGripPhysics(float DeltaTime, FBodyInstance* BodyInstance);
	FCalculateCustomPhysics OnCalculateCustomWalkingPhysics;
	void customWalkingPhysics(float DeltaTime, FBodyInstance* BodyInstance);
	
	void controlCameraDirection(float DeltaTime);
	void initCamera();

	void initUpperBody();
	void initUpperBodyJoints();

	void initWeapon();
	void initWeaponJoints();

	void initLegs();
	void initLegJoints();

	void initPIDs();

	void initCustomPhysics();

	//calculates the inertia of a bodyinstance relative to a point in worldspace
	void calculateRelativeInertia(FBodyInstance* offset_bi, const FVector& cor, FMatrix* out_inertia);
	float inertiaAboutAxis(const FMatrix& inertia_mat, const FVector& axis);
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

	//Camera control --------------------------------------------------------------
	//camera_direction_control
	//X = horizontal rotation
	//Y = vertical rotation
	PIDData2D cdc;

	FVector locked_target_dir;
	FVector locked_target_dir_xy;

	//Weapon control --------------------------------------------------------------
	//weapon control states
	bool was_standing_still = true;
	bool wep_extended = false;
	bool rot_forward = true;

	//arm_direction_controller
	//X = direction control
	//Y = rotational speed control
	PIDData2D adc;
	//arm_twist_controller
	PIDData atc;

	//g_direction_controller
	//X = direction control
	//Y = rotational speed control
	PIDData3D gdc;
	//weapon_twist_controller
	PIDData wtc;

	//weapon_grab_direction_controller
	PIDData2D wgdc;
	//weapon_grab_controller
	PIDData3D wgc;

	//general variables used across several function
	FVector weapon_twist_solder;
	FVector weapon_twist_target;
	FVector input_dir;
	FVector prev_input_dir;
	FVector target_arm_dir;
	FVector target_arm_dir_xy;
	FVector prev_target_arm_dir_xy;

	FVector target_wep_dir;
	FVector prev_target_wep_dir;
	FVector prev_target_wep_dir_xy;
	FVector target_wep_dir_xy;
	FVector target_wep_dir_curr_wep_proj;
	
	//variables used for readability across several function
	FVector ga_pos;
	FVector ga_forward;
	FVector ga_right;
	FVector ga_up;
	FVector ga_prev_up;
	
	FVector g_pos;
	FVector g_pos_offset;
	FVector g_forward;
	FVector g_right;
	FVector g_up;
	FVector g_prev_up;
	
	FVector current_griph_xydir;

	FVector w_pos;
	FVector w_up;
	FVector w_prev_up;

	//Movement control --------------------------------------------------------------

	UPROPERTY(EditAnywhere)
	float target_hover_height = 200;
	PIDData hover_height;

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
	float target_speed = 300;
	UPROPERTY(EditAnywhere)
	float time_to_target_speed = 0.05f;
	PIDData2D movement_velocity;
};
