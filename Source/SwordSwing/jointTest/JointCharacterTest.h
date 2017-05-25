// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "PhysicsEngine/PhysicsConstraintComponent.h"

//#include "ForceFeedbackComponent.h"

#include "Weapon.h"
#include "GameFramework/Pawn.h"
#include "JointCharacterTest.generated.h"

USTRUCT(BlueprintType)
struct FPIDData
{
	GENERATED_USTRUCT_BODY()
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	float target;
	
	float error;
	float prev_err;
	float integral;
	float derivative;
	float adjustment;

	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	float max_adjustment;
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	float P;
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	float I;
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	float D;
};
USTRUCT(BlueprintType)
struct FPIDData2D
{
	GENERATED_USTRUCT_BODY()
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	FVector2D target;

	FVector2D error;
	FVector2D prev_err;
	FVector2D integral;
	FVector2D derivative;
	FVector2D adjustment;

	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	FVector2D max_adjustment;
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	FVector2D P;
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	FVector2D I;
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	FVector2D D;
};
USTRUCT(BlueprintType)
struct FPIDData3D
{
	GENERATED_USTRUCT_BODY()
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	FVector target;

	FVector error;
	FVector prev_err;
	FVector integral;
	FVector derivative;
	FVector adjustment;

	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	FVector max_adjustment;
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	FVector P;
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	FVector I;
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	FVector D;
};


USTRUCT()
struct FLimbState
{
	GENERATED_USTRUCT_BODY()
		
	FVector pos;
	FVector forward;
	FVector right;
	FVector up;
	FVector prev_up;
};

USTRUCT()
struct FLimbTarget
{
	GENERATED_USTRUCT_BODY()

	FVector dir;
	FVector prev_dir;
	FVector dir_xy;
	FVector prev_dir_xy;

	FVector twist_dir;
};

USTRUCT()
struct FLimbNode
{
	GENERATED_USTRUCT_BODY()

	FBodyInstance* bi;
	FLimbNode* next;

	FLimbState state;

	FPIDData3D* pid;
	FLimbTarget target;

};

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
	UPROPERTY(Category = "Arm", VisibleAnywhere)
	USphereComponent* grip_axis;
	UPROPERTY(Category = "Arm", VisibleAnywhere)
	UStaticMeshComponent* grip_axis_vis;
	UPROPERTY(Category = "Arm", VisibleAnywhere)
	UPhysicsConstraintComponent* grip_axis_attachment;

	UPROPERTY(Category = "Grip", VisibleAnywhere)
	USphereComponent* grip;
	UPROPERTY(Category = "Grip", VisibleAnywhere)
	UStaticMeshComponent* grip_vis;
	UPROPERTY(Category = "Grip", VisibleAnywhere)
	UPhysicsConstraintComponent* grip_attachment;
	UPROPERTY(Category = "Grip", VisibleAnywhere)
	UPhysicsConstraintComponent* wep_attachment;

	UPROPERTY(Category = "Grip", VisibleAnywhere)
	UDecalComponent* grip_indicator_decal;
	UPROPERTY(Category = "Grip", VisibleAnywhere)
	UParticleSystemComponent* grip_indicator_beam;
	UPROPERTY(Category = "Grip", VisibleAnywhere)
	UAudioComponent* object_attach_audio;

	TArray<FBodyInstance*> arm_BIs;


	//FBodyInstance* grip_axis_bi;
	//FBodyInstance* grip_bi;
	//FVector offset_wep_inertia;

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
	USkeletalMeshComponent* body;
	
	//upper body joint chain root
	FLimbNode upbr;
	void setTorsoTargets();
	void setPelvisTargets();

	//right leg joint chain root
	FLimbNode rlr;
	void setRThighTargets();
	void setRShinTargets();
	//left leg joint chain root
	FLimbNode llr;
	void setLThighTargets();
	void setLShinTargets();

	FBodyInstance* torsoBI;


	void cameraCalculations(float DeltaTime);
	FVector2D camera_input;
	float scaled_inverted_cam_input_size = 0.f;
	void movementCalculations(float DeltaTime);
	FVector2D movement_input;
	FVector target_direction;

	void moveForward(float AxisValue);
	void moveRight(float AxisValue);
	//cm/s
	
	void gripIndicatorCalculations(float DeltaTime);
	
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

	bool isAlive();

	bool isGuarding();

	void setCanMove(bool new_state);

	void setCanSwing(bool new_state);
	void disableSwingAbility();
	void enableSwingAbility();

	//PIDS =================================================================================
	//CAMERA -----------------------------------------------------------------
	//camera_direction_control
	//X = horizontal rotation
	//Y = vertical rotation
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	FPIDData2D cdc;

	//WEAPON -----------------------------------------------------------------
	
	//arm joint direction controllers(plural)
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	TArray<FPIDData2D> ajdc;
	//arm joint direction targets(plural)
	TArray<FLimbTarget> ajdc_targets;
	
	//arm_twist_controller
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	FPIDData atc;

	//weapon_twist_controller
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	FPIDData wtc;

	//weapon_grab_direction_controller
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	FPIDData2D wgdc;
	//weapon_grab_controller
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	FPIDData3D wgc;

	//Movement control --------------------------------------------------------------

	UPROPERTY(EditAnywhere)
	float target_hover_height = 200;
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	FPIDData hover_height;

	UPROPERTY(EditAnywhere)
	float target_speed = 400;
	UPROPERTY(EditAnywhere)
	float time_to_target_speed = 0.05f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	FPIDData2D movement_velocity;

	//BODY PIDS -----------------------------------------------------

	//Right thigh controller
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "BodyPID")
	FPIDData3D right_thigh_controller;
	//Right shin controller
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "BodyPID")
	FPIDData3D right_shin_controller;

	//Left thigh controller
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "BodyPID")
	FPIDData3D left_thigh_controller;
	//Left shin controller
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "BodyPID")
	FPIDData3D left_shin_controller;

	//pelvis controller
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "BodyPID")
	FPIDData3D pelvis_controller;
	//torso controller
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "BodyPID")
	FPIDData3D torso_controller;



private:

	void initInputVars();
	void setArmTwistTargets();

	FCalculateCustomPhysics OnCalculateCustomHoverPhysics; //HOVER
	void customHoverPhysics(float DeltaTime, FBodyInstance* BodyInstance);
	
	FCalculateCustomPhysics OnCalculateControlGripPhysics; //GRIP
	void ControlGripPhysics(float DeltaTime, FBodyInstance* BodyInstance);
	FCalculateCustomPhysics OnCalculateControlArmJointDirectionPhysics; //GRIP POSITION
	void ControlArmJointDirectionPhysics(float DeltaTime, FBodyInstance* BodyInstance);
	FCalculateCustomPhysics OnCalculateControlArmTwistPhysics; // WEAPON TWist
	void ControlArmTwistPhysics(float DeltaTime, FBodyInstance* BodyInstance);
	FCalculateCustomPhysics OnCalculateControlWeaponTwistPhysics; // WEAPON TWist
	void ControlWeaponTwistPhysics(float DeltaTime, FBodyInstance* BodyInstance);
	
	FCalculateCustomPhysics OnCalculateWeaponGrabControl; // WEAPON Grabbing
	void weaponGrabControl(float DeltaTime, FBodyInstance* BodyInstance);
	FCalculateCustomPhysics OnCalculateCustomInitGripPhysics;
	void customInitGripPhysics(float DeltaTime, FBodyInstance* BodyInstance);

	FCalculateCustomPhysics CalculateControlBody; //GRIP POSITION
	void ControlBody(float DeltaTime, FBodyInstance* BodyInstance);

	void updateLimbStates(FLimbNode* limb);
	void ControlLimb(float DeltaTime, FLimbNode* limb);
	
	void controlCameraDirection(float DeltaTime);
	void initCamera();

	void initBody();

	void initWeapon();
	void initWeaponJoints();

	void initPIDs();
	void initBodyJoints();

	void initCustomPhysics();

	//calculates the inertia of a bodyinstance relative to a point in worldspace
	void calculateRelativeInertia(FBodyInstance* offset_bi, const FVector& cor, FMatrix* out_inertia);
	float inertiaAboutAxis(const FMatrix& inertia_mat, const FVector& axis);
	

	//Camera control --------------------------------------------------------------
	FVector locked_target_dir;
	FVector locked_target_dir_xy;

	//Weapon control --------------------------------------------------------------
	//weapon control states
	bool was_standing_still = true;
	bool wep_extended = false;
	bool rot_forward = true;

	//general variables used across several function
	FVector weapon_twist_solder;
	FVector weapon_twist_target;
	FVector input_dir;
	FVector prev_input_dir;	
	
	//variables used for readability across several function

	//arm body instance states
	TArray<FLimbState> abis;

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


};
