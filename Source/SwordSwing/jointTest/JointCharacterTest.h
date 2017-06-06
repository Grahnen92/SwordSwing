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
	
	//character states ===========================================================================
	UPROPERTY(EditAnywhere, Category = "CharacterStates")
	bool mortal = true;
	UPROPERTY(EditAnywhere, Category = "CharacterStates" )
	bool alive = true;
	UPROPERTY(EditAnywhere, Category = "CharacterStates")
	bool can_move = true;
	UPROPERTY(EditAnywhere, Category = "CharacterStates")
	bool fight_mode;

	bool dashing = false;
	UPROPERTY(EditAnywhere)
	float dash_speed = 50000;
	UPROPERTY(EditAnywhere)
	float dash_cd = 1.5f;
	float dash_cd_timer = 0.0f;
	float dash_force;
	const float dash_force_time = 0.1f;
	float dash_force_timer = 0.f;

	bool jumping;
	UPROPERTY(EditAnywhere)
	float jump_height = 100;
	float jump_force;
	const float jump_force_time = 0.1f;
	float curr_jump_time = 0.f;
	//int jump_frame_count;
	//const int jump_frame_nr = 3;

	// body components ==============================================================================
	UPROPERTY(Category = "Body", VisibleAnywhere)
		USkeletalMeshComponent* body;

	UPROPERTY(Category = "Body", VisibleAnywhere)
		UParticleSystemComponent* body_trail;

	UPROPERTY(Category = "Body", VisibleAnywhere)
		UParticleSystemComponent* left_thruster;
	UPROPERTY(Category = "Body", VisibleAnywhere)
		UParticleSystemComponent* right_thruster;

	//Weapon States ==============================================================================

	
	UPROPERTY(EditAnywhere, Category = "CharacterStates")
	bool arm_disabled = false;
	UPROPERTY(EditAnywhere, Category = "CharacterStates")
	float arm_disable_duration = 0.5f;
	float arm_disable_start;


	// 0 disengaged
	// 1 engaging
	// 2 engaged
	// 3 disengaging
	int fight_mode_state;
	float fight_t;
	UPROPERTY(EditAnywhere)
	float fight_mode_engage_time = 0.25;
	// 0 normal
	// 1 guarding
	// 2 guarding locked
	// 3 disabled
	int fight_stance = 0;

	bool was_standing_still = true;
	bool wep_extended = false;
	bool rot_forward = true;

	UPROPERTY(EditAnywhere, Category = "CharacterStates")
	bool guarding = false;
	bool guard_locked = false;

	bool grabbing_weapon = false;
	bool holding_weapon = false;
	bool holding_object = false;

	void fightModeOn();
	void fightModeOff();
	void release();
	void grab();
	void abortGrab();
	UFUNCTION(BlueprintCallable)
	void attachWeapon(AWeapon* _wep);
	AWeapon* held_weapon;
	UObject* held_object;

	void guard();
	void abortGuard();
	void lockGuard();
	void unlockGuard();

	//weapon components ==============================================================================
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



	//camera ==============================================================================
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
	USphereComponent* targeting_aura;
	UPROPERTY(Category = "Weapon", VisibleAnywhere)
	USphereComponent* targeting_point;
	
	UFUNCTION()
	void addPotentialTarget(UPrimitiveComponent* OverlappedComp, AActor* OtherActor, UPrimitiveComponent* OtherComp, int32 OtherBodyIndex, bool bFromSweep, const FHitResult& SweepResult);
	UFUNCTION()
	void removePotentialTarget(UPrimitiveComponent* OverlappedComp, AActor* OtherActor, UPrimitiveComponent* OtherComp, int32 OtherBodyIndex);
	void aquireTarget();

	void calculateWepInertia();

	FRotator axis_target_rot;
	FRotator arm_target_rot;
	FRotator axis_rot_before_fight;
	FRotator arm_rot_before_fight;
	float arm_length_before_fight;
	float target_arm_length;


	//Camera INPUT ==============================================================================
	void pitchCamera(float AxisValue);
	void yawCamera(float AxisValue);
	void cameraCalculations(float DeltaTime);
	FVector2D camera_input;
	float scaled_inverted_cam_input_size = 0.f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Input")
	FVector input_dir;
	FVector prev_input_dir;

	//movement INPUT ==============================================================================
	void movementCalculations(float DeltaTime);
	FVector2D movement_input;
	FVector target_direction;

	void moveForward(float AxisValue);
	void moveRight(float AxisValue);

	void dash();
	void jump();
	//cm/s
	


public:	


	// UNREAL FUNCTIONS ==============================================================================
	virtual void Tick(float DeltaTime) override;
	virtual void SetupPlayerInputComponent(class UInputComponent* PlayerInputComponent) override;
	
	// STATE checkers/setters/getters ==============================================================================
	void setPlayerSpecificMaterial(UMaterial* mat);
	void setAlive(bool new_state);
	bool isAlive();

	bool isGuarding();

	void setCanMove(bool new_state);

	void setArmDisabled(bool new_state);
	void disableArm(float duration);
	void enableArm();
	void setInputDir(FVector _dir);

	void setFOV(int _fov);

	//PIDS =================================================================================
	//CAMERA -----------------------------------------------------------------
	//camera_direction_control
	//X = horizontal rotation
	//Y = vertical rotation
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	FPIDData2D cdc;

	FVector locked_target_dir;
	FVector locked_target_dir_xy;

	//WEAPON -----------------------------------------------------------------
	
	//arm joint direction controllers(plural)
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	TArray<FPIDData2D> ajdc;
	//arm joint direction targets(plural)
	TArray<FLimbTarget> ajdc_targets;
	
	//general variables used across several function
	FVector weapon_twist_solder;
	FVector weapon_twist_target;

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



protected:

	//COLLISION FUNCTIONS =================================================================================
	void FellOutOfWorld(const class UDamageType& dmgType);
	UFUNCTION()
	void OnBodyHit(UPrimitiveComponent* HitComp, AActor* OtherActor, UPrimitiveComponent* OtherComp, FVector NormalImpulse, const FHitResult& Hit);


	//CONTROL FUNCTIONS =================================================================================
	void initInputVars();
	void setNormalStanceTargets();
	void setGuardingStanceTargets();
	void setDisabledStanceTargets();
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
	FCalculateCustomPhysics CalculateControlBody;
	void ControlBody(float DeltaTime, FBodyInstance* BodyInstance);

	void updateLimbStates(FLimbNode* limb);
	void ControlLimb(float DeltaTime, FLimbNode* limb);
	
	void controlCameraDirection(float DeltaTime);

	void gripIndicatorCalculations(float DeltaTime);

	// INIT FUNCTIONS ====================================================================================================
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
