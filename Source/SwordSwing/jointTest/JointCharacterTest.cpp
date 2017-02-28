// Fill out your copyright notice in the Description page of Project Settings.

#include "SwordSwing.h"
#include "JointCharacterTest.h"


// Sets default values
AJointCharacterTest::AJointCharacterTest()
{
 	// Set this pawn to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
	RootComponent = CreateDefaultSubobject<USceneComponent>(TEXT("RootComponent"));
	
	rolling_body = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("PhysicalCharacter"));
	rolling_body->SetupAttachment(RootComponent);

	// Weapon settings
	weapon_axis = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("WeaponAxis"));
	weapon_axis->SetupAttachment(RootComponent);
	weapon_axis->SetRelativeLocation(FVector(0.f, 0.f, 110.f));
	weapon_axis->SetRelativeScale3D(FVector(1.0f, 1.0f, 0.05f));

	weapon_axis_attachment = CreateDefaultSubobject<UPhysicsConstraintComponent>(TEXT("WeaponAxisAttachment"));
	weapon_axis_attachment->SetupAttachment(rolling_body);
	weapon_axis_attachment->SetRelativeLocation(FVector(0.f, 0.f, 50.f));

	weapon_motor = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("WeaponMotor"));
	weapon_motor->SetupAttachment(RootComponent);
	weapon_motor->SetRelativeLocation(FVector(0.f, 0.f, 110.f));
	weapon_motor->SetRelativeScale3D(FVector(0.5f, 0.5f, 0.05f));

	weapon_motor_attachment = CreateDefaultSubobject<UPhysicsConstraintComponent>(TEXT("WeaponMotorAttachment"));
	weapon_motor_attachment->SetupAttachment(weapon_axis);
	weapon_motor_attachment->SetRelativeLocation(FVector(0.f, 0.f, 50.f));

	weapon = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("Weapon"));
	weapon->SetupAttachment(RootComponent);
	weapon->SetRelativeLocation(FVector(180.f, 0.f, 110.f));
	weapon->SetRelativeScale3D(FVector(2.0f, 0.2f, 0.05f));

	weapon_attachment = CreateDefaultSubobject<UPhysicsConstraintComponent>( TEXT("WeaponAttachment"));
	weapon_attachment->SetupAttachment(weapon_motor);
	//weapon_attachment->SetRelativeLocation(FVector(0.f, 0.f, 50.f));
	
	//camera settings
	camera_axis = CreateDefaultSubobject<USceneComponent>(TEXT("CameraAxis"));
	camera_axis->SetupAttachment(RootComponent);

	camera_spring_arm = CreateDefaultSubobject<USpringArmComponent>(TEXT("CameraSpringArm"));
	camera_spring_arm->SetupAttachment(camera_axis);
	camera_spring_arm->SetRelativeLocationAndRotation(FVector(0.0f, 0.0f, 50.0f), FRotator(-60.0f, 0.0f, 0.0f));
	camera_spring_arm->TargetArmLength = 400.f;
	camera_spring_arm->bEnableCameraLag = true;
	camera_spring_arm->CameraLagSpeed = 6.0f;

	camera = CreateDefaultSubobject<UCameraComponent>(TEXT("GameCamera"));
	camera->SetupAttachment(camera_spring_arm, USpringArmComponent::SocketName);

	//player settings
	AutoPossessPlayer = EAutoReceiveInput::Player0;
	//git test
	SetupJoints();
}

// Called when the game starts or when spawned
void AJointCharacterTest::BeginPlay()
{
	Super::BeginPlay();
	hover_height.wanted = wanted_hover_height;
	hover_height.max_adjustment = 1000000;

	sword_rotation.wanted = 0.0f;
	sword_rotation.max_adjustment = 1000000;

	axis_target_rot = FRotator(0.f, 150.f, 0.f);
	arm_target_rot = FRotator(-65.f, 0.f, 0.f);
	target_arm_length = 1000.f;

	
	
}

// Called every frame
void AJointCharacterTest::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	cameraCalculations(DeltaTime);
	movementCalculations(DeltaTime);
	//hover(DeltaTime);
	//UE_LOG(LogTemp, Warning, TEXT("tick!"));


}



void AJointCharacterTest::cameraCalculations(float DeltaTime) 
{
	camera_axis->SetWorldLocation(rolling_body->GetCenterOfMass());

	if (fight_mode) {
		if (fight_mode_state == 1) 
		{
			//camera_axis->SetWorldRotation(FMath::Lerp<float>(axis_rot_before_fight, axis_target_rot, fight_t));
			camera_spring_arm->SetRelativeRotation(FMath::Lerp<float>(arm_rot_before_fight , arm_target_rot, fight_t));
			camera_spring_arm->TargetArmLength = FMath::Lerp<float>(arm_length_before_fight, target_arm_length, fight_t);
			fight_t += DeltaTime/ fight_mode_engage_time;

			if (fight_t > 1.0f)
			{
				fight_mode_state = 2;
				fight_t = 1.0f;

				//yaw
				//camera_axis->SetWorldRotation(axis_target_rot);
				//pitch
				camera_spring_arm->SetRelativeRotation(arm_target_rot);
				//arm length
				camera_spring_arm->TargetArmLength = target_arm_length;
				
			}
		}
		
		if (!camera_input.IsZero()) 
		{
			rolling_body->AddForce(FVector::UpVector * hover_height.adjustment * rolling_body->GetMass());
			FVector2D sword_direction = camera_input.GetSafeNormal();
			float sword_angle = FMath::Atan2(sword_direction.Y, sword_direction.X)*180.f / 3.14f - 90;
			
			sword_rotation.wanted = sword_angle;

			//the camera axis switches x and y relative to the movement axis
			FVector target_wep_dir = (camera_axis->GetForwardVector()*camera_input.Y + camera_axis->GetRightVector()* camera_input.X);
			FVector current_wep_dir = weapon_motor->GetForwardVector();
			current_wep_dir.Z = 0.0f;


			sword_rotation.error = FMath::Acos(FVector::DotProduct(target_wep_dir.GetSafeNormal(), current_wep_dir.GetSafeNormal()))*180.f / 3.14f;
			UE_LOG(LogTemp, Warning, TEXT("sword_rotation.error: %f"), sword_rotation.error);
			UE_LOG(LogTemp, Warning, TEXT("twm x: %f"), target_wep_dir.X);
			UE_LOG(LogTemp, Warning, TEXT("twm y: %f"), target_wep_dir.Y);
			UE_LOG(LogTemp, Warning, TEXT("twm z: %f \n"), target_wep_dir.Z);
			
			UE_LOG(LogTemp, Warning, TEXT("wm x: %f"), current_wep_dir.X);
			UE_LOG(LogTemp, Warning, TEXT("wm y: %f"), current_wep_dir.Y);
			UE_LOG(LogTemp, Warning, TEXT("wm z: %f\n"), current_wep_dir.Z);

			UE_LOG(LogTemp, Warning, TEXT("ca x: %f"), camera_axis->GetForwardVector().X);
			UE_LOG(LogTemp, Warning, TEXT("ca y: %f"), camera_axis->GetForwardVector().Y);
			UE_LOG(LogTemp, Warning, TEXT("ca z: %f\n"), camera_axis->GetForwardVector().Z);

			if (FVector::CrossProduct(target_wep_dir.GetSafeNormal(), current_wep_dir.GetSafeNormal()).Z < 0)
				sword_rotation.error = -sword_rotation.error;

			sword_rotation.integral = sword_rotation.integral + sword_rotation.error * DeltaTime;
			sword_rotation.derivative = (sword_rotation.error - sword_rotation.prev_err) / DeltaTime;

			sword_rotation.adjustment = 100000000000.0f * sword_rotation.error + 5.0f * sword_rotation.integral + 100000000.0f * sword_rotation.derivative;
			sword_rotation.prev_err = sword_rotation.error;

			//weapon_attachment->SetAngularOrientationTarget(FRotator(0.f, sword_angle, 0.f));
			weapon_motor->AddTorque(FVector(0.f, 0.f, -sword_rotation.adjustment));
			//weapon->AddTorque(FVector(0.f, 100000.f, 0.f));
			//weapon->AddTorque(FVector(100000.f, 0.f, 0.f));
			
		}
		


	}
	else {
		if (fight_mode_state == 3)
		{
			camera_spring_arm->TargetArmLength = FMath::Lerp<float>(arm_length_before_fight, target_arm_length, fight_t);
			camera_spring_arm->SetRelativeRotation(FMath::Lerp<float>(arm_rot_before_fight, arm_target_rot, fight_t));
			fight_t -= DeltaTime / fight_mode_engage_time;
			if (fight_t < 0.0f)
			{
				fight_mode_state = 0;
				camera_spring_arm->TargetArmLength = arm_length_before_fight;
				camera_spring_arm->SetRelativeRotation(arm_rot_before_fight);
			}
		}

		FRotator axis_rotation = camera_axis->GetComponentRotation();
		axis_rotation.Yaw += camera_input.X;
		camera_axis->SetWorldRotation(axis_rotation);
			
		FRotator arm_rotation = camera_spring_arm->GetComponentRotation();
		arm_rotation.Pitch = FMath::Clamp(arm_rotation.Pitch + camera_input.Y, -80.0f, 80.0f);
		camera_spring_arm->SetWorldRotation(arm_rotation);
			
		UE_LOG(LogTemp, Warning, TEXT("left x: %f\n"), movement_input.X);
		UE_LOG(LogTemp, Warning, TEXT("left y: %f"), movement_input.Y);
		UE_LOG(LogTemp, Warning, TEXT("right x: %f"), camera_input.X);
		UE_LOG(LogTemp, Warning, TEXT("right y: %f\n"), camera_input.Y);

	}
	
}

void AJointCharacterTest::movementCalculations(float DeltaTime)
{

	//if (!movement_input.IsZero() && rolling_body->GetComponentVelocity().Size() < target_speed)
	if (!movement_input.IsZero())
	{
		movement_input = movement_input.GetSafeNormal();

		FVector curr_vel = rolling_body->GetComponentVelocity();

		FVector target_vel = (camera_axis->GetForwardVector()*movement_input.X + camera_axis->GetRightVector()* movement_input.Y)*target_speed;

		FVector move_force = ((target_vel - curr_vel) )*rolling_body->GetMass();
		move_force.Z = 0;

		rolling_body->AddForce(move_force );

	}
	if (jumping) 
	{

		if (curr_jump_time == 0.f)
		{//the factor of 100 is because unreal apparently applies forces in kg*cm*s^(-2)
			jump_force = (((FMath::Sqrt(2.f*1000.f*jump_height) - rolling_body->GetComponentVelocity().Z) / jump_force_time) + 1000.f)*rolling_body->GetMass() * 100;
		}

		rolling_body->AddForce(FVector::UpVector*jump_force*DeltaTime);

		curr_jump_time += DeltaTime;
		if (curr_jump_time > jump_force_time) {
			jumping = false;
			curr_jump_time = 0.0f;
		}

	}
}

void AJointCharacterTest::hover(float DeltaTime)
{
	FCollisionQueryParams RV_TraceParams = FCollisionQueryParams(FName(TEXT("RV_Trace")), true, this);
	RV_TraceParams.bTraceComplex = true;
	RV_TraceParams.bTraceAsyncScene = true;
	RV_TraceParams.bReturnPhysicalMaterial = false;

	//Re-initialize hit info
	FHitResult rv_hit(ForceInit);

	FVector start = rolling_body->GetCenterOfMass();
	FVector end = start - FVector::UpVector*100.0f;

	//call GetWorld() from within an actor extending class
	GetWorld()->LineTraceSingleByChannel(
		rv_hit,        //result
		start,    //start
		end, //end
		ECC_WorldStatic, //collision channel
		RV_TraceParams
	);

	if (rv_hit.bBlockingHit) 
	{
		hover_height.error = hover_height.wanted - rv_hit.Distance;
		hover_height.integral = hover_height.integral + hover_height.error * DeltaTime;
		hover_height.derivative = (hover_height.error - hover_height.prev_err) / DeltaTime;

		hover_height.adjustment = 30.0f * hover_height.error + 5.0f * hover_height.integral + 10.0f * hover_height.derivative;
		hover_height.adjustment = FMath::Min(FMath::Max(0.0f, hover_height.adjustment), hover_height.max_adjustment);

		hover_height.prev_err = hover_height.error;

		rolling_body->AddForce(FVector::UpVector * hover_height.adjustment * rolling_body->GetMass());

	}
	else 
	{
		hover_height.prev_err = 0.0;
		hover_height.integral = 0.0;
	}

		//UE_LOG(LogTemp, Warning, TEXT("hit!"));
}


// Called to bind functionality to input
void AJointCharacterTest::SetupPlayerInputComponent(UInputComponent* PlayerInputComponent)
{
	Super::SetupPlayerInputComponent(PlayerInputComponent);

	//Hook up events for "ZoomIn"
	InputComponent->BindAction("FightMode", IE_Pressed, this, &AJointCharacterTest::fightModeOn);
	InputComponent->BindAction("FightMode", IE_Released, this, &AJointCharacterTest::fightModeOff);

	InputComponent->BindAction("ZoomIn", IE_Pressed, this, &AJointCharacterTest::zoomIn);
	InputComponent->BindAction("ZoomIn", IE_Released, this, &AJointCharacterTest::zoomOut);
	
	InputComponent->BindAction("Jump", IE_Pressed, this, &AJointCharacterTest::jump);

	//Hook up every-frame handling for our four axes
	InputComponent->BindAxis("MoveForward", this, &AJointCharacterTest::moveForward);
	InputComponent->BindAxis("MoveRight", this, &AJointCharacterTest::moveRight);
	InputComponent->BindAxis("CameraPitch", this, &AJointCharacterTest::pitchCamera);
	InputComponent->BindAxis("CameraYaw", this, &AJointCharacterTest::yawCamera);

}

//Input functions
void AJointCharacterTest::moveForward(float AxisValue)
{
	movement_input.X = FMath::Clamp<float>(AxisValue, -1.0f, 1.0f);
}

void AJointCharacterTest::moveRight(float AxisValue)
{
	movement_input.Y = FMath::Clamp<float>(AxisValue, -1.0f, 1.0f);
}

void AJointCharacterTest::jump()
{
	jumping = true;
	curr_jump_time = 0.0f;
}

void AJointCharacterTest::pitchCamera(float AxisValue)
{
	camera_input.Y = AxisValue;
}

void AJointCharacterTest::yawCamera(float AxisValue)
{
	camera_input.X = AxisValue;
}

void AJointCharacterTest::zoomIn()
{
	zooming = true;
}

void AJointCharacterTest::zoomOut()
{
	zooming = false;
}

void AJointCharacterTest::fightModeOn()
{
	fight_mode = true;
	fight_mode_state = 1;
	fight_t = 0.0f;

	axis_rot_before_fight = camera_axis->GetComponentRotation();
	//arm_rot_before_fight = camera_spring_arm->GetComponentRotation();
	arm_rot_before_fight = FRotator(camera_spring_arm->GetRelativeTransform().GetRotation());
	arm_length_before_fight = camera_spring_arm->TargetArmLength;
	//cam_before_fight = FVector(camera_axis->GetComponentRotation().Yaw, camera_spring_arm->GetComponentRotation().Pitch, camera_spring_arm->TargetArmLength);

	//FRotator axis_rotation = FRotator(0.f, 180.f, 0.f);
	////axis_rotation.Yaw = 180;
	//camera_axis->SetWorldRotation(axis_rotation);

	////Rotate our camera's pitch, but limit it so we're always looking downward

	//FRotator arm_rotation = FRotator(-50.f, 180.f, 0.f);
	////arm_rotation.Pitch = FMath::Clamp(arm_rotation.Pitch + camera_input.Y, -80.0f, 80.0f);
	//camera_spring_arm->SetWorldRotation(arm_rotation);

	//camera_spring_arm->TargetArmLength = 800.f;

}

void AJointCharacterTest::fightModeOff()
{
	fight_mode = false;
	fight_mode_state = 3;
}

void AJointCharacterTest::SetupJoints() 
{
	FConstrainComponentPropName bodyName;
	bodyName.ComponentName = FName("PhysicalCharacter");
	FConstrainComponentPropName weaponName;
	weaponName.ComponentName = FName("Weapon");

	//weapon_attachment->ComponentName1 = bodyName;
	//weapon_attachment->OverrideComponent1 = rolling_body;
	//weapon_attachment->ComponentName2 = weaponName;
	//weapon_attachment->OverrideComponent2 = weapon;
	//weapon_attachment->UpdateConstraintFrames();
	//weapon_attachment->InitComponentConstraint();

	//weapon_attachment->SetConstraintReferencePosition
	//weapon_attachment->SetConstraintReferenceFrame()

	weapon_axis_attachment->SetConstrainedComponents(rolling_body, NAME_None, weapon_axis, NAME_None);
	weapon_axis_attachment->SetLinearXLimit(ELinearConstraintMotion::LCM_Locked, 4.0f);
	weapon_axis_attachment->SetLinearYLimit(ELinearConstraintMotion::LCM_Locked, 4.0f);
	weapon_axis_attachment->SetLinearZLimit(ELinearConstraintMotion::LCM_Locked, 4.0f);
	weapon_axis_attachment->SetAngularSwing1Limit(EAngularConstraintMotion::ACM_Free, 0.0f);
	weapon_axis_attachment->SetAngularSwing2Limit(EAngularConstraintMotion::ACM_Free, 0.0f);
	weapon_axis_attachment->SetAngularTwistLimit(EAngularConstraintMotion::ACM_Free, 0.0f);
	weapon_axis_attachment->SetDisableCollision(true);

	weapon_motor_attachment->SetConstrainedComponents(weapon_axis, NAME_None, weapon_motor, NAME_None);
	weapon_motor_attachment->SetLinearXLimit(ELinearConstraintMotion::LCM_Locked, 0.0f);
	weapon_motor_attachment->SetLinearYLimit(ELinearConstraintMotion::LCM_Locked, 0.0f);
	weapon_motor_attachment->SetLinearZLimit(ELinearConstraintMotion::LCM_Locked, 0.0f);
	weapon_motor_attachment->SetAngularSwing1Limit(EAngularConstraintMotion::ACM_Free, 0.0f);
	weapon_motor_attachment->SetAngularSwing2Limit(EAngularConstraintMotion::ACM_Locked, 0.0f);
	weapon_motor_attachment->SetAngularTwistLimit(EAngularConstraintMotion::ACM_Locked, 0.0f);
	weapon_motor_attachment->SetDisableCollision(true);

	weapon_attachment->SetConstrainedComponents(weapon_motor, NAME_None, weapon, NAME_None);
	weapon_attachment->SetLinearXLimit(ELinearConstraintMotion::LCM_Locked, 0.0f);
	weapon_attachment->SetLinearYLimit(ELinearConstraintMotion::LCM_Locked, 0.0f);
	weapon_attachment->SetLinearZLimit(ELinearConstraintMotion::LCM_Locked, 0.0f);
	weapon_attachment->SetAngularSwing1Limit(EAngularConstraintMotion::ACM_Locked, 0.0f);
	weapon_attachment->SetAngularSwing2Limit(EAngularConstraintMotion::ACM_Limited, 90.0f);
	weapon_attachment->SetAngularTwistLimit(EAngularConstraintMotion::ACM_Locked, 0.0f);
	weapon_attachment->SetDisableCollision(true);

	//weapon_attachment->SetAngularDriveMode(EAngularDriveMode::TwistAndSwing);
	//weapon_attachment->SetAngularOrientationTarget(FRotator(0.f, 0.f, 0.f));
	//weapon_attachment->SetAngularDriveParams(10000.f, 0.f, 1000000.f);
	//weapon_attachment->SetOrientationDriveTwistAndSwing(true, true);
 }

