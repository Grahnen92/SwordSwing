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
	rolling_body->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);
	rolling_body->SetCollisionResponseToAllChannels(ECR_Block);
	rolling_body->SetNotifyRigidBodyCollision(true);
	

	// Weapon settings
	weapon_axis = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("WeaponAxis"));
	weapon_axis->SetupAttachment(RootComponent);
	weapon_axis->SetRelativeLocation(FVector(0.f, 0.f, 110.f));
	weapon_axis->SetRelativeScale3D(FVector(0.8f, 0.8f, 0.05f));

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
	weapon->SetRelativeLocation(FVector(0.f, 0.f, 112.f));
	weapon->SetRelativeScale3D(FVector(0.1f, 0.1f, 2.0f));

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
	hover_height.target = target_hover_height;
	hover_height.max_adjustment = 1000000;
	hover_height.P = 30.f;
	hover_height.I = 5;
	hover_height.D = 10.0f;
	hover_height.integral = 0.f;
	hover_height.prev_err = 0.f;

	sword_motor_pos.target = FVector::UpVector*60.f;
	sword_motor_pos.max_adjustment = FVector(100000.f, 100000.f, 100000.f);
	sword_motor_pos.P = FVector(10.f, 10.f, 10.f);
	sword_motor_pos.I = FVector(0.f, 0.f, 0.f);
	sword_motor_pos.D = FVector(5.f, 5.f, 5.f);
	sword_motor_pos.integral = FVector::ZeroVector;


	target_wep_dir = FVector::UpVector;
	prev_target_wep_dir = FVector::ForwardVector;
	sword_rotation.target = 0.0f;
	sword_rotation.max_adjustment = 1000000;
	sword_rotation.P = 4.f;
	sword_rotation.I = 0.f;
	sword_rotation.D = 0.8f;
	sword_rotation.integral =0.f;

	sword_rotation_speed.target = 0.0f;
	sword_rotation_speed.max_adjustment = 1000000;
	sword_rotation_speed.P = 0.f;
	sword_rotation_speed.I = 0.f;
	sword_rotation_speed.D = 0.0f;
	sword_rotation_speed.integral = 0.f;


	sword_incline.target = 0.0f;
	sword_incline.max_adjustment = 1000000;
	sword_incline.P = 1500;
	sword_incline.I = 0.0f;
	sword_incline.D = 180.0f;
	sword_incline.integral = 0.f;

	movement_velocity.max_adjustment = FVector2D(1000000.f, 1000000.f);
	movement_velocity.P = FVector2D(5.f, 5.f);
	movement_velocity.I = FVector2D(0.f, 0.f);
	movement_velocity.D = FVector2D(0.1f, 0.1f);
	movement_velocity.integral = FVector2D::ZeroVector;


	axis_target_rot = FRotator(0.f, 150.f, 0.f);
	arm_target_rot = FRotator(-65.f, 0.f, 0.f);
	target_arm_length = 1000.f;

	OnCalculateCustomHoverPhysics.BindUObject(this, &AJointCharacterTest::customHoverPhysics);
	rolling_body_bi = rolling_body->GetBodyInstance();
	

	weapon_axis_bi = weapon_axis->GetBodyInstance();

	weapon_motor_bi = weapon_motor->GetBodyInstance();
	OnCalculateCustomWeaponPhysics.BindUObject(this, &AJointCharacterTest::customWeaponPhysics);

	weapon_bi = weapon->GetBodyInstance();

	rolling_body->OnComponentHit.AddDynamic(this, &AJointCharacterTest::OnHit);
	weapon_axis->OnComponentHit.AddDynamic(this, &AJointCharacterTest::OnHit);
}

// Called every frame
void AJointCharacterTest::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	if (fight_mode) 
	{
		weapon_motor_bi->AddCustomPhysics(OnCalculateCustomWeaponPhysics);
		//swordCalculations(DeltaTime);
	}

	cameraCalculations(DeltaTime);
	movementCalculations(DeltaTime);
	//rolling_body_bi->AddCustomPhysics(OnCalculateCustomHoverPhysics);
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
		axis_rotation.Yaw += camera_input.X*2.0f;
		camera_axis->SetWorldRotation(axis_rotation);
			
		FRotator arm_rotation = camera_spring_arm->GetComponentRotation();
		arm_rotation.Pitch = FMath::Clamp(arm_rotation.Pitch + camera_input.Y*2.0f, -80.0f, 80.0f);
		camera_spring_arm->SetWorldRotation(arm_rotation);

	}
	
}
void AJointCharacterTest::swordCalculations(float DeltaTime)
{
	//calculations regarding the position of the weapon ----------------------------------------------------------
	FVector current_wep_dir = weapon_motor->GetForwardVector().GetSafeNormal();
	current_wep_dir.Z = 0.0f;

	FVector2D camera_input_norm = camera_input.GetSafeNormal(0.000000001f);
	float scaled_inverted_cam_input_size = FMath::Pow(1 - camera_input.Size(), 2.0f);
	float scaled_cam_input_size = FMath::Pow(camera_input.Size(), 2.0f);
	target_wep_dir = (camera_axis->GetForwardVector()*camera_input.Y + camera_axis->GetRightVector()* camera_input.X) + camera_axis->GetUpVector()*scaled_inverted_cam_input_size;
	target_wep_dir.Normalize();
	FVector target_wep_dir_xy = target_wep_dir - FVector::DotProduct(target_wep_dir, camera_axis->GetUpVector())*camera_axis->GetUpVector();
	FVector target_wep_dir_curr_wep_proj = target_wep_dir - FVector::DotProduct(target_wep_dir, weapon_motor->GetRightVector())*weapon_motor->GetRightVector();


	float current_wep_incline = FMath::Acos(FVector::DotProduct(weapon_motor->GetUpVector(), weapon->GetUpVector()))*180.f / PI;
	current_wep_incline = FMath::Fmod(current_wep_incline, 90.0f);
	//Position-------------------------------------------------------------------------------------------------

	sword_motor_pos.error = (weapon_axis->GetComponentLocation() + target_wep_dir * 60.f) - weapon_motor->GetComponentLocation();
	sword_motor_pos.integral = sword_motor_pos.integral + sword_motor_pos.error * DeltaTime;
	sword_motor_pos.derivative = (sword_motor_pos.error - sword_motor_pos.prev_err) / DeltaTime;

	sword_motor_pos.adjustment = sword_motor_pos.P * sword_motor_pos.error +
		sword_motor_pos.I * sword_motor_pos.integral +
		sword_motor_pos.D * sword_motor_pos.derivative;
	sword_motor_pos.prev_err = sword_motor_pos.error;

	weapon_motor->AddForce(sword_motor_pos.adjustment);
	//sword_motor_pos.error = weapon_axis->get
	
	//Rotation and direction-------------------------------------------------------------------------------------------------
	
	//Control weapon rotation -------------------------------------------------------------------------------------------------
	float target_ang_speed;
	if (target_wep_dir_xy.IsNearlyZero()) {
		//sword_rotation.error = 0.0f;
		target_wep_dir_xy = prev_target_wep_dir;
		was_standing_still = true;
		target_ang_speed = 0.f;
	}
	else
	{	
		if (camera_input.Size() > 0.5 && !was_standing_still)
		{
			target_ang_speed = FMath::Acos(FVector::DotProduct(target_wep_dir_xy.GetSafeNormal(), prev_target_wep_dir))*180.f / PI;
			if (FVector::CrossProduct(target_wep_dir_xy.GetSafeNormal(), prev_target_wep_dir).Z < 0.f)
				target_ang_speed = -target_ang_speed;
		}
		else 
		{
			target_ang_speed = 0.f;
		 }

		prev_target_wep_dir = target_wep_dir_xy.GetSafeNormal();
		was_standing_still = false;
	}
	//used to make the target be a little ahead of the actual target when the player is swinging quickly
	
	UE_LOG(LogTemp, Warning, TEXT("target_ang_speed: %f"), target_ang_speed);

	float tmp_forward_error = (FMath::Acos(FVector::DotProduct(target_wep_dir_xy.GetSafeNormal(), current_wep_dir.GetSafeNormal()))*180.f / PI) + target_ang_speed;
	float tmp_backwards_error = (FMath::Acos(FVector::DotProduct(target_wep_dir_xy.GetSafeNormal(), -current_wep_dir.GetSafeNormal()))*180.f / PI) + target_ang_speed;

	if (tmp_forward_error < tmp_backwards_error)
	{
		sword_rotation.error = tmp_forward_error;
		if (FVector::CrossProduct(target_wep_dir_xy.GetSafeNormal(), current_wep_dir.GetSafeNormal()).Z < 0)
			sword_rotation.error = -sword_rotation.error;
	}
	else
	{
		sword_rotation.error = tmp_backwards_error;
		if (FVector::CrossProduct(target_wep_dir_xy.GetSafeNormal(), -current_wep_dir.GetSafeNormal()).Z < 0)
			sword_rotation.error = -sword_rotation.error;
	}
	//sword_rotation.error = FMath::Acos(FVector::DotProduct(target_wep_dir_xy.GetSafeNormal(), current_wep_dir.GetSafeNormal()))*180.f / PI;
	

	sword_rotation.integral = sword_rotation.integral + sword_rotation.error * DeltaTime;
	sword_rotation.derivative = (sword_rotation.error - sword_rotation.prev_err) / DeltaTime;

	sword_rotation.adjustment = sword_rotation.P * sword_rotation.error +
								sword_rotation.I * sword_rotation.integral +
								sword_rotation.D * sword_rotation.derivative;
	sword_rotation.prev_err = sword_rotation.error;

	//// inertia calculations -------------------------------------------------
	//FVector wep_inertia = weapon->GetInertiaTensor();
	//FMatrix diagonal_inertia(FVector(wep_inertia.X, 0.f, 0.f),
	//	FVector(0.f, wep_inertia.Y, 0.f),
	//	FVector(0.f, 0.f, wep_inertia.Z),
	//	FVector(0.f, 0.f, 0.f));

	////float incline_cos = FMath::Cos(FMath::DegreesToRadians(FMath::Abs(weapon_incline)));
	///*FVector tip_distance = weapon_motor->GetComponentLocation() - (weapon->GetComponentLocation() + 100 * weapon->GetForwardVector());
	//tip_distance.Z = 0.f;*/
	//FVector com_d = FVector(-140.0f, 0.0f, 0.0f) ;
	////com_d.Z = 0.f;
	//FMatrix inertia_translator(FVector(0.f, -com_d.Z, com_d.Y),
	//	FVector(com_d.Z, 0.f, -com_d.X),
	//	FVector(-com_d.Y, com_d.X, 0.f),
	//	FVector(0.f, 0.f, 0.f));
	//inertia_translator = inertia_translator*inertia_translator;
	//for (int i = 0; i < 4; i++) 
	//{
	//	for(int j = 0; j < 4; j++)
	//	{
	//		inertia_translator.M[i][j] = inertia_translator.M[i][j] * (-weapon->GetMass());
	//	} 
	//}
	////inertia_translator.ApplyScale(-weapon->GetMass());

	//FRotationMatrix inertia_rotator(FRotator(weapon_incline, 0.f, 0.f));

	//FMatrix Tdiagonal_inertia = diagonal_inertia + inertia_translator;
	//Tdiagonal_inertia.M[3][3] = 1.f;


	//FMatrix Rdiagonal_inertia = inertia_rotator*Tdiagonal_inertia*inertia_rotator.GetTransposed();

	FVector rot_torque = weapon_motor->GetUpVector()*
		-sword_rotation.adjustment*
		FMath::Lerp((weapon_motor->GetMass()*FMath::Pow(40.0f, 2) / 2.0f), weapon->GetMass()*FMath::Pow(250.0f, 2) / 3.0f, FMath::Pow(current_wep_incline/90.f, 3));
	weapon_motor->AddTorque(rot_torque);

	//Control rotational speed --------------------------------------------------------------------------------------------------
	float tmp_rot_speed = weapon_motor->GetPhysicsAngularVelocity().Size();
	sword_rotation_speed.error = FMath::Pow((sword_rotation.error/180.f), 1.0f)*5000 - weapon_motor->GetPhysicsAngularVelocity().Size();


	sword_rotation_speed.integral = sword_rotation_speed.integral + sword_rotation_speed.error * DeltaTime;
	sword_rotation_speed.derivative = (sword_rotation_speed.error - sword_rotation_speed.prev_err) / DeltaTime;

	sword_rotation_speed.adjustment = sword_rotation_speed.P * sword_rotation_speed.error +
		sword_rotation_speed.I * sword_rotation_speed.integral +
		sword_rotation_speed.D * sword_rotation_speed.derivative;
	sword_rotation_speed.prev_err = sword_rotation_speed.error;
	weapon_motor->AddTorque(weapon_motor->GetUpVector()*sword_rotation_speed.adjustment);
	
	//Control weapon incline --------------------------------------------------------------------------------------------------
	
	if (!target_wep_dir_curr_wep_proj.IsNearlyZero())
	{
		sword_incline.error = /*90.f*FMath::Pow(1-camera_input.Size(),4)*/  -FMath::Acos(FVector::DotProduct(weapon->GetUpVector(), target_wep_dir_curr_wep_proj.GetSafeNormal()))*180.f / PI;
		if (FVector::Coincident(FVector::CrossProduct(weapon->GetUpVector(), target_wep_dir_curr_wep_proj.GetSafeNormal()), weapon->GetRightVector(), 0.00001f))
			sword_incline.error = -sword_incline.error;	

		if(weapon->GetUpVector().Z < 0.f && (weapon->GetForwardVector()*sword_incline.error).Z < 0.f)
			sword_incline.error = -sword_incline.error;
	}
	else {
			sword_incline.error = 0.0f;
	}
	
	//sword_incline.error = 90.f - weapon_incline;

	sword_incline.integral = sword_incline.integral + sword_incline.error * DeltaTime;
	sword_incline.derivative = (sword_incline.error - sword_incline.prev_err) / DeltaTime;

	sword_incline.adjustment = sword_incline.P * sword_incline.error +
							   sword_incline.I * sword_incline.integral +
							   sword_incline.D * sword_incline.derivative;
	sword_incline.prev_err = sword_incline.error;

	//weapon->AddTorque(weapon->GetRightVector()*sword_incline.adjustment);		
	weapon->AddForceAtLocation(weapon->GetForwardVector()*sword_incline.adjustment, weapon->GetComponentLocation() + 100 * weapon->GetUpVector());
	weapon->AddForceAtLocation(weapon->GetForwardVector()*-sword_incline.adjustment, weapon->GetComponentLocation() - 200 * weapon->GetUpVector());

	DrawDebugPoint(
		GetWorld(),
		weapon_motor->GetComponentLocation() + 100 * current_wep_dir,
		20,  					//size
		FColor(0, 255, 0),  //pink
		true,  				//persistent (never goes away)
		0.03 					//point leaves a trail on moving object
	);

	DrawDebugPoint(
		GetWorld(),
		weapon_motor->GetComponentLocation() + 100 * target_wep_dir,
		20,  					//size
		FColor(0, 255, 255),  //pink
		true,  				//persistent (never goes away)
		0.03 					//point leaves a trail on moving object
	);

	DrawDebugPoint(
		GetWorld(),
		weapon_motor->GetComponentLocation() + 100 * target_wep_dir_xy.GetSafeNormal(),
		20,  					//size
		FColor(255, 0, 0),  //pink
		true,  				//persistent (never goes away)
		0.03 					//point leaves a trail on moving object
	);
	
}

void AJointCharacterTest::movementCalculations(float DeltaTime)
{

	//Planar movement speed -------------------------------------------------------------------------------------

	//if (!movement_input.IsZero() && rolling_body->GetComponentVelocity().Size() < target_speed)
	//if (!movement_input.IsZero())
	if(true)
	{
		FVector2D movement_input_norm = movement_input.GetSafeNormal();

		FVector curr_vel = rolling_body->GetComponentVelocity();
		FVector2D curr_vel2D = FVector2D(curr_vel.X, curr_vel.Y);

		FVector target_vel = (camera_axis->GetForwardVector()*movement_input.X + camera_axis->GetRightVector()* movement_input.Y)*target_speed;
		FVector2D target_vel2D = FVector2D(target_vel.X, target_vel.Y);

		movement_velocity.error = target_vel2D - curr_vel2D;
		movement_velocity.integral = movement_velocity.integral + movement_velocity.error * DeltaTime;
		movement_velocity.derivative = (movement_velocity.error - movement_velocity.prev_err) / DeltaTime;

		movement_velocity.adjustment = movement_velocity.P * movement_velocity.error +
									   movement_velocity.I * movement_velocity.integral +
									   movement_velocity.D * movement_velocity.derivative;
		movement_velocity.prev_err = movement_velocity.error;

		//FVector move_force = ((target_vel - curr_vel) )*rolling_body->GetMass();
		//move_force.Z = 0;
		FVector move_force = FVector(movement_velocity.adjustment.X, movement_velocity.adjustment.Y, 0.f)*rolling_body->GetMass();
	
		rolling_body->AddForce(move_force );
	}
	//Jumping -------------------------------------------------------------------------------------

	if (jumping) 
	{

		if (curr_jump_time == 0.f)
		{//the factor of 100 is because unreal apparently applies forces in kg*cm*s^(-2)
			jump_force = (((FMath::Sqrt(2.f*1000.f*jump_height) - rolling_body->GetComponentVelocity().Z) / jump_force_time) + 1000.f)*(rolling_body->GetMass() + weapon->GetMass() + weapon_axis->GetMass() + weapon_motor->GetMass()) * 100;
		}

		rolling_body->AddForce(FVector::UpVector*jump_force*DeltaTime);

		curr_jump_time += DeltaTime;
		if (curr_jump_time > jump_force_time) {
			jumping = false;
			curr_jump_time = 0.0f;
		}

	}
}

void AJointCharacterTest::OnHit(UPrimitiveComponent* HitComp, AActor* OtherActor, UPrimitiveComponent* OtherComp, FVector NormalImpulse, const FHitResult& Hit)
{
	if (NormalImpulse.Size() > 200000) {
		UE_LOG(LogTemp, Warning, TEXT("hit: %s"), *NormalImpulse.ToString());
		UE_LOG(LogTemp, Warning, TEXT("ded"));
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
		hover_height.error = hover_height.target - rv_hit.Distance;
		hover_height.integral = hover_height.integral + hover_height.error * DeltaTime;
		hover_height.derivative = (hover_height.error - hover_height.prev_err) / DeltaTime;

		hover_height.adjustment = hover_height.P * hover_height.error +
			hover_height.I * hover_height.integral +
			hover_height.D * hover_height.derivative;
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
	if (fight_mode == false)
	{
		fight_mode = true;
		fight_mode_state = 1;
		fight_t = 0.0f;

		axis_rot_before_fight = camera_axis->GetComponentRotation();
		//arm_rot_before_fight = camera_spring_arm->GetComponentRotation();
		arm_rot_before_fight = FRotator(camera_spring_arm->GetRelativeTransform().GetRotation());
		arm_length_before_fight = camera_spring_arm->TargetArmLength;

		weapon_attachment->SetAngularSwing1Limit(EAngularConstraintMotion::ACM_Locked, 0.0f);
		weapon_attachment->SetAngularSwing2Limit(EAngularConstraintMotion::ACM_Limited, 90.f);
		weapon_attachment->ConstraintInstance.AngularRotationOffset = FRotator(0.f, 0.f, 0.f);
		weapon_attachment->SetAngularTwistLimit(EAngularConstraintMotion::ACM_Locked, 0.0f);

		weapon_motor_attachment->SetLinearXLimit(ELinearConstraintMotion::LCM_Limited, 150.0f);
		weapon_motor_attachment->SetLinearYLimit(ELinearConstraintMotion::LCM_Limited, 150.0f);
		weapon_motor_attachment->SetLinearZLimit(ELinearConstraintMotion::LCM_Limited, 150.0f);
		weapon_motor_attachment->SetAngularSwing1Limit(EAngularConstraintMotion::ACM_Free, 0.0f);
		weapon_motor_attachment->SetConstraintReferencePosition(EConstraintFrame::Frame2, FVector(0.f, 0.f, 41.6666f));


		weapon->SetEnableGravity(false);
	}
	else
	{
		fight_mode = false;
		fight_mode_state = 3;

		weapon_attachment->SetAngularSwing1Limit(EAngularConstraintMotion::ACM_Free, 0.0f);
		weapon_attachment->SetAngularSwing2Limit(EAngularConstraintMotion::ACM_Free, 90.f);
		weapon_attachment->SetAngularTwistLimit(EAngularConstraintMotion::ACM_Free, 0.0f);

		weapon_motor_attachment->SetLinearXLimit(ELinearConstraintMotion::LCM_Locked, 0.0f);
		weapon_motor_attachment->SetLinearYLimit(ELinearConstraintMotion::LCM_Locked, 0.0f);
		weapon_motor_attachment->SetLinearZLimit(ELinearConstraintMotion::LCM_Locked, 0.0f);
		weapon_motor_attachment->SetAngularSwing1Limit(EAngularConstraintMotion::ACM_Locked, 0.0f);
		weapon_motor_attachment->SetConstraintReferencePosition(EConstraintFrame::Frame2, FVector(-20.f, 0.f, -50.f));

		weapon->SetEnableGravity(true);
	}

	

}

void AJointCharacterTest::fightModeOff()
{
	/*fight_mode = false;
	fight_mode_state = 3;

	weapon_attachment->SetAngularSwing1Limit(EAngularConstraintMotion::ACM_Free, 0.0f);
	weapon_attachment->SetAngularSwing2Limit(EAngularConstraintMotion::ACM_Free, 90.f);
	weapon_attachment->SetAngularTwistLimit(EAngularConstraintMotion::ACM_Free, 0.0f);

	weapon_motor_attachment->SetLinearXLimit(ELinearConstraintMotion::LCM_Locked, 0.0f);
	weapon_motor_attachment->SetLinearYLimit(ELinearConstraintMotion::LCM_Locked, 0.0f);
	weapon_motor_attachment->SetLinearZLimit(ELinearConstraintMotion::LCM_Locked, 0.0f);
	weapon_motor_attachment->SetConstraintReferencePosition(EConstraintFrame::Frame2, FVector(-20.f, 0.f, -50.f));

	weapon->SetEnableGravity(true);*/

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
	weapon_motor_attachment->SetLinearXLimit(ELinearConstraintMotion::LCM_Limited, 0.0f);
	weapon_motor_attachment->SetLinearYLimit(ELinearConstraintMotion::LCM_Limited, 0.0f);
	weapon_motor_attachment->SetLinearZLimit(ELinearConstraintMotion::LCM_Limited, 0.0f);
	weapon_motor_attachment->SetAngularSwing1Limit(EAngularConstraintMotion::ACM_Free, 0.0f);
	weapon_motor_attachment->SetAngularSwing2Limit(EAngularConstraintMotion::ACM_Locked, 0.0f);
	weapon_motor_attachment->SetAngularTwistLimit(EAngularConstraintMotion::ACM_Locked, 0.0f);
	weapon_motor_attachment->SetDisableCollision(true);
	weapon_motor_attachment->SetAngularDriveMode(EAngularDriveMode::TwistAndSwing);
	weapon_motor_attachment->SetAngularVelocityDriveTwistAndSwing(true, true);
	weapon_motor_attachment->SetAngularDriveParams(0.f, 10000.f, 10000000000000000000000.f);


	weapon_attachment->SetConstrainedComponents(weapon_motor, NAME_None, weapon, NAME_None);
	weapon_attachment->SetLinearXLimit(ELinearConstraintMotion::LCM_Locked, 0.0f);
	weapon_attachment->SetLinearYLimit(ELinearConstraintMotion::LCM_Locked, 0.0f);
	weapon_attachment->SetLinearZLimit(ELinearConstraintMotion::LCM_Locked, 0.0f);
	weapon_attachment->SetAngularSwing1Limit(EAngularConstraintMotion::ACM_Locked, 0.0f);
	weapon_attachment->SetAngularSwing2Limit(EAngularConstraintMotion::ACM_Limited, 90.f);
	weapon_attachment->ConstraintInstance.AngularRotationOffset = FRotator(0.f, 0.f, 0.f);
	weapon_attachment->SetAngularTwistLimit(EAngularConstraintMotion::ACM_Locked, 0.0f);
	weapon_attachment->SetDisableCollision(true);
	weapon_attachment->SetAngularDriveMode(EAngularDriveMode::TwistAndSwing);
	weapon_attachment->SetAngularVelocityDriveTwistAndSwing(true, true);
	weapon_attachment->SetAngularDriveParams(0.f, 10000.f, 10000000000000000000000.f);
	

	weapon_motor->SetPhysicsMaxAngularVelocity(5000.f);
	weapon->SetPhysicsMaxAngularVelocity(5000.f);

	//weapon_attachment->SetAngularDriveMode(EAngularDriveMode::TwistAndSwing);
	//weapon_attachment->SetAngularOrientationTarget(FRotator(0.f, 0.f, 0.f));
	//weapon_attachment->SetAngularDriveParams(10000.f, 0.f, 1000000.f);
	//weapon_attachment->SetOrientationDriveTwistAndSwing(true, true);
 }

void AJointCharacterTest::customHoverPhysics(float DeltaTime, FBodyInstance* BodyInstance)
{
	FCollisionQueryParams RV_TraceParams = FCollisionQueryParams(FName(TEXT("RV_Trace")), true, this);
	RV_TraceParams.bTraceComplex = true;
	RV_TraceParams.bTraceAsyncScene = true;
	RV_TraceParams.bReturnPhysicalMaterial = false;

	//Re-initialize hit info
	FHitResult rv_hit(ForceInit);

	FVector start = BodyInstance->GetCOMPosition();
	FVector end = start - FVector::UpVector*hover_height.target*2.f;

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
		hover_height.error = hover_height.target - rv_hit.Distance;
		hover_height.integral = hover_height.integral + hover_height.error * DeltaTime;
		hover_height.derivative = (hover_height.error - hover_height.prev_err) / DeltaTime;

		hover_height.adjustment = hover_height.P * hover_height.error +
			hover_height.I * hover_height.integral +
			hover_height.D * hover_height.derivative;
		hover_height.adjustment = FMath::Min(FMath::Max(0.0f, hover_height.adjustment), hover_height.max_adjustment);

		hover_height.prev_err = hover_height.error;

		BodyInstance->AddForce(FVector::UpVector * hover_height.adjustment * rolling_body->GetMass());

	}
	else
	{
		hover_height.prev_err = 0.0;
		hover_height.integral = 0.0;
	}
}

void AJointCharacterTest::customWeaponPhysics(float DeltaTime, FBodyInstance* BodyInstance)
{
	//calculations regarding the position of the weapon ----------------------------------------------------------
	FVector wa_pos = weapon_axis_bi->GetUnrealWorldTransform().GetLocation();
	
	FVector wm_pos = weapon_motor_bi->GetUnrealWorldTransform().GetLocation();
	FVector wm_forward = weapon_motor_bi->GetUnrealWorldTransform().GetUnitAxis(EAxis::X);
	FVector wm_right = weapon_motor_bi->GetUnrealWorldTransform().GetUnitAxis(EAxis::Y);
	FVector wm_up = weapon_motor_bi->GetUnrealWorldTransform().GetUnitAxis(EAxis::Z);

	FVector w_pos = weapon_bi->GetUnrealWorldTransform().GetLocation();
	FVector w_forward = weapon_bi->GetUnrealWorldTransform().GetUnitAxis(EAxis::X);
	FVector w_right = weapon_bi->GetUnrealWorldTransform().GetUnitAxis(EAxis::Y);
	FVector w_up = weapon_bi->GetUnrealWorldTransform().GetUnitAxis(EAxis::Z);
	
	FVector current_wep_dir = wm_forward;
	current_wep_dir.Z = 0.0f;

	FVector2D camera_input_norm = camera_input.GetSafeNormal(0.000000001f);
	float scaled_inverted_cam_input_size = FMath::Pow(1 - camera_input.Size(), 2.0f);
	float scaled_cam_input_size = FMath::Pow(camera_input.Size(), 2.0f);
	target_wep_dir = (camera_axis->GetForwardVector()*camera_input.Y + camera_axis->GetRightVector()* camera_input.X) + camera_axis->GetUpVector()*scaled_inverted_cam_input_size;
	target_wep_dir.Normalize();
	//UE_LOG(LogTemp, Warning, TEXT("target dir:  %s"), target_wep_dir.ToString());
	//UE_LOG(LogTemp, Warning, TEXT("target dir: %s"), *target_wep_dir.ToString());
	FVector target_wep_dir_xy = target_wep_dir - FVector::DotProduct(target_wep_dir, camera_axis->GetUpVector())*camera_axis->GetUpVector();
	FVector target_wep_dir_curr_wep_proj = target_wep_dir - FVector::DotProduct(target_wep_dir, wm_right)*wm_right;


	float current_wep_incline = FMath::Acos(FVector::DotProduct(wm_up, w_up))*180.f / PI;
	current_wep_incline = FMath::Fmod(current_wep_incline, 90.0f);
	//Position-------------------------------------------------------------------------------------------------

	sword_motor_pos.error = (wa_pos + target_wep_dir * 70.f) - wm_pos;
	sword_motor_pos.integral = sword_motor_pos.integral + sword_motor_pos.error * DeltaTime;
	sword_motor_pos.derivative = (sword_motor_pos.error - sword_motor_pos.prev_err) / DeltaTime;

	sword_motor_pos.adjustment = sword_motor_pos.P * sword_motor_pos.error +
		sword_motor_pos.I * sword_motor_pos.integral +
		sword_motor_pos.D * sword_motor_pos.derivative;
	sword_motor_pos.prev_err = sword_motor_pos.error;

	weapon_motor_bi->AddImpulse(sword_motor_pos.adjustment, false);

	//Rotation and direction-------------------------------------------------------------------------------------------------

	//Control weapon rotation -------------------------------------------------------------------------------------------------
	//used to make the target be a little ahead of the actual target when the player is swinging quickly
	float target_ang_speed;
	if (target_wep_dir_xy.IsNearlyZero()) {
		//sword_rotation.error = 0.0f;
		target_wep_dir_xy = prev_target_wep_dir;
		was_standing_still = true;
		target_ang_speed = 0.f;
	}
	else
	{
		if (camera_input.Size() > 0.5 && !was_standing_still)
		{
			target_ang_speed = FMath::Acos(FVector::DotProduct(target_wep_dir_xy.GetSafeNormal(), prev_target_wep_dir))*180.f / PI;
			if (FVector::CrossProduct(target_wep_dir_xy.GetSafeNormal(), prev_target_wep_dir).Z < 0.f)
				target_ang_speed = -target_ang_speed;
		}
		else
		{
			target_ang_speed = 0.f;
		}

		prev_target_wep_dir = target_wep_dir_xy.GetSafeNormal();
		was_standing_still = false;
	}

	//UE_LOG(LogTemp, Warning, TEXT("target_ang_speed: %f"), target_ang_speed);

	float tmp_forward_error = (FMath::Acos(FVector::DotProduct(target_wep_dir_xy.GetSafeNormal(), current_wep_dir))*180.f / PI);// +target_ang_speed;
	float tmp_backwards_error = (FMath::Acos(FVector::DotProduct(target_wep_dir_xy.GetSafeNormal(), -current_wep_dir))*180.f / PI);// +target_ang_speed;

	if (tmp_forward_error < tmp_backwards_error)
	{
		sword_rotation.error = tmp_forward_error;
		if (FVector::CrossProduct(target_wep_dir_xy.GetSafeNormal(), current_wep_dir).Z < 0)
			sword_rotation.error = -sword_rotation.error;
	}
	else
	{
		sword_rotation.error = tmp_backwards_error;
		if (FVector::CrossProduct(target_wep_dir_xy.GetSafeNormal(), -current_wep_dir).Z < 0)
			sword_rotation.error = -sword_rotation.error;
	}
	//UE_LOG(LogTemp, Warning, TEXT("sword_rotation.error: %f"), sword_rotation.error);

	sword_rotation.integral = sword_rotation.integral + sword_rotation.error * DeltaTime;
	sword_rotation.derivative = (sword_rotation.error - sword_rotation.prev_err) / DeltaTime;

	sword_rotation.adjustment = sword_rotation.P * sword_rotation.error +
		sword_rotation.I * sword_rotation.integral +
		sword_rotation.D * sword_rotation.derivative;
	sword_rotation.prev_err = sword_rotation.error;

	FVector rot_torque = wm_up*
		-sword_rotation.adjustment*
		FMath::Lerp((weapon_motor->GetMass()*FMath::Pow(40.0f, 2) / 2.0f), weapon->GetMass()*FMath::Pow(250.0f, 2) / 3.0f, FMath::Pow(current_wep_incline / 90.f, 3));
	weapon_motor_bi->AddTorque(rot_torque, false);

	//Control rotational speed --------------------------------------------------------------------------------------------------
	sword_rotation_speed.error = FMath::Pow((sword_rotation.error / 180.f), 1.0f) * 5000 - weapon_motor_bi->GetUnrealWorldAngularVelocity().Size();


	sword_rotation_speed.integral = sword_rotation_speed.integral + sword_rotation_speed.error * DeltaTime;
	sword_rotation_speed.derivative = (sword_rotation_speed.error - sword_rotation_speed.prev_err) / DeltaTime;

	sword_rotation_speed.adjustment = sword_rotation_speed.P * sword_rotation_speed.error +
		sword_rotation_speed.I * sword_rotation_speed.integral +
		sword_rotation_speed.D * sword_rotation_speed.derivative;
	sword_rotation_speed.prev_err = sword_rotation_speed.error;
	//weapon_motor_bi->AddTorque(-wm_up*sword_rotation_speed.adjustment, false);

	//Control weapon incline --------------------------------------------------------------------------------------------------

	if (!target_wep_dir_curr_wep_proj.IsNearlyZero())
	{
		sword_incline.error = /*90.f*FMath::Pow(1-camera_input.Size(),4)*/  -FMath::Acos(FVector::DotProduct(w_up, target_wep_dir_curr_wep_proj.GetSafeNormal()))*180.f / PI;
		if (FVector::Coincident(FVector::CrossProduct(w_up, target_wep_dir_curr_wep_proj.GetSafeNormal()), w_right, 0.00001f))
			sword_incline.error = -sword_incline.error;

		if (w_up.Z < 0.f && (w_forward*sword_incline.error).Z < 0.f)
			sword_incline.error = -sword_incline.error;
	}
	else {
		sword_incline.error = 0.0f;
	}

	sword_incline.integral = sword_incline.integral + sword_incline.error * DeltaTime;
	sword_incline.derivative = (sword_incline.error - sword_incline.prev_err) / DeltaTime;

	sword_incline.adjustment = sword_incline.P * sword_incline.error +
		sword_incline.I * sword_incline.integral +
		sword_incline.D * sword_incline.derivative;
	sword_incline.prev_err = sword_incline.error;

	//weapon->AddTorque(weapon->GetRightVector()*sword_incline.adjustment);	
	weapon_bi->AddForceAtPosition(w_forward*sword_incline.adjustment, w_pos + 100 * w_up, false);
	weapon_bi->AddForceAtPosition(w_forward*-sword_incline.adjustment, w_pos - 200 * w_up, false);

	DrawDebugPoint(
		GetWorld(),
		wm_pos + 100 * current_wep_dir,
		20,  					//size
		FColor(0, 255, 0),  //pink
		true,  				//persistent (never goes away)
		0.03 					//point leaves a trail on moving object
	);

	DrawDebugPoint(
		GetWorld(),
		wm_pos + 100 * target_wep_dir,
		20,  					//size
		FColor(0, 255, 255),  //pink
		true,  				//persistent (never goes away)
		0.03 					//point leaves a trail on moving object
	);

	DrawDebugPoint(
		GetWorld(),
		wm_pos + 100 * target_wep_dir_xy.GetSafeNormal(),
		20,  					//size
		FColor(255, 0, 0),  //pink
		true,  				//persistent (never goes away)
		0.03 					//point leaves a trail on moving object
	);

}

void AJointCharacterTest::weaponRotationPhysics(float DeltaTime, FBodyInstance* BodyInstance)
{
}

void AJointCharacterTest::weaponInclinePhysics(float DeltaTime, FBodyInstance* BodyInstance)
{
	
}

void AJointCharacterTest::weaponPositionPhysics(float DeltaTime, FBodyInstance* BodyInstance)
{
}