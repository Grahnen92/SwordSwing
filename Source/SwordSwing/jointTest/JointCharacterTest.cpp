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

	target_wep_dir = FVector::UpVector;
	wd.target = FVector::ZeroVector;
	wd.max_adjustment = FVector(1000000.f, 1000000.f, 1000000.f);
	wd.P = FVector(0.f, 5.0f, 0.f);
	wd.I = FVector(0.f, 0.f, 0.f);
	wd.D = FVector(0.f, 0.000000f, 0.f);
	wd.integral = FVector::ZeroVector;

	ws.target = 0.f;
	ws.max_adjustment = 1000000.f, 1000000.f;
	ws.P =1000.1f;
	ws.I = 0.f;
	ws.D = 100.0f;
	ws.integral = 0.f;


	movement_velocity.max_adjustment = FVector2D(1000000.f, 1000000.f);
	movement_velocity.P = FVector2D(5.f, 5.f);
	movement_velocity.I = FVector2D(0.f, 0.f);
	movement_velocity.D = FVector2D(0.1f, 0.1f);
	movement_velocity.integral = FVector2D::ZeroVector;


	axis_target_rot = FRotator(0.f, 150.f, 0.f);
	arm_target_rot = FRotator(-65.f, 0.f, 0.f);
	target_arm_length = 1000.f;

	FVector tit = weapon->GetBodyInstance()->GetBodyInertiaTensor();
	wep_inertia.SetAxis(0, FVector(tit.X, 0.f, 0.f));
	wep_inertia.SetAxis(1, FVector(0.f, tit.Y, 0.f));
	wep_inertia.SetAxis(2, FVector(0.f, 0.f, tit.Z));
	FVector d = FVector(0.f, 0.f, 100.f);
	FMatrix tmp_d;
	tmp_d.SetAxis(0, FVector(0.f, d.Z, -d.Y));
	tmp_d.SetAxis(1, FVector(-d.Z, 0.f, d.X));
	tmp_d.SetAxis(2, FVector(d.Y, d.X, 0.f));
	wep_inertia = wep_inertia + (tmp_d*tmp_d)*(-weapon->GetMass());
	FVector::DotProduct(wep_inertia.TransformVector(FVector::UpVector),FVector::UpVector);
}

// Called every frame
void AJointCharacterTest::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	cameraCalculations(DeltaTime);
	movementCalculations(DeltaTime);
	//hover(DeltaTime);
	//UE_LOG(LogTemp, Warning, TEXT("tick!"));

	
	/*UE_LOG(LogTemp, Warning, TEXT("inerta: %s"), *weapon->GetBodyInstance()->GetMassSpaceToWorldSpace().TransformVector(weapon->GetBodyInstance()->GetBodyInertiaTensor()).ToString());
	UE_LOG(LogTemp, Warning, TEXT("inerta: %f"), (1.f /12.f)*weapon->GetMass()*(200.f *200.f + 10.f *10.f));
	UE_LOG(LogTemp, Warning, TEXT("inerta: %f"), (1.f / 12.f)*weapon->GetMass()*(10.f *10.f + 10.f *10.f));
	UE_LOG(LogTemp, Warning, TEXT("inerta: %f"), (1.f / 12.f)*weapon->GetMass()*(10.f *10.f + 200.f *200.f));
	*/
	FVector tmp_u = FVector(0.f, 0.f, 1.f);
	FVector tmp_r = FVector(0.f, 1.f, 0.f);
	FVector tmp_f = FVector(1.f, 0.f, 0.f);
	FVector tmp_u_f = (tmp_u + tmp_f).GetSafeNormal();
	FVector tmp_r_f = (tmp_r + tmp_f).GetSafeNormal();
	FVector tmp_u_r = (tmp_u + tmp_r).GetSafeNormal();
	/*UE_LOG(LogTemp, Warning, TEXT("inerta up: %f"), FVector::DotProduct(wep_inertia.TransformVector(tmp_u), tmp_u));
	UE_LOG(LogTemp, Warning, TEXT("inerta right: %f"), FVector::DotProduct(wep_inertia.TransformVector(tmp_r), tmp_r));
	UE_LOG(LogTemp, Warning, TEXT("inerta forward: %f"), FVector::DotProduct(wep_inertia.TransformVector(tmp_f), tmp_f));
	UE_LOG(LogTemp, Warning, TEXT("inerta uf: %f"), FVector::DotProduct(wep_inertia.TransformVector(tmp_u_f), tmp_u_f));
	UE_LOG(LogTemp, Warning, TEXT("inerta rf: %f"), FVector::DotProduct(wep_inertia.TransformVector(tmp_r_f), tmp_r_f));
	UE_LOG(LogTemp, Warning, TEXT("inerta ur: %f"), FVector::DotProduct(wep_inertia.TransformVector(tmp_u_r), tmp_u_r));*/
	//UE_LOG(LogTemp, Warning, TEXT("motorcom: %s"), *weapon_motor->GetBodyInstance()->GetCOMPosition().ToString());



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
		
		//calculations regarding the position of the weapon ----------------------------------------------------------
		FVector current_wep_dir = weapon_motor->GetUpVector();
		//current_wep_dir.Z = 0.0f;
		FVector2D camera_input_norm = camera_input.GetSafeNormal(0.000000001f);
		target_wep_dir = (camera_axis->GetForwardVector()*camera_input.Y + camera_axis->GetRightVector()* camera_input.X + camera_axis->GetUpVector()*(1 - camera_input.Size()));
		//target_wep_dir = (camera_axis->GetForwardVector()*camera_input.Y + camera_axis->GetRightVector()*(1 - camera_input.Size()) + camera_axis->GetUpVector()* camera_input.X);

		target_wep_dir.Normalize();
		wd.error.X = FMath::Acos(FVector::DotProduct(target_wep_dir.GetSafeNormal(), current_wep_dir))*180.f / PI;
		FVector target_rot_axis = FVector::CrossProduct(target_wep_dir.GetSafeNormal(), current_wep_dir).GetSafeNormal();

		FVector current_rot_vel = weapon_motor->GetPhysicsAngularVelocity();
		//FVector current_rot_vel_proj_on_target_dir = (FVector::DotProduct(current_rot_vel, target_wep_dir)/target_wep_dir.Size())*target_wep_dir.GetSafeNormal();
		//FVector current_rot_vel_proj_on_current_dir = (FVector::DotProduct(current_rot_vel, current_wep_dir) / current_wep_dir.Size())*current_wep_dir.GetSafeNormal();
		//wd.error.Y = FVector::DotProduct(current_rot_vel.GetSafeNormal(), target_rot_axis);
		wd.error.Y = -current_rot_vel.Size();

		wd.integral = wd.integral + wd.error * DeltaTime;
		wd.derivative = (wd.error - wd.prev_err) / DeltaTime;

		wd.adjustment = wd.P * wd.error +
						wd.I * wd.integral +
						wd.D * wd.derivative;
		wd.prev_err = wd.error;
		weapon_motor->AddTorque(target_rot_axis*
									-wd.adjustment.X*FVector::DotProduct(wep_inertia.TransformVector(FVector::ForwardVector), FVector::ForwardVector));
		float tmp_angle = FMath::Acos(FVector::DotProduct(current_rot_vel.GetSafeNormal(), current_wep_dir))*180.f / PI;
		float current_inertia;
		if (tmp_angle > 3.f) 
		{
			FVector inertia_offset = FVector::UpVector.RotateAngleAxis(tmp_angle,
				FVector::CrossProduct(current_rot_vel.GetSafeNormal(), current_wep_dir).GetSafeNormal());
			current_inertia = FVector::DotProduct(wep_inertia.TransformVector(inertia_offset), inertia_offset);
		}
		else
		{
			current_inertia = FVector::DotProduct(wep_inertia.TransformVector(FVector::UpVector), FVector::UpVector);
		}
		
		UE_LOG(LogTemp, Warning, TEXT("current_inertia: %f"), current_inertia);
		weapon_motor->AddAngularImpulse(wep_inertia.TransformVector(current_rot_vel.GetSafeNormal()*
			wd.adjustment.Y));
		
		
		/*FMatrix tmp_rot_mat;
		tmp_rot_mat.SetIdentity();
		FVector tmp_cross = FVector::CrossProduct(FVector::UpVector, target_wep_dir);
		FMatrix tmp_cross_mat;
		tmp_cross_mat.SetAxis(0, FVector(0.f, tmp_cross.Z, -tmp_cross.Y));
		tmp_cross_mat.SetAxis(1, FVector( -tmp_cross.Z, 0.f,  tmp_cross.X));
		tmp_cross_mat.SetAxis(2, FVector(tmp_cross.Y, -tmp_cross.X, 0.f));
		tmp_rot_mat = tmp_rot_mat + tmp_cross_mat + tmp_cross_mat*tmp_cross_mat*(1 / (1 + FVector::DotProduct(FVector::UpVector, target_wep_dir)));
		weapon_motor->SetWorldRotation(tmp_rot_mat.Rotator());
		weapon_motor->SetWorldLocation(weapon_axis->GetCenterOfMass());
		UE_LOG(LogTemp, Warning, TEXT("motorcom: %s"), *target_wep_dir.Rotation().ToString());
		UE_LOG(LogTemp, Warning, TEXT("motorcom: %s"), *camera_input.ToString());*/


		//Draw Debug points ---------------------------------------------------------------------------------------------------
		DrawDebugPoint(
			GetWorld(),
			weapon_motor->GetComponentLocation() + 100* current_wep_dir,
			20,  					//size
			FColor(255, 0, 0),  //pink
			true,  				//persistent (never goes away)
			0.03 					//point leaves a trail on moving object
		);

		DrawDebugPoint(
			GetWorld(),
			weapon_motor->GetComponentLocation() + 100 * target_wep_dir,
			20,  					//size
			FColor(0, 255, 0),  //pink
			true,  				//persistent (never goes away)
			0.03 					//point leaves a trail on moving object
		);

		DrawDebugLine(
			GetWorld(),
			weapon_motor->GetComponentLocation()  + 100 * current_wep_dir,
			weapon_motor->GetComponentLocation()  + 100 * current_wep_dir.GetSafeNormal() + target_rot_axis*100, 					//size
			FColor(0, 0, 255),  //pink
			true,  				//persistent (never goes away)
			0.01, 					//point leaves a trail on moving object
			10,
			5.f
		);
		DrawDebugLine(
			GetWorld(),
			weapon_motor->GetComponentLocation() + FVector::RightVector*40.f,
			weapon_motor->GetComponentLocation() + FVector::RightVector*40.f + current_rot_vel.GetSafeNormal() * 100, 					//size
			FColor(255, 0, 0),  //pink
			true,  				//persistent (never goes away)
			0.01, 					//point leaves a trail on moving object
			10,
			5.f
		);
		
		
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

	}
	
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
	
	weapon->SetRelativeLocation(FVector(0.f, 0.f, 155.f));
	weapon->SetRelativeScale3D(FVector(0.1f, 0.1f, 2.0f));
	weapon->SetupAttachment(RootComponent);
	weapon_attachment = CreateDefaultSubobject<UPhysicsConstraintComponent>(TEXT("WeaponAttachment"));
	weapon_attachment->SetupAttachment(weapon_motor);

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
	weapon_motor_attachment->SetAngularSwing1Limit(EAngularConstraintMotion::ACM_Free, 90.0f);
	weapon_motor_attachment->SetAngularSwing2Limit(EAngularConstraintMotion::ACM_Limited, 90.0f);
	weapon_motor_attachment->SetAngularTwistLimit(EAngularConstraintMotion::ACM_Limited, 90.0f);
	weapon_motor_attachment->SetDisableCollision(true);

	weapon_attachment->SetConstrainedComponents(weapon_motor, NAME_None, weapon, NAME_None);
	weapon_attachment->SetLinearXLimit(ELinearConstraintMotion::LCM_Locked, 0.0f);
	weapon_attachment->SetLinearYLimit(ELinearConstraintMotion::LCM_Locked, 0.0f);
	weapon_attachment->SetLinearZLimit(ELinearConstraintMotion::LCM_Locked, 0.0f);
	weapon_attachment->SetAngularSwing1Limit(EAngularConstraintMotion::ACM_Locked, 0.0f);
	weapon_attachment->SetAngularSwing2Limit(EAngularConstraintMotion::ACM_Locked, 45.0f);
	weapon_attachment->ConstraintInstance.AngularRotationOffset = FRotator(0.f, -45.f, 0.f);
	weapon_attachment->SetAngularTwistLimit(EAngularConstraintMotion::ACM_Locked, 0.0f);
	weapon_attachment->SetDisableCollision(true);

	//weapon_attachment->SetAngularDriveMode(EAngularDriveMode::TwistAndSwing);
	//weapon_attachment->SetAngularOrientationTarget(FRotator(0.f, 0.f, 0.f));
	//weapon_attachment->SetAngularDriveParams(10000.f, 0.f, 1000000.f);
	//weapon_attachment->SetOrientationDriveTwistAndSwing(true, true);
 }

