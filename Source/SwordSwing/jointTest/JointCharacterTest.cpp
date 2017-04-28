// Fill out your copyright notice in the Description page of Project Settings.

#include "SwordSwing.h"
#include "AudioDevice.h"
#include "FightMode.h"
#include "ActiveSound.h"
#include "JointCharacterTest.h"



// Sets default values
AJointCharacterTest::AJointCharacterTest()
{
 	// Set this pawn to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
	RootComponent = CreateDefaultSubobject<USceneComponent>(TEXT("RootComponent"));

	//body settings
	
	initLegs();
	initUpperBody();
	initWeapon();

	initCamera();

	initUpperBodyJoints();
	initWeaponJoints();
	initLegJoints();


	//AutoPossessPlayer = EAutoReceiveInput::Player0;
}

// Called when the game starts or when spawned
void AJointCharacterTest::BeginPlay()
{
	Super::BeginPlay();
	
	initPIDs();
	initCustomPhysics();

	camera->FieldOfView = 50.f;

	
}

// Called every frame
void AJointCharacterTest::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
	
	cameraCalculations(DeltaTime);

	//DrawDebugPoint(
	//	GetWorld(),
	//	grip_bi->GetCOMPosition(),
	//	10,  					//size
	//	FColor(255, 0, 255),  //pink
	//	false,  				//persistent (never goes away)
	//	0.03 					//point leaves a trail on moving object
	//);

	//DrawDebugLine(
	//	GetWorld(),
	//	grip->GetComponentLocation(),
	//	grip->GetComponentLocation() - 200 * FVector::UpVector, 					//size
	//	FColor(255, 0, 0),  //pink
	//	true,  				//persistent (never goes away)
	//	0.03, 					//point leaves a trail on moving object
	//	10,
	//	5.f
	//);



	if (alive)
	{
		camera_axis->SetWorldLocation(torso->GetCenterOfMass()+ FVector::UpVector*40.f);
		//grip_axis->SetWorldLocation(torso->GetCenterOfMass() + FVector(0.f, 0.f, -15.f));
		//grip_axis_bi->SetBodyTransform(FTransform(torso_bi->GetCOMPosition() + FVector(0.f, 0.f, -15.f)), true);
		
		if (can_swing)
		{
			//if (fight_mode)
			{
				grip_bi->AddCustomPhysics(OnCalculateControlGripPhysics);
			}
		}


		torso_bi->AddCustomPhysics(OnCalculateCustomHoverPhysics);

		if (can_move)
		{
			movementCalculations(DeltaTime);
			torso_bi->AddCustomPhysics(OnCalculateCustomStabilizerPhysics);
			pelvis_bi->AddCustomPhysics(OnCalculateCustomWalkingPhysics);
		}
		

		
		
	}
}


void AJointCharacterTest::cameraCalculations(float DeltaTime) 
{
	if (fight_mode) {
		if (fight_mode_state == 1) 
		{
			//camera_spring_arm->SetRelativeRotation(FMath::Lerp<float>(arm_rot_before_fight , arm_target_rot, fight_t));
			camera_spring_arm->TargetArmLength = FMath::Lerp<float>(arm_length_before_fight, target_arm_length, fight_t);
			fight_t += DeltaTime/ fight_mode_engage_time;

			if (fight_t > 1.0f)
			{
				fight_mode_state = 2;
				fight_t = 1.0f;

				//camera_spring_arm->SetRelativeRotation(arm_target_rot);
				camera_spring_arm->TargetArmLength = target_arm_length;
				
			}
		}

		if (locked_target)
		{
			/*FVector lockon_direction = lock_on_target->GetComponentLocation() - camera_axis->GetComponentLocation();
			lockon_direction.Z = 0.f;
			FRotator look_rot = FRotationMatrix::MakeFromX(lockon_direction).Rotator();
			camera_axis->SetWorldRotation(look_rot);*/
			controlCameraDirection(DeltaTime);
		}
		
	}
	else {
		if (fight_mode_state == 3)
		{
			camera_spring_arm->TargetArmLength = FMath::Lerp<float>(arm_length_before_fight, target_arm_length, fight_t);
			//camera_spring_arm->SetRelativeRotation(FMath::Lerp<float>(arm_rot_before_fight, arm_target_rot, fight_t));
			fight_t -= DeltaTime / fight_mode_engage_time;
			if (fight_t < 0.0f)
			{
				fight_mode_state = 0;
				camera_spring_arm->TargetArmLength = arm_length_before_fight;
			//	camera_spring_arm->SetRelativeRotation(arm_rot_before_fight);
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

void AJointCharacterTest::movementCalculations(float DeltaTime)
{

	//Planar movement speed -------------------------------------------------------------------------------------

	if(!movement_input.IsZero())
		target_direction = (camera_axis->GetForwardVector()*movement_input.X + camera_axis->GetRightVector()* movement_input.Y).GetSafeNormal();

	//if (!movement_input.IsZero() && rolling_body->GetComponentVelocity().Size() < target_speed)
	//if (!movement_input.IsZero())
	if(true)
	{
		FVector2D movement_input_norm = movement_input.GetSafeNormal();

		FVector curr_vel = torso->GetComponentVelocity();
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
		FVector move_force = FVector(movement_velocity.adjustment.X, movement_velocity.adjustment.Y, 0.f)*torso->GetMass();
	
		torso->AddForce(move_force );
	}
	//Jumping -------------------------------------------------------------------------------------

	if (jumping) 
	{

		if (curr_jump_time == 0.f)
		{//the factor of 100 is because unreal apparently applies forces in kg*cm*s^(-2)
			jump_force = (((FMath::Sqrt(2.f*1000.f*jump_height) - torso->GetComponentVelocity().Z) / jump_force_time) + 1000.f)*(torso->GetMass() + grip->GetMass() + grip_axis->GetMass()) * 100;
		}

		torso->AddForce(FVector::UpVector*jump_force*DeltaTime);

		curr_jump_time += DeltaTime;
		if (curr_jump_time > jump_force_time) {
			jumping = false;
			curr_jump_time = 0.0f;
		}

	}
}
//SETTERS / GETTERS ===========================================================================

void AJointCharacterTest::setCanMove(bool new_state)
{
	can_move = new_state;
}
void AJointCharacterTest::setCanSwing(bool new_state)
{
	can_swing = new_state;
}

void AJointCharacterTest::setPlayerSpecificMaterial(UMaterial* mat)
{
	torso_vis->SetMaterial(0, mat);
}

bool AJointCharacterTest::isAlive()
{
	return alive;
}

//COLLISION RESPONSE CALCULATIONS ===========================================================================

void AJointCharacterTest::OnBodyHit(UPrimitiveComponent* HitComp, AActor* OtherActor, UPrimitiveComponent* OtherComp, FVector NormalImpulse, const FHitResult& Hit)
{
	if (NormalImpulse.Size() > 16000.f)
	{
		TArray<USceneComponent*> tmp_child_comps;
		HitComp->GetChildrenComponents(true, tmp_child_comps);
		for (int i = 0; i < tmp_child_comps.Num(); i++)
		{
			/*UE_LOG(LogTemp, Warning, TEXT(" %s"), *tmp_child_comps[i]->GetFullName());
			if (tmp_child_comps[i]->GetFullName().Contains("Attachment"))
			{
			UPhysicsConstraintComponent* tmp_constraint = dynamic_cast<UPhysicsConstraintComponent*>(tmp_child_comps[i]);
			tmp_constraint->BreakConstraint();
			}*/
			//tmp_child_comps[i]->DestroyComponent();

			spine_attachment->BreakConstraint();

		}

		release();

		alive = false;
		fight_mode = false;
		torso->OnComponentHit.RemoveAll(this);
		spine->OnComponentHit.RemoveAll(this);

		Cast<AFightMode>(GetWorld()->GetAuthGameMode())->registerDeath(this->GetNetOwningPlayer()->PlayerController);
		//HitComp->DestroyComponent();
	}
}

//CAMERA CONTROL =======================================================================================================
void AJointCharacterTest::controlCameraDirection(float DeltaTime)
{

	locked_target_dir = (locked_target->GetComponentLocation() - camera_axis->GetComponentLocation()).GetSafeNormal();
	locked_target_dir_xy = locked_target_dir; locked_target_dir_xy.Z = 0.f;
	locked_target_dir_xy = locked_target_dir_xy.GetSafeNormal();

	FVector cam_forward = camera_axis->GetForwardVector();
	FVector cam_up = camera_axis->GetUpVector();

	FVector cam_arm_forward = camera_spring_arm->GetForwardVector();

	cdc.error.X = 1.f - FVector::DotProduct(locked_target_dir_xy, cam_forward);
	if (FVector::DotProduct(FVector::CrossProduct(locked_target_dir_xy, cam_forward), cam_up) > 0.f)
		cdc.error.X = -cdc.error.X;

	cdc.error.Y = FVector::DotProduct(locked_target_dir, FVector::UpVector) - FVector::DotProduct(cam_arm_forward, FVector::UpVector);

	cdc.integral = cdc.integral + cdc.error * DeltaTime;
	cdc.derivative = (cdc.error - cdc.prev_err) / DeltaTime;

	cdc.adjustment = cdc.P * cdc.error +
		cdc.I * cdc.integral +
		cdc.D * cdc.derivative;
	cdc.prev_err = cdc.error;

	camera_axis->AddLocalRotation(FRotator(0.f, cdc.adjustment.X, 0.f));

	camera_spring_arm->AddRelativeRotation(FRotator( cdc.adjustment.Y, 0.f, 0.f ));
}

void AJointCharacterTest::addPotentialTarget(UPrimitiveComponent* OverlappedComp, AActor* OtherActor, UPrimitiveComponent* OtherComp, int32 OtherBodyIndex, bool bFromSweep, const FHitResult& SweepResult)
{
	if (OtherActor != this)
		lock_on_targets.Add(OtherActor->GetRootComponent());
}
void AJointCharacterTest::removePotentialTarget(UPrimitiveComponent* OverlappedComp, AActor* OtherActor, UPrimitiveComponent* OtherComp, int32 OtherBodyIndex)
{
	if (OtherActor != this)
		lock_on_targets.Remove(OtherActor->GetRootComponent());
}

void AJointCharacterTest::aquireTarget()
{
	if (lock_on_targets.Num() > 0)
	{
		locked_target = lock_on_targets.Top();
	}
	//locked_target = this->GetNetOwningPlayer()->PlayerController->StartSpot->GetRootComponent();
}

//MOVEMENT CONTROL PHYSICS =======================================================================================================

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
	GetWorld()->LineTraceSingleByObjectType(rv_hit,        //result
		start,    //start
		end, //end
		FCollisionObjectQueryParams::AllStaticObjects, //collision channel
		RV_TraceParams);
	//GetWorld()->LineTraceSingleByChannel(
	//	rv_hit,        //result
	//	start,    //start
	//	end, //end
	//	ECC_WorldStatic, //collision channel
	//	RV_TraceParams
	//);


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
		BodyInstance->AddForce(FVector::UpVector * hover_height.adjustment * BodyInstance->GetBodyMass());

	}
	else
	{
		hover_height.prev_err = 0.0;
		hover_height.integral = 0.0;
	}
}

void AJointCharacterTest::customStabilizerPhysics(float DeltaTime, FBodyInstance* BodyInstance) 
{
	//this is the right vector of the hips
	FVector pelvis_up = pelvis_bi->GetUnrealWorldTransform().GetUnitAxis(EAxis::Z);

	FVector spine_up = spine_bi->GetUnrealWorldTransform().GetUnitAxis(EAxis::Z);

	//this is the right vector of the torso
	FVector torso_right= torso_bi->GetUnrealWorldTransform().GetUnitAxis(EAxis::X);

	////torso_position
	//FVector tmp_pose_error = -(pelvis_bi->GetCOMPosition() - torso_bi->GetCOMPosition());
	//pose_controller.error.X = tmp_pose_error.X; pose_controller.error.Y = tmp_pose_error.Y;
	//pose_controller.integral = pose_controller.integral + pose_controller.error * DeltaTime;
	//pose_controller.derivative = (pose_controller.error - pose_controller.prev_err) / DeltaTime;

	//pose_controller.adjustment = pose_controller.P * pose_controller.error +
	//	pose_controller.I * pose_controller.integral +
	//	pose_controller.D * pose_controller.derivative;
	//pose_controller.prev_err = pose_controller.error;

	////FVector move_force = ((target_vel - curr_vel) )*rolling_body->GetMass();
	////move_force.Z = 0;
	//FVector stabilizer_force = FVector(pose_controller.adjustment.X, pose_controller.adjustment.Y, 0.f)*torso->GetMass();
	////torso_bi->AddForce(stabilizer_force);
	//torso_bi->AddImpulse(stabilizer_force, false);

	FVector spine2hip_proj = spine_up - pelvis_up*FVector::DotProduct(pelvis_up, spine_up);
	spine2hip_proj.Normalize();

	ssc.error.X = FMath::Acos(FVector::DotProduct(ssc.target,
		spine2hip_proj))*180.f / PI;
	ssc.integral = ssc.integral + ssc.error * DeltaTime;
	ssc.derivative = (ssc.error - ssc.prev_err) / DeltaTime;

	ssc.adjustment = ssc.P * ssc.error +
		ssc.I * ssc.integral +
		ssc.D * ssc.derivative;
	ssc.prev_err = ssc.error;

	FVector tmp_rot_axis = FVector::CrossProduct(spine2hip_proj, ssc.target);
	//DrawDebugLine(
	//	GetWorld(),
	//	spine_bi->GetUnrealWorldTransform().GetLocation(),
	//	spine_bi->GetUnrealWorldTransform().GetLocation() + 100 * tmp_rot_axis.GetSafeNormal(), 					//size
	//	FColor(255, 0, 0),  //pink
	//	true,  				//persistent (never goes away)
	//	0.01, 					//point leaves a trail on moving object
	//	10,
	//	5.f
	//);
	
	//pelvis_bi->AddAngularImpulse(tmp_rot_axis*ssc.adjustment.X, false);
	
	//gravitational compensaiton
	float upper_mass = pelvis_bi->GetBodyMass() + spine_bi->GetBodyMass() + torso_bi->GetBodyMass();
	FVector upper_com = (pelvis_bi->GetCOMPosition()*pelvis_bi->GetBodyMass() + spine_bi->GetCOMPosition()*spine_bi->GetBodyMass() + torso_bi->GetCOMPosition()*torso_bi->GetBodyMass()) / (upper_mass);
	//pelvis_bi->AddForceAtPosition(FVector::UpVector*upper_mass * 980.f, upper_com, false);
	//pelvis_bi->AddTorque(-FVector::CrossProduct( FVector::UpVector*upper_mass * 980.f, upper_com - pelvis_bi->GetCOMPosition() ), false);

	FVector torso_current_forward = FVector::CrossProduct(FVector::UpVector, torso_right);
	
	ttc.error.X = FMath::Acos(FVector::DotProduct(target_direction,
		torso_current_forward))*180.f / PI;
	if (FVector::CrossProduct(target_direction, torso_current_forward).Z < 0.f)
		ttc.error.X = -ttc.error.X;

	ttc.integral = ttc.integral + ttc.error * DeltaTime;
	ttc.derivative = (ttc.error - ttc.prev_err) / DeltaTime;

	ttc.adjustment = ttc.P * ttc.error +
		ttc.I * ttc.integral +
		ttc.D * ttc.derivative;
	ttc.prev_err = ttc.error;

	//torso_bi->AddTorque(spine_up*-ttc.adjustment.X, false);

	FVector pelvis_current_forward = FVector::CrossProduct(FVector::UpVector, pelvis_up);

	ptc.error.X = FMath::Acos(FVector::DotProduct(target_direction,
		pelvis_current_forward))*180.f / PI;
	if (FVector::CrossProduct(target_direction, pelvis_current_forward).Z < 0.f)
		ptc.error.X = -ptc.error.X;

	ptc.integral = ptc.integral + ptc.error * DeltaTime;
	ptc.derivative = (ptc.error - ptc.prev_err) / DeltaTime;

	ptc.adjustment = ptc.P * ptc.error +
		ptc.I * ptc.integral +
		ptc.D * ptc.derivative;
	ptc.prev_err = ptc.error;

	//pelvis_bi->AddTorque(spine_up*ptc.adjustment.X, false);


}

void AJointCharacterTest::customWalkingPhysics(float DeltaTime, FBodyInstance* BodyInstance)
{
	//hip movement
	FMatrix from_pelvis_to_world_transform = pelvis->ComponentToWorld.ToMatrixWithScale();
	FMatrix from_world_to_hip_transform = r_hip_motor_bi->GetUnrealWorldTransform().ToMatrixWithScale();

	FVector r_thigh_target = from_pelvis_to_world_transform.TransformVector(rtdc.target);

	rtdc.error.X = FMath::Acos(FVector::DotProduct(r_thigh_target, -r_thigh_bi->GetUnrealWorldTransform().GetUnitAxis(EAxis::Z)))*180.f / PI;
	rtdc.integral = rtdc.integral + rtdc.error * DeltaTime;
	rtdc.derivative = (rtdc.error - rtdc.prev_err) / DeltaTime;

	rtdc.adjustment = rtdc.P * rtdc.error +
		rtdc.I * rtdc.integral +
		rtdc.D * rtdc.derivative;
	rtdc.prev_err = rtdc.error;

	FVector tmp_rot_axis = FVector::CrossProduct(-r_thigh_bi->GetUnrealWorldTransform().GetUnitAxis(EAxis::Z), r_thigh_target);
	//r_hip_motor_bi->AddAngularImpulse(tmp_rot_axis*rtdc.adjustment.X, false);
	
	//gravitational compensaiton
	float r_leg_mass = r_hip_motor_bi->GetBodyMass() + r_thigh_bi->GetBodyMass() + r_knee_motor_bi->GetBodyMass() + r_shin_bi->GetBodyMass();
	FVector r_leg_com = (r_hip_motor_bi->GetCOMPosition()*r_hip_motor_bi->GetBodyMass() + r_thigh_bi->GetCOMPosition()*r_thigh_bi->GetBodyMass() + r_knee_motor_bi->GetCOMPosition()*r_knee_motor_bi->GetBodyMass() + r_shin_bi->GetCOMPosition()*r_shin_bi->GetBodyMass() ) / (r_leg_mass);
	//r_hip_motor_bi->AddTorque(-FVector::CrossProduct(FVector::UpVector*r_leg_mass * 980.f, r_leg_com - r_hip_motor_bi->GetCOMPosition()), false);


	FVector l_thigh_target = from_pelvis_to_world_transform.TransformVector(ltdc.target);

	ltdc.error.X = FMath::Acos(FVector::DotProduct(l_thigh_target, -l_thigh_bi->GetUnrealWorldTransform().GetUnitAxis(EAxis::Z)))*180.f / PI;
	ltdc.integral = ltdc.integral + ltdc.error * DeltaTime;
	ltdc.derivative = (ltdc.error - ltdc.prev_err) / DeltaTime;

	ltdc.adjustment = ltdc.P * ltdc.error +
		ltdc.I * ltdc.integral +
		ltdc.D * ltdc.derivative;
	ltdc.prev_err = ltdc.error;

	tmp_rot_axis = FVector::CrossProduct(-l_thigh_bi->GetUnrealWorldTransform().GetUnitAxis(EAxis::Z), l_thigh_target);
	//l_hip_motor_bi->AddAngularImpulse(tmp_rot_axis*ltdc.adjustment.X, false);

	//gravitational compensaiton
	float l_leg_mass = l_hip_motor_bi->GetBodyMass() + l_thigh_bi->GetBodyMass() + l_knee_motor_bi->GetBodyMass() + l_shin_bi->GetBodyMass();
	FVector l_leg_com = (l_hip_motor_bi->GetCOMPosition()*l_hip_motor_bi->GetBodyMass() + l_thigh_bi->GetCOMPosition()*l_thigh_bi->GetBodyMass() + l_knee_motor_bi->GetCOMPosition()*l_knee_motor_bi->GetBodyMass() + l_shin_bi->GetCOMPosition()*l_shin_bi->GetBodyMass() ) / (l_leg_mass);
	//l_hip_motor_bi->AddTorque(-FVector::CrossProduct(FVector::UpVector*l_leg_mass * 980.f, l_leg_com - l_hip_motor_bi->GetCOMPosition()), false);

	//knee movement

	rkec.integral = rkec.integral + rkec.error * DeltaTime;
	rkec.derivative = (rkec.error - rkec.prev_err) / DeltaTime;

	rkec.adjustment = rkec.P * rkec.error +
		rkec.I * rkec.integral +
		rkec.D * rkec.derivative;
	rkec.prev_err = rkec.error;

	
	lkec.integral = lkec.integral + lkec.error * DeltaTime;
	lkec.derivative = (lkec.error - lkec.prev_err) / DeltaTime;

	lkec.adjustment = lkec.P * lkec.error +
		lkec.I * lkec.integral +
		lkec.D * lkec.derivative;
	lkec.prev_err = lkec.error;
	
	//pelvis_bi->AddAngularImpulse(FVector::UpVector * 500, false);

	//foot movement

}

//WEAPON CONTROL PHYSICS =======================================================================================================

void AJointCharacterTest::ControlGripPhysics(float DeltaTime, FBodyInstance* BodyInstance)
{
	initInputVars();
	

	customInitGripPhysics(DeltaTime, BodyInstance);
	ControlArmDirectionPhysics(DeltaTime, BodyInstance);
	ControlArmTwistPhysics(DeltaTime, BodyInstance);
	if (!guard_locked)
	{
		ControlGripDirectionPhysics(DeltaTime, BodyInstance);
		ControlGripInclinePhysics(DeltaTime, BodyInstance);
	}

	if (guarding && !guard_locked && (1.f - FVector::DotProduct(target_arm_dir, ga_up)) < 0.01f && (1.f - FVector::DotProduct(target_wep_dir, g_up)) < 0.01f && FMath::Abs(atc.error) < 0.01f)
		lockGuard();

	if (holding_weapon && !guard_locked)
	{
		ControlWeaponTwistPhysics(DeltaTime, BodyInstance);
	}
	else if (grabbing_weapon)
	{
		weaponGrabControl(DeltaTime, BodyInstance);
	}
}

void AJointCharacterTest::initInputVars()
{

	if (!fight_mode)
	{
		if (!guarding)
		{
			wep_extended = false;
			target_wep_dir = (-camera_axis->GetRightVector()*0.3f + camera_axis->GetForwardVector() + camera_axis->GetUpVector()*0.4f).GetSafeNormal();
			target_arm_dir = (camera_axis->GetRightVector() - camera_axis->GetUpVector()*0.3f + camera_axis->GetForwardVector()*0.4f).GetSafeNormal();
		}
		else
		{
			input_dir = camera_axis->GetForwardVector();
			wep_extended = false;
			target_wep_dir = input_dir;
			target_arm_dir = input_dir;

			target_wep_dir_xy = target_wep_dir - FVector::DotProduct(target_wep_dir, camera_axis->GetUpVector())*camera_axis->GetUpVector();

			target_arm_dir_xy = target_arm_dir - FVector::DotProduct(target_wep_dir, camera_axis->GetUpVector())*camera_axis->GetUpVector();
			target_arm_dir_xy.Normalize();

			if (target_wep_dir_xy.IsNearlyZero())
				target_wep_dir_xy = prev_target_wep_dir_xy;

			FVector guard_pos(FMath::Cos(PI / 2.5f), 0.f, FMath::Sin(PI / 2.5f));
			guard_pos.Normalize();

			float tmp_angle = FMath::Acos(FVector::DotProduct(target_wep_dir_xy.GetSafeNormal(), FVector::ForwardVector))*180.f / PI;
			if (FVector::CrossProduct(FVector::ForwardVector, target_wep_dir_xy.GetSafeNormal()).Z < 0)
				tmp_angle = -tmp_angle;
			target_arm_dir = guard_pos.RotateAngleAxis(tmp_angle, FVector::UpVector);

			FVector guard_dir(FMath::Cos(-PI / 3.f), 0.f, FMath::Sin(-PI / 3.f));
			guard_dir.Normalize();
			target_wep_dir = guard_dir.RotateAngleAxis(tmp_angle, FVector::UpVector);

			target_wep_dir_curr_wep_proj = target_wep_dir - FVector::DotProduct(target_wep_dir, g_right)*g_right;
		}
	}
	else
	{

		if (guarding)
		{
			wep_extended = false;
			target_wep_dir = input_dir;
			target_arm_dir = input_dir;

			target_wep_dir_xy = target_wep_dir - FVector::DotProduct(target_wep_dir, camera_axis->GetUpVector())*camera_axis->GetUpVector();

			target_arm_dir_xy = target_arm_dir - FVector::DotProduct(target_wep_dir, camera_axis->GetUpVector())*camera_axis->GetUpVector();
			target_arm_dir_xy.Normalize();

			if (target_wep_dir_xy.IsNearlyZero())
				target_wep_dir_xy = prev_target_wep_dir_xy;

			FVector guard_pos(FMath::Cos(PI / 2.5f), 0.f, FMath::Sin(PI / 2.5f));
			guard_pos.Normalize();

			float tmp_angle = FMath::Acos(FVector::DotProduct(target_wep_dir_xy.GetSafeNormal(), FVector::ForwardVector))*180.f / PI;
			if (FVector::CrossProduct(FVector::ForwardVector, target_wep_dir_xy.GetSafeNormal()).Z < 0)
				tmp_angle = -tmp_angle;
			target_arm_dir = guard_pos.RotateAngleAxis(tmp_angle, FVector::UpVector);

			FVector guard_dir(FMath::Cos(-PI / 3.f), 0.f, FMath::Sin(-PI / 3.f));
			guard_dir.Normalize();
			target_wep_dir = guard_dir.RotateAngleAxis(tmp_angle, FVector::UpVector);

			target_wep_dir_curr_wep_proj = target_wep_dir - FVector::DotProduct(target_wep_dir, g_right)*g_right;

		}
		else
		{
			//direction target
			target_wep_dir = input_dir;
			//target_wep_dir_xy.Normalize();
			target_wep_dir_curr_wep_proj = target_wep_dir - FVector::DotProduct(target_wep_dir, g_right)*g_right;
			if (target_wep_dir_xy.IsNearlyZero()) {
				target_wep_dir_xy = prev_target_wep_dir_xy;
			}
			else
			{
				prev_target_wep_dir_xy = target_wep_dir_xy;
			}

			target_wep_dir_xy = target_wep_dir - FVector::DotProduct(target_wep_dir, camera_axis->GetUpVector())*camera_axis->GetUpVector();

			//position target
			target_arm_dir = input_dir;
			target_arm_dir_xy = target_arm_dir - FVector::DotProduct(target_arm_dir, camera_axis->GetUpVector())*camera_axis->GetUpVector();
			/*if (target_arm_dir_xy.IsNearlyZero()) {
			target_arm_dir_xy = prev_target_arm_dir_xy;
			}
			else
			{
			target_arm_dir_xy.Normalize();
			prev_target_arm_dir_xy = target_arm_dir_xy;
			}*/
		}
	}


	
}

void AJointCharacterTest::customInitGripPhysics(float DeltaTime, FBodyInstance* BodyInstance)
{
	//grip_axis_bi->SetBodyTransform(FTransform(torso_bi->GetCOMPosition() + FVector(0.f, 0.f, -15.f)), true);
	ga_pos = grip_axis_bi->GetUnrealWorldTransform().GetLocation();
	ga_forward = grip_axis_bi->GetUnrealWorldTransform().GetUnitAxis(EAxis::X);
	ga_right = grip_axis_bi->GetUnrealWorldTransform().GetUnitAxis(EAxis::Y);
	ga_prev_up = ga_up;
	ga_up = grip_axis_bi->GetUnrealWorldTransform().GetUnitAxis(EAxis::Z);

	g_pos = grip_bi->GetUnrealWorldTransform().GetLocation();
	g_forward = grip_bi->GetUnrealWorldTransform().GetUnitAxis(EAxis::X);
	g_right = grip_bi->GetUnrealWorldTransform().GetUnitAxis(EAxis::Y);
	g_prev_up = g_up;
	g_up = grip_bi->GetUnrealWorldTransform().GetUnitAxis(EAxis::Z);

	current_griph_xydir = g_forward;
	current_griph_xydir.Z = 0.0f;

	/*float current_wep_incline = FMath::Acos(FVector::DotProduct(g_up, w_up))*180.f / PI;
	current_wep_incline = FMath::Fmod(current_wep_incline, 90.0f);*/
}


void AJointCharacterTest::ControlArmDirectionPhysics(float DeltaTime, FBodyInstance* BodyInstance)
{
	
	//adc.error.X = (FMath::Acos(FVector::DotProduct(target_arm_dir, ga_up))*180.f / PI);
	adc.error.X = 1.f - FVector::DotProduct(target_arm_dir, ga_up);
	FVector rot_ref = FVector::CrossProduct(ga_up, target_arm_dir);
	rot_ref = rot_ref.GetSafeNormal();

	//calculate rotational speed around rotation point
	FVector vel_axis = (ga_up - ga_prev_up) / DeltaTime;
	adc.error.Y = -vel_axis.Size();
	vel_axis = vel_axis.GetSafeNormal();

	//calculate current inertial properties of the grip axis, grip (and weapon when attached)
	//float wi_ref_i = 0.f;
	//float wi_va_i = 0.f;
	//if (holding_weapon)
	//{
	//	FMatrix wep_inertia;
	//	calculateRelativeInertia(held_weapon->getShaftComponent()->GetBodyInstance(), ga_pos, &wep_inertia);
	//	wi_ref_i = inertiaAboutAxis(wep_inertia, rot_ref);
	//	wi_va_i = inertiaAboutAxis(wep_inertia, FVector::CrossProduct(ga_up, vel_axis).GetSafeNormal());
	//	/*UE_LOG(LogTemp, Warning, TEXT("adc.adjustment: %s"), *adc.adjustment.ToString());
	//	UE_LOG(LogTemp, Warning, TEXT("wep inertia vel: %f"), wi_va_i);
	//	UE_LOG(LogTemp, Warning, TEXT("wep inertia x: %f"), inertiaAboutAxis(wep_inertia, FVector::ForwardVector));*/
	//}
	FMatrix grip_inertia;
	calculateRelativeInertia(grip_bi, ga_pos, &grip_inertia);
	float gi_ref_i = inertiaAboutAxis(grip_inertia, rot_ref);
	float gi_va_i = inertiaAboutAxis(grip_inertia, FVector::CrossProduct(ga_up, vel_axis).GetSafeNormal());
	
	//calculate PID
	adc.error.X = adc.error.X*(/*wi_ref_i + */gi_ref_i + grip_axis_bi->GetBodyInertiaTensor().Z);
	adc.error.Y = adc.error.Y*(/*wi_va_i +*/ gi_va_i + grip_axis_bi->GetBodyInertiaTensor().Z);

	adc.integral = adc.integral + adc.error * DeltaTime;
	adc.derivative = (adc.error - adc.prev_err) / DeltaTime;

	adc.adjustment = adc.P * adc.error +
		adc.I * adc.integral +
		adc.D * adc.derivative;
	adc.prev_err = adc.error;
	
	//reset constraint rotation
	FVector WPri = grip_axis_attachment->ComponentToWorld.GetUnitAxis(EAxis::X);
	FVector WOrth = grip_axis_attachment->ComponentToWorld.GetUnitAxis(EAxis::Y);
	FVector PriAxis1 = torso->GetComponentTransform().InverseTransformVectorNoScale(WPri);
	FVector SecAxis1 = torso->GetComponentTransform().InverseTransformVectorNoScale(WOrth);
	grip_axis_attachment->SetConstraintReferenceOrientation(EConstraintFrame::Frame1, PriAxis1, SecAxis1);

	//apply pid adjusted forces
	//grip_axis_bi->AddTorque(rot_ref*adc.adjustment.X, false);
	rot_ref = FVector::CrossProduct(rot_ref, ga_up).GetSafeNormal();
	grip_axis_bi->AddForceAtPosition(rot_ref*adc.adjustment.X, ga_pos + ga_up, false);
	grip_axis_bi->AddForceAtPosition(rot_ref*-adc.adjustment.X, ga_pos - ga_up, false);

	grip_axis_bi->AddForceAtPosition(vel_axis*adc.adjustment.Y, ga_pos + ga_up, false);
	grip_axis_bi->AddForceAtPosition(vel_axis*-adc.adjustment.Y, ga_pos - ga_up, false);
}

void AJointCharacterTest::ControlArmTwistPhysics(float DeltaTime, FBodyInstance* BodyInstance)
{
	FVector ga_up_xy = ga_up;
	ga_up_xy.Z = 0.f;
	//ga_up_xy = ga_up_xy.GetSafeNormal(0.001f);
	if (!ga_up_xy.IsNearlyZero(0.09f))
	{
		ga_up_xy.Normalize();
		FVector projection_vector = (ga_forward - FVector::DotProduct(ga_forward, ga_up_xy)*ga_up_xy).GetSafeNormal();

		atc.error = 1.f - FVector::DotProduct(projection_vector, -FVector::UpVector);
		if (FVector::DotProduct(ga_up_xy, FVector::CrossProduct(-FVector::UpVector, projection_vector).GetSafeNormal()) > 0.f)
			atc.error = -atc.error;
	}
	else
	{
		FVector ga_forward_xy = ga_forward;
		ga_forward_xy.Z = 0.f;
		ga_forward_xy.Normalize();

		atc.error = 1.f - FVector::DotProduct(ga_forward_xy, camera_axis->GetForwardVector());
		if (FVector::CrossProduct(ga_forward_xy, camera_axis->GetForwardVector()).GetSafeNormal().Z < 0.f)
			atc.error = -atc.error;
	}

	

	FMatrix grip_inertia;
	calculateRelativeInertia(grip_bi, ga_pos, &grip_inertia);
	float gi_ref_i = inertiaAboutAxis(grip_inertia, ga_up);

	//atc.error = atc.error*(gi_ref_i + grip_axis_bi->GetBodyInertiaTensor().Z);
	//arm twist control error with inertia
	float atcewi = atc.error*(gi_ref_i + grip_axis_bi->GetBodyInertiaTensor().Z);
	
	atc.integral = atc.integral + atcewi * DeltaTime;
	atc.derivative = (atcewi - atc.prev_err) / DeltaTime;

	atc.adjustment = atc.P * atcewi +
		atc.I * atc.integral +
		atc.D * atc.derivative;
	atc.prev_err = atcewi;

	grip_axis_bi->AddTorque(ga_up*atc.adjustment, false);
}


void AJointCharacterTest::ControlGripDirectionPhysics(float DeltaTime, FBodyInstance* BodyInstance)
{
	//gdc.error.X = (FMath::Acos(FVector::DotProduct(target_arm_dir, g_up))*180.f / PI);
	//gdc.error.X = FMath::Acos(FVector::DotProduct(target_arm_dir, g_up));
	gdc.error.X = 1.f - FVector::DotProduct(target_wep_dir, g_up);
	FVector rot_ref = FVector::CrossProduct(g_up, target_wep_dir);
	//UE_LOG(LogTemp, Warning, TEXT("rot_ref: %s"), *rot_ref.ToString());
	rot_ref = rot_ref.GetSafeNormal();

	//calculate rotational speed around rotation point
	FVector vel_axis = (g_up - g_prev_up)/DeltaTime;
	gdc.error.Y = -vel_axis.Size();
	vel_axis = vel_axis.GetSafeNormal();

	//calculate current inertial properties of the grip (and weapon when attached)
	FMatrix grip_inertia;
	calculateRelativeInertia(grip_bi, g_pos, &grip_inertia);
	float gi_ref_i = inertiaAboutAxis(grip_inertia, rot_ref);
	float gi_va_i = inertiaAboutAxis(grip_inertia, FVector::CrossProduct(g_up, vel_axis).GetSafeNormal());

	//calculate PID
	gdc.error.X = gdc.error.X*(gi_ref_i);
	gdc.error.Y = gdc.error.Y*(gi_va_i );

	gdc.integral = gdc.integral + gdc.error * DeltaTime;
	gdc.derivative = (gdc.error - gdc.prev_err) / DeltaTime;

	gdc.adjustment = gdc.P * gdc.error +
		gdc.I * gdc.integral +
		gdc.D * gdc.derivative;
	gdc.prev_err = gdc.error;

	//reset constraint rotation
	FVector WPri = grip_attachment->ComponentToWorld.GetUnitAxis(EAxis::X);
	FVector WOrth = grip_attachment->ComponentToWorld.GetUnitAxis(EAxis::Y);

	FVector PriAxis1 = grip_axis->GetComponentTransform().InverseTransformVectorNoScale(WPri);
	FVector SecAxis1 = grip_axis->GetComponentTransform().InverseTransformVectorNoScale(WOrth);
	grip_attachment->SetConstraintReferenceOrientation(EConstraintFrame::Frame1, PriAxis1, SecAxis1);

	//apply pid adjusted forces
	//grip_bi->AddTorque(rot_ref*gdc.adjustment.X, false, false);
	rot_ref = FVector::CrossProduct(rot_ref, g_up).GetSafeNormal();
	grip_bi->AddForceAtPosition(rot_ref*gdc.adjustment.X, g_pos + g_up, false);
	grip_bi->AddForceAtPosition(rot_ref*-gdc.adjustment.X, g_pos - g_up, false);
	
	//grip_bi->AddTorque(vel_axis*gdc.adjustment.Y, false, false);
	//grip_bi->AddTorque(vel_axis*gdc.adjustment.Y*(20000.f), false);
	grip_bi->AddForceAtPosition(vel_axis*gdc.adjustment.Y, g_pos + g_up, false);
	grip_bi->AddForceAtPosition(vel_axis*-gdc.adjustment.Y, g_pos - g_up, false);

	//DrawDebugLine(
	//	GetWorld(),
	//	grip->GetComponentLocation(),
	//	grip->GetComponentLocation() + 100.f * vel_axis, 					//size
	//	FColor(255, 0, 0),  //pink
	//	true,  				//persistent (never goes away)
	//	0.03, 					//point leaves a trail on moving object
	//	10,
	//	5.f
	//);

}

void AJointCharacterTest::ControlGripInclinePhysics(float DeltaTime, FBodyInstance* BodyInstance)
{

}
void AJointCharacterTest::ControlWeaponTwistPhysics(float DeltaTime, FBodyInstance* BodyInstance)
{
	FVector solder_dir = g_up - weapon_twist_solder;
	if (solder_dir.Size() > 0.05f)
	{
		weapon_twist_target = solder_dir.GetSafeNormal();
		weapon_twist_solder = weapon_twist_solder + solder_dir*(0.05f / solder_dir.Size());
	}

	weapon_twist_target = weapon_twist_target.GetSafeNormal() - FVector::DotProduct(weapon_twist_target.GetSafeNormal(), g_up)*g_up;
	weapon_twist_target = weapon_twist_target.GetSafeNormal();

	wtc.error = 1 - FVector::DotProduct(g_forward, weapon_twist_target);
	float tmp_forward_angle_ref = 1 - FVector::DotProduct(-g_forward, weapon_twist_target);

	if (tmp_forward_angle_ref < wtc.error)
	{
		wtc.error = tmp_forward_angle_ref;
		g_forward = -g_forward;
	}
	//weapon_vis->SetRelativeRotation(FRotator(0.f, sword_twist.error, 0.f));
	FVector tmp_ref_vec = FVector::CrossProduct(g_forward, weapon_twist_target);

	if (!FVector::Coincident(tmp_ref_vec, g_up, 0.0001f))
		wtc.error = -wtc.error;

	wtc.integral = wtc.integral + wtc.error * DeltaTime;
	wtc.derivative = (wtc.error - wtc.prev_err) / DeltaTime;

	wtc.adjustment = wtc.P * wtc.error +
		wtc.I * wtc.integral +
		wtc.D * wtc.derivative;
	wtc.prev_err = wtc.error;

	float tmp_inertia = grip_bi->GetBodyInertiaTensor().Z;
	grip_bi->AddTorque(g_up*wtc.adjustment*tmp_inertia, false);

	//held_weapon->getShaftComponent()->AddLocalRotation(FRotator(0.f, wtc.adjustment, 0.f));
	//held_weapon->getShaftComponent()->AddLocalRotation(FRotator(0.f, 1.f, 0.f));
}


void AJointCharacterTest::weaponGrabControl(float DeltaTime, FBodyInstance* BodyInstance)
{
	FBodyInstance* tmp_shaft = held_weapon->getShaftComponent()->GetBodyInstance();
	w_prev_up = w_up;
	w_up = tmp_shaft->GetUnrealWorldTransform().GetUnitAxis(EAxis::Z);
	w_pos = tmp_shaft->GetUnrealWorldTransform().GetLocation();
	USceneComponent* tmp_attachment = held_weapon->getAttachmentPoint();

	wgc.error = g_pos - tmp_attachment->GetComponentLocation();

	//weapon position --------------------------------------------------------------
	wgc.integral = wgc.integral + wgc.error * DeltaTime;
	wgc.derivative = (wgc.error - wgc.prev_err) / DeltaTime;

	wgc.adjustment = wgc.P * wgc.error +
		wgc.I * wgc.integral +
		wgc.D * wgc.derivative;
	wgc.prev_err = wgc.error;

	float mass = tmp_shaft->GetBodyMass();
	//tmp_shaft->AddForceAtPosition(wgc.adjustment*(mass)/*+mass*980.f*FVector::UpVector*/, tmp_attachment->GetComponentLocation(), false);
	tmp_shaft->AddForce(wgc.adjustment*(mass), false);
	////weapon rotation --------------------------------------------------------------
	wgdc.error.X = 1.f - FVector::DotProduct(g_up, w_up);
	UE_LOG(LogTemp, Warning, TEXT("wgdc.error.X  %f"), wgdc.error.X);
	FVector rot_ref = FVector::CrossProduct(w_up, g_up);
	rot_ref = rot_ref.GetSafeNormal();

	if (wgc.error.Size() < 10.f && FMath::Abs(wgdc.error.X) < 0.1f)
	{
		held_weapon->getShaftComponent()->SetAngularDamping(0.0f);
		held_weapon->getShaftComponent()->SetLinearDamping(0.01f);
		attachWeapon(held_weapon);

		grabbing_weapon = false;
	}

	FVector vel_axis = (w_up - w_prev_up) / DeltaTime;
	wgdc.error.Y = -vel_axis.Size();
	vel_axis = vel_axis.GetSafeNormal();

	FMatrix wep_inertia;
	calculateRelativeInertia(tmp_shaft, w_pos, &wep_inertia);
	float wi_ref_i = inertiaAboutAxis(wep_inertia, rot_ref);
	float wi_va_i = inertiaAboutAxis(wep_inertia, FVector::CrossProduct(w_up, vel_axis).GetSafeNormal());

	wgdc.error.X = wgdc.error.X*(wi_ref_i);
	wgdc.error.Y = wgdc.error.Y*(wi_va_i);

	wgdc.integral = wgdc.integral + wgdc.error * DeltaTime;
	wgdc.derivative = (wgdc.error - wgdc.prev_err) / DeltaTime;

	wgdc.adjustment = wgdc.P * wgdc.error +
		wgdc.I * wgdc.integral +
		wgdc.D * wgdc.derivative;
	wgdc.prev_err = wgdc.error;

	rot_ref = FVector::CrossProduct(rot_ref, w_up).GetSafeNormal();
	tmp_shaft->AddForceAtPosition(rot_ref*wgdc.adjustment.X, w_pos + w_up, false);
	tmp_shaft->AddForceAtPosition(rot_ref*-wgdc.adjustment.X, w_pos - w_up, false);

	tmp_shaft->AddForceAtPosition(vel_axis*wgdc.adjustment.Y, w_pos + w_up, false);
	tmp_shaft->AddForceAtPosition(vel_axis*-wgdc.adjustment.Y, w_pos - w_up, false);
	//if (!grabbing_weapon)
	//{
	//	wgc.prev_err = FVector::ZeroVector;
	//	wgrc.prev_err = 0.f;
	//	wgic.prev_err = 0.f;
	//}

}

void AJointCharacterTest::lockGuard()
{
	////Grip_h
	//FVector WPri = grip_h_attachment->ComponentToWorld.GetUnitAxis(EAxis::X);
	//FVector WOrth = grip_h_attachment->ComponentToWorld.GetUnitAxis(EAxis::Y);

	//FVector PriAxis1 = grip_axis->GetComponentTransform().InverseTransformVectorNoScale(WPri);
	//FVector SecAxis1 = grip_axis->GetComponentTransform().InverseTransformVectorNoScale(WOrth);
	//grip_h_attachment->SetConstraintReferenceOrientation(EConstraintFrame::Frame1, PriAxis1, SecAxis1);

	grip_attachment->SetAngularSwing1Limit(EAngularConstraintMotion::ACM_Locked, 0.0f);
	grip_attachment->SetAngularSwing2Limit(EAngularConstraintMotion::ACM_Locked, 0.0f);
	grip_attachment->SetAngularTwistLimit(EAngularConstraintMotion::ACM_Locked, 0.0f);


	guard_locked = true;
}

void AJointCharacterTest::unlockGuard()
{

	grip_attachment->SetAngularSwing1Limit(EAngularConstraintMotion::ACM_Free, 0.0f);
	grip_attachment->SetAngularSwing2Limit(EAngularConstraintMotion::ACM_Free, 0.0f);
	grip_attachment->SetAngularTwistLimit(EAngularConstraintMotion::ACM_Free, 0.0f);

	guard_locked = false;
}

// Called to bind functionality to input
void AJointCharacterTest::SetupPlayerInputComponent(UInputComponent* PlayerInputComponent)
{
	Super::SetupPlayerInputComponent(PlayerInputComponent);

	//Hook up events for "ZoomIn"
	InputComponent->BindAction("FightMode", IE_Pressed, this, &AJointCharacterTest::fightModeOn);
	InputComponent->BindAction("FightMode", IE_Released, this, &AJointCharacterTest::fightModeOff);

	InputComponent->BindAction("Guard", IE_Pressed, this, &AJointCharacterTest::guard);
	InputComponent->BindAction("Guard", IE_Released, this, &AJointCharacterTest::abortGuard);

	InputComponent->BindAction("Grab", IE_Pressed, this, &AJointCharacterTest::release);
	InputComponent->BindAction("Grab", IE_Repeat, this, &AJointCharacterTest::grab);
	InputComponent->BindAction("Grab", IE_Released, this, &AJointCharacterTest::abortGrab);

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
	scaled_inverted_cam_input_size = FMath::Pow(1 - camera_input.Size(), 2.0f);
	prev_input_dir = input_dir;
	input_dir = (camera_axis->GetForwardVector()*camera_input.Y + camera_axis->GetRightVector()* camera_input.X) + camera_axis->GetUpVector()*scaled_inverted_cam_input_size;
	input_dir.Normalize();
}

void AJointCharacterTest::yawCamera(float AxisValue)
{
	camera_input.X = AxisValue;
	scaled_inverted_cam_input_size = FMath::Pow(1 - camera_input.Size(), 2.0f);
	prev_input_dir = input_dir;
	input_dir = (camera_axis->GetForwardVector()*camera_input.Y + camera_axis->GetRightVector()* camera_input.X) + camera_axis->GetUpVector()*scaled_inverted_cam_input_size;
	input_dir.Normalize();
}

void AJointCharacterTest::zoomIn()
{
	zooming = true;
}

void AJointCharacterTest::zoomOut()
{
	zooming = false;
}

void AJointCharacterTest::guard()
{
	guarding = true;
}

void AJointCharacterTest::abortGuard()
{
	guarding = false;
	unlockGuard();
}

void AJointCharacterTest::fightModeOn()
{

	fight_mode = true;
	fight_mode_state = 1;
	fight_t = 0.0f;

	axis_rot_before_fight = camera_axis->GetComponentRotation();
		
	arm_rot_before_fight = FRotator(camera_spring_arm->GetRelativeTransform().GetRotation());
	arm_length_before_fight = camera_spring_arm->TargetArmLength;

	aquireTarget();
}

void AJointCharacterTest::fightModeOff()
{
	fight_mode = false;
	fight_mode_state = 3;

	locked_target = nullptr;
}

void AJointCharacterTest::release()
{
	wep_extended = false;
	if (holding_weapon)
	{
		/*UE_LOG(LogTemp, Warning, TEXT("----------------------"));
		UE_LOG(LogTemp, Warning, TEXT("grip_vmass before unweld: %f"), grip_v_bi->GetBodyMass());
		UE_LOG(LogTemp, Warning, TEXT("grip_v inertia before unweld: %s"), *grip_v_bi->GetBodyInertiaTensor().ToString());*/

		//wep_attachment->BreakConstraint();
		grip->UnWeldChildren();
		grip->UpdateBodySetup();
		held_weapon->DetachFromActor(FDetachmentTransformRules::KeepWorldTransform);
		held_weapon->getShaftComponent()->SetSimulatePhysics(true);
		held_weapon->getShaftComponent()->SetPhysicsLinearVelocity(grip->GetPhysicsLinearVelocity());
		held_weapon->getShaftComponent()->SetPhysicsAngularVelocity(grip->GetPhysicsAngularVelocity());

		held_weapon->getShaftComponent()->SetEnableGravity(true);

		held_weapon->deInitGrabbed();

		holding_weapon = false;
		wep_extended = false;

		UE_LOG(LogTemp, Warning, TEXT("----------------------"));
		return;
	}
	else if (holding_object)
	{
		/*grip->UnWeldChildren();
		grip->UpdateBodySetup();*/
		holding_object = false;
		return;
	}

}

void AJointCharacterTest::grab()
{
	if (!grabbing_weapon && !holding_weapon)
	{
		TSet<AActor*> overlaps;
		grip->GetOverlappingActors(overlaps);

		TArray<FHitResult> traces;
		FCollisionQueryParams RV_TraceParams = FCollisionQueryParams(FName(TEXT("RV_Trace")), true, this);
		RV_TraceParams.bTraceComplex = false;
		RV_TraceParams.bTraceAsyncScene = true;
		RV_TraceParams.bReturnPhysicalMaterial = false;

		//GetWorld()->LineTraceMultiByChannel(traces, grip->GetComponentLocation() + FVector::UpVector*150.f, grip->GetComponentLocation() - FVector::UpVector*300.f, ECollisionChannel::ECC_PhysicsBody, RV_TraceParams);
		GetWorld()->SweepMultiByChannel(traces, grip->GetComponentLocation() + FVector::UpVector*150.f, grip->GetComponentLocation() - FVector::UpVector*300.f, FQuat(), ECollisionChannel::ECC_PhysicsBody, FCollisionShape::MakeSphere(grip->GetUnscaledSphereRadius()), RV_TraceParams);
		if (overlaps.Num() > 0)
		{
			//overlaps.Remove
			int one = 1;
			TSet<AActor*>::TIterator it = overlaps.CreateIterator();
			//(*it)->AttachToComponent(grip_h);
			held_weapon = dynamic_cast<AWeapon*>((*it));
			if (held_weapon)
			{
				grabbing_weapon = true;
				held_weapon->getShaftComponent()->SetEnableGravity(false);
				//held_weapon->getShaftComponent()->SetAngularDamping(1.0f);
				//held_weapon->getShaftComponent()->SetLinearDamping(1.0f);

			}
			else
			{
				auto object_root = (*it)->GetRootComponent();

				/*

				(*it)->SetActorRotation(grip_v->GetComponentRotation());
				(*it)->SetActorLocation(grip_v->GetComponentLocation());

				object_root->WeldTo(grip_v);
				grip_v->UpdateBodySetup();*/

				return;
			}


		}
		else if (traces.Num() > 0)
		{
			held_weapon = dynamic_cast<AWeapon*>(traces[0].GetActor());
			if (held_weapon)
			{
				grabbing_weapon = true;
				held_weapon->getShaftComponent()->SetEnableGravity(false);
				//held_weapon->getShaftComponent()->SetAngularDamping(1.0f);
				//held_weapon->getShaftComponent()->SetLinearDamping(1.0f);
			}
		}
	}
}

void AJointCharacterTest::attachWeapon(AWeapon* _wep)
{
	UCapsuleComponent* tmp_shaft = dynamic_cast<UCapsuleComponent*>(held_weapon->GetRootComponent());
	tmp_shaft->SetSimulatePhysics(false);
	USceneComponent* tmp_handle = dynamic_cast<USceneComponent*>(held_weapon->GetComponentsByTag(USceneComponent::StaticClass(), "handle_point").Top());
	held_weapon->SetActorRotation(grip->GetComponentRotation());

	FVector handle_offset = held_weapon->GetActorLocation() - tmp_handle->GetComponentLocation();
	held_weapon->SetActorLocation(grip->GetComponentLocation() + (handle_offset));

	//wep_attachment->ConstraintActor2 = held_weapon;
	//wep_attachment->SetConstrainedComponents(grip, NAME_None, tmp_shaft, NAME_None);
	held_weapon->AttachRootComponentTo(grip, NAME_None, EAttachLocation::KeepWorldPosition, true);
	grip->UpdateBodySetup();

	w_up = FVector::ZeroVector;

	holding_weapon = true;
	wep_extended = false;
	held_weapon->initGrabbed(this);
}


void AJointCharacterTest::abortGrab()
{
	if (grabbing_weapon)
	{
		held_weapon->getShaftComponent()->SetEnableGravity(true);
		held_weapon = nullptr;
		grabbing_weapon = false;
	}

}

//INITIALIZATION ==========================================================================

void AJointCharacterTest::initCamera()
{
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

	targeting_sphere = CreateDefaultSubobject<USphereComponent>(TEXT("TargetingSphere"));
	targeting_sphere->SetupAttachment(torso);
	targeting_sphere->SetSphereRadius(1500);
}

void AJointCharacterTest::initUpperBody()
{

	spine = CreateDefaultSubobject<UCapsuleComponent>(TEXT("Spine"));
	spine->SetupAttachment(pelvis);
	spine->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);
	spine->SetNotifyRigidBodyCollision(true);
	spine_vis = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("SpineVisualization"));
	spine_vis->SetupAttachment(spine);
	spine_attachment = CreateDefaultSubobject<UPhysicsConstraintComponent>(TEXT("SpineAttachment"));
	spine_attachment->SetupAttachment(spine);
	spine_attachment->SetRelativeLocation(FVector(0.f, 0.f, 20.f));

	torso = CreateDefaultSubobject<UBoxComponent>(TEXT("Torso"));
	torso->SetupAttachment(spine);
	torso->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);
	torso->SetNotifyRigidBodyCollision(true);
	torso_vis = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("TorsoVisualization"));
	torso_vis->SetupAttachment(torso);

}

void AJointCharacterTest::initUpperBodyJoints()
{
	spine_attachment->SetConstrainedComponents(spine, NAME_None, torso, NAME_None);
	spine_attachment->SetLinearXLimit(ELinearConstraintMotion::LCM_Locked, 4.0f);
	spine_attachment->SetLinearYLimit(ELinearConstraintMotion::LCM_Locked, 4.0f);
	spine_attachment->SetLinearZLimit(ELinearConstraintMotion::LCM_Locked, 4.0f);
	spine_attachment->SetAngularSwing1Limit(EAngularConstraintMotion::ACM_Locked, 0.0f);
	spine_attachment->SetAngularSwing2Limit(EAngularConstraintMotion::ACM_Locked, 0.0f);
	spine_attachment->SetAngularTwistLimit(EAngularConstraintMotion::ACM_Locked, 0.0f);
	spine_attachment->SetDisableCollision(true);
}

void AJointCharacterTest::initWeapon()
{
	// Weapon settings
	grip_axis = CreateDefaultSubobject<USphereComponent>(TEXT("WeaponAxis"));
	grip_axis->SetupAttachment(torso);
	grip_axis->SetRelativeLocation(FVector(0.f, 0.f, 20));
	grip_axis->SetRelativeScale3D(FVector(1.f, 1.f, 1.f));
	grip_axis->SetSphereRadius(30.f);
	grip_axis->SetSimulatePhysics(true);
	grip_axis->SetEnableGravity(false);
	grip_axis->SetMassOverrideInKg(NAME_None, 20.f, true);
	grip_axis->SetPhysicsMaxAngularVelocity(5000.f);
	grip_axis_vis = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("WeaponAxisVis"));
	grip_axis_vis->SetupAttachment(grip_axis);
	grip_axis_attachment = CreateDefaultSubobject<UPhysicsConstraintComponent>(TEXT("GripAxisAttachment"));
	grip_axis_attachment->SetupAttachment(grip_axis);
	grip_axis_attachment->SetRelativeLocation(FVector(0.f, 0.f, 0.f));

	grip = CreateDefaultSubobject<USphereComponent>(TEXT("WeaponHandle1"));
	grip->SetupAttachment(grip_axis);
	grip->SetRelativeLocation(FVector(0.f, 0.f, 0.f));
	grip->SetRelativeScale3D(FVector(1.f, 1.f, 1.f));
	grip->SetSphereRadius(20.f);
	grip->SetSimulatePhysics(true);
	grip->SetEnableGravity(false);
	grip->SetMassOverrideInKg(NAME_None, 15.f, true);
	grip->SetPhysicsMaxAngularVelocity(5000.f);
	grip_vis = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("WeaponHandle1Vis"));
	grip_vis->SetupAttachment(grip);
	
	grip_attachment = CreateDefaultSubobject<UPhysicsConstraintComponent>(TEXT("GripAttachment"));
	grip_attachment->SetupAttachment(grip);
	grip_attachment->SetRelativeLocation(FVector(0.f, 0.f, 0.f));

	wep_attachment = CreateDefaultSubobject<UPhysicsConstraintComponent>(TEXT("WeaponAttachment"));
	wep_attachment->SetupAttachment(grip);
	wep_attachment->SetRelativeLocation(FVector(0.f, 0.f, 0.f));
	
}

void AJointCharacterTest::initWeaponJoints()
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

	grip_axis_attachment->SetConstrainedComponents(torso, NAME_None, grip_axis, NAME_None);
	grip_axis_attachment->SetLinearXLimit(ELinearConstraintMotion::LCM_Locked, 4.0f);
	grip_axis_attachment->SetLinearYLimit(ELinearConstraintMotion::LCM_Locked, 4.0f);
	grip_axis_attachment->SetLinearZLimit(ELinearConstraintMotion::LCM_Locked, 4.0f);
	grip_axis_attachment->SetAngularSwing1Limit(EAngularConstraintMotion::ACM_Free, 0.0f);
	grip_axis_attachment->SetAngularSwing2Limit(EAngularConstraintMotion::ACM_Free, 0.0f);
	grip_axis_attachment->SetAngularTwistLimit(EAngularConstraintMotion::ACM_Free, 0.0f);
	grip_axis_attachment->SetDisableCollision(true);

	grip_attachment->SetConstrainedComponents(grip_axis, NAME_None, grip, NAME_None);
	grip_attachment->SetLinearXLimit(ELinearConstraintMotion::LCM_Limited, 0.0f);
	grip_attachment->SetLinearYLimit(ELinearConstraintMotion::LCM_Limited, 0.0f);
	grip_attachment->SetLinearZLimit(ELinearConstraintMotion::LCM_Limited, 0.0f);
	grip_attachment->SetAngularSwing1Limit(EAngularConstraintMotion::ACM_Free, 0.0f);
	grip_attachment->SetAngularSwing2Limit(EAngularConstraintMotion::ACM_Free, 0.0f);
	grip_attachment->SetAngularTwistLimit(EAngularConstraintMotion::ACM_Free, 0.0f);
	grip_attachment->SetDisableCollision(true);

	//wep_attachment->SetConstrainedComponents(grip_axis, NAME_None, grip, NAME_None);
	wep_attachment->SetLinearXLimit(ELinearConstraintMotion::LCM_Locked, 0.0f);
	wep_attachment->SetLinearYLimit(ELinearConstraintMotion::LCM_Locked, 0.0f);
	wep_attachment->SetLinearZLimit(ELinearConstraintMotion::LCM_Locked, 0.0f);
	wep_attachment->SetAngularSwing1Limit(EAngularConstraintMotion::ACM_Free, 0.0f);
	wep_attachment->SetAngularSwing2Limit(EAngularConstraintMotion::ACM_Locked, 0.0f);
	wep_attachment->SetAngularTwistLimit(EAngularConstraintMotion::ACM_Locked, 0.0f);
	wep_attachment->SetDisableCollision(true);
}

void AJointCharacterTest::initLegs()
{
	pelvis = CreateDefaultSubobject<UCapsuleComponent>(TEXT("PelvisCol"));
	pelvis->SetupAttachment(RootComponent);
	pelvis->SetRelativeLocation(FVector(0.f, 0.f, 0));
	pelvis->SetRelativeScale3D(FVector(1.f, 1.f, 1.f));
	pelvis->GetBodyInstance()->bLockXRotation = true;
	pelvis->GetBodyInstance()->bLockYRotation = true;
	pelvis->GetBodyInstance()->bLockZRotation = true;

	right_leg_axis = CreateDefaultSubobject<USceneComponent>(TEXT("RightLegAxis"));
	right_leg_axis->SetupAttachment(pelvis);

	left_leg_axis = CreateDefaultSubobject<USceneComponent>(TEXT("LeftLegAxis"));
	left_leg_axis->SetupAttachment(pelvis);
	
	pelvis_attachment = CreateDefaultSubobject<UPhysicsConstraintComponent>(TEXT("PelvisAttachment"));
	pelvis_attachment->SetupAttachment(pelvis);
	pelvis_attachment->SetRelativeLocation(FVector(0.f, 0.f, 0.f));

	pelvis_visu = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("PelvisVisualization"));
	pelvis_visu->SetupAttachment(pelvis);
	// Right ----------------------------------------------------------------------------------------------
	r_hip_motor = CreateDefaultSubobject<USphereComponent>(TEXT("RHipCol"));
	r_hip_motor->SetupAttachment(right_leg_axis);
	r_hip_motor->SetRelativeLocation(FVector(0.f, 0.f, 0));
	r_hip_motor->SetRelativeScale3D(FVector(1.f, 1.f, 1.f));
	r_hip_motor_vis = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("RHip"));
	r_hip_motor_vis->SetupAttachment(r_hip_motor);
	r_hip_attachment = CreateDefaultSubobject<UPhysicsConstraintComponent>(TEXT("RHipAttachment"));
	r_hip_attachment->SetupAttachment(r_hip_motor);
	r_hip_attachment->SetRelativeLocation(FVector(0.f, 0.f, 0.f));
	
	r_thigh = CreateDefaultSubobject<UCapsuleComponent>(TEXT("RThighCol"));
	r_thigh->SetupAttachment(right_leg_axis);
	r_thigh->SetRelativeLocation(FVector(0.f, 0.f, 0));
	r_thigh->SetRelativeScale3D(FVector(1.f, 1.f, 1.f));
	r_thigh_vis = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("RThigh"));
	r_thigh_vis->SetupAttachment(r_thigh);
	r_thigh_attachment = CreateDefaultSubobject<UPhysicsConstraintComponent>(TEXT("RThighAttachment"));
	r_thigh_attachment->SetupAttachment(r_hip_motor);
	r_thigh_attachment->SetRelativeLocation(FVector(0.f, 0.f, 0.f));

	r_knee_motor = CreateDefaultSubobject<USphereComponent>(TEXT("RKneeCol"));
	r_knee_motor->SetupAttachment(right_leg_axis);
	r_knee_motor->SetRelativeLocation(FVector(0.f, 0.f, 0));
	r_knee_motor->SetRelativeScale3D(FVector(1.f, 1.f, 1.f));
	r_knee_motor_vis = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("RKnee"));
	r_knee_motor_vis->SetupAttachment(r_knee_motor);
	r_knee_attachment = CreateDefaultSubobject<UPhysicsConstraintComponent>(TEXT("RKneeAttachment"));
	r_knee_attachment->SetupAttachment(r_knee_motor);
	r_knee_attachment->SetRelativeLocation(FVector(0.f, 0.f, 0.f));

	r_shin = CreateDefaultSubobject<UCapsuleComponent>(TEXT("RShinCol"));
	r_shin->SetupAttachment(right_leg_axis);
	r_shin->SetRelativeLocation(FVector(0.f, 0.f, 0));
	r_shin->SetRelativeScale3D(FVector(1.f, 1.f, 1.f));
	r_shin_vis = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("RShin"));
	r_shin_vis->SetupAttachment(r_shin);
	r_shin_attachment = CreateDefaultSubobject<UPhysicsConstraintComponent>(TEXT("RShinAttachment"));
	r_shin_attachment->SetupAttachment(r_shin);
	r_shin_attachment->SetRelativeLocation(FVector(0.f, 0.f, 0.f));

	r_toe = CreateDefaultSubobject<UCapsuleComponent>(TEXT("RToeCol"));
	r_toe->SetupAttachment(r_shin);
	r_toe->SetRelativeLocation(FVector(0.f, 0.f, 0));
	r_toe->SetRelativeScale3D(FVector(1.f, 1.f, 1.f));
	r_toe_vis = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("RToeVisualization"));
	r_toe_vis->SetupAttachment(r_toe);
	//r_toe_attachment = CreateDefaultSubobject<UPhysicsConstraintComponent>(TEXT("RToeAttachment"));
	//r_toe_attachment->SetupAttachment(r_toe);
	//r_toe_attachment->SetRelativeLocation(FVector(0.f, 0.f, 0.f));

	// LEFT ----------------------------------------------------------------------------------------------
	l_hip_motor = CreateDefaultSubobject<USphereComponent>(TEXT("LHipCol"));
	l_hip_motor->SetupAttachment(left_leg_axis);
	l_hip_motor->SetRelativeLocation(FVector(0.f, 0.f, 0));
	l_hip_motor->SetRelativeScale3D(FVector(1.f, 1.f, 1.f));
	l_hip_motor_vis = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("LHip"));
	l_hip_motor_vis->SetupAttachment(l_hip_motor);
	l_hip_attachment = CreateDefaultSubobject<UPhysicsConstraintComponent>(TEXT("LHipAttachment"));
	l_hip_attachment->SetupAttachment(l_hip_motor);
	l_hip_attachment->SetRelativeLocation(FVector(0.f, 0.f, 0.f));

	l_thigh = CreateDefaultSubobject<UCapsuleComponent>(TEXT("LThighCol"));
	l_thigh->SetupAttachment(left_leg_axis);
	l_thigh->SetRelativeLocation(FVector(0.f, 0.f, 0));
	l_thigh->SetRelativeScale3D(FVector(1.f, 1.f, 1.f));
	l_thigh_vis = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("LThigh"));
	l_thigh_vis->SetupAttachment(l_thigh);
	l_thigh_attachment = CreateDefaultSubobject<UPhysicsConstraintComponent>(TEXT("LThighAttachment"));
	l_thigh_attachment->SetupAttachment(l_hip_motor);
	l_thigh_attachment->SetRelativeLocation(FVector(0.f, 0.f, 0.f));

	l_knee_motor = CreateDefaultSubobject<USphereComponent>(TEXT("LKneeCol"));
	l_knee_motor->SetupAttachment(left_leg_axis);
	l_knee_motor->SetRelativeLocation(FVector(0.f, 0.f, 0));
	l_knee_motor->SetRelativeScale3D(FVector(1.f, 1.f, 1.f));
	l_knee_motor_vis = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("LKnee"));
	l_knee_motor_vis->SetupAttachment(l_knee_motor);
	l_knee_attachment = CreateDefaultSubobject<UPhysicsConstraintComponent>(TEXT("LKneeAttachment"));
	l_knee_attachment->SetupAttachment(l_knee_motor);
	l_knee_attachment->SetRelativeLocation(FVector(0.f, 0.f, 0.f));

	l_shin = CreateDefaultSubobject<UCapsuleComponent>(TEXT("LShinCol"));
	l_shin->SetupAttachment(left_leg_axis);
	l_shin->SetRelativeLocation(FVector(0.f, 0.f, 0));
	l_shin->SetRelativeScale3D(FVector(1.f, 1.f, 1.f));
	l_shin_vis = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("LShin"));
	l_shin_vis->SetupAttachment(l_shin);
	l_shin_attachment = CreateDefaultSubobject<UPhysicsConstraintComponent>(TEXT("LShinAttachment"));
	l_shin_attachment->SetupAttachment(l_shin);
	l_shin_attachment->SetRelativeLocation(FVector(0.f, 0.f, 0.f));

	l_toe = CreateDefaultSubobject<UCapsuleComponent>(TEXT("LToeCol"));
	l_toe->SetupAttachment(l_shin);
	l_toe->SetRelativeLocation(FVector(0.f, 0.f, 0));
	l_toe->SetRelativeScale3D(FVector(1.f, 1.f, 1.f));
	l_toe_vis = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("LToeVisualization"));
	l_toe_vis->SetupAttachment(l_toe);
	//l_toe_attachment = CreateDefaultSubobject<UPhysicsConstraintComponent>(TEXT("LToeAttachment"));
	//l_toe_attachment->SetupAttachment(l_toe);
	//l_toe_attachment->SetRelativeLocation(FVector(0.f, 0.f, 0.f));
}

void AJointCharacterTest::initLegJoints()
{
	pelvis_attachment->SetConstrainedComponents(pelvis, NAME_None, spine, NAME_None);
	pelvis_attachment->SetLinearXLimit(ELinearConstraintMotion::LCM_Locked, 4.0f);
	pelvis_attachment->SetLinearYLimit(ELinearConstraintMotion::LCM_Locked, 4.0f);
	pelvis_attachment->SetLinearZLimit(ELinearConstraintMotion::LCM_Locked, 4.0f);
	pelvis_attachment->SetAngularSwing1Limit(EAngularConstraintMotion::ACM_Limited, 90.0f);
	pelvis_attachment->SetAngularSwing2Limit(EAngularConstraintMotion::ACM_Locked, 0.0f);
	pelvis_attachment->SetAngularTwistLimit(EAngularConstraintMotion::ACM_Locked, 0.0f);
	pelvis_attachment->SetDisableCollision(true);

	// RIGHT ----------------------------------------------------------------------------------------------
	r_hip_attachment->SetConstrainedComponents(pelvis, NAME_None, r_hip_motor, NAME_None);
	r_hip_attachment->SetLinearXLimit(ELinearConstraintMotion::LCM_Limited, 0.0f);
	r_hip_attachment->SetLinearYLimit(ELinearConstraintMotion::LCM_Limited, 0.0f);
	r_hip_attachment->SetLinearZLimit(ELinearConstraintMotion::LCM_Limited, 0.0f);
	r_hip_attachment->ConstraintInstance.bScaleLinearLimits = false;
	r_hip_attachment->ConstraintInstance.ProfileInstance.LinearLimit.bSoftConstraint = true;
	r_hip_attachment->ConstraintInstance.ProfileInstance.LinearLimit.Stiffness = 100.0f;
	r_hip_attachment->ConstraintInstance.ProfileInstance.LinearLimit.Damping = 10.0f;
	r_hip_attachment->SetAngularSwing1Limit(EAngularConstraintMotion::ACM_Locked, 45.0f);
	r_hip_attachment->SetAngularSwing2Limit(EAngularConstraintMotion::ACM_Locked, 0.0f);
	r_hip_attachment->SetAngularTwistLimit(EAngularConstraintMotion::ACM_Limited, 100);
	r_hip_attachment->ConstraintInstance.AngularRotationOffset = FRotator(0.f, 0.f, 45.0f);
	r_hip_attachment->ConstraintInstance.ProfileInstance.TwistLimit.bSoftConstraint = true;
	r_hip_attachment->ConstraintInstance.ProfileInstance.TwistLimit.Stiffness = 50;
	r_hip_attachment->ConstraintInstance.ProfileInstance.TwistLimit.Damping = 5;
	r_hip_attachment->SetDisableCollision(true);

	/*r_hip_attachment->SetAngularDriveMode(EAngularDriveMode::TwistAndSwing);
	r_hip_attachment->SetAngularVelocityDriveTwistAndSwing(true, true);
	r_hip_attachment->SetAngularDriveParams(0.f, 10000.f, 10000000000000000000000.f);*/

	r_thigh_attachment->SetConstrainedComponents(r_hip_motor, NAME_None, r_thigh, NAME_None);
	r_thigh_attachment->SetLinearXLimit(ELinearConstraintMotion::LCM_Locked, 0.0f);
	r_thigh_attachment->SetLinearYLimit(ELinearConstraintMotion::LCM_Locked, 0.0f);
	r_thigh_attachment->SetLinearZLimit(ELinearConstraintMotion::LCM_Locked, 0.0f);
	r_thigh_attachment->SetAngularSwing1Limit(EAngularConstraintMotion::ACM_Locked, 0.0f);
	r_thigh_attachment->SetAngularSwing2Limit(EAngularConstraintMotion::ACM_Limited, 80.f);
	r_thigh_attachment->SetAngularTwistLimit(EAngularConstraintMotion::ACM_Locked, 80.f);
	r_thigh_attachment->ConstraintInstance.AngularRotationOffset = FRotator(0.f, 0.f, 0.f);
	r_thigh_attachment->SetDisableCollision(true);


	r_knee_attachment->SetConstrainedComponents(r_thigh, NAME_None, r_knee_motor, NAME_None);
	r_knee_attachment->SetLinearXLimit(ELinearConstraintMotion::LCM_Limited, 0.0f);
	r_knee_attachment->SetLinearYLimit(ELinearConstraintMotion::LCM_Limited, 0.0f);
	r_knee_attachment->SetLinearZLimit(ELinearConstraintMotion::LCM_Limited, 0.0f);
	r_knee_attachment->ConstraintInstance.bScaleLinearLimits = false;
	r_knee_attachment->SetAngularSwing1Limit(EAngularConstraintMotion::ACM_Locked, 0.0f);
	r_knee_attachment->SetAngularSwing2Limit(EAngularConstraintMotion::ACM_Locked, 0.0f);
	r_knee_attachment->SetAngularTwistLimit(EAngularConstraintMotion::ACM_Limited, 85.0f);
	r_knee_attachment->ConstraintInstance.AngularRotationOffset = FRotator(0.f, 0.f, -90.f);
	r_knee_attachment->ConstraintInstance.ProfileInstance.TwistLimit.bSoftConstraint = true;
	r_knee_attachment->ConstraintInstance.ProfileInstance.TwistLimit.Stiffness = 1000;
	r_knee_attachment->ConstraintInstance.ProfileInstance.TwistLimit.Damping = 100;
	r_knee_attachment->ConstraintInstance.ProfileInstance.TwistLimit.ContactDistance = 45.0f;
	r_knee_attachment->SetDisableCollision(true);
	/*r_knee_attachment->SetAngularDriveMode(EAngularDriveMode::TwistAndSwing);
	r_knee_attachment->SetAngularVelocityDriveTwistAndSwing(true, true);
	r_knee_attachment->SetAngularDriveParams(0.f, 10000.f, 10000000000000000000000.f);*/

	r_shin_attachment->SetConstrainedComponents(r_knee_motor, NAME_None, r_shin, NAME_None);
	r_shin_attachment->SetLinearXLimit(ELinearConstraintMotion::LCM_Locked, 0.0f);
	r_shin_attachment->SetLinearYLimit(ELinearConstraintMotion::LCM_Locked, 0.0f);
	r_shin_attachment->SetLinearZLimit(ELinearConstraintMotion::LCM_Locked, 0.0f);
	r_shin_attachment->SetAngularSwing1Limit(EAngularConstraintMotion::ACM_Locked, 0.0f);
	r_shin_attachment->SetAngularSwing2Limit(EAngularConstraintMotion::ACM_Locked, 00.f);
	r_shin_attachment->SetAngularTwistLimit(EAngularConstraintMotion::ACM_Locked, 0.0f);
	r_shin_attachment->SetDisableCollision(true);


	//r_toe_attachment->SetConstrainedComponents(r_shin, NAME_None, r_toe, NAME_None);
	//r_toe_attachment->SetLinearXLimit(ELinearConstraintMotion::LCM_Locked, 0.0f);
	//r_toe_attachment->SetLinearYLimit(ELinearConstraintMotion::LCM_Locked, 0.0f);
	//r_toe_attachment->SetLinearZLimit(ELinearConstraintMotion::LCM_Locked, 0.0f);
	//r_toe_attachment->ConstraintInstance.bScaleLinearLimits = false;
	//r_toe_attachment->SetAngularSwing1Limit(EAngularConstraintMotion::ACM_Locked, 0.0f);
	//r_toe_attachment->SetAngularSwing2Limit(EAngularConstraintMotion::ACM_Locked, 0.0f);
	//r_toe_attachment->SetAngularTwistLimit(EAngularConstraintMotion::ACM_Free, 85.0f);
	//r_toe_attachment->ConstraintInstance.AngularRotationOffset = FRotator(0.f, 0.f, 0.f);
	//r_toe_attachment->SetDisableCollision(true);

	// LEFT ----------------------------------------------------------------------------------------------

	l_hip_attachment->SetConstrainedComponents(pelvis, NAME_None, l_hip_motor, NAME_None);
	l_hip_attachment->SetLinearXLimit(ELinearConstraintMotion::LCM_Limited, 0.0f);
	l_hip_attachment->SetLinearYLimit(ELinearConstraintMotion::LCM_Limited, 0.0f);
	l_hip_attachment->SetLinearZLimit(ELinearConstraintMotion::LCM_Limited, 0.0f);
	l_hip_attachment->ConstraintInstance.bScaleLinearLimits = false;
	l_hip_attachment->ConstraintInstance.ProfileInstance.LinearLimit.bSoftConstraint = true;
	l_hip_attachment->ConstraintInstance.ProfileInstance.LinearLimit.Stiffness = 100.0f;
	l_hip_attachment->ConstraintInstance.ProfileInstance.LinearLimit.Damping = 10.0f;
	l_hip_attachment->SetAngularSwing1Limit(EAngularConstraintMotion::ACM_Locked, 45.0f);
	l_hip_attachment->SetAngularSwing2Limit(EAngularConstraintMotion::ACM_Locked, 0.0f);
	l_hip_attachment->SetAngularTwistLimit(EAngularConstraintMotion::ACM_Limited, 100);
	l_hip_attachment->ConstraintInstance.AngularRotationOffset = FRotator(0.f, 0.f, 45.f);
	l_hip_attachment->ConstraintInstance.ProfileInstance.TwistLimit.bSoftConstraint = true;
	l_hip_attachment->ConstraintInstance.ProfileInstance.TwistLimit.Stiffness = 50;
	l_hip_attachment->ConstraintInstance.ProfileInstance.TwistLimit.Damping = 5;
	l_hip_attachment->SetDisableCollision(true);

	/*l_hip_attachment->SetAngularDriveMode(EAngularDriveMode::TwistAndSwing);
	l_hip_attachment->SetAngularVelocityDriveTwistAndSwing(true, true);
	l_hip_attachment->SetAngularDriveParams(0.f, 10000.f, 10000000000000000000000.f);*/

	l_thigh_attachment->SetConstrainedComponents(l_hip_motor, NAME_None, l_thigh, NAME_None);
	l_thigh_attachment->SetLinearXLimit(ELinearConstraintMotion::LCM_Locked, 0.0f);
	l_thigh_attachment->SetLinearYLimit(ELinearConstraintMotion::LCM_Locked, 0.0f);
	l_thigh_attachment->SetLinearZLimit(ELinearConstraintMotion::LCM_Locked, 0.0f);
	l_thigh_attachment->SetAngularSwing1Limit(EAngularConstraintMotion::ACM_Locked, 0.0f);
	l_thigh_attachment->SetAngularSwing2Limit(EAngularConstraintMotion::ACM_Limited, 80.f);
	l_thigh_attachment->SetAngularTwistLimit(EAngularConstraintMotion::ACM_Locked, 80.f);
	l_thigh_attachment->ConstraintInstance.AngularRotationOffset = FRotator(0.f, 0.f, 0.f);
	l_thigh_attachment->SetDisableCollision(true);

	l_knee_attachment->SetConstrainedComponents(l_thigh, NAME_None, l_knee_motor, NAME_None);
	l_knee_attachment->SetLinearXLimit(ELinearConstraintMotion::LCM_Limited, 0.0f);
	l_knee_attachment->SetLinearYLimit(ELinearConstraintMotion::LCM_Limited, 0.0f);
	l_knee_attachment->SetLinearZLimit(ELinearConstraintMotion::LCM_Limited, 0.0f);
	l_knee_attachment->ConstraintInstance.bScaleLinearLimits = false;
	l_knee_attachment->SetAngularSwing1Limit(EAngularConstraintMotion::ACM_Locked, 0.0f);
	l_knee_attachment->SetAngularSwing2Limit(EAngularConstraintMotion::ACM_Locked, 0.0f);
	l_knee_attachment->SetAngularTwistLimit(EAngularConstraintMotion::ACM_Limited, 85.0f);
	l_knee_attachment->ConstraintInstance.AngularRotationOffset = FRotator(0.f, 0.f, -90.f);
	l_knee_attachment->ConstraintInstance.ProfileInstance.TwistLimit.bSoftConstraint = true;
	l_knee_attachment->ConstraintInstance.ProfileInstance.TwistLimit.Stiffness = 1000;
	l_knee_attachment->ConstraintInstance.ProfileInstance.TwistLimit.Damping = 100;
	l_knee_attachment->ConstraintInstance.ProfileInstance.TwistLimit.ContactDistance = 45.0f;
	l_knee_attachment->SetDisableCollision(true);
	/*l_knee_attachment->SetAngularDriveMode(EAngularDriveMode::TwistAndSwing);
	l_knee_attachment->SetAngularVelocityDriveTwistAndSwing(true, true);
	l_knee_attachment->SetAngularDriveParams(0.f, 10000.f, 10000000000000000000000.f);*/

	l_shin_attachment->SetConstrainedComponents(l_knee_motor, NAME_None, l_shin, NAME_None);
	l_shin_attachment->SetLinearXLimit(ELinearConstraintMotion::LCM_Locked, 0.0f);
	l_shin_attachment->SetLinearYLimit(ELinearConstraintMotion::LCM_Locked, 0.0f);
	l_shin_attachment->SetLinearZLimit(ELinearConstraintMotion::LCM_Locked, 0.0f);
	l_shin_attachment->SetAngularSwing1Limit(EAngularConstraintMotion::ACM_Locked, 0.0f);
	l_shin_attachment->SetAngularSwing2Limit(EAngularConstraintMotion::ACM_Locked, 00.f);
	l_shin_attachment->SetAngularTwistLimit(EAngularConstraintMotion::ACM_Locked, 0.0f);
	l_shin_attachment->SetDisableCollision(true);

	//l_toe_attachment->SetConstrainedComponents(l_shin, NAME_None, l_toe, NAME_None);
	//l_toe_attachment->SetLinearXLimit(ELinearConstraintMotion::LCM_Locked, 0.0f);
	//l_toe_attachment->SetLinearYLimit(ELinearConstraintMotion::LCM_Locked, 0.0f);
	//l_toe_attachment->SetLinearZLimit(ELinearConstraintMotion::LCM_Locked, 0.0f);
	//l_toe_attachment->ConstraintInstance.bScaleLinearLimits = false;
	//l_toe_attachment->SetAngularSwing1Limit(EAngularConstraintMotion::ACM_Locked, 0.0f);
	//l_toe_attachment->SetAngularSwing2Limit(EAngularConstraintMotion::ACM_Locked, 0.0f);
	//l_toe_attachment->SetAngularTwistLimit(EAngularConstraintMotion::ACM_Free, 85.0f);
	//l_toe_attachment->ConstraintInstance.AngularRotationOffset = FRotator(0.f, 0.f, 0.f);
	//l_toe_attachment->SetDisableCollision(true);
}

void AJointCharacterTest::initPIDs()
{
	//camera ------------------------------------------------
	axis_target_rot = FRotator(0.f, 150.f, 0.f);
	arm_target_rot = FRotator(-35.f, 0.f, 0.f);
	target_arm_length = 300.f;

	cdc.target = FVector2D(0.f, 0.f);
	cdc.max_adjustment = FVector2D(0.f, 0.f);
	cdc.P = FVector2D(10.f, 10.f);
	cdc.I = FVector2D(0.f, 0.f);
	cdc.D = FVector2D(0.005f, 0.005f);
	cdc.integral = FVector2D::ZeroVector;

	//hovering ------------------------------------------------
	hover_height.target = target_hover_height;
	hover_height.max_adjustment = 1000000;
	hover_height.P = 300.f;
	hover_height.I = 5;
	hover_height.D = 10.0f;
	hover_height.integral = 0.f;
	hover_height.prev_err = 0.f;

	//arm and weapon ------------------------------------------------
	target_wep_dir = FVector::UpVector;
	weapon_twist_solder = FVector::UpVector;
	weapon_twist_target = FVector::RightVector;
	prev_target_wep_dir = FVector::ForwardVector;
	prev_target_wep_dir_xy = FVector::ForwardVector;
	target_arm_dir = FVector::UpVector;
	prev_target_arm_dir_xy = FVector::ForwardVector;

	adc.target = FVector2D(0.f, 0.f);
	adc.max_adjustment = FVector2D(0.f, 0.f);
	adc.P = FVector2D(600.f, 10.f);
	adc.I = FVector2D(0.f, 0.f);
	adc.D = FVector2D(6.f, 0.f);
	adc.integral = FVector2D::ZeroVector;
	
	atc.target = 0.0f;
	atc.max_adjustment = 5;
	atc.P = 500;
	atc.I = 0.0f;
	atc.D = 100.1f;
	atc.integral = 0.f;

	gdc.target = FVector::ZeroVector;
	gdc.max_adjustment = FVector::ZeroVector;
	gdc.P = FVector(500.f, 10.f, 10000000.f);
	gdc.I = FVector::ZeroVector;
	gdc.D = FVector(10.0f, 0.f, 0.25f);
	gdc.integral = FVector::ZeroVector;

	wtc.target = 0.0f;
	wtc.max_adjustment = 5;
	wtc.P = 150.f;
	wtc.I = 0.0f;
	wtc.D = 60.1f;
	wtc.integral = 0.f;

	//grabbing  ------------------------------------------------

	wgdc.target = FVector2D(0.f, 0.f);
	wgdc.max_adjustment = FVector2D(0.f, 0.f);
	wgdc.P = FVector2D(200, 4.f);
	wgdc.I = FVector2D(0.f, 0.f);
	wgdc.D = FVector2D(4.f, 0.f);
	wgdc.integral = FVector2D::ZeroVector;

	wgc.max_adjustment = FVector(100000.f, 100000.f, 100000.f);
	wgc.P = FVector(16.f, 16.f, 16.f);
	wgc.I = FVector(0.f, 0.f, 0.f);
	wgc.D = FVector(12.f, 12.f, 12.f);
	wgc.integral = FVector::ZeroVector;

	

	//movement  ------------------------------------------------

	movement_velocity.max_adjustment = FVector2D(1000000.f, 1000000.f);
	movement_velocity.P = FVector2D(5.f, 5.f);
	movement_velocity.I = FVector2D(0.f, 0.f);
	movement_velocity.D = FVector2D(0.1f, 0.1f);
	movement_velocity.integral = FVector2D::ZeroVector;

	pose_controller.max_adjustment = FVector2D(1000000.f, 1000000.f);
	pose_controller.target = FVector2D::ZeroVector;
	pose_controller.P = FVector2D(0.1f, 0.1f);
	pose_controller.I = FVector2D(0.f, 0.f);
	pose_controller.D = FVector2D(0.01f, 0.01f);
	pose_controller.integral = FVector2D::ZeroVector;

	ssc.max_adjustment = FVector(1000000.f, 1000000.f, 1000000.f);
	ssc.target = FVector::UpVector;
	ssc.P = FVector(10000.f, 1000.f, 1000.f);
	ssc.I = FVector(0.f, 0.f, 0.f);
	ssc.D = FVector(1.f, 1.f, 1.f);
	ssc.integral = FVector::ZeroVector;

	ptc.max_adjustment = FVector(1000000.f, 1000000.f, 1000000.f);
	ptc.target = FVector::UpVector;
	ptc.P = FVector(5000.f, 5000.f, 5000.f);
	ptc.I = FVector(0.f, 0.f, 0.f);
	ptc.D = FVector(100.f, 100.f, 100.f);
	ptc.integral = FVector::ZeroVector;

	ttc.max_adjustment = FVector(1000000.f, 1000000.f, 1000000.f);
	ttc.target = FVector::UpVector;
	ttc.P = FVector(5000.f, 5000.f, 5000.f);
	ttc.I = FVector(0.f, 0.f, 0.f);
	ttc.D = FVector(100.f, 100.f, 100.f);
	ttc.integral = FVector::ZeroVector;

	rtdc.max_adjustment = FVector(1000000.f, 1000000.f, 1000000.f);
	rtdc.target = -FVector::ForwardVector;
	//r_thigh_direction_controller.target = -FVector::RightVector;
	//r_thigh_direction_controller.target = FVector::ForwardVector;
	rtdc.P = FVector(100000.1f, 100000.1f, 100000.1f);
	rtdc.I = FVector(0.f, 0.f, 0.f);
	rtdc.D = FVector(1000.01f, 1000.01f, 1000.01f);
	rtdc.integral = FVector::ZeroVector;

	ltdc.max_adjustment = FVector(1000000.f, 1000000.f, 1000000.f);
	ltdc.target = -FVector::ForwardVector;
	//l_thigh_direction_controller.target = -FVector::RightVector;
	//l_thigh_direction_controller.target = -FVector::ForwardVector;
	ltdc.P = FVector(100000.1f, 100000.1f, 100000.1f);
	ltdc.I = FVector(0.f, 0.f, 0.f);
	ltdc.D = FVector(1000.01f, 1000.01f, 1000.01f);
	ltdc.integral = FVector::ZeroVector;

	rkec.target = 53;
	rkec.max_adjustment = 1000000;
	rkec.P = 200;
	rkec.I = 0.0f;
	rkec.D = 0.f;
	rkec.integral = 0.f;

	lkec.target = 53;
	lkec.max_adjustment = 1000000;
	lkec.P = 200	;
	lkec.I = 0.0f;
	lkec.D = 0.f;
	lkec.integral = 0.f;
}

void AJointCharacterTest::initCustomPhysics()
{
	OnCalculateCustomHoverPhysics.BindUObject(this, &AJointCharacterTest::customHoverPhysics);
	OnCalculateCustomStabilizerPhysics.BindUObject(this, &AJointCharacterTest::customStabilizerPhysics);
	torso_bi = torso->GetBodyInstance();
	spine_bi = spine->GetBodyInstance();

	OnCalculateControlGripPhysics.BindUObject(this, &AJointCharacterTest::ControlGripPhysics);
	OnCalculateCustomInitGripPhysics.BindUObject(this, &AJointCharacterTest::customInitGripPhysics);
	OnCalculateControlArmDirectionPhysics.BindUObject(this, &AJointCharacterTest::ControlArmDirectionPhysics);
	OnCalculateControlGripDirectionPhysics.BindUObject(this, &AJointCharacterTest::ControlGripDirectionPhysics);
	OnCalculateControlGripInclinePhysics.BindUObject(this, &AJointCharacterTest::ControlGripInclinePhysics);
	OnCalculateControlWeaponTwistPhysics.BindUObject(this, &AJointCharacterTest::ControlWeaponTwistPhysics);
	OnCalculateControlArmTwistPhysics.BindUObject(this, &AJointCharacterTest::ControlArmTwistPhysics);


	OnCalculateWeaponGrabControl.BindUObject(this, &AJointCharacterTest::weaponGrabControl);

	grip_axis_bi = grip_axis->GetBodyInstance();
	grip_bi = grip->GetBodyInstance();
	g_pos_offset = FVector::ZeroVector;

	ga_prev_up = FVector::ZeroVector;
	g_prev_up = FVector::ZeroVector;
	w_prev_up = FVector::ZeroVector;
	//weapon_handle_2_bi = weapon_handle_2->GetBodyInstance();
	
	calculateWepInertia();

	//rolling_body->OnComponentHit.AddDynamic(this, &AJointCharacterTest::OnHit);
	torso->OnComponentHit.AddDynamic(this, &AJointCharacterTest::OnBodyHit);
	spine->OnComponentHit.AddDynamic(this, &AJointCharacterTest::OnBodyHit);
	targeting_sphere->OnComponentBeginOverlap.AddDynamic(this, &AJointCharacterTest::addPotentialTarget);
	targeting_sphere->OnComponentEndOverlap.AddDynamic(this, &AJointCharacterTest::removePotentialTarget);

	OnCalculateCustomWalkingPhysics.BindUObject(this, &AJointCharacterTest::customWalkingPhysics);
	pelvis_bi = pelvis->GetBodyInstance();
	r_hip_motor_bi = r_hip_motor->GetBodyInstance();
	r_thigh_bi = r_thigh->GetBodyInstance();
	r_knee_motor_bi = r_knee_motor->GetBodyInstance();
	r_shin_bi = r_shin->GetBodyInstance();
	r_toe_bi = r_toe->GetBodyInstance();

	l_hip_motor_bi = l_hip_motor->GetBodyInstance();
	l_thigh_bi = l_thigh->GetBodyInstance();
	l_knee_motor_bi = l_knee_motor->GetBodyInstance();
	l_shin_bi = l_shin->GetBodyInstance();
	l_toe_bi = l_toe->GetBodyInstance();
	
	grip_bi->SetBodyTransform(FTransform(grip_bi->GetUnrealWorldTransform().GetTranslation() + FVector::UpVector*70.f), true);
	//grip_v_bi->SetBodyTransform(FTransform(grip_v_bi->GetUnrealWorldTransform().GetTranslation() + FVector::UpVector*70.f), true);
}

void AJointCharacterTest::calculateWepInertia()
{
	FVector ti = grip_bi->GetBodyInertiaTensor();
	//FVector tp = FVector(0.f, 0.f, 75.f);
	
	FVector tp = grip->ComponentToWorld.InverseTransformPositionNoScale(grip_bi->GetCOMPosition());// -grip_v->GetComponentLocation();
	offset_wep_inertia = FVector(ti.X + grip_bi->GetBodyMass()*(tp.Y*tp.Y + tp.Z*tp.Z),
		ti.Y + grip_bi->GetBodyMass()*(tp.X*tp.X + tp.Z*tp.Z),
		ti.Z + grip_bi->GetBodyMass()*(tp.X*tp.X + tp.Y*tp.Y));

	UE_LOG(LogTemp, Warning, TEXT("offset inertia: %s"), *offset_wep_inertia.ToString());
}

void AJointCharacterTest::calculateRelativeInertia(FBodyInstance* offset_bi, const FVector& cor, FMatrix* out_inertia)
{
	FVector bi_inrt = offset_bi->GetBodyInertiaTensor();
	FMatrix bi_T = offset_bi->GetUnrealWorldTransform().ToMatrixWithScale();
	bi_T.ScaleTranslation(FVector(0.f, 0.f, 0.f));
	FMatrix bi_di = FMatrix(FVector(bi_inrt.X, 0.f, 0.f), FVector(0.f, bi_inrt.Y, 0.f), FVector(0.f, 0.f, bi_inrt.Z), FVector(0.f, 0.f, 0.f));
	*out_inertia = bi_T*bi_di*bi_T.GetTransposed();
	FVector r = offset_bi->GetCOMPosition() - cor;
	float bi_m = offset_bi->GetBodyMass();
	out_inertia->M[0][0] = out_inertia->M[0][0] + bi_m*(r.Y*r.Y + r.Z*r.Z);
	out_inertia->M[1][1] = out_inertia->M[1][1] + bi_m*(r.X*r.X + r.Z*r.Z);
	out_inertia->M[2][2] = out_inertia->M[2][2] + bi_m*(r.X*r.X + r.Y*r.Y);
	out_inertia->M[0][1] = out_inertia->M[0][1] + bi_m*(r.X*r.Y);
	out_inertia->M[0][2] = out_inertia->M[0][2] + bi_m*(r.X*r.Z);
	out_inertia->M[1][2] = out_inertia->M[1][2] + bi_m*(r.Y*r.Z);
}

float AJointCharacterTest::inertiaAboutAxis(const FMatrix& inertia_mat, const FVector& axis)
{
	return inertia_mat.M[0][0] *axis.X*axis.X + inertia_mat.M[1][1] *axis.Y*axis.Y + inertia_mat.M[2][2] *axis.Z*axis.Z
		- 2.f*inertia_mat.M[0][1] *axis.X*axis.Y - 2.f*inertia_mat.M[1][2] *axis.Y*axis.Z - 2.f*inertia_mat.M[0][2] *axis.X*axis.Z;
}