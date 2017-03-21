// Fill out your copyright notice in the Description page of Project Settings.

#include "SwordSwing.h"
#include "JointCharacterTest.h"


// Sets default values
AJointCharacterTest::AJointCharacterTest()
{
 	// Set this pawn to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
	RootComponent = CreateDefaultSubobject<USceneComponent>(TEXT("RootComponent"));

	initCamera();

	//body settings
	initUpperBody();
	initWeapon();
	initLegs();
	initUpperBodyJoints();
	initWeaponJoints();
	initLegJoints();


	AutoPossessPlayer = EAutoReceiveInput::Player0;
}

// Called when the game starts or when spawned
void AJointCharacterTest::BeginPlay()
{
	Super::BeginPlay();
	
	initPIDs();
	initCustomPhysics();
	
}

// Called every frame
void AJointCharacterTest::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	if (fight_mode) 
		weapon_motor_bi->AddCustomPhysics(OnCalculateCustomWeaponPhysics);
		//swordCalculations(DeltaTime);

	cameraCalculations(DeltaTime);
	movementCalculations(DeltaTime);

	torso_bi->AddCustomPhysics(OnCalculateCustomHoverPhysics);
	torso_bi->AddCustomPhysics(OnCalculateCustomStabilizerPhysics);

	pelvis_bi->AddCustomPhysics(OnCalculateCustomWalkingPhysics);
	
	
	//UE_LOG(LogTemp, Warning, TEXT("tick!"));
}



void AJointCharacterTest::cameraCalculations(float DeltaTime) 
{
	camera_axis->SetWorldLocation(torso->GetCenterOfMass());

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
			jump_force = (((FMath::Sqrt(2.f*1000.f*jump_height) - torso->GetComponentVelocity().Z) / jump_force_time) + 1000.f)*(torso->GetMass() + weapon->GetMass() + weapon_axis->GetMass() + weapon_motor->GetMass()) * 100;
		}

		torso->AddForce(FVector::UpVector*jump_force*DeltaTime);

		curr_jump_time += DeltaTime;
		if (curr_jump_time > jump_force_time) {
			jumping = false;
			curr_jump_time = 0.0f;
		}

	}
}

void AJointCharacterTest::OnHit(UPrimitiveComponent* HitComp, AActor* OtherActor, UPrimitiveComponent* OtherComp, FVector NormalImpulse, const FHitResult& Hit)
{
		UE_LOG(LogTemp, Warning, TEXT("hit: %f"), NormalImpulse.Size());
		UE_LOG(LogTemp, Warning, TEXT("ded"));
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
	DrawDebugLine(
		GetWorld(),
		spine_bi->GetUnrealWorldTransform().GetLocation(),
		spine_bi->GetUnrealWorldTransform().GetLocation() + 100 * tmp_rot_axis.GetSafeNormal(), 					//size
		FColor(255, 0, 0),  //pink
		true,  				//persistent (never goes away)
		0.01, 					//point leaves a trail on moving object
		10,
		5.f
	);
	
	pelvis_bi->AddAngularImpulse(tmp_rot_axis*ssc.adjustment.X, false);
	
	//gravitational compensaiton
	float upper_mass = pelvis_bi->GetBodyMass() + spine_bi->GetBodyMass() + torso_bi->GetBodyMass();
	FVector upper_com = (pelvis_bi->GetCOMPosition()*pelvis_bi->GetBodyMass() + spine_bi->GetCOMPosition()*spine_bi->GetBodyMass() + torso_bi->GetCOMPosition()*torso_bi->GetBodyMass()) / (upper_mass);
	//pelvis_bi->AddForceAtPosition(FVector::UpVector*upper_mass * 980.f, upper_com, false);
	pelvis_bi->AddTorque(-FVector::CrossProduct( FVector::UpVector*upper_mass * 980.f, upper_com - pelvis_bi->GetCOMPosition() ), false);

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

	torso_bi->AddTorque(spine_up*-ttc.adjustment.X, false);

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

	pelvis_bi->AddTorque(spine_up*ptc.adjustment.X, false);


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
	r_hip_motor_bi->AddAngularImpulse(tmp_rot_axis*rtdc.adjustment.X, false);
	
	//gravitational compensaiton
	float r_leg_mass = r_hip_motor_bi->GetBodyMass() + r_thigh_bi->GetBodyMass() + r_thigh_hydraulic_bi->GetBodyMass() + r_knee_motor_bi->GetBodyMass() + r_shin_bi->GetBodyMass() + r_shin_hydraulic_bi->GetBodyMass();
	FVector r_leg_com = (r_hip_motor_bi->GetCOMPosition()*r_hip_motor_bi->GetBodyMass() + r_thigh_bi->GetCOMPosition()*r_thigh_bi->GetBodyMass() + r_thigh_hydraulic_bi->GetCOMPosition()*r_thigh_hydraulic_bi->GetBodyMass() + r_knee_motor_bi->GetCOMPosition()*r_knee_motor_bi->GetBodyMass() + r_shin_bi->GetCOMPosition()*r_shin_bi->GetBodyMass() + r_shin_hydraulic_bi->GetCOMPosition()*r_shin_hydraulic_bi->GetBodyMass()) / (r_leg_mass);
	r_hip_motor_bi->AddTorque(-FVector::CrossProduct(FVector::UpVector*r_leg_mass * 980.f, r_leg_com - r_hip_motor_bi->GetCOMPosition()), false);


	FVector l_thigh_target = from_pelvis_to_world_transform.TransformVector(ltdc.target);

	ltdc.error.X = FMath::Acos(FVector::DotProduct(l_thigh_target, -l_thigh_bi->GetUnrealWorldTransform().GetUnitAxis(EAxis::Z)))*180.f / PI;
	ltdc.integral = ltdc.integral + ltdc.error * DeltaTime;
	ltdc.derivative = (ltdc.error - ltdc.prev_err) / DeltaTime;

	ltdc.adjustment = ltdc.P * ltdc.error +
		ltdc.I * ltdc.integral +
		ltdc.D * ltdc.derivative;
	ltdc.prev_err = ltdc.error;

	tmp_rot_axis = FVector::CrossProduct(-l_thigh_bi->GetUnrealWorldTransform().GetUnitAxis(EAxis::Z), l_thigh_target);
	l_hip_motor_bi->AddAngularImpulse(tmp_rot_axis*ltdc.adjustment.X, false);

	//gravitational compensaiton
	float l_leg_mass = l_hip_motor_bi->GetBodyMass() + l_thigh_bi->GetBodyMass() + l_thigh_hydraulic_bi->GetBodyMass() + l_knee_motor_bi->GetBodyMass() + l_shin_bi->GetBodyMass() + l_shin_hydraulic_bi->GetBodyMass();
	FVector l_leg_com = (l_hip_motor_bi->GetCOMPosition()*l_hip_motor_bi->GetBodyMass() + l_thigh_bi->GetCOMPosition()*l_thigh_bi->GetBodyMass() + l_thigh_hydraulic_bi->GetCOMPosition()*l_thigh_hydraulic_bi->GetBodyMass() + l_knee_motor_bi->GetCOMPosition()*l_knee_motor_bi->GetBodyMass() + l_shin_bi->GetCOMPosition()*l_shin_bi->GetBodyMass() + l_shin_hydraulic_bi->GetCOMPosition()*l_shin_hydraulic_bi->GetBodyMass()) / (l_leg_mass);
	l_hip_motor_bi->AddTorque(-FVector::CrossProduct(FVector::UpVector*l_leg_mass * 980.f, l_leg_com - l_hip_motor_bi->GetCOMPosition()), false);

	//knee movement
	FVector r_hydraulic_dir = r_shin_hydraulic_bi->GetUnrealWorldTransform().GetLocation() - r_thigh_hydraulic_bi->GetUnrealWorldTransform().GetLocation() ;
	rkec.error = rkec.target - r_hydraulic_dir.Size();
	rkec.integral = rkec.integral + rkec.error * DeltaTime;
	rkec.derivative = (rkec.error - rkec.prev_err) / DeltaTime;

	rkec.adjustment = rkec.P * rkec.error +
		rkec.I * rkec.integral +
		rkec.D * rkec.derivative;
	rkec.prev_err = rkec.error;
	r_shin_hydraulic_bi->AddImpulse(r_hydraulic_dir.GetSafeNormal()*rkec.adjustment, false);
	r_thigh_hydraulic_bi->AddImpulse(-r_hydraulic_dir.GetSafeNormal()*rkec.adjustment, false);

	FVector l_hydraulic_dir = l_shin_hydraulic_bi->GetUnrealWorldTransform().GetLocation() - l_thigh_hydraulic_bi->GetUnrealWorldTransform().GetLocation() ;
	lkec.error = lkec.target - l_hydraulic_dir.Size();
	lkec.integral = lkec.integral + lkec.error * DeltaTime;
	lkec.derivative = (lkec.error - lkec.prev_err) / DeltaTime;

	lkec.adjustment = lkec.P * lkec.error +
		lkec.I * lkec.integral +
		lkec.D * lkec.derivative;
	lkec.prev_err = lkec.error;
	l_shin_hydraulic_bi->AddImpulse(l_hydraulic_dir.GetSafeNormal()*lkec.adjustment, false);
	l_thigh_hydraulic_bi->AddImpulse(-l_hydraulic_dir.GetSafeNormal()*lkec.adjustment, false);

	//pelvis_bi->AddAngularImpulse(FVector::UpVector * 500, false);

	//foot movement

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

	sword_motor_pos.error = ((wa_pos + weapon_motor_attachment->RelativeLocation) + target_wep_dir * 70.f) - wm_pos;
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

	weapon_axis_attachment->BreakConstraint();
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
}

void AJointCharacterTest::initUpperBody()
{
	torso = CreateDefaultSubobject<UBoxComponent>(TEXT("Torso"));
	torso->SetupAttachment(RootComponent);
	torso->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);
	torso->SetNotifyRigidBodyCollision(true);
	torso_vis = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("TorsoVisualization"));
	torso_vis->SetupAttachment(torso);

	spine = CreateDefaultSubobject<UCapsuleComponent>(TEXT("Spine"));
	spine->SetupAttachment(RootComponent);
	spine->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);
	spine->SetNotifyRigidBodyCollision(true);
	spine_vis = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("SpineVisualization"));
	spine_vis->SetupAttachment(spine);
	spine_attachment = CreateDefaultSubobject<UPhysicsConstraintComponent>(TEXT("SpineAttachment"));
	spine_attachment->SetupAttachment(spine);
	spine_attachment->SetRelativeLocation(FVector(0.f, 0.f, 20.f));

}

void AJointCharacterTest::initUpperBodyJoints()
{
	spine_attachment->SetConstrainedComponents(spine, NAME_None, torso, NAME_None);
	spine_attachment->SetLinearXLimit(ELinearConstraintMotion::LCM_Locked, 4.0f);
	spine_attachment->SetLinearYLimit(ELinearConstraintMotion::LCM_Locked, 4.0f);
	spine_attachment->SetLinearZLimit(ELinearConstraintMotion::LCM_Locked, 4.0f);
	spine_attachment->SetAngularSwing1Limit(EAngularConstraintMotion::ACM_Free, 0.0f);
	spine_attachment->SetAngularSwing2Limit(EAngularConstraintMotion::ACM_Free, 0.0f);
	spine_attachment->SetAngularTwistLimit(EAngularConstraintMotion::ACM_Free, 0.0f);
	spine_attachment->SetDisableCollision(true);
}

void AJointCharacterTest::initWeapon()
{
	// Weapon settings
	weapon_axis = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("WeaponAxis"));
	weapon_axis->SetupAttachment(RootComponent);
	weapon_axis->SetRelativeLocation(FVector(0.f, 0.f, 110.f));
	weapon_axis->SetRelativeScale3D(FVector(1.f, 1.f, 1.f));

	weapon_axis_attachment = CreateDefaultSubobject<UPhysicsConstraintComponent>(TEXT("WeaponAxisAttachment"));
	weapon_axis_attachment->SetupAttachment(torso);
	weapon_axis_attachment->SetRelativeLocation(FVector(0.f, 0.f, 0.f));

	weapon_motor = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("WeaponMotor"));
	weapon_motor->SetupAttachment(RootComponent);
	weapon_motor->SetRelativeLocation(FVector(0.f, 0.f, 160.f));
	weapon_motor->SetRelativeScale3D(FVector(0.5f, 0.5f, 0.05f));

	weapon_motor_attachment = CreateDefaultSubobject<UPhysicsConstraintComponent>(TEXT("WeaponMotorAttachment"));
	weapon_motor_attachment->SetupAttachment(weapon_axis);
	weapon_motor_attachment->SetRelativeLocation(FVector(0.f, 0.f, 50.f));

	weapon = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("Weapon"));
	weapon->SetupAttachment(RootComponent);
	weapon->SetRelativeLocation(FVector(0.f, 0.f, 163.f));
	weapon->SetRelativeScale3D(FVector(0.1f, 0.1f, 2.0f));

	weapon_motor->SetPhysicsMaxAngularVelocity(5000.f);
	weapon->SetPhysicsMaxAngularVelocity(5000.f);
}

void AJointCharacterTest::initWeaponJoints()
{
	weapon_attachment = CreateDefaultSubobject<UPhysicsConstraintComponent>(TEXT("WeaponAttachment"));
	weapon_attachment->SetupAttachment(weapon_motor);
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

	weapon_axis_attachment->SetConstrainedComponents(torso, NAME_None, weapon_axis, NAME_None);
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


	//weapon_attachment->SetAngularDriveMode(EAngularDriveMode::TwistAndSwing);
	//weapon_attachment->SetAngularOrientationTarget(FRotator(0.f, 0.f, 0.f));
	//weapon_attachment->SetAngularDriveParams(10000.f, 0.f, 1000000.f);
	//weapon_attachment->SetOrientationDriveTwistAndSwing(true, true);
}

void AJointCharacterTest::initLegs()
{

	right_leg_axis = CreateDefaultSubobject<USceneComponent>(TEXT("RightLegAxis"));
	right_leg_axis->SetupAttachment(RootComponent);

	left_leg_axis = CreateDefaultSubobject<USceneComponent>(TEXT("LeftLegAxis"));
	left_leg_axis->SetupAttachment(RootComponent);

	pelvis = CreateDefaultSubobject<UCapsuleComponent>(TEXT("PelvisCol"));
	pelvis->SetupAttachment(RootComponent);
	pelvis->SetRelativeLocation(FVector(0.f, 0.f, 0));
	pelvis->SetRelativeScale3D(FVector(1.f, 1.f, 1.f));
	pelvis->GetBodyInstance()->bLockXRotation = true;
	pelvis->GetBodyInstance()->bLockYRotation = true;
	pelvis->GetBodyInstance()->bLockZRotation = true;
	
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
	r_thigh_attachment->SetupAttachment(r_thigh);
	r_thigh_attachment->SetRelativeLocation(FVector(0.f, 0.f, 0.f));

	r_thigh_hydraulic = CreateDefaultSubobject<USphereComponent>(TEXT("RThighHydraulicCol"));
	r_thigh_hydraulic->SetupAttachment(right_leg_axis);
	r_thigh_hydraulic->SetRelativeLocation(FVector(0.f, 0.f, 0));
	r_thigh_hydraulic->SetRelativeScale3D(FVector(1.f, 1.f, 1.f));
	r_thigh_hydraulic_vis = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("RThighHydraulic"));
	r_thigh_hydraulic_vis->SetupAttachment(r_thigh_hydraulic);
	r_thigh_hydraulic_attachment = CreateDefaultSubobject<UPhysicsConstraintComponent>(TEXT("RThighHydraulicAttachment"));
	r_thigh_hydraulic_attachment->SetupAttachment(r_thigh_hydraulic);
	r_thigh_hydraulic_attachment->SetRelativeLocation(FVector(0.f, 0.f, 0.f));
	r_thigh_hydraulic_piston = CreateDefaultSubobject<UPhysicsConstraintComponent>(TEXT("RThighHydraulicPiston"));
	r_thigh_hydraulic_piston->SetupAttachment(r_thigh_hydraulic);
	r_thigh_hydraulic_piston->SetRelativeLocation(FVector(0.f, 0.f, 0.f));

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

	r_shin_hydraulic = CreateDefaultSubobject<USphereComponent>(TEXT("RShinHydraulicCol"));
	r_shin_hydraulic->SetupAttachment(right_leg_axis);
	r_shin_hydraulic->SetRelativeLocation(FVector(0.f, 0.f, 0));
	r_shin_hydraulic->SetRelativeScale3D(FVector(1.f, 1.f, 1.f));
	r_shin_hydraulic_vis = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("RShinHydraulic"));
	r_shin_hydraulic_vis->SetupAttachment(r_shin_hydraulic);
	r_shin_hydraulic_attachment = CreateDefaultSubobject<UPhysicsConstraintComponent>(TEXT("RShinHydraulicAttachment"));
	r_shin_hydraulic_attachment->SetupAttachment(r_shin_hydraulic);
	r_shin_hydraulic_attachment->SetRelativeLocation(FVector(0.f, 0.f, 0.f));

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
	l_thigh_attachment->SetupAttachment(l_thigh);
	l_thigh_attachment->SetRelativeLocation(FVector(0.f, 0.f, 0.f));

	l_thigh_hydraulic = CreateDefaultSubobject<USphereComponent>(TEXT("LThighHydraulicCol"));
	l_thigh_hydraulic->SetupAttachment(left_leg_axis);
	l_thigh_hydraulic->SetRelativeLocation(FVector(0.f, 0.f, 0));
	l_thigh_hydraulic->SetRelativeScale3D(FVector(1.f, 1.f, 1.f));
	l_thigh_hydraulic_vis = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("LThighHydraulic"));
	l_thigh_hydraulic_vis->SetupAttachment(l_thigh_hydraulic);
	l_thigh_hydraulic_attachment = CreateDefaultSubobject<UPhysicsConstraintComponent>(TEXT("LThighHydraulicAttachment"));
	l_thigh_hydraulic_attachment->SetupAttachment(l_thigh_hydraulic);
	l_thigh_hydraulic_attachment->SetRelativeLocation(FVector(0.f, 0.f, 0.f));
	l_thigh_hydraulic_piston = CreateDefaultSubobject<UPhysicsConstraintComponent>(TEXT("LThighHydraulicPiston"));
	l_thigh_hydraulic_piston->SetupAttachment(l_thigh_hydraulic);
	l_thigh_hydraulic_piston->SetRelativeLocation(FVector(0.f, 0.f, 0.f));

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

	l_shin_hydraulic = CreateDefaultSubobject<USphereComponent>(TEXT("LShinHydraulicCol"));
	l_shin_hydraulic->SetupAttachment(left_leg_axis);
	l_shin_hydraulic->SetRelativeLocation(FVector(0.f, 0.f, 0));
	l_shin_hydraulic->SetRelativeScale3D(FVector(1.f, 1.f, 1.f));
	l_shin_hydraulic_vis = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("LShinHydraulic"));
	l_shin_hydraulic_vis->SetupAttachment(l_shin_hydraulic);
	l_shin_hydraulic_attachment = CreateDefaultSubobject<UPhysicsConstraintComponent>(TEXT("LShinHydraulicAttachment"));
	l_shin_hydraulic_attachment->SetupAttachment(l_shin_hydraulic);
	l_shin_hydraulic_attachment->SetRelativeLocation(FVector(0.f, 0.f, 0.f));

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
	pelvis_attachment->SetAngularSwing1Limit(EAngularConstraintMotion::ACM_Free, 0.0f);
	pelvis_attachment->SetAngularSwing2Limit(EAngularConstraintMotion::ACM_Free, 0.0f);
	pelvis_attachment->SetAngularTwistLimit(EAngularConstraintMotion::ACM_Free, 0.0f);
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
	r_hip_attachment->SetAngularSwing1Limit(EAngularConstraintMotion::ACM_Limited, 45.0f);
	r_hip_attachment->SetAngularSwing2Limit(EAngularConstraintMotion::ACM_Locked, 0.0f);
	r_hip_attachment->SetAngularTwistLimit(EAngularConstraintMotion::ACM_Free, 160);
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
	r_thigh_attachment->SetAngularSwing2Limit(EAngularConstraintMotion::ACM_Locked, 00.f);
	r_thigh_attachment->SetAngularTwistLimit(EAngularConstraintMotion::ACM_Locked, 0.0f);
	r_thigh_attachment->SetDisableCollision(true);

	r_thigh_hydraulic_attachment->SetConstrainedComponents(r_thigh_hydraulic, NAME_None, r_thigh, NAME_None);
	r_thigh_hydraulic_attachment->SetLinearXLimit(ELinearConstraintMotion::LCM_Locked, 0.0f);
	r_thigh_hydraulic_attachment->SetLinearYLimit(ELinearConstraintMotion::LCM_Locked, 0.0f);
	r_thigh_hydraulic_attachment->SetLinearZLimit(ELinearConstraintMotion::LCM_Locked, 0.0f);
	r_thigh_hydraulic_attachment->SetAngularSwing1Limit(EAngularConstraintMotion::ACM_Locked, 0.0f);
	r_thigh_hydraulic_attachment->SetAngularSwing2Limit(EAngularConstraintMotion::ACM_Locked, 00.f);
	r_thigh_hydraulic_attachment->SetAngularTwistLimit(EAngularConstraintMotion::ACM_Free, 0.0f);
	r_thigh_hydraulic_attachment->SetDisableCollision(true);

	r_thigh_hydraulic_piston->SetConstrainedComponents(r_thigh_hydraulic, NAME_None, r_shin_hydraulic, NAME_None);
	r_thigh_hydraulic_piston->SetLinearXLimit(ELinearConstraintMotion::LCM_Locked, 0.0f);
	r_thigh_hydraulic_piston->SetLinearYLimit(ELinearConstraintMotion::LCM_Locked, 0.0f);
	r_thigh_hydraulic_piston->SetLinearZLimit(ELinearConstraintMotion::LCM_Limited, 28);
	r_thigh_hydraulic_piston->ConstraintInstance.bScaleLinearLimits = false;
	r_thigh_hydraulic_piston->ConstraintInstance.ProfileInstance.LinearLimit.bSoftConstraint = true;
	r_thigh_hydraulic_piston->ConstraintInstance.ProfileInstance.LinearLimit.Stiffness = 1000.0f;
	r_thigh_hydraulic_piston->ConstraintInstance.ProfileInstance.LinearLimit.Damping = 10.0f;
	r_thigh_hydraulic_piston->SetAngularSwing1Limit(EAngularConstraintMotion::ACM_Locked, 0.0f);
	r_thigh_hydraulic_piston->SetAngularSwing2Limit(EAngularConstraintMotion::ACM_Locked, 00.f);
	r_thigh_hydraulic_piston->SetAngularTwistLimit(EAngularConstraintMotion::ACM_Locked, 0.0f);
	r_thigh_hydraulic_piston->SetDisableCollision(true);


	r_knee_attachment->SetConstrainedComponents(r_thigh, NAME_None, r_knee_motor, NAME_None);
	r_knee_attachment->SetLinearXLimit(ELinearConstraintMotion::LCM_Limited, 0.0f);
	r_knee_attachment->SetLinearYLimit(ELinearConstraintMotion::LCM_Limited, 0.0f);
	r_knee_attachment->SetLinearZLimit(ELinearConstraintMotion::LCM_Limited, 0.0f);
	r_knee_attachment->ConstraintInstance.bScaleLinearLimits = false;
	r_knee_attachment->SetAngularSwing1Limit(EAngularConstraintMotion::ACM_Locked, 0.0f);
	r_knee_attachment->SetAngularSwing2Limit(EAngularConstraintMotion::ACM_Locked, 0.0f);
	r_knee_attachment->SetAngularTwistLimit(EAngularConstraintMotion::ACM_Limited, 85.0f);
	r_knee_attachment->ConstraintInstance.AngularRotationOffset = FRotator(0.f, 0.f, 0.f);
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

	r_shin_hydraulic_attachment->SetConstrainedComponents(r_shin_hydraulic, NAME_None, r_shin, NAME_None);
	r_shin_hydraulic_attachment->SetLinearXLimit(ELinearConstraintMotion::LCM_Locked, 0.0f);
	r_shin_hydraulic_attachment->SetLinearYLimit(ELinearConstraintMotion::LCM_Locked, 0.0f);
	r_shin_hydraulic_attachment->SetLinearZLimit(ELinearConstraintMotion::LCM_Locked, 0.0f);
	r_shin_hydraulic_attachment->SetAngularSwing1Limit(EAngularConstraintMotion::ACM_Locked, 0.0f);
	r_shin_hydraulic_attachment->SetAngularSwing2Limit(EAngularConstraintMotion::ACM_Locked, 00.f);
	r_shin_hydraulic_attachment->SetAngularTwistLimit(EAngularConstraintMotion::ACM_Free, 0.0f);
	r_shin_hydraulic_attachment->SetDisableCollision(true);

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
	l_hip_attachment->SetAngularSwing1Limit(EAngularConstraintMotion::ACM_Limited, 45.0f);
	l_hip_attachment->SetAngularSwing2Limit(EAngularConstraintMotion::ACM_Locked, 0.0f);
	l_hip_attachment->SetAngularTwistLimit(EAngularConstraintMotion::ACM_Free, 160);
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
	l_thigh_attachment->SetAngularSwing2Limit(EAngularConstraintMotion::ACM_Locked, 00.f);
	l_thigh_attachment->SetAngularTwistLimit(EAngularConstraintMotion::ACM_Locked, 0.0f);
	l_thigh_attachment->SetDisableCollision(true);

	l_thigh_hydraulic_attachment->SetConstrainedComponents(l_thigh_hydraulic, NAME_None, l_thigh, NAME_None);
	l_thigh_hydraulic_attachment->SetLinearXLimit(ELinearConstraintMotion::LCM_Locked, 0.0f);
	l_thigh_hydraulic_attachment->SetLinearYLimit(ELinearConstraintMotion::LCM_Locked, 0.0f);
	l_thigh_hydraulic_attachment->SetLinearZLimit(ELinearConstraintMotion::LCM_Locked, 0.0f);
	l_thigh_hydraulic_attachment->SetAngularSwing1Limit(EAngularConstraintMotion::ACM_Locked, 0.0f);
	l_thigh_hydraulic_attachment->SetAngularSwing2Limit(EAngularConstraintMotion::ACM_Locked, 00.f);
	l_thigh_hydraulic_attachment->SetAngularTwistLimit(EAngularConstraintMotion::ACM_Free, 0.0f);
	l_thigh_hydraulic_attachment->SetDisableCollision(true);

	l_thigh_hydraulic_piston->SetConstrainedComponents(l_thigh_hydraulic, NAME_None, l_shin_hydraulic, NAME_None);
	l_thigh_hydraulic_piston->SetLinearXLimit(ELinearConstraintMotion::LCM_Locked, 0.0f);
	l_thigh_hydraulic_piston->SetLinearYLimit(ELinearConstraintMotion::LCM_Locked, 0.0f);
	l_thigh_hydraulic_piston->SetLinearZLimit(ELinearConstraintMotion::LCM_Limited, 28);
	l_thigh_hydraulic_piston->ConstraintInstance.bScaleLinearLimits = false;
	l_thigh_hydraulic_piston->ConstraintInstance.ProfileInstance.LinearLimit.bSoftConstraint = true;
	l_thigh_hydraulic_piston->ConstraintInstance.ProfileInstance.LinearLimit.Stiffness = 1000.0f;
	l_thigh_hydraulic_piston->ConstraintInstance.ProfileInstance.LinearLimit.Damping = 10.0f;
	l_thigh_hydraulic_piston->SetAngularSwing1Limit(EAngularConstraintMotion::ACM_Locked, 0.0f);
	l_thigh_hydraulic_piston->SetAngularSwing2Limit(EAngularConstraintMotion::ACM_Locked, 00.f);
	l_thigh_hydraulic_piston->SetAngularTwistLimit(EAngularConstraintMotion::ACM_Locked, 0.0f);
	l_thigh_hydraulic_piston->SetDisableCollision(true);


	l_knee_attachment->SetConstrainedComponents(l_thigh, NAME_None, l_knee_motor, NAME_None);
	l_knee_attachment->SetLinearXLimit(ELinearConstraintMotion::LCM_Limited, 0.0f);
	l_knee_attachment->SetLinearYLimit(ELinearConstraintMotion::LCM_Limited, 0.0f);
	l_knee_attachment->SetLinearZLimit(ELinearConstraintMotion::LCM_Limited, 0.0f);
	l_knee_attachment->ConstraintInstance.bScaleLinearLimits = false;
	l_knee_attachment->SetAngularSwing1Limit(EAngularConstraintMotion::ACM_Locked, 0.0f);
	l_knee_attachment->SetAngularSwing2Limit(EAngularConstraintMotion::ACM_Locked, 0.0f);
	l_knee_attachment->SetAngularTwistLimit(EAngularConstraintMotion::ACM_Limited, 85.0f);
	l_knee_attachment->ConstraintInstance.AngularRotationOffset = FRotator(0.f, 0.f, 0.f);
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

	l_shin_hydraulic_attachment->SetConstrainedComponents(l_shin_hydraulic, NAME_None, l_shin, NAME_None);
	l_shin_hydraulic_attachment->SetLinearXLimit(ELinearConstraintMotion::LCM_Locked, 0.0f);
	l_shin_hydraulic_attachment->SetLinearYLimit(ELinearConstraintMotion::LCM_Locked, 0.0f);
	l_shin_hydraulic_attachment->SetLinearZLimit(ELinearConstraintMotion::LCM_Locked, 0.0f);
	l_shin_hydraulic_attachment->SetAngularSwing1Limit(EAngularConstraintMotion::ACM_Locked, 0.0f);
	l_shin_hydraulic_attachment->SetAngularSwing2Limit(EAngularConstraintMotion::ACM_Locked, 00.f);
	l_shin_hydraulic_attachment->SetAngularTwistLimit(EAngularConstraintMotion::ACM_Free, 0.0f);
	l_shin_hydraulic_attachment->SetDisableCollision(true);

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
	hover_height.target = target_hover_height;
	hover_height.max_adjustment = 1000000;
	hover_height.P = 300.f;
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
	sword_rotation.integral = 0.f;

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
	rtdc.target = FVector::RightVector;
	//r_thigh_direction_controller.target = -FVector::RightVector;
	//r_thigh_direction_controller.target = FVector::ForwardVector;
	rtdc.P = FVector(100000.1f, 100000.1f, 100000.1f);
	rtdc.I = FVector(0.f, 0.f, 0.f);
	rtdc.D = FVector(1000.01f, 1000.01f, 1000.01f);
	rtdc.integral = FVector::ZeroVector;

	ltdc.max_adjustment = FVector(1000000.f, 1000000.f, 1000000.f);
	ltdc.target = FVector::RightVector;
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

	axis_target_rot = FRotator(0.f, 150.f, 0.f);
	arm_target_rot = FRotator(-65.f, 0.f, 0.f);
	target_arm_length = 1000.f;
}

void AJointCharacterTest::initCustomPhysics()
{
	OnCalculateCustomHoverPhysics.BindUObject(this, &AJointCharacterTest::customHoverPhysics);
	OnCalculateCustomStabilizerPhysics.BindUObject(this, &AJointCharacterTest::customStabilizerPhysics);
	torso_bi = torso->GetBodyInstance();
	spine_bi = spine->GetBodyInstance();

	OnCalculateCustomWeaponPhysics.BindUObject(this, &AJointCharacterTest::customWeaponPhysics);
	weapon_axis_bi = weapon_axis->GetBodyInstance();
	weapon_motor_bi = weapon_motor->GetBodyInstance();
	weapon_bi = weapon->GetBodyInstance();

	//rolling_body->OnComponentHit.AddDynamic(this, &AJointCharacterTest::OnHit);
	weapon_axis->OnComponentHit.AddDynamic(this, &AJointCharacterTest::OnHit);

	OnCalculateCustomWalkingPhysics.BindUObject(this, &AJointCharacterTest::customWalkingPhysics);
	pelvis_bi = pelvis->GetBodyInstance();
	r_hip_motor_bi = r_hip_motor->GetBodyInstance();
	r_thigh_bi = r_thigh->GetBodyInstance();
	r_thigh_hydraulic_bi = r_thigh_hydraulic->GetBodyInstance();
	r_knee_motor_bi = r_knee_motor->GetBodyInstance();
	r_shin_bi = r_shin->GetBodyInstance();
	r_shin_hydraulic_bi = r_shin_hydraulic->GetBodyInstance();
	r_toe_bi = r_toe->GetBodyInstance();

	l_hip_motor_bi = l_hip_motor->GetBodyInstance();
	l_thigh_bi = l_thigh->GetBodyInstance();
	l_thigh_hydraulic_bi = l_thigh_hydraulic->GetBodyInstance();
	l_knee_motor_bi = l_knee_motor->GetBodyInstance();
	l_shin_bi = l_shin->GetBodyInstance();
	l_shin_hydraulic_bi = l_shin_hydraulic->GetBodyInstance();
	l_toe_bi = l_toe->GetBodyInstance();
}