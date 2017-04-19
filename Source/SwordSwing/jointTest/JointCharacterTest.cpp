// Fill out your copyright notice in the Description page of Project Settings.

#include "SwordSwing.h"
#include "AudioDevice.h"

#include "ActiveSound.h"
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


	//AutoPossessPlayer = EAutoReceiveInput::Player0;
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
	
	cameraCalculations(DeltaTime);

	//DrawDebugPoint(
	//	GetWorld(),
	//	grip_v_bi->GetCOMPosition(),
	//	20,  					//size
	//	FColor(255, 0, 255),  //pink
	//	false,  				//persistent (never goes away)
	//	0.03 					//point leaves a trail on moving object
	//);

	DrawDebugLine(
		GetWorld(),
		grip_v->GetComponentLocation(),
		grip_v->GetComponentLocation() - 200 * FVector::UpVector, 					//size
		FColor(255, 0, 0),  //pink
		true,  				//persistent (never goes away)
		0.03, 					//point leaves a trail on moving object
		10,
		5.f
	);



	if (alive)
	{
		camera_axis->SetWorldLocation(torso->GetCenterOfMass());
		//weapon_axis->SetWorldLocation(torso->GetCenterOfMass() + FVector(0.f, 0.f, -15.f));
		weapon_axis_bi->SetBodyTransform(FTransform(torso_bi->GetCOMPosition() + FVector(0.f, 0.f, -15.f)), true);
		
		
		if (fight_mode)
		{
			grip_h_bi->AddCustomPhysics(OnCalculateControlGripPhysics);
		}

		movementCalculations(DeltaTime);

		torso_bi->AddCustomPhysics(OnCalculateCustomHoverPhysics);
		torso_bi->AddCustomPhysics(OnCalculateCustomStabilizerPhysics);

		pelvis_bi->AddCustomPhysics(OnCalculateCustomWalkingPhysics);
	}
}



void AJointCharacterTest::cameraCalculations(float DeltaTime) 
{

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
			jump_force = (((FMath::Sqrt(2.f*1000.f*jump_height) - torso->GetComponentVelocity().Z) / jump_force_time) + 1000.f)*(torso->GetMass() + grip_v->GetMass() + weapon_axis->GetMass() + grip_h->GetMass()) * 100;
		}

		torso->AddForce(FVector::UpVector*jump_force*DeltaTime);

		curr_jump_time += DeltaTime;
		if (curr_jump_time > jump_force_time) {
			jumping = false;
			curr_jump_time = 0.0f;
		}

	}
}

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

		//HitComp->DestroyComponent();
	}
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

void AJointCharacterTest::ControlGripPhysics(float DeltaTime, FBodyInstance* BodyInstance)
{
	initInputVars();
	

	customInitGripPhysics(DeltaTime, BodyInstance);
	ControlGripPositionPhysics(DeltaTime, BodyInstance);
	ControlGripDirectionPhysics(DeltaTime, BodyInstance);
	ControlGripInclinePhysics(DeltaTime, BodyInstance);

	if (holding_weapon)
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
	if (guarding)
	{
		FVector rot_ref;
		wep_extended = false;
		target_wep_dir = input_dir;
		target_wep_pos = input_dir;

		target_wep_dir_xy = target_wep_dir - FVector::DotProduct(target_wep_dir, camera_axis->GetUpVector())*camera_axis->GetUpVector();

		target_wep_pos_xy = target_wep_pos - FVector::DotProduct(target_wep_dir, camera_axis->GetUpVector())*camera_axis->GetUpVector();
		target_wep_pos_xy.Normalize();

		if (target_wep_dir_xy.IsNearlyZero())
			target_wep_dir_xy = prev_target_wep_dir_xy;

		FVector guard_pos(FMath::Cos(PI / 2.5f), 0.f , FMath::Sin(PI / 2.5f));
		guard_pos.Normalize();

		float tmp_angle = FMath::Acos(FVector::DotProduct(target_wep_dir_xy.GetSafeNormal(), FVector::ForwardVector))*180.f / PI;
		if (FVector::CrossProduct( FVector::ForwardVector, target_wep_dir_xy.GetSafeNormal() ).Z < 0)
			tmp_angle = -tmp_angle;
		target_wep_pos = guard_pos.RotateAngleAxis(tmp_angle, FVector::UpVector);
		
		FVector guard_dir(FMath::Cos(-PI / 3.f), 0.f, FMath::Sin(-PI / 3.f));
		guard_dir.Normalize();
		target_wep_dir = guard_dir.RotateAngleAxis(tmp_angle, FVector::UpVector);

		target_wep_dir_curr_wep_proj = target_wep_dir - FVector::DotProduct(target_wep_dir, gh_right)*gh_right;

		DrawDebugLine(
			GetWorld(),
			torso->GetComponentLocation(),
			torso->GetComponentLocation() + 100 * target_wep_dir, 					//size
			FColor(255, 0, 0),  //pink
			true,  				//persistent (never goes away)
			0.03, 					//point leaves a trail on moving object
			10,
			5.f
		);
	}
	else
	{
		target_wep_dir = input_dir;
		target_wep_pos = input_dir;

		target_wep_dir_xy = target_wep_dir - FVector::DotProduct(target_wep_dir, camera_axis->GetUpVector())*camera_axis->GetUpVector();
		//target_wep_dir_xy.Normalize();
		target_wep_dir_curr_wep_proj = target_wep_dir - FVector::DotProduct(target_wep_dir, gh_right)*gh_right;

		target_wep_pos_xy = target_wep_pos - FVector::DotProduct(target_wep_dir, camera_axis->GetUpVector())*camera_axis->GetUpVector();
		target_wep_pos_xy.Normalize();
	}

	
}

void AJointCharacterTest::customInitGripPhysics(float DeltaTime, FBodyInstance* BodyInstance)
{
	weapon_axis_bi->SetBodyTransform(FTransform(torso_bi->GetCOMPosition() + FVector(0.f, 0.f, -15.f)), true);
	wa_pos = weapon_axis_bi->GetUnrealWorldTransform().GetLocation();

	gh_pos = grip_h_bi->GetUnrealWorldTransform().GetLocation();
	gh_forward = grip_h_bi->GetUnrealWorldTransform().GetUnitAxis(EAxis::X);
	gh_right = grip_h_bi->GetUnrealWorldTransform().GetUnitAxis(EAxis::Y);
	gh_up = grip_h_bi->GetUnrealWorldTransform().GetUnitAxis(EAxis::Z);

	gv_pos = grip_v_bi->GetUnrealWorldTransform().GetLocation();
	gv_forward = grip_v_bi->GetUnrealWorldTransform().GetUnitAxis(EAxis::X);
	gv_right = grip_v_bi->GetUnrealWorldTransform().GetUnitAxis(EAxis::Y);
	gv_up = grip_v_bi->GetUnrealWorldTransform().GetUnitAxis(EAxis::Z);

	current_griph_xydir = gh_forward;
	current_griph_xydir.Z = 0.0f;

	/*float current_wep_incline = FMath::Acos(FVector::DotProduct(gh_up, w_up))*180.f / PI;
	current_wep_incline = FMath::Fmod(current_wep_incline, 90.0f);*/
}


void AJointCharacterTest::ControlGripPositionPhysics(float DeltaTime, FBodyInstance* BodyInstance)
{
/*	if(grabbing_weapon)
		gpc.error = target_wep_pos - gh_pos;
	else */
	if (wep_extended)
		gpc.error = ((wa_pos) +target_wep_pos_xy.GetSafeNormal() * 70.f) - gh_pos;
	else
		gpc.error = ((wa_pos) +target_wep_pos * 70.f) - gh_pos;

	gpc.integral = gpc.integral + gpc.error * DeltaTime;
	gpc.derivative = (gpc.error - gpc.prev_err) / DeltaTime;

	gpc.adjustment = gpc.P * gpc.error +
		gpc.I * gpc.integral +
		gpc.D * gpc.derivative;
	gpc.prev_err = gpc.error;

	grip_h_bi->AddImpulse(gpc.adjustment*(grip_h_bi->GetBodyMass() + grip_v_bi->GetBodyMass()), false);
}

void AJointCharacterTest::ControlGripDirectionPhysics(float DeltaTime, FBodyInstance* BodyInstance)
{
	//used to make the target be a little ahead of the actual target when the player is swinging quickly
	float target_ang_speed;

	if (target_wep_dir_xy.IsNearlyZero()) {
		//grc.error = 0.0f;
		target_wep_dir_xy = prev_target_wep_dir_xy;
		was_standing_still = true;
		target_ang_speed = 0.f;
	}
	else
	{
		if (camera_input.Size() > 0.5 && !was_standing_still)
		{
			target_ang_speed = FMath::Acos(FVector::DotProduct(target_wep_dir_xy.GetSafeNormal(), prev_target_wep_dir_xy))*180.f / PI;
			if (FVector::CrossProduct(target_wep_dir_xy.GetSafeNormal(), prev_target_wep_dir_xy).Z < 0.f)
				target_ang_speed = -target_ang_speed;
		}
		else
		{
			target_ang_speed = 0.f;
		}

		prev_target_wep_dir_xy = target_wep_dir_xy.GetSafeNormal();
		was_standing_still = false;
	}

	//UE_LOG(LogTemp, Warning, TEXT("target_ang_speed: %f"), target_ang_speed);

	if (wep_extended)
	{
		if (rot_forward)
		{
			float tmp_forward_error = (FMath::Acos(FVector::DotProduct(target_wep_dir_xy.GetSafeNormal(), current_griph_xydir))*180.f / PI);// +target_ang_speed;
			grc.error = tmp_forward_error;
			if (FVector::CrossProduct(target_wep_dir_xy.GetSafeNormal(), current_griph_xydir).Z < 0)
				grc.error = -grc.error;
		}
		else
		{
			float tmp_backwards_error = (FMath::Acos(FVector::DotProduct(target_wep_dir_xy.GetSafeNormal(), -current_griph_xydir))*180.f / PI);// +target_ang_speed;
			grc.error = tmp_backwards_error;
			if (FVector::CrossProduct(target_wep_dir_xy.GetSafeNormal(), -current_griph_xydir).Z < 0)
				grc.error = -grc.error;
		}
	}
	else
	{
		float tmp_forward_error = (FMath::Acos(FVector::DotProduct(target_wep_dir_xy.GetSafeNormal(), current_griph_xydir))*180.f / PI);// +target_ang_speed;
		float tmp_backwards_error = (FMath::Acos(FVector::DotProduct(target_wep_dir_xy.GetSafeNormal(), -current_griph_xydir))*180.f / PI);// +target_ang_speed;

		//TODO: look at the logic of this if
		if (tmp_forward_error < tmp_backwards_error)
		{
			rot_forward = true;
			grc.error = tmp_forward_error;
			if (FVector::CrossProduct(target_wep_dir_xy.GetSafeNormal(), current_griph_xydir).Z < 0)
				grc.error = -grc.error;
		}
		else
		{
			rot_forward = false;
			grc.error = tmp_backwards_error;
			if (FVector::CrossProduct(target_wep_dir_xy.GetSafeNormal(), -current_griph_xydir).Z < 0)
				grc.error = -grc.error;
		}
		//UE_LOG(LogTemp, Warning, TEXT("sword_rotation.error: %f"), sword_rotation.error);
	}

	grc.integral = grc.integral + grc.error * DeltaTime;
	grc.derivative = (grc.error - grc.prev_err) / DeltaTime;

	grc.adjustment = grc.P * grc.error +
		grc.I * grc.integral +
		grc.D * grc.derivative;
	grc.prev_err = grc.error;

	float z_inertia = (FMath::Pow(grip_h->GetUnscaledSphereRadius(), 2)*grip_h_bi->GetBodyMass()*2.f / 5.f);
	if (holding_weapon)
	{
		FVector tmpDIr = FVector(FVector::DotProduct(gv_up, FVector::ForwardVector), FVector::DotProduct(gv_up, FVector::RightVector), FVector::DotProduct(gv_up, FVector::UpVector));
		//float z_inertia = ((FMath::Pow(FVector::DotProduct(w_up, FVector::ForwardVector), 2) + FMath::Pow(FVector::DotProduct(w_up, FVector::RightVector), 2))*weapon->GetMass()*FMath::Pow(weapon->GetUnscaledCapsuleHalfHeight(), 2) / 3.0f) + 1;
		z_inertia += offset_wep_inertia.X*gv_up.X*gv_up.X + offset_wep_inertia.Y*gv_up.Y*gv_up.Y + offset_wep_inertia.Z*gv_up.Z*gv_up.Z;
		//z_inertia = z_inertia + (FMath::Pow(grip_h->GetUnscaledSphereRadius(), 2)*grip_h_bi->GetBodyMass()*2.f / 5.f);// +(FMath::Pow(weapon_handle_2->GetUnscaledSphereRadius(), 2)*weapon_handle_2_bi->GetBodyMass()*2.f / 5.f);																														//UE_LOG(LogTemp, Warning, TEXT("z_inertia: %f"), z_inertia);

	}
	FVector rot_torque = gh_up*-grc.adjustment*z_inertia;
	/*UE_LOG(LogTemp, Warning, TEXT("rot_torque.Size(): %f"), rot_torque.Size());
	UE_LOG(LogTemp, Warning, TEXT("DeltaTime: %f"), DeltaTime);*/
	//FMath::Lerp((grip_h->GetMass()*FMath::Pow(40.0f, 2) / 2.0f), weapon->GetMass()*FMath::Pow(250.0f, 2) / 3.0f, FMath::Pow(current_wep_incline / 90.f, 3));
	grip_h_bi->AddTorque(rot_torque, false);
	//grip_h_bi->AddForceAtPosition(FVector::ForwardVector*weapon->GetMass()*10000, gh_pos + FVector::ForwardVector*10.f, false);
	//grip_h_bi->AddForceAtPosition(-FVector::ForwardVector*weapon->GetMass()* 1000000000, gh_pos - FVector::ForwardVector*10.f, false);
}

void AJointCharacterTest::ControlGripInclinePhysics(float DeltaTime, FBodyInstance* BodyInstance)
{
	if (!target_wep_dir_curr_wep_proj.IsNearlyZero())
	{

		if (wep_extended)
		{
			if (FMath::Acos(FVector::DotProduct(gh_up, target_wep_dir_curr_wep_proj.GetSafeNormal()))*180.f / PI < 30)
			{
				wep_extended = false;
			}
			else
			{
				target_wep_dir_curr_wep_proj = gv_up;
				target_wep_dir_curr_wep_proj.Z = 0.f;
				target_wep_dir_curr_wep_proj.Normalize();

			}
		}
		//keep weapon raised if input is less than a certain input
		else if (FMath::Acos(FVector::DotProduct(gh_up, target_wep_dir_curr_wep_proj.GetSafeNormal()))*180.f / PI < 10)
		{
			target_wep_dir_curr_wep_proj = FVector::UpVector;
		}
		//if input is extended beyond a certain angle it should be kept in an extended stance
		else if (FMath::Acos(FVector::DotProduct(gh_up, target_wep_dir_curr_wep_proj.GetSafeNormal()))*180.f / PI > 80 && FMath::Abs(gic.error) < 5.0f)
		{
			wep_extended = true;
		}


		gic.error = -FMath::Acos(FVector::DotProduct(gv_up, target_wep_dir_curr_wep_proj.GetSafeNormal()))*180.f / PI;
		if (FVector::Coincident(FVector::CrossProduct(gv_up, target_wep_dir_curr_wep_proj.GetSafeNormal()), gv_right, 0.00001f))
			gic.error = -gic.error;

		if (gv_up.Z < 0.f && (gv_forward*gic.error).Z < 0.f && !guarding)
			gic.error = -gic.error;
	}
	else {
		gic.error = 0.0f;
	}

	gic.integral = gic.integral + gic.error * DeltaTime;
	gic.derivative = (gic.error - gic.prev_err) / DeltaTime;

	gic.adjustment = gic.P * gic.error +
		gic.I * gic.integral +
		gic.D * gic.derivative;
	gic.prev_err = gic.error;

	//weapon->AddTorque(weapon->GetRightVector()*sword_incline.adjustment);	

	grip_v_bi->AddForceAtPosition(gv_forward*gic.adjustment, gv_pos + 100 * gv_up, false);
	grip_v_bi->AddForceAtPosition(gv_forward*-gic.adjustment, gv_pos - 100 * gv_up, false);
	
	FVector centripetal = current_griph_xydir; centripetal.Z = 0.f; centripetal.Normalize();
	FVector tmp_dist = (grip_v_bi->GetCOMPosition() - grip_h_bi->GetCOMPosition());
	tmp_dist.Z = 0.f;
	FVector temp_W = grip_h_bi->GetUnrealWorldAngularVelocity();
	centripetal = -centripetal*grip_v_bi->GetBodyMass()*tmp_dist.Size()*FMath::DegreesToRadians(grip_h_bi->GetUnrealWorldAngularVelocity().SizeSquared());
	//weapon_bi->AddForce(centripetal, false);

}
void AJointCharacterTest::ControlWeaponTwistPhysics(float DeltaTime, FBodyInstance* BodyInstance)
{
	FVector tmp_vel_proj = gv_up - weapon_twist_solder;
	if (tmp_vel_proj.Size() > 0.05f)
	{
		weapon_twist_target = tmp_vel_proj.GetSafeNormal();
		weapon_twist_solder = weapon_twist_solder + tmp_vel_proj*(0.05f / tmp_vel_proj.Size());
	}

	weapon_twist_target = weapon_twist_target.GetSafeNormal() - FVector::DotProduct(weapon_twist_target.GetSafeNormal(), gv_up)*gv_up;
	weapon_twist_target = weapon_twist_target.GetSafeNormal();
	FVector tmp_forward_ref;

	tmp_forward_ref = held_weapon->getHeadComponent()->GetForwardVector();
	//held_weapon->getforwa

	wtc.error = FMath::Acos(FVector::DotProduct(tmp_forward_ref, weapon_twist_target))*180.f / PI;
	float tmp_forward_angle_ref = FMath::Acos(FVector::DotProduct(-tmp_forward_ref, weapon_twist_target))*180.f / PI;

	if (tmp_forward_angle_ref < wtc.error)
	{
		wtc.error = tmp_forward_angle_ref;
		tmp_forward_ref = -tmp_forward_ref;
	}
	//weapon_vis->SetRelativeRotation(FRotator(0.f, sword_twist.error, 0.f));
	FVector tmp_ref_vec = FVector::CrossProduct(tmp_forward_ref, weapon_twist_target);
	FVector tmp_vis_u = gv_up;
	if (!FVector::Coincident(tmp_ref_vec, tmp_vis_u, 0.0001f))
		wtc.error = -wtc.error;

	wtc.integral = wtc.integral + wtc.error * DeltaTime;
	wtc.derivative = (wtc.error - wtc.prev_err) / DeltaTime;

	wtc.adjustment = wtc.P * wtc.error +
		wtc.I * wtc.integral +
		wtc.D * wtc.derivative;
	wtc.prev_err = wtc.error;

	held_weapon->getHeadComponent()->AddLocalRotation(FRotator(0.f, wtc.adjustment, 0.f));

}


void AJointCharacterTest::weaponGrabControl(float DeltaTime, FBodyInstance* BodyInstance)
{
	FBodyInstance* tmp_shaft = held_weapon->getShaftComponent()->GetBodyInstance();
	USceneComponent* tmp_attachment = held_weapon->getAttachmentPoint();

	wgc.error = gv_pos - tmp_attachment->GetComponentLocation();

	if (wgc.error.Size() < 10.f && FMath::Abs(wgrc.error) < 7.f && FMath::Abs(wgic.error) < 7.f)
	{
		held_weapon->getShaftComponent()->SetAngularDamping(0.0f);
		held_weapon->getShaftComponent()->SetLinearDamping(0.01f);
		attachWeapon(held_weapon);

		grabbing_weapon = false;
	}

	//weapon position --------------------------------------------------------------
	wgc.integral = wgc.integral + wgc.error * DeltaTime;
	wgc.derivative = (wgc.error - wgc.prev_err) / DeltaTime;

	wgc.adjustment = wgc.P * wgc.error +
		wgc.I * wgc.integral +
		wgc.D * wgc.derivative;
	wgc.prev_err = wgc.error;

	float mass = tmp_shaft->GetBodyMass();
	tmp_shaft->AddForceAtPosition(wgc.adjustment*(mass)/*+mass*980.f*FVector::UpVector*/, tmp_attachment->GetComponentLocation(), false);

	//weapon rotation --------------------------------------------------------------
	FVector tmp_wep_forward = tmp_shaft->GetUnrealWorldTransform().GetUnitAxis(EAxis::Z);
	FVector tmp_wep_forward_xy = tmp_wep_forward;
	tmp_wep_forward_xy.Z = 0.f;
	tmp_wep_forward_xy.Normalize();
	if (rot_forward)
	{
		wgrc.error = (FMath::Acos(FVector::DotProduct(current_griph_xydir, tmp_wep_forward_xy))*180.f / PI);// +target_ang_speed;;
		if (FVector::CrossProduct(tmp_wep_forward_xy, current_griph_xydir).Z < 0)
			wgrc.error = -wgrc.error;
	}
	else
	{
		wgrc.error = (FMath::Acos(FVector::DotProduct(-current_griph_xydir, tmp_wep_forward_xy))*180.f / PI);// +target_ang_speed;;
		if (FVector::CrossProduct(tmp_wep_forward_xy,  - current_griph_xydir).Z < 0)
			wgrc.error = -wgrc.error;
	}
	

	wgrc.integral = wgrc.integral + wgrc.error * DeltaTime;
	wgrc.derivative = (wgrc.error - wgrc.prev_err) / DeltaTime;

	wgrc.adjustment = wgrc.P * wgrc.error +
		wgrc.I * wgrc.integral +
		wgrc.D * wgrc.derivative;
	wgrc.prev_err = wgrc.error;

	FVector tmp_bi = tmp_shaft->GetBodyInertiaTensor();
	float tmp_inertia = tmp_bi.X*tmp_wep_forward.X*tmp_wep_forward.X + tmp_bi.Y*tmp_wep_forward.Y*tmp_wep_forward.Y + tmp_bi.Z*tmp_wep_forward.Z*tmp_wep_forward.Z;
	FVector rot_torque = FVector::UpVector*-wgrc.adjustment*tmp_inertia;
	/*tmp_shaft->AddTorque(rot_torque, false);*/

	FVector tmp_rot_force = FVector::CrossProduct(FVector::UpVector, tmp_wep_forward_xy);
	tmp_shaft->AddForceAtPosition(tmp_rot_force*wgrc.adjustment, tmp_attachment->GetComponentLocation() + 100 * tmp_wep_forward, false);
	tmp_shaft->AddForceAtPosition(tmp_rot_force*-wgrc.adjustment, tmp_attachment->GetComponentLocation() - 100 * tmp_wep_forward, false);

	//weapon incline --------------------------------------------------------------
	wgic.error = (FMath::Acos(FVector::DotProduct(gv_up, gh_up))*180.f / PI) - (FMath::Acos(FVector::DotProduct(tmp_wep_forward, gh_up))*180.f / PI);
	wgic.integral = wgic.integral + wgic.error * DeltaTime;
	wgic.derivative = (wgic.error - wgic.prev_err) / DeltaTime;

	wgic.adjustment = wgic.P * wgic.error +
		wgic.I * wgic.integral +
		wgic.D * wgic.derivative;
	wgic.prev_err = wgic.error;

	FVector tmp_right = FVector::CrossProduct(FVector::UpVector, tmp_wep_forward);
	tmp_rot_force = FVector::CrossProduct(tmp_right.GetSafeNormal(), tmp_wep_forward);
	tmp_shaft->AddForceAtPosition(tmp_rot_force*wgic.adjustment, tmp_attachment->GetComponentLocation() + 100 * tmp_wep_forward, false);
	tmp_shaft->AddForceAtPosition(tmp_rot_force*-wgic.adjustment, tmp_attachment->GetComponentLocation() - 100 * tmp_wep_forward, false);

	if (!grabbing_weapon)
	{
		wgc.prev_err = FVector::ZeroVector;
		wgrc.prev_err = 0.f;
		wgic.prev_err = 0.f;
	}

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

		grip_v_attachment->SetAngularSwing1Limit(EAngularConstraintMotion::ACM_Locked, 0.0f);
		grip_v_attachment->SetAngularSwing2Limit(EAngularConstraintMotion::ACM_Free, 120);
		grip_v_attachment->ConstraintInstance.AngularRotationOffset = FRotator(0.f, 0.f, 0.f);
		grip_v_attachment->SetAngularTwistLimit(EAngularConstraintMotion::ACM_Locked, 0.0f);

		grip_h_attachment->SetLinearXLimit(ELinearConstraintMotion::LCM_Limited, 150.0f);
		grip_h_attachment->SetLinearYLimit(ELinearConstraintMotion::LCM_Limited, 150.0f);
		grip_h_attachment->SetLinearZLimit(ELinearConstraintMotion::LCM_Limited, 150.0f);
		grip_h_attachment->SetAngularSwing1Limit(EAngularConstraintMotion::ACM_Free, 0.0f);
		grip_h_attachment->SetConstraintReferencePosition(EConstraintFrame::Frame2, FVector(0.f, 0.f, 41.6666f));


		grip_v->SetEnableGravity(false);
	}
	else
	{
		fight_mode = false;
		fight_mode_state = 3;

		grip_v_attachment->SetAngularSwing1Limit(EAngularConstraintMotion::ACM_Free, 0.0f);
		grip_v_attachment->SetAngularSwing2Limit(EAngularConstraintMotion::ACM_Free, 90.f);
		grip_v_attachment->SetAngularTwistLimit(EAngularConstraintMotion::ACM_Free, 0.0f);

		grip_h_attachment->SetConstraintReferencePosition(EConstraintFrame::Frame2, FVector(0.f, -20.f, -50.f));
		grip_h_attachment->SetLinearXLimit(ELinearConstraintMotion::LCM_Locked, 0.0f);
		grip_h_attachment->SetLinearYLimit(ELinearConstraintMotion::LCM_Locked, 0.0f);
		grip_h_attachment->SetLinearZLimit(ELinearConstraintMotion::LCM_Locked, 0.0f);
		grip_h_attachment->SetAngularSwing1Limit(EAngularConstraintMotion::ACM_Locked, 0.0f);
		
		//weapon_axis_attachment->SetConstraintReferencePosition(EConstraintFrame::Frame2, FVector(0.f, -20.f, 0.f));

		grip_v->SetEnableGravity(true);
	}
}

void AJointCharacterTest::fightModeOff()
{
	/*fight_mode = false;
	fight_mode_state = 3;

	weapon_attachment->SetAngularSwing1Limit(EAngularConstraintMotion::ACM_Free, 0.0f);
	weapon_attachment->SetAngularSwing2Limit(EAngularConstraintMotion::ACM_Free, 90.f);
	weapon_attachment->SetAngularTwistLimit(EAngularConstraintMotion::ACM_Free, 0.0f);

	grip_h_attachment->SetLinearXLimit(ELinearConstraintMotion::LCM_Locked, 0.0f);
	grip_h_attachment->SetLinearYLimit(ELinearConstraintMotion::LCM_Locked, 0.0f);
	grip_h_attachment->SetLinearZLimit(ELinearConstraintMotion::LCM_Locked, 0.0f);
	grip_h_attachment->SetConstraintReferencePosition(EConstraintFrame::Frame2, FVector(-20.f, 0.f, -50.f));

	weapon->SetEnableGravity(true);*/

}

void AJointCharacterTest::release()
{
	wep_extended = false;
	if (holding_weapon)
	{
		/*UE_LOG(LogTemp, Warning, TEXT("----------------------"));
		UE_LOG(LogTemp, Warning, TEXT("grip_vmass before unweld: %f"), grip_v_bi->GetBodyMass());
		UE_LOG(LogTemp, Warning, TEXT("grip_v inertia before unweld: %s"), *grip_v_bi->GetBodyInertiaTensor().ToString());*/

		grip_v->UnWeldChildren();
		grip_v->UpdateBodySetup();
		UE_LOG(LogTemp, Warning, TEXT("grip_vmass after unweld: %f"), grip_v_bi->GetBodyMass());

		held_weapon->DetachFromActor(FDetachmentTransformRules::KeepWorldTransform);
		held_weapon->getShaftComponent()->SetSimulatePhysics(true);
		held_weapon->getShaftComponent()->SetEnableGravity(true);
		held_weapon->getShaftComponent()->SetPhysicsLinearVelocity(grip_v->GetPhysicsLinearVelocity());
		held_weapon->getShaftComponent()->SetPhysicsAngularVelocity(grip_v->GetPhysicsAngularVelocity());

		held_weapon->deInitGrabbed();

		/*grip_v_attachment->BreakConstraint();
		grip_v->SetEnableGravity(true);*/
		holding_weapon = false;
		wep_extended = false;

		UE_LOG(LogTemp, Warning, TEXT("----------------------"));
		return;
	}
	else if (holding_object)
	{
		grip_v->UnWeldChildren();
		grip_v->UpdateBodySetup();
		holding_object = false;
		return;
	}

}

void AJointCharacterTest::grab()
{
	if (!grabbing_weapon && !holding_weapon)
	{
		TSet<AActor*> overlaps;
		grip_v->GetOverlappingActors(overlaps);

		TArray<FHitResult> traces;
		FCollisionQueryParams RV_TraceParams = FCollisionQueryParams(FName(TEXT("RV_Trace")), true, this);
		RV_TraceParams.bTraceComplex = true;
		RV_TraceParams.bTraceAsyncScene = true;
		RV_TraceParams.bReturnPhysicalMaterial = false;

		GetWorld()->LineTraceMultiByChannel(traces, grip_h->GetComponentLocation() + FVector::UpVector*150.f, grip_h->GetComponentLocation() - FVector::UpVector*300.f, ECollisionChannel::ECC_PhysicsBody, RV_TraceParams);

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
	UE_LOG(LogTemp, Warning, TEXT("----------------------"));

	UE_LOG(LogTemp, Warning, TEXT("grip_v mass before weld: %f"), grip_v_bi->GetBodyMass());
	UE_LOG(LogTemp, Warning, TEXT("grip_v inertia before weld: %s"), *grip_v_bi->GetBodyInertiaTensor().ToString());

	UCapsuleComponent* tmp_shaft = dynamic_cast<UCapsuleComponent*>(held_weapon->GetRootComponent());
	//tmp_shaft->SetSimulatePhysics(false);
	UE_LOG(LogTemp, Warning, TEXT("tmp_shaft mass before weld: %f"), tmp_shaft->GetBodyInstance()->GetBodyMass());
	UE_LOG(LogTemp, Warning, TEXT("tmp_shaft inertia before weld: %s"), *tmp_shaft->GetBodyInstance()->GetBodyInertiaTensor().ToString());

	USceneComponent* tmp_handle = dynamic_cast<USceneComponent*>(held_weapon->GetComponentsByTag(USceneComponent::StaticClass(), "handle_point").Top());
	held_weapon->SetActorRotation(grip_v->GetComponentRotation());
	//float tmp_forward_error = (FMath::Acos(FVector::DotProduct(target_wep_dir_xy.GetSafeNormal(), current_griph_xydir))*180.f / PI);// +target_ang_speed;
	//if (tmp_forward_error > 90.f)
	//{
	//	held_weapon->AddActorWorldRotation();
	//}
	FVector handle_offset = held_weapon->GetActorLocation() - tmp_handle->GetComponentLocation();
	held_weapon->SetActorLocation(grip_v->GetComponentLocation() + (handle_offset));

	//UBoxComponent* tmp_head = dynamic_cast<UBoxComponent*>(held_weapon->GetComponentsByTag(UBoxComponent::StaticClass(), "head").Top());

	//tmp_shaft->GetBodyInstance()->Weld(grip_v_bi, )
	held_weapon->AttachRootComponentTo(grip_v, NAME_None, EAttachLocation::KeepWorldPosition, true);
	//tmp_shaft->WeldTo(grip_v);
	grip_v->UpdateBodySetup();
	calculateWepInertia();

	UE_LOG(LogTemp, Warning, TEXT("grip_v mass after weld: %f"), grip_v_bi->GetBodyMass());
	UE_LOG(LogTemp, Warning, TEXT("grip_v inertia after weld: %s"), *grip_v_bi->GetBodyInertiaTensor().ToString());

	UE_LOG(LogTemp, Warning, TEXT("----------------------"));
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
	spine_attachment->SetAngularSwing1Limit(EAngularConstraintMotion::ACM_Locked, 0.0f);
	spine_attachment->SetAngularSwing2Limit(EAngularConstraintMotion::ACM_Locked, 0.0f);
	spine_attachment->SetAngularTwistLimit(EAngularConstraintMotion::ACM_Locked, 0.0f);
	spine_attachment->SetDisableCollision(true);
}

void AJointCharacterTest::initWeapon()
{
	// Weapon settings
	weapon_axis = CreateDefaultSubobject<USphereComponent>(TEXT("WeaponAxis"));
	weapon_axis->SetupAttachment(RootComponent);
	weapon_axis->SetRelativeLocation(FVector(0.f, 0.f, 50));
	weapon_axis->SetRelativeScale3D(FVector(1.f, 1.f, 1.f));
	weapon_axis->SetSphereRadius(15.f);
	weapon_axis->SetSimulatePhysics(false);
	weapon_axis->SetEnableGravity(false);
	weapon_axis_vis = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("WeaponAxisVis"));
	weapon_axis_vis->SetupAttachment(weapon_axis);
	

	weapon_axis_attachment = CreateDefaultSubobject<UPhysicsConstraintComponent>(TEXT("WeaponAxisAttachment"));
	weapon_axis_attachment->SetupAttachment(torso);
	weapon_axis_attachment->SetRelativeLocation(FVector(0.f, 0.f, 0.f));

	grip_h = CreateDefaultSubobject<USphereComponent>(TEXT("WeaponHandle1"));
	grip_h->SetupAttachment(weapon_axis);
	grip_h->SetRelativeLocation(FVector(0.f, 0.f, 0.f));
	grip_h->SetRelativeScale3D(FVector(1.f, 1.f, 1.f));
	grip_h->SetSphereRadius(20.f);
	grip_h->SetSimulatePhysics(true);
	grip_h->SetEnableGravity(false);
	grip_h->SetMassOverrideInKg(NAME_None, 15.f, true);
	grip_h->SetPhysicsMaxAngularVelocity(5000.f);
	grip_h_vis = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("WeaponHandle1Vis"));
	grip_h_vis->SetupAttachment(grip_h);
	
	grip_h_attachment = CreateDefaultSubobject<UPhysicsConstraintComponent>(TEXT("WeaponHandle1Attachment"));
	grip_h_attachment->SetupAttachment(weapon_axis);
	grip_h_attachment->SetRelativeLocation(FVector(0.f, 0.f, 0.f));
	
	grip_v = CreateDefaultSubobject<USphereComponent>(TEXT("grip_v"));
	grip_v->SetupAttachment(grip_h);
	grip_v->SetRelativeLocation(FVector(0.f, 0.f, 0.f));
	grip_v->SetRelativeScale3D(FVector(1.f, 1.f, 1.f));
	grip_v->SetSphereRadius(20.f);
	grip_v->SetSimulatePhysics(true);
	grip_v->SetEnableGravity(false);
	grip_v->SetPhysicsMaxAngularVelocity(5000.f);
	grip_v->SetMassOverrideInKg(NAME_None, 7.5f, true);
	grip_v_vis = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("grip_vVis"));
	grip_v_vis->SetupAttachment(grip_v);
	grip_v_attachment = CreateDefaultSubobject<UPhysicsConstraintComponent>(TEXT("grip_vAttachment"));
	grip_v_attachment->SetupAttachment(grip_h);
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

	/*weapon_axis_attachment->SetConstrainedComponents(torso, NAME_None, weapon_axis, NAME_None);
	weapon_axis_attachment->SetLinearXLimit(ELinearConstraintMotion::LCM_Locked, 4.0f);
	weapon_axis_attachment->SetLinearYLimit(ELinearConstraintMotion::LCM_Locked, 4.0f);
	weapon_axis_attachment->SetLinearZLimit(ELinearConstraintMotion::LCM_Locked, 4.0f);
	weapon_axis_attachment->SetAngularSwing1Limit(EAngularConstraintMotion::ACM_Locked, 0.0f);
	weapon_axis_attachment->SetAngularSwing2Limit(EAngularConstraintMotion::ACM_Locked, 0.0f);
	weapon_axis_attachment->SetAngularTwistLimit(EAngularConstraintMotion::ACM_Locked, 0.0f);
	weapon_axis_attachment->SetDisableCollision(true);*/

	grip_h_attachment->SetConstrainedComponents(weapon_axis, NAME_None, grip_h, NAME_None);
	grip_h_attachment->SetLinearXLimit(ELinearConstraintMotion::LCM_Limited, 0.0f);
	grip_h_attachment->SetLinearYLimit(ELinearConstraintMotion::LCM_Limited, 0.0f);
	grip_h_attachment->SetLinearZLimit(ELinearConstraintMotion::LCM_Limited, 0.0f);
	grip_h_attachment->SetAngularSwing1Limit(EAngularConstraintMotion::ACM_Free, 0.0f);
	grip_h_attachment->SetAngularSwing2Limit(EAngularConstraintMotion::ACM_Locked, 0.0f);
	grip_h_attachment->SetAngularTwistLimit(EAngularConstraintMotion::ACM_Locked, 0.0f);
	grip_h_attachment->SetDisableCollision(true);

	grip_v_attachment->SetConstrainedComponents(grip_h, NAME_None, grip_v, NAME_None);
	grip_v_attachment->SetLinearXLimit(ELinearConstraintMotion::LCM_Locked, 0.0f);
	grip_v_attachment->SetLinearYLimit(ELinearConstraintMotion::LCM_Locked, 0.0f);
	grip_v_attachment->SetLinearZLimit(ELinearConstraintMotion::LCM_Locked, 0.0f);
	grip_v_attachment->SetAngularSwing1Limit(EAngularConstraintMotion::ACM_Locked, 0.0f);
	grip_v_attachment->SetAngularSwing2Limit(EAngularConstraintMotion::ACM_Limited, 90.f);
	grip_v_attachment->ConstraintInstance.AngularRotationOffset = FRotator(0.f, 0.f, 0.f);
	grip_v_attachment->SetAngularTwistLimit(EAngularConstraintMotion::ACM_Locked, 0.0f);
	grip_v_attachment->SetDisableCollision(true);
	grip_v_attachment->SetAngularDriveMode(EAngularDriveMode::TwistAndSwing);
	grip_v_attachment->SetAngularVelocityDriveTwistAndSwing(true, true);
	grip_v_attachment->SetAngularDriveParams(0.f, 10000.f, 10000000000000000000000.f);


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
	hover_height.target = target_hover_height;
	hover_height.max_adjustment = 1000000;
	hover_height.P = 300.f;
	hover_height.I = 5;
	hover_height.D = 10.0f;
	hover_height.integral = 0.f;
	hover_height.prev_err = 0.f;

	gpc.target = FVector::UpVector*60.f;
	gpc.max_adjustment = FVector(100000.f, 100000.f, 100000.f);
	gpc.P = FVector(1.f, 1.f, 1.f);
	gpc.I = FVector(0.f, 0.f, 0.f);
	gpc.D = FVector(0.5f, 0.5f, 0.5f);
	gpc.integral = FVector::ZeroVector;

	target_wep_dir = FVector::UpVector;
	weapon_twist_solder = FVector::UpVector;
	weapon_twist_target = FVector::RightVector;
	prev_target_wep_dir = FVector::ForwardVector;
	prev_target_wep_dir_xy = FVector::ForwardVector;
	grc.target = 0.0f;
	grc.max_adjustment = 1000000;
	grc.P = 8.f;
	grc.I = 0.f;
	grc.D = 0.6f;
	grc.integral = 0.f;

	grsc.target = 0.0f;
	grsc.max_adjustment = 1000000;
	grsc.P = 0.f;
	grsc.I = 0.f;
	grsc.D = 0.0f;
	grsc.integral = 0.f;

	gic.target = 0.0f;
	gic.max_adjustment = 1000000;
	gic.P = 5000;
	gic.I = 0.0f;
	gic.D = 100;
	gic.integral = 0.f;

	wtc.target = 0.0f;
	wtc.max_adjustment = 5;
	wtc.P = 0.1;
	wtc.I = 0.0f;
	wtc.D = 0.0001f;
	wtc.integral = 0.f;

	wgc.max_adjustment = FVector(100000.f, 100000.f, 100000.f);
	wgc.P = FVector(10.f, 10.f, 10.f);
	wgc.I = FVector(0.f, 0.f, 0.f);
	wgc.D = FVector(5.5f, 5.5f, 5.5f);
	wgc.integral = FVector::ZeroVector;

	wgrc.target = 0.0f;
	wgrc.max_adjustment = 1000000;
	wgrc.P = 100.f;
	wgrc.I = 0.f;
	wgrc.D = 8.1f;
	wgrc.integral = 0.f;

	wgic.target = 0.0f;
	wgic.max_adjustment = 1000000;
	wgic.P = 5000.f;
	wgic.I = 0.f;
	wgic.D = 100.f;
	wgic.integral = 0.f;

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

	axis_target_rot = FRotator(0.f, 150.f, 0.f);
	arm_target_rot = FRotator(-65.f, 0.f, 0.f);
	target_arm_length = 500.f;
}

void AJointCharacterTest::initCustomPhysics()
{
	OnCalculateCustomHoverPhysics.BindUObject(this, &AJointCharacterTest::customHoverPhysics);
	OnCalculateCustomStabilizerPhysics.BindUObject(this, &AJointCharacterTest::customStabilizerPhysics);
	torso_bi = torso->GetBodyInstance();
	spine_bi = spine->GetBodyInstance();

	OnCalculateControlGripPhysics.BindUObject(this, &AJointCharacterTest::ControlGripPhysics);
	OnCalculateCustomInitGripPhysics.BindUObject(this, &AJointCharacterTest::customInitGripPhysics);
	OnCalculateControlGripPositionPhysics.BindUObject(this, &AJointCharacterTest::ControlGripPositionPhysics);
	OnCalculateControlGripDirectionPhysics.BindUObject(this, &AJointCharacterTest::ControlGripDirectionPhysics);
	OnCalculateControlGripInclinePhysics.BindUObject(this, &AJointCharacterTest::ControlGripInclinePhysics);
	OnCalculateControlWeaponTwistPhysics.BindUObject(this, &AJointCharacterTest::ControlWeaponTwistPhysics);

	OnCalculateWeaponGrabControl.BindUObject(this, &AJointCharacterTest::weaponGrabControl);

	weapon_axis_bi = weapon_axis->GetBodyInstance();
	grip_h_bi = grip_h->GetBodyInstance();
	//weapon_handle_2_bi = weapon_handle_2->GetBodyInstance();
	
	grip_v_bi = grip_v->GetBodyInstance();

	UE_LOG(LogTemp, Warning, TEXT("grip+wep inertia %s"), *offset_wep_inertia.ToString());

	//rolling_body->OnComponentHit.AddDynamic(this, &AJointCharacterTest::OnHit);
	torso->OnComponentHit.AddDynamic(this, &AJointCharacterTest::OnBodyHit);
	spine->OnComponentHit.AddDynamic(this, &AJointCharacterTest::OnBodyHit);

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
	
	grip_h_bi->SetBodyTransform(FTransform(grip_h_bi->GetUnrealWorldTransform().GetTranslation() + FVector::UpVector*70.f), true);
	grip_v_bi->SetBodyTransform(FTransform(grip_v_bi->GetUnrealWorldTransform().GetTranslation() + FVector::UpVector*70.f), true);
}

void AJointCharacterTest::calculateWepInertia()
{
	FVector ti = grip_v_bi->GetBodyInertiaTensor();
	//FVector tp = FVector(0.f, 0.f, 75.f);
	
	FVector tp = grip_v->ComponentToWorld.InverseTransformPositionNoScale(grip_v_bi->GetCOMPosition());// -grip_v->GetComponentLocation();
	offset_wep_inertia = FVector(ti.X + grip_v_bi->GetBodyMass()*(tp.Y*tp.Y + tp.Z*tp.Z),
		ti.Y + grip_v_bi->GetBodyMass()*(tp.X*tp.X + tp.Z*tp.Z),
		ti.Z + grip_v_bi->GetBodyMass()*(tp.X*tp.X + tp.Y*tp.Y));

	UE_LOG(LogTemp, Warning, TEXT("offset inertia: %s"), *offset_wep_inertia.ToString());
}