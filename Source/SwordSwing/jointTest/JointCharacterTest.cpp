// Fill out your copyright notice in the Description page of Project Settings.

#include "SwordSwing.h"
#include "AudioDevice.h"
#include "FightMode.h"
#include "FightModeBase.h"
#include "LastManMode.h"
#include "ActiveSound.h"
#include "JointCharacterTest.h"



// Sets default values
AJointCharacterTest::AJointCharacterTest()
{
 	// Set this pawn to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
	RootComponent = CreateDefaultSubobject<USceneComponent>(TEXT("RootComponent"));

	this->bCanBeDamaged = true;
	//body settings
	
	initBody();
	initWeapon();

	initCamera();

	initWeaponJoints();
	//AutoPossessPlayer = EAutoReceiveInput::Player0;
}

// Called when the game starts or when spawned
void AJointCharacterTest::BeginPlay()
{
	Super::BeginPlay();
	
	initBodyJoints();
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
	//	grip_bi->GetCOMPosition(),
	//	10,  					//size
	//	FColor(255, 0, 255),  //pink
	//	false,  				//persistent (never goes away)
	//	0.03 					//point leaves a trail on moving object
	//);

	if (alive)
	//	if (false)
	{
		camera_axis->SetWorldLocation(torsoBI->GetCOMPosition()+ FVector::UpVector*30.f);
		//grip_axis->SetWorldLocation(torso->GetCenterOfMass() + FVector(0.f, 0.f, -15.f));
		//grip_axis_bi->SetBodyTransform(FTransform(torso_bi->GetCOMPosition() + FVector(0.f, 0.f, -15.f)), true);
		
		if (can_swing)
		{
			//if (fight_mode)
			{
				arm_BIs[1]->AddCustomPhysics(OnCalculateControlGripPhysics);
			}
		}

		if (!holding_weapon)
		{
			gripIndicatorCalculations(DeltaTime);
		}


		torsoBI->AddCustomPhysics(OnCalculateCustomHoverPhysics);
		torsoBI->AddCustomPhysics(CalculateControlBody);

		if (can_move)
		{
			movementCalculations(DeltaTime);
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
void AJointCharacterTest::setFOV(int _fov)
{
	camera->SetFieldOfView(_fov);
}

void AJointCharacterTest::movementCalculations(float DeltaTime)
{

	//Planar movement speed -------------------------------------------------------------------------------------

	if(!movement_input.IsZero())
		target_direction = (camera_axis->GetForwardVector()*movement_input.X + camera_axis->GetRightVector()* movement_input.Y).GetSafeNormal();

	FVector2D movement_input_norm = movement_input.GetSafeNormal();

	FVector curr_vel = torsoBI->GetUnrealWorldVelocity();
	FVector2D curr_vel2D = FVector2D(curr_vel.X, curr_vel.Y);
	UE_LOG(LogTemp, Warning, TEXT("current vel %f"), curr_vel2D.Size());
	UE_LOG(LogTemp, Warning, TEXT("current err %f"), movement_velocity.error.Size());

	FVector target_vel;
	//if (!dashing){
		target_vel = (camera_axis->GetForwardVector()*movement_input.X + camera_axis->GetRightVector()* movement_input.Y)*target_speed;
		FVector2D target_vel2D = FVector2D(target_vel.X, target_vel.Y);
		movement_velocity.error = target_vel2D - curr_vel2D;
	//}
	//else {
	//	target_vel = (camera_axis->GetForwardVector()*movement_input.X + camera_axis->GetRightVector()* movement_input.Y)*dash_speed;
	//	FVector2D target_vel2D = FVector2D(target_vel.X, target_vel.Y);
	//	movement_velocity.error = target_vel2D - curr_vel2D;
	//	if (movement_velocity.error.Size() < 100.f)
	//		dashing = false;
	//}

	

	
	movement_velocity.integral = movement_velocity.integral + movement_velocity.error * DeltaTime;
	movement_velocity.derivative = (movement_velocity.error - movement_velocity.prev_err) / DeltaTime;

	movement_velocity.adjustment = movement_velocity.P * movement_velocity.error +
									movement_velocity.I * movement_velocity.integral +
									movement_velocity.D * movement_velocity.derivative;
	movement_velocity.prev_err = movement_velocity.error;

	//FVector move_force = ((target_vel - curr_vel) )*rolling_body->GetMass();
	//move_force.Z = 0;
	FVector move_force = FVector(movement_velocity.adjustment.X, movement_velocity.adjustment.Y, 0.f)*torsoBI->GetBodyMass();
	
	torsoBI->AddForce(move_force );

	//dashing -------------------------------------------------------------------------------------

	if (dashing) {
		if (dash_force_timer == 0.f) {//the factor of 100 is because unreal apparently applies forces in kg*cm*s^(-2)
			dash_force = (body->GetMass()*dash_speed - body->GetMass()*curr_vel2D.Size()) / jump_force_time;
		}
		if (dash_force_timer < dash_force_time) {
			torsoBI->AddForce(target_direction*dash_force*DeltaTime);
			dash_force_timer += DeltaTime;
		}

		dash_cd_timer += DeltaTime;
		body_trail->SetFloatParameter(FName("trailLifetime"), FMath::Lerp(1.0f, 0.0f, dash_cd_timer/ dash_cd));

		if (dash_cd_timer > dash_cd) {
			dashing = false;
			dash_force_timer = 0.0f;
			dash_cd_timer = 0.0f;
			body_trail->EndTrails();
		}
	}

	//Jumping -------------------------------------------------------------------------------------

	if (jumping) {
		if (curr_jump_time == 0.f){//the factor of 100 is because unreal apparently applies forces in kg*cm*s^(-2)
			jump_force = (((FMath::Sqrt(2.f*1000.f*jump_height) - torsoBI->GetUnrealWorldVelocity().Z) / jump_force_time) + 1000.f)*(body->GetMass()) * 100.f;
		}

		torsoBI->AddForce(FVector::UpVector*jump_force*DeltaTime);

		curr_jump_time += DeltaTime;
		if (curr_jump_time > jump_force_time) {
			jumping = false;
			curr_jump_time = 0.0f;
		}
	}
}
//SETTERS / GETTERS ===========================================================================

void  AJointCharacterTest::gripIndicatorCalculations(float DeltaTime)
{
	FCollisionQueryParams RV_TraceParams = FCollisionQueryParams(FName(TEXT("RV_Trace")), true, this);
	RV_TraceParams.bTraceComplex = true;
	RV_TraceParams.bTraceAsyncScene = false;
	RV_TraceParams.bReturnPhysicalMaterial = false;

	FCollisionObjectQueryParams rv_objects;
	rv_objects.AddObjectTypesToQuery(ECollisionChannel::ECC_WorldStatic);
	rv_objects.AddObjectTypesToQuery(ECollisionChannel::ECC_PhysicsBody);


	//Re-initialize hit info
	FHitResult rv_hit(ForceInit);

	FVector start = g_pos;
	FVector end = start - FVector::UpVector*200.f;

	//call GetWorld() from within an actor extending class
	GetWorld()->LineTraceSingleByObjectType(rv_hit,        //result
		start,    //start
		end, //end
		rv_objects, //collision channel
		RV_TraceParams);

	if (rv_hit.bBlockingHit)
	{
		grip_indicator_decal->SetVisibility(true);
		grip_indicator_decal->SetWorldLocation(rv_hit.Location);
		grip_indicator_beam->SetBeamTargetPoint(0, g_pos -FVector::UpVector*rv_hit.Distance, 0);
		

		DrawDebugPoint(
			GetWorld(),
			rv_hit.Location,
			5,  					//size
			FColor(255, 0, 255),  //pink
			false,  				//persistent (never goes away)
			0.03 					//point leaves a trail on moving object
		);
		
	}
	else
	{
		grip_indicator_beam->SetBeamTargetPoint(0, g_pos - FVector::UpVector*200.f, 0);
		grip_indicator_decal->SetVisibility(false);
	}


	
}

void AJointCharacterTest::setCanMove(bool new_state)
{
	can_move = new_state;
}
void AJointCharacterTest::setCanSwing(bool new_state)
{
	can_swing = new_state;
}

void AJointCharacterTest::disableSwingAbility()
{
	can_swing = false;
}
void AJointCharacterTest::enableSwingAbility()
{
	can_swing = true;
}

void AJointCharacterTest::setPlayerSpecificMaterial(UMaterial* mat)
{
	body->SetMaterial(3, mat);
}

bool AJointCharacterTest::isAlive()
{
	return alive;
}

bool AJointCharacterTest::isGuarding()
{
	return guarding;
}

//COLLISION RESPONSE CALCULATIONS ===========================================================================

void AJointCharacterTest::OnBodyHit(UPrimitiveComponent* HitComp, AActor* OtherActor, UPrimitiveComponent* OtherComp, FVector NormalImpulse, const FHitResult& Hit)
{
	if (OtherActor != this)
	{
		if (NormalImpulse.Size() > 30000.f)
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

				//body->BreakConstraint();
			}

			release();

			//body->BreakConstraint(FVector::ZeroVector, FVector::ZeroVector, Hit.BoneName);
			FConstraintInstance* tmp_const = body->FindConstraintInstance(Hit.BoneName);
			if (tmp_const)
				tmp_const->TermConstraint();

			alive = false;
			fight_mode = false;
			body->OnComponentHit.RemoveAll(this);

			AFightModeBase* tmp_mode = Cast<AFightModeBase>(GetWorld()->GetAuthGameMode());
			if (tmp_mode)
			{
				if (this->GetNetOwningPlayer())
					tmp_mode->registerDeath(this->GetNetOwningPlayer()->PlayerController);
				else
					tmp_mode->registerDeath(nullptr);
			}
			//HitComp->DestroyComponent();
		}
	}
}

void AJointCharacterTest::FellOutOfWorld(const class UDamageType& dmgType)
{
	Super::FellOutOfWorld(dmgType);
	AFightModeBase* tmp_mode = Cast<AFightModeBase>(GetWorld()->GetAuthGameMode());
	if (tmp_mode)
	{
		if (this->GetNetOwningPlayer())
			tmp_mode->registerDeath(this->GetNetOwningPlayer()->PlayerController);
		else
			tmp_mode->registerDeath(nullptr);
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
		lock_on_targets.Add(OtherComp);
}
void AJointCharacterTest::removePotentialTarget(UPrimitiveComponent* OverlappedComp, AActor* OtherActor, UPrimitiveComponent* OtherComp, int32 OtherBodyIndex)
{
	if (OtherActor != this)
		lock_on_targets.Remove(OtherComp);
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
	RV_TraceParams.bTraceComplex = false;
	RV_TraceParams.bReturnPhysicalMaterial = false;
	RV_TraceParams.MobilityType = EQueryMobilityType::Any;
	//Re-initialize hit info
	FHitResult rv_hit(ForceInit);

	FVector start = BodyInstance->GetCOMPosition();
	FVector end = start - FVector::UpVector*hover_height.target*2.f;

	//call GetWorld() from within an actor extending class
	//GetWorld()->LineTraceSingleByObjectType(rv_hit,        //result
	//	start,    //start
	//	end, //end
	//	FCollisionObjectQueryParams::AllStaticObjects, //collision channel
	//	RV_TraceParams);

	//GetWorld()->SweepSingleByChannel(rv_hit, start, end, FQuat(), ECollisionChannel::ECC_WorldStatic, FCollisionShape::MakeSphere(20.f), RV_TraceParams);
	//GetWorld()->SweepSingleByObjectType(rv_hit, start, end, FQuat(), FCollisionObjectQueryParams::AllObjects, FCollisionShape::MakeSphere(20.f), RV_TraceParams);
	GetWorld()->SweepSingleByProfile(rv_hit, start, end, FQuat(), FName("OverlapAllTerrain"), FCollisionShape::MakeSphere(20.f), RV_TraceParams);
	//GetWorld()->LineTraceSingleByChannel(
	//	rv_hit,        //result
	//	start,    //start
	//	end, //end
	//	ECC_WorldStatic, //collision channel
	//	RV_TraceParams
	//);

	//DrawDebugPoint(
	//	GetWorld(),
	//	BodyInstance->GetUnrealWorldTransform().GetLocation(),
	//	5,  					//size
	//	FColor(255, 0, 0),  //pink
	//	false,  				//persistent (never goes away)
	//	0.03 					//point leaves a trail on moving object
	//);
	//UE_LOG(LogTemp, Warning, TEXT("body mass %f"), BodyInstance->GetBodyMass());


	if (rv_hit.bBlockingHit)
	{
		hover_height.error = hover_height.target - rv_hit.Distance;
		hover_height.integral = hover_height.integral + hover_height.error * DeltaTime;
		hover_height.derivative = (hover_height.error - hover_height.prev_err) / DeltaTime;

		hover_height.adjustment = hover_height.P * hover_height.error +
			hover_height.I * hover_height.integral +
			hover_height.D * hover_height.derivative;
		hover_height.adjustment = FMath::Min(FMath::Max(0.0f, hover_height.adjustment), hover_height.max_adjustment) * body->GetMass();

		hover_height.prev_err = hover_height.error;
		BodyInstance->AddForce(FVector::UpVector * hover_height.adjustment);

	}
	else
	{
		hover_height.prev_err = 0.0;
		hover_height.integral = 0.0;
	}
}
//WEAPON CONTROL PHYSICS =======================================================================================================

void AJointCharacterTest::ControlBody(float DeltaTime, FBodyInstance* BodyInstance)
{

	//upperbody
	updateLimbStates(&upbr);
	setPelvisTargets();
	ControlLimb(DeltaTime, &upbr);

	updateLimbStates(upbr.next);
	setTorsoTargets();
	ControlLimb(DeltaTime, upbr.next);

	//right leg
	updateLimbStates(&rlr);
	setRThighTargets();
	ControlLimb(DeltaTime, &rlr);

	updateLimbStates(rlr.next);
	setRShinTargets();
	ControlLimb(DeltaTime, rlr.next);
	//left leg
	updateLimbStates(&llr);
	setLThighTargets();
	ControlLimb(DeltaTime, &llr);

	updateLimbStates(llr.next);
	setLShinTargets();
	ControlLimb(DeltaTime, llr.next);
}

void AJointCharacterTest::setTorsoTargets()
{
	FLimbTarget* torso_tar = &upbr.next->target;
	torso_tar->prev_dir = torso_tar->dir;
	torso_tar->dir = FVector::UpVector;
	torso_tar->prev_dir_xy = torso_tar->dir_xy;
	torso_tar->dir_xy = FVector::ZeroVector;

	torso_tar->twist_dir = (camera_axis->GetForwardVector() - FVector::DotProduct(camera_axis->GetForwardVector(), upbr.next->state.up)* upbr.next->state.up).GetSafeNormal();

}
void AJointCharacterTest::setPelvisTargets()
{
	FLimbTarget* pelv_tar = &upbr.target;
	pelv_tar->prev_dir = pelv_tar->dir;
	pelv_tar->dir = FVector::UpVector;
	pelv_tar->prev_dir_xy = pelv_tar->dir_xy;
	pelv_tar->dir_xy = FVector::ZeroVector;


	pelv_tar->twist_dir = (camera_axis->GetForwardVector() - FVector::DotProduct(camera_axis->GetForwardVector(), upbr.state.up)* upbr.state.up).GetSafeNormal();

	//DrawDebugLine(GetWorld(), upbr.state.pos, upbr.state.pos + pelv_tar->twist_dir*100, FColor(255, 0, 255), false, 0.03, 10, 1.f);
	//DrawDebugLine(GetWorld(), upbr.state.pos, upbr.state.pos + -upbr.state.up * 100, FColor(255, 0, 255), false, 0.03, 10, 1.f);
}

void AJointCharacterTest::setRThighTargets()
{
	FLimbTarget* tmp_rThigh = &rlr.target;
	tmp_rThigh->prev_dir = tmp_rThigh->dir;
	tmp_rThigh->dir = -FVector::UpVector;
	tmp_rThigh->prev_dir_xy = tmp_rThigh->dir_xy;
	tmp_rThigh->dir_xy = FVector::ZeroVector;

	tmp_rThigh->twist_dir = (upbr.state.forward - FVector::DotProduct(upbr.state.forward, rlr.state.up)* rlr.state.up).GetSafeNormal();

}
void AJointCharacterTest::setRShinTargets()
{

}

void AJointCharacterTest::setLThighTargets()
{
	FLimbTarget* tmp_lThigh = &llr.target;
	tmp_lThigh->prev_dir = tmp_lThigh->dir;
	tmp_lThigh->dir = -FVector::UpVector;
	tmp_lThigh->prev_dir_xy = tmp_lThigh->dir_xy;
	tmp_lThigh->dir_xy = FVector::ZeroVector;

	tmp_lThigh->twist_dir = (upbr.state.forward - FVector::DotProduct(upbr.state.forward, llr.state.up)* llr.state.up).GetSafeNormal();
}
void AJointCharacterTest::setLShinTargets()
{

}

void AJointCharacterTest::updateLimbStates(FLimbNode* limb)
{
	limb->state.pos = limb->bi->GetUnrealWorldTransform().GetLocation();
	//x and z axis are swapped
	limb->state.forward = limb->bi->GetUnrealWorldTransform().GetUnitAxis(EAxis::Z);
	limb->state.right = limb->bi->GetUnrealWorldTransform().GetUnitAxis(EAxis::Y);
	limb->state.prev_up = limb->state.up;
	limb->state.up = limb->bi->GetUnrealWorldTransform().GetUnitAxis(EAxis::X);

	//DrawDebugLine(GetWorld(), limb->state.pos, limb->state.pos + limb->state.forward * 100, FColor(255, 0, 0), false, 0.03, 10, 1.f);
	//DrawDebugLine(GetWorld(), limb->state.pos, limb->state.pos + limb->state.right * 100, FColor(0, 255, 0), false, 0.03, 10, 1.f);
	//DrawDebugLine(GetWorld(), limb->state.pos, limb->state.pos + limb->state.up * 100, FColor(0, 0, 255), false, 0.03, 10, 1.f);

	//DrawDebugPoint(
	//	GetWorld(),
	//	limb->state.pos,
	//	10,  					//size
	//	FColor(255, 0, 255),  //pink
	//	false,  				//persistent (never goes away)
	//	0.03 					//point leaves a trail on moving object
	//);


}

void AJointCharacterTest::ControlLimb(float DeltaTime, FLimbNode* limb)
{

	limb->pid->error.X = 1.f - FVector::DotProduct(limb->target.dir, limb->state.up);
	FVector rot_ref = FVector::CrossProduct(limb->state.up, limb->target.dir);
	rot_ref = rot_ref.GetSafeNormal();

	//calculate rotational speed around rotation point
	FVector vel_axis = (limb->state.up - limb->state.prev_up) / DeltaTime;
	limb->pid->error.Y = -vel_axis.Size();
	vel_axis = vel_axis.GetSafeNormal();

	//twist
	limb->pid->error.Z = 1.f - FVector::DotProduct(limb->target.twist_dir, limb->state.forward);
	if (FVector::DotProduct(-limb->state.up, FVector::CrossProduct(limb->state.forward, limb->target.twist_dir).GetSafeNormal()) > 0.f)
		limb->pid->error.Z = -limb->pid->error.Z;

	//Limb chain inertia around velocity and ref axis
	float jvi = 0.f, jri = 0.f, lui = 0.f;
	FMatrix limb_inertia;

	//TODO:: här var du
	FLimbNode* limb_it = limb;
	calculateRelativeInertia(limb_it->bi, limb->state.pos, &limb_inertia);
	jvi += inertiaAboutAxis(limb_inertia, FVector::CrossProduct(limb->state.up, vel_axis).GetSafeNormal());
	jri += inertiaAboutAxis(limb_inertia, rot_ref);
	lui += limb->bi->GetBodyInertiaTensor().Z;
	limb_it = limb_it->next;
	while (limb_it)
	{
		calculateRelativeInertia(limb_it->bi, limb->state.pos, &limb_inertia);
		jvi += inertiaAboutAxis(limb_inertia, FVector::CrossProduct(limb->state.up, vel_axis).GetSafeNormal());
		jri += inertiaAboutAxis(limb_inertia, rot_ref);
		lui += inertiaAboutAxis(limb_inertia, limb->state.up);
		limb_it = limb_it->next;
	}

	//calculate PID
	limb->pid->error.X = limb->pid->error.X*(jri);
	limb->pid->error.Y = limb->pid->error.Y*(jvi);
	limb->pid->error.Z = limb->pid->error.Z*lui;
	

	limb->pid->integral = limb->pid->integral + limb->pid->error * DeltaTime;
	limb->pid->derivative = (limb->pid->error - limb->pid->prev_err) / DeltaTime;

	limb->pid->adjustment = limb->pid->P * limb->pid->error +
		limb->pid->I * limb->pid->integral +
		limb->pid->D * limb->pid->derivative;
	limb->pid->prev_err = limb->pid->error;

	rot_ref = FVector::CrossProduct(rot_ref, limb->state.up).GetSafeNormal();
	limb->bi->AddForceAtPosition(rot_ref*limb->pid->adjustment.X, limb->state.pos + limb->state.up, false);
	limb->bi->AddForceAtPosition(rot_ref*-limb->pid->adjustment.X, limb->state.pos - limb->state.up, false);

	limb->bi->AddForceAtPosition(vel_axis*limb->pid->adjustment.Y, limb->state.pos + limb->state.up, false);
	limb->bi->AddForceAtPosition(vel_axis*-limb->pid->adjustment.Y, limb->state.pos - limb->state.up, false);

	limb->bi->AddTorque(limb->state.up*limb->pid->adjustment.Z, false);
	
}

void AJointCharacterTest::ControlGripPhysics(float DeltaTime, FBodyInstance* BodyInstance)
{
	initInputVars();
	customInitGripPhysics(DeltaTime, BodyInstance);

	ControlArmJointDirectionPhysics(DeltaTime, arm_BIs[0]);
	//ControlArmDirectionPhysics(DeltaTime, BodyInstance);
	ControlArmTwistPhysics(DeltaTime, BodyInstance);
	if (!guard_locked)
	{
		ControlArmJointDirectionPhysics(DeltaTime, arm_BIs[1]);
		//ControlGripDirectionPhysics(DeltaTime, BodyInstance);
	}

	if (guarding && !guard_locked && (1.f - FVector::DotProduct(ajdc_targets[0].dir, ga_up)) < 0.01f && (1.f - FVector::DotProduct(ajdc_targets[1].dir, g_up)) < 0.01f && FMath::Abs(atc.error) < 0.01f)
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
			ajdc_targets[1].dir = (-camera_axis->GetRightVector()*0.3f + camera_axis->GetForwardVector() + camera_axis->GetUpVector()*0.4f).GetSafeNormal();
			ajdc_targets[0].dir = (camera_axis->GetRightVector() - camera_axis->GetUpVector()*0.3f + camera_axis->GetForwardVector()*0.4f).GetSafeNormal();
		}
		else
		{
			input_dir = camera_axis->GetForwardVector();
			wep_extended = false;
			ajdc_targets[1].dir = input_dir;
			ajdc_targets[0].dir = input_dir;

			ajdc_targets[1].dir_xy = ajdc_targets[1].dir - FVector::DotProduct(ajdc_targets[1].dir, camera_axis->GetUpVector())*camera_axis->GetUpVector();

			ajdc_targets[0].dir_xy = ajdc_targets[0].dir - FVector::DotProduct(ajdc_targets[0].dir, camera_axis->GetUpVector())*camera_axis->GetUpVector();
			ajdc_targets[0].dir_xy.Normalize();

			if (ajdc_targets[1].dir_xy.IsNearlyZero())
				ajdc_targets[1].dir_xy = ajdc_targets[1].prev_dir_xy;

			FVector guard_pos(FMath::Cos(PI / 2.5f), 0.f, FMath::Sin(PI / 2.5f));
			guard_pos.Normalize();

			float tmp_angle = FMath::Acos(FVector::DotProduct(ajdc_targets[1].dir_xy.GetSafeNormal(), FVector::ForwardVector))*180.f / PI;
			if (FVector::CrossProduct(FVector::ForwardVector, ajdc_targets[1].dir_xy.GetSafeNormal()).Z < 0)
				tmp_angle = -tmp_angle;
			ajdc_targets[0].dir = guard_pos.RotateAngleAxis(tmp_angle, FVector::UpVector);

			FVector guard_dir(FMath::Cos(-PI / 3.f), 0.f, FMath::Sin(-PI / 3.f));
			guard_dir.Normalize();
			ajdc_targets[1].dir = guard_dir.RotateAngleAxis(tmp_angle, FVector::UpVector);

			
		}
	}
	else
	{

		if (guarding)
		{
			wep_extended = false;
			ajdc_targets[1].dir = input_dir;
			ajdc_targets[0].dir = input_dir;

			ajdc_targets[1].dir_xy = ajdc_targets[1].dir - FVector::DotProduct(ajdc_targets[1].dir, camera_axis->GetUpVector())*camera_axis->GetUpVector();

			ajdc_targets[0].dir_xy = ajdc_targets[0].dir - FVector::DotProduct(ajdc_targets[1].dir, camera_axis->GetUpVector())*camera_axis->GetUpVector();
			ajdc_targets[0].dir_xy.Normalize();

			if (ajdc_targets[1].dir_xy.IsNearlyZero())
				ajdc_targets[1].dir_xy = ajdc_targets[1].prev_dir_xy;

			FVector guard_pos(FMath::Cos(PI / 2.5f), 0.f, FMath::Sin(PI / 2.5f));
			guard_pos.Normalize();

			float tmp_angle = FMath::Acos(FVector::DotProduct(ajdc_targets[1].dir_xy.GetSafeNormal(), FVector::ForwardVector))*180.f / PI;
			if (FVector::CrossProduct(FVector::ForwardVector, ajdc_targets[1].dir_xy.GetSafeNormal()).Z < 0)
				tmp_angle = -tmp_angle;
			ajdc_targets[0].dir = guard_pos.RotateAngleAxis(tmp_angle, FVector::UpVector);

			FVector guard_dir(FMath::Cos(-PI / 3.f), 0.f, FMath::Sin(-PI / 3.f));
			guard_dir.Normalize();
			ajdc_targets[1].dir = guard_dir.RotateAngleAxis(tmp_angle, FVector::UpVector);


		}
		else
		{
			//direction target
			ajdc_targets[1].dir = input_dir;
			if (ajdc_targets[1].dir_xy.IsNearlyZero()) {
				ajdc_targets[1].dir_xy = ajdc_targets[1].prev_dir_xy;
			}
			else
			{
				ajdc_targets[1].prev_dir_xy = ajdc_targets[1].dir_xy;
			}

			ajdc_targets[1].dir_xy = ajdc_targets[1].dir - FVector::DotProduct(ajdc_targets[1].dir, camera_axis->GetUpVector())*camera_axis->GetUpVector();

			ajdc_targets[0].dir = input_dir;
			ajdc_targets[0].dir_xy = ajdc_targets[0].dir - FVector::DotProduct(ajdc_targets[0].dir, camera_axis->GetUpVector())*camera_axis->GetUpVector();
		}
	}


	
}

void AJointCharacterTest::setArmTwistTargets()
{
	FVector ga_up_xy = ga_up;
	ga_up_xy.Z = 0.f;
	//ga_up_xy = ga_up_xy.GetSafeNormal(0.001f);
	if (!ga_up_xy.IsNearlyZero(0.09f))
	{
		ga_up_xy.Normalize();
		FVector projection_vector = (ga_forward - FVector::DotProduct(ga_forward, ga_up_xy)*ga_up_xy).GetSafeNormal();

		ajdc_targets[0].twist_dir = projection_vector;
		atc.error = 1.f - FVector::DotProduct(projection_vector, -FVector::UpVector);
		if (FVector::DotProduct(ga_up_xy, FVector::CrossProduct(-FVector::UpVector, projection_vector).GetSafeNormal()) > 0.f)
			ajdc_targets[0].twist_dir = -ajdc_targets[0].twist_dir;
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
}

void AJointCharacterTest::customInitGripPhysics(float DeltaTime, FBodyInstance* BodyInstance)
{
	//grip_axis_bi->SetBodyTransform(FTransform(torso_bi->GetCOMPosition() + FVector(0.f, 0.f, -15.f)), true);
	ga_pos = arm_BIs[0]->GetUnrealWorldTransform().GetLocation();
	ga_forward = arm_BIs[0]->GetUnrealWorldTransform().GetUnitAxis(EAxis::X);
	ga_right = arm_BIs[0]->GetUnrealWorldTransform().GetUnitAxis(EAxis::Y);
	ga_prev_up = ga_up;
	ga_up = arm_BIs[0]->GetUnrealWorldTransform().GetUnitAxis(EAxis::Z);

	g_pos = arm_BIs[1]->GetUnrealWorldTransform().GetLocation();
	g_forward = arm_BIs[1]->GetUnrealWorldTransform().GetUnitAxis(EAxis::X);
	g_right = arm_BIs[1]->GetUnrealWorldTransform().GetUnitAxis(EAxis::Y);
	g_prev_up = g_up;
	g_up = arm_BIs[1]->GetUnrealWorldTransform().GetUnitAxis(EAxis::Z);

	for (int i = 0; i < arm_BIs.Num(); i++)
	{
		
		abis[i].pos = arm_BIs[i]->GetUnrealWorldTransform().GetLocation();
		abis[i].forward = arm_BIs[i]->GetUnrealWorldTransform().GetUnitAxis(EAxis::X);
		abis[i].right = arm_BIs[i]->GetUnrealWorldTransform().GetUnitAxis(EAxis::Y);
		abis[i].prev_up = abis[i].up;
		abis[i].up = arm_BIs[i]->GetUnrealWorldTransform().GetUnitAxis(EAxis::Z);

	}

	/*float current_wep_incline = FMath::Acos(FVector::DotProduct(g_up, w_up))*180.f / PI;
	current_wep_incline = FMath::Fmod(current_wep_incline, 90.0f);*/
}

void AJointCharacterTest::ControlArmJointDirectionPhysics(float DeltaTime, FBodyInstance* BodyInstance)
{
	//find joint index
	int ji = arm_BIs.Num();
	for (int i = 0; i < arm_BIs.Num(); i++) {
		if (arm_BIs[i] == BodyInstance) {
			ji = i;
			break;
		}
	}

	ajdc[ji].error.X = 1.f - FVector::DotProduct(ajdc_targets[ji].dir, abis[ji].up);
	FVector rot_ref = FVector::CrossProduct(abis[ji].up, ajdc_targets[ji].dir);
	rot_ref = rot_ref.GetSafeNormal();

	//calculate rotational speed around rotation point
	FVector vel_axis = (abis[ji].up - abis[ji].prev_up) / DeltaTime;
	ajdc[ji].error.Y = -vel_axis.Size();
	vel_axis = vel_axis.GetSafeNormal();

	//Joint inertia around velocity and ref axis
	float jvi = 0.f, jri = 0.f;
	FMatrix joint_inertia;

	for (int i = ji; i < arm_BIs.Num(); i++) {
		calculateRelativeInertia(arm_BIs[i], abis[ji].pos, &joint_inertia);
		jvi += inertiaAboutAxis(joint_inertia, FVector::CrossProduct(abis[ji].up, vel_axis).GetSafeNormal());
		jri += inertiaAboutAxis(joint_inertia, rot_ref);
	}
	
	//calculate PID
	ajdc[ji].error.X = ajdc[ji].error.X*(jri);
	ajdc[ji].error.Y = ajdc[ji].error.Y*(jvi);

	ajdc[ji].integral = ajdc[ji].integral + ajdc[ji].error * DeltaTime;
	ajdc[ji].derivative = (ajdc[ji].error - ajdc[ji].prev_err) / DeltaTime;

	ajdc[ji].adjustment = ajdc[ji].P * ajdc[ji].error +
		ajdc[ji].I * ajdc[ji].integral +
		ajdc[ji].D * ajdc[ji].derivative;
	ajdc[ji].prev_err = ajdc[ji].error;

	////reset constraint rotation
	//FVector WPri = grip_axis_attachment->ComponentToWorld.GetUnitAxis(EAxis::X);
	//FVector WOrth = grip_axis_attachment->ComponentToWorld.GetUnitAxis(EAxis::Y);
	//FVector PriAxis1 = torsoBI->GetUnrealWorldTransform().InverseTransformVectorNoScale(WPri);
	//FVector SecAxis1 = torsoBI->GetUnrealWorldTransform().InverseTransformVectorNoScale(WOrth);
	//grip_axis_attachment->SetConstraintReferenceOrientation(EConstraintFrame::Frame1, PriAxis1, SecAxis1);

	//apply pid adjusted forces
	//grip_axis_bi->AddTorque(rot_ref*adc.adjustment.X, false);
	rot_ref = FVector::CrossProduct(rot_ref, abis[ji].up).GetSafeNormal();
	arm_BIs[ji]->AddForceAtPosition(rot_ref*ajdc[ji].adjustment.X, abis[ji].pos + abis[ji].up, false);
	arm_BIs[ji]->AddForceAtPosition(rot_ref*-ajdc[ji].adjustment.X, abis[ji].pos - abis[ji].up, false);

	arm_BIs[ji]->AddForceAtPosition(vel_axis*ajdc[ji].adjustment.Y, abis[ji].pos + abis[ji].up, false);
	arm_BIs[ji]->AddForceAtPosition(vel_axis*-ajdc[ji].adjustment.Y, abis[ji].pos - abis[ji].up, false);
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
	calculateRelativeInertia(arm_BIs[1], ga_pos, &grip_inertia);
	float gi_ref_i = inertiaAboutAxis(grip_inertia, ga_up);

	//atc.error = atc.error*(gi_ref_i + grip_axis_bi->GetBodyInertiaTensor().Z);
	//arm twist control error with inertia
	float atcewi = atc.error*(gi_ref_i + arm_BIs[0]->GetBodyInertiaTensor().Z);
	
	atc.integral = atc.integral + atcewi * DeltaTime;
	atc.derivative = (atcewi - atc.prev_err) / DeltaTime;

	atc.adjustment = atc.P * atcewi +
		atc.I * atc.integral +
		atc.D * atc.derivative;
	atc.prev_err = atcewi;

	arm_BIs[0]->AddTorque(ga_up*atc.adjustment, false);
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

	float tmp_inertia = arm_BIs[1]->GetBodyInertiaTensor().Z;
	arm_BIs[1]->AddTorque(g_up*wtc.adjustment*tmp_inertia, false);

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
	//Grip_h
	FVector WPri = grip_attachment->ComponentToWorld.GetUnitAxis(EAxis::X);
	FVector WOrth = grip_attachment->ComponentToWorld.GetUnitAxis(EAxis::Y);

	FVector PriAxis1 = grip_axis->GetComponentTransform().InverseTransformVectorNoScale(WPri);
	FVector SecAxis1 = grip_axis->GetComponentTransform().InverseTransformVectorNoScale(WOrth);
	grip_attachment->SetConstraintReferenceOrientation(EConstraintFrame::Frame1, PriAxis1, SecAxis1);

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

	InputComponent->BindAction("Dash", IE_Pressed, this, &AJointCharacterTest::dash);

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

void AJointCharacterTest::dash()
{
	if (!dashing)
	{
		dashing = true;
		dash_force_timer = 0.f;
		body_trail->BeginTrails(FName("Spine01"), FName("Neck"), ETrailWidthMode::ETrailWidthMode_FromCentre, 1.0f);
	}	
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

	target_speed = 220.f;

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

	target_speed = 400.f;
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

		/*UE_LOG(LogTemp, Warning, TEXT("----------------------"));*/
		grip_indicator_beam->SetVisibility(true);
		grip_indicator_decal->SetVisibility(true);
		return;
	}
	else if (holding_object)
	{
		grip_indicator_beam->SetVisibility(true);
		grip_indicator_decal->SetVisibility(true);
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

	object_attach_audio->Activate();

	grip_indicator_beam->SetVisibility(false);
	grip_indicator_decal->SetVisibility(false);

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
	camera_spring_arm->SetRelativeLocationAndRotation(FVector(0.0f, 0.0f, 50.0f), FRotator(0.f, 0.0f, 0.0f));
	camera_spring_arm->TargetArmLength = 400.f;
	camera_spring_arm->bEnableCameraLag = true;
	camera_spring_arm->CameraLagSpeed = 6.0f;

	camera = CreateDefaultSubobject<UCameraComponent>(TEXT("GameCamera"));
	camera->SetupAttachment(camera_spring_arm, USpringArmComponent::SocketName);

	targeting_sphere = CreateDefaultSubobject<USphereComponent>(TEXT("TargetingSphere"));
	targeting_sphere->SetupAttachment(camera_axis);
	targeting_sphere->SetSphereRadius(1500);
}

void AJointCharacterTest::initBody()
{

	body = CreateDefaultSubobject<USkeletalMeshComponent>(TEXT("Body"));
	body->SetupAttachment(RootComponent);

	static ConstructorHelpers::FObjectFinder<USkeletalMesh>  jointSKeletalMesh(TEXT("/Game/JointCharacter/mesh/robotWithArm_2/featherBotUVNoArm.featherBotUVNoArm"));
	body->SetSkeletalMesh(jointSKeletalMesh.Object);
	
	body->SetPhysicsAsset(jointSKeletalMesh.Object->PhysicsAsset);
	body->SetSimulatePhysics(true);

	body_trail = CreateDefaultSubobject<UParticleSystemComponent>(TEXT("BodyTrail"));
	body_trail->SetupAttachment(body, FName("Spine02"));
	//UE_LOG(LogTemp, Warning, TEXT("constraintnum %d"), body->Constraints.Num());

	//UPhysicsAsset* test = body->GetPhysicsAsset();
	
}

void AJointCharacterTest::initWeapon()
{
	// Weapon settings
	grip_axis = CreateDefaultSubobject<USphereComponent>(TEXT("WeaponAxis"));
	grip_axis->SetupAttachment(body, FName("Spine02"));
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

	grip_indicator_decal = CreateDefaultSubobject<UDecalComponent>(TEXT("GripIndicatorDecal"));
	grip_indicator_decal->SetupAttachment(grip);
	grip_indicator_decal->bAbsoluteRotation = true;

	grip_indicator_beam = CreateDefaultSubobject<UParticleSystemComponent>(TEXT("GripIndicatorBeam"));
	grip_indicator_beam->SetupAttachment(grip);
	grip_indicator_beam->bAbsoluteRotation = true;

	object_attach_audio = CreateDefaultSubobject<UAudioComponent>(TEXT("GripAttachmentAudio"));
	object_attach_audio->SetupAttachment(grip);

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

	grip_axis_attachment->SetConstrainedComponents(body, FName("Spine02"), grip_axis, NAME_None);
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

void AJointCharacterTest::initBodyJoints()
{
	torsoBI = body->GetBodyInstance(FName("Spine02"));

	//pelvis
	upbr.bi = body->GetBodyInstance(FName("Spine01"));
	upbr.pid = &pelvis_controller;
	//torso
	upbr.next = new FLimbNode;
	upbr.next->bi = body->GetBodyInstance(FName("Spine02"));
	upbr.next->pid = &torso_controller;
	upbr.next->next = nullptr;

	//right thigh
	rlr.bi = body->GetBodyInstance(FName("rThigh"));
	rlr.pid = &right_thigh_controller;
	//right shin
	rlr.next = new FLimbNode;
	rlr.next->bi = body->GetBodyInstance(FName("rShin"));
	rlr.next->pid = &right_shin_controller;
	rlr.next->next = nullptr;

	//left thigh
	llr.bi = body->GetBodyInstance(FName("lThigh"));
	llr.pid = &left_thigh_controller;
	//left shin
	llr.next = new FLimbNode;
	llr.next->bi = body->GetBodyInstance(FName("lShin"));
	llr.next->pid = &left_shin_controller;
	llr.next->next = nullptr;
}

void AJointCharacterTest::initPIDs()
{
	//camera ------------------------------------------------
	axis_target_rot = FRotator(0.f, 150.f, 0.f);
	arm_target_rot = FRotator(-35.f, 0.f, 0.f);
	target_arm_length = 500.f;

	cdc.target = FVector2D(0.f, 0.f);
	cdc.max_adjustment = FVector2D(0.f, 0.f);
	cdc.P = FVector2D(50.f, 50.f);
	cdc.I = FVector2D(0.f, 0.f);
	cdc.D = FVector2D(0.005f, 0.005f);
	cdc.integral = FVector2D::ZeroVector;

	//hovering ------------------------------------------------
	/*hover_height.target = target_hover_height;
	hover_height.max_adjustment = 100000;
	hover_height.P = 15;
	hover_height.I = 0;
	hover_height.D = 10;
	hover_height.integral = 0.f;
	hover_height.prev_err = 0.f;*/

	//arm and weapon ------------------------------------------------
	ajdc_targets.SetNum(2);
	ajdc_targets[0].dir = FVector::UpVector;
	ajdc_targets[0].prev_dir_xy = FVector::ForwardVector;

	ajdc_targets[1].dir = FVector::UpVector;
	ajdc_targets[1].prev_dir = FVector::ForwardVector;
	ajdc_targets[1].prev_dir_xy = FVector::ForwardVector;

	weapon_twist_solder = FVector::UpVector;
	weapon_twist_target = FVector::RightVector;

	
	atc.target = 0.0f;
	atc.max_adjustment = 5;
	atc.P = 100;
	atc.I = 0.0f;
	atc.D = 20.1f;
	atc.integral = 0.f;

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
	movement_velocity.P = FVector2D(10.f, 10.f);
	movement_velocity.I = FVector2D(0.f, 0.f);
	movement_velocity.D = FVector2D(0.5f, 0.5f);
	movement_velocity.integral = FVector2D::ZeroVector;

}

void AJointCharacterTest::initCustomPhysics()
{
	OnCalculateCustomHoverPhysics.BindUObject(this, &AJointCharacterTest::customHoverPhysics);

	OnCalculateControlGripPhysics.BindUObject(this, &AJointCharacterTest::ControlGripPhysics);
	OnCalculateCustomInitGripPhysics.BindUObject(this, &AJointCharacterTest::customInitGripPhysics);
	OnCalculateControlArmJointDirectionPhysics.BindUObject(this, &AJointCharacterTest::ControlArmJointDirectionPhysics);
	OnCalculateControlWeaponTwistPhysics.BindUObject(this, &AJointCharacterTest::ControlWeaponTwistPhysics);
	OnCalculateControlArmTwistPhysics.BindUObject(this, &AJointCharacterTest::ControlArmTwistPhysics);

	OnCalculateWeaponGrabControl.BindUObject(this, &AJointCharacterTest::weaponGrabControl);

	CalculateControlBody.BindUObject(this, &AJointCharacterTest::ControlBody);

	arm_BIs.Add(grip_axis->GetBodyInstance());
	arm_BIs.Add(grip->GetBodyInstance());

	abis.SetNum(arm_BIs.Num());
	abis[0].prev_up = FVector::UpVector;
	abis[1].prev_up = FVector::UpVector;

	//grip_axis_bi = grip_axis->GetBodyInstance();
	//grip_bi = grip->GetBodyInstance();
	g_pos_offset = FVector::ZeroVector;

	ga_prev_up = FVector::ZeroVector;
	g_prev_up = FVector::ZeroVector;
	w_prev_up = FVector::ZeroVector;
	//weapon_handle_2_bi = weapon_handle_2->GetBodyInstance();
	
	calculateWepInertia();

	//rolling_body->OnComponentHit.AddDynamic(this, &AJointCharacterTest::OnHit);
	body->OnComponentHit.AddDynamic(this, &AJointCharacterTest::OnBodyHit);
	targeting_sphere->OnComponentBeginOverlap.AddDynamic(this, &AJointCharacterTest::addPotentialTarget);
	targeting_sphere->OnComponentEndOverlap.AddDynamic(this, &AJointCharacterTest::removePotentialTarget);

	//arm_BIs[1]->SetBodyTransform(FTransform(arm_BIs[1]->GetUnrealWorldTransform().GetTranslation() + FVector::UpVector*70.f), true);
	//grip_v_bi->SetBodyTransform(FTransform(grip_v_bi->GetUnrealWorldTransform().GetTranslation() + FVector::UpVector*70.f), true);
}

void AJointCharacterTest::calculateWepInertia()
{
	//FVector ti = grip_bi->GetBodyInertiaTensor();
	////FVector tp = FVector(0.f, 0.f, 75.f);
	//
	//FVector tp = grip->ComponentToWorld.InverseTransformPositionNoScale(grip_bi->GetCOMPosition());// -grip_v->GetComponentLocation();
	//offset_wep_inertia = FVector(ti.X + grip_bi->GetBodyMass()*(tp.Y*tp.Y + tp.Z*tp.Z),
	//	ti.Y + grip_bi->GetBodyMass()*(tp.X*tp.X + tp.Z*tp.Z),
	//	ti.Z + grip_bi->GetBodyMass()*(tp.X*tp.X + tp.Y*tp.Y));

	//UE_LOG(LogTemp, Warning, TEXT("offset inertia: %s"), *offset_wep_inertia.ToString());
}

void AJointCharacterTest::calculateRelativeInertia(FBodyInstance* offset_bi, const FVector& cor, FMatrix* out_inertia)
{
	FVector bi_inrt = offset_bi->GetBodyInertiaTensor();
	FMatrix bi_T = offset_bi->GetUnrealWorldTransform().ToMatrixWithScale();
	//bi_T.ScaleTranslation(FVector(0.f, 0.f, 0.f));
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