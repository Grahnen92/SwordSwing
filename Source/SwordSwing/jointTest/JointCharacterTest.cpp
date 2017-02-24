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
	//rolling_body = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("PhysicalCharacter"));
	
	camera_axis = CreateDefaultSubobject<USceneComponent>(TEXT("CameraAxis"));
	camera_axis->SetupAttachment(RootComponent);

	camera_spring_arm = CreateDefaultSubobject<USpringArmComponent>(TEXT("CameraSpringArm"));
	camera_spring_arm->SetupAttachment(camera_axis);
	camera_spring_arm->SetRelativeLocationAndRotation(FVector(0.0f, 0.0f, 50.0f), FRotator(-60.0f, 0.0f, 0.0f));
	camera_spring_arm->TargetArmLength = 400.f;
	camera_spring_arm->bEnableCameraLag = false;
	camera_spring_arm->CameraLagSpeed = 3.0f;

	camera = CreateDefaultSubobject<UCameraComponent>(TEXT("GameCamera"));
	camera->SetupAttachment(camera_spring_arm, USpringArmComponent::SocketName);

	AutoPossessPlayer = EAutoReceiveInput::Player0;

	
}

// Called when the game starts or when spawned
void AJointCharacterTest::BeginPlay()
{
	Super::BeginPlay();
	hover_height.wanted = wanted_hover_height;
	hover_height.max_adjustment = 1000000;

	//the factor of 100 is because unreal apparently applies forces in kg*cm*s^(-2)
	//jump_force = ((FMath::Sqrt(2.f*9.81f*jump_height) / jump_force_time) + 9.81f)*rolling_body->GetMass()*100;
}

// Called every frame
void AJointCharacterTest::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	cameraCalculations(DeltaTime);
	movementCalculations(DeltaTime);
	hover(DeltaTime);
	//UE_LOG(LogTemp, Warning, TEXT("tick!"));
}



void AJointCharacterTest::cameraCalculations(float DeltaTime) 
{
	if (zooming)
	{
		zoom_factor += DeltaTime / 0.5f;         //Zoom in over half a second
	}
	else
	{
		zoom_factor -= DeltaTime / 0.25f;        //Zoom out over a quarter of a second
	}
	zoom_factor = FMath::Clamp<float>(zoom_factor, 0.0f, 1.0f);
	//Blend our camera's FOV and our SpringArm's length based on ZoomFactor
	camera->FieldOfView = FMath::Lerp<float>(90.0f, 60.0f, zoom_factor);
	camera_spring_arm->TargetArmLength = FMath::Lerp<float>(400.0f, 300.0f, zoom_factor);

	{
		camera_axis->SetWorldLocation(rolling_body->GetCenterOfMass());

		//Rotate our actor's yaw, which will turn our camera because we're attached to it
		{
			//FRotator NewRotation = GetActorRotation();
			FRotator NewRotation = camera_axis->GetComponentRotation();
			NewRotation.Yaw += camera_input.X;
			camera_axis->SetWorldRotation(NewRotation);
		}

		//Rotate our camera's pitch, but limit it so we're always looking downward
		{
			FRotator NewRotation = camera_spring_arm->GetComponentRotation();
			NewRotation.Pitch = FMath::Clamp(NewRotation.Pitch + camera_input.Y, -80.0f, 80.0f);
			camera_spring_arm->SetWorldRotation(NewRotation);
		}
	}
}

void AJointCharacterTest::movementCalculations(float DeltaTime)
{
	UE_LOG(LogTemp, Warning, TEXT("x: %f"), movement_input.X);
	UE_LOG(LogTemp, Warning, TEXT("y: %f"), movement_input.Y);
	if (!movement_input.IsZero() && rolling_body->GetComponentVelocity().Size() < target_speed)
	{
		//Scale our movement input axis values by 100 units per second
		movement_input = movement_input.SafeNormal();

		FVector curr_vel = rolling_body->GetComponentVelocity();
		curr_vel.Z = 0.0f;
		FVector target_vel = FVector(movement_input.X, movement_input.Y, 0.0f)*target_speed;
		//FVector move_force = ((target_vel - curr_vel) / time_to_target_speed)*rolling_body->GetMass();
		FVector move_force = ((target_vel - curr_vel) )*rolling_body->GetMass();

		rolling_body->AddForce(camera_axis->GetForwardVector()* move_force.X );
		rolling_body->AddForce(camera_axis->GetRightVector()* move_force.Y );

	}
	if (jumping) {

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

