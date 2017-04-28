// Fill out your copyright notice in the Description page of Project Settings.

#include "SwordSwing.h"
#include "InitPawn.h"


// Sets default values
AInitPawn::AInitPawn()
{
 	// Set this pawn to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

	initCamera();

}

// Called when the game starts or when spawned
void AInitPawn::BeginPlay()
{
	Super::BeginPlay();
	
}

// Called every frame
void AInitPawn::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	cameraCalculations(DeltaTime);

}


void AInitPawn::cameraCalculations(float DeltaTime)
{
	FRotator axis_rotation = camera_axis->GetComponentRotation();
	axis_rotation.Yaw += camera_input.X*2.0f;
	camera_axis->SetWorldRotation(axis_rotation);

	FRotator arm_rotation = camera_spring_arm->GetComponentRotation();
	arm_rotation.Pitch = FMath::Clamp(arm_rotation.Pitch + camera_input.Y*2.0f, -80.0f, 80.0f);
	camera_spring_arm->SetWorldRotation(arm_rotation);
}

// Called to bind functionality to input
void AInitPawn::SetupPlayerInputComponent(UInputComponent* PlayerInputComponent)
{
	Super::SetupPlayerInputComponent(PlayerInputComponent);

	InputComponent->BindAxis("CameraPitch", this, &AInitPawn::pitchCamera);
	InputComponent->BindAxis("CameraYaw", this, &AInitPawn::yawCamera);

}

void AInitPawn::pitchCamera(float AxisValue)
{
	camera_input.Y = AxisValue;
}

void AInitPawn::yawCamera(float AxisValue)
{
	camera_input.X = AxisValue;
}

void AInitPawn::initCamera()
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

