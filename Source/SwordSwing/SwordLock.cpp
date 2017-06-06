// Fill out your copyright notice in the Description page of Project Settings.

#include "SwordSwing.h"
#include "SwordLock.h"


// Sets default values
ASwordLock::ASwordLock()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

	root = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));
	lock_center = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("LockCenter"));
	lock_center->SetupAttachment(root);
	target_point = CreateDefaultSubobject<USphereComponent>(TEXT("TargetPoint"));
	target_point->SetupAttachment(root);
	left_wall = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("LeftWall"));
	left_wall->SetupAttachment(root);
	right_wall = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("RightWall"));
	right_wall->SetupAttachment(root);
	light = CreateDefaultSubobject<UPointLightComponent>(TEXT("Light"));
	light->SetupAttachment(root);
}

// Called when the game starts or when spawned
void ASwordLock::BeginPlay()
{
	Super::BeginPlay();
	
}

// Called every frame
void ASwordLock::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

}

