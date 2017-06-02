// Fill out your copyright notice in the Description page of Project Settings.

#include "SwordSwing.h"
#include "jointTest/Weapon.h"
#include "WeaponSpawner.h"


// Sets default values
AWeaponSpawner::AWeaponSpawner()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
	RootComponent = CreateDefaultSubobject<USceneComponent>(TEXT("RootComponent"));
}

// Called when the game starts or when spawned
void AWeaponSpawner::BeginPlay()
{
	Super::BeginPlay();
	if (weapon_template) // Check if the Asset is assigned in the blueprint.
	{
		UE_LOG(LogTemp, Warning, TEXT("Weapon template has been succesfully assigned"));
	}
	else
	{
		UE_LOG(LogTemp, Warning, TEXT("Weapon template has not been succesfully assigned"));
	}
}

// Called every frame
void AWeaponSpawner::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

}


void AWeaponSpawner::spawnWeapon()
{
	FActorSpawnParameters SpawnInfo;
	//weapons.Add(GetWorld()->SpawnActor<AWeapon>(weapon_template, GetActorTransform() + FVector::UpVector*100.f, SpawnInfo));
	weapons.Add(GetWorld()->SpawnActor<AWeapon>(weapon_template, GetActorLocation() + FVector::UpVector*100.f, GetActorRotation().Add(0.f, 0.f, 90.f), SpawnInfo));
}
void AWeaponSpawner::despawnWeapon(int w_index)
{
	//weapons.Top()->Destroy();
	
}
void AWeaponSpawner::despawnAllWeapons()
{
	//weapons.RemoveAll();
	for (auto &wep : weapons)
	{
		wep->Destroy();
	}
	weapons.Empty();
}
