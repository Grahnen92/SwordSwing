// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "GameFramework/Actor.h"
#include "Weapon.generated.h"

UCLASS()
class SWORDSWING_API AWeapon : public AActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	AWeapon();

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

	UPROPERTY(Category = "WeaponPart", VisibleAnywhere)
	UCapsuleComponent* weapon_shaft;
	UPROPERTY(Category = "WeaponPart", VisibleAnywhere)
	UStaticMeshComponent* weapon_shaft_vis;
	FBodyInstance* weapon_shaft_bi;

	UPROPERTY(Category = "WeaponPart", VisibleAnywhere)
	USceneComponent* weapon_handle_point;

	UPROPERTY(Category = "WeaponPart", VisibleAnywhere)
	UBoxComponent* weapon_head;
	UPROPERTY(Category = "WeaponPart", VisibleAnywhere)
	UStaticMeshComponent* weapon_head_vis;
	FBodyInstance* weapon_head_bi;

	UPROPERTY(Category = "WeaponPart", VisibleAnywhere)
	UParticleSystemComponent* weapon_trail;

	UPROPERTY(Category = "WeaponPart", VisibleAnywhere)
	UAudioComponent* weapon_swish_audio;
	UPROPERTY(Category = "WeaponPart", VisibleAnywhere)
	UAudioComponent* weapon_wood_impact_audio;

	UForceFeedbackEffect* weapon_impact;

	bool held = false;
	APawn* holder;

public:	
	// Called every frame
	virtual void Tick(float DeltaTime) override;

	UFUNCTION()
	void OnSwordHit(UPrimitiveComponent* HitComp, AActor* OtherActor, UPrimitiveComponent* OtherComp, FVector NormalImpulse, const FHitResult& Hit);

	void initWeapon();
	
};
