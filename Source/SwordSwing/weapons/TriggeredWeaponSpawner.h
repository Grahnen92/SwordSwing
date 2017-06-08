// Fill out your copyright notice in the Description page of Project Settings.

#pragma once


#include "weapons/WeaponSpawner.h"
#include "TriggeredWeaponSpawner.generated.h"


/**
 * 
 */
UCLASS()
class SWORDSWING_API ATriggeredWeaponSpawner : public AWeaponSpawner
{
	GENERATED_BODY()
	
	
public:
	// Sets default values for this actor's properties
	ATriggeredWeaponSpawner();

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

public:
	// Called every frame
	virtual void Tick(float DeltaTime) override;

	UFUNCTION()
	void OnOverlapBegin(UPrimitiveComponent* OverlappedComp, AActor* OtherActor, UPrimitiveComponent* OtherComp, int32 OtherBodyIndex, bool bFromSweep, const FHitResult& SweepResult);
	UFUNCTION()
	void OnOverlapEnd(UPrimitiveComponent* OverlappedComp, AActor* OtherActor, UPrimitiveComponent* OtherComp, int32 OtherBodyIndex);

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Trigger")
	USceneComponent* root;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Trigger")
	UCapsuleComponent* trigger;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "TriggerText")
	UTextRenderComponent* trigger_ftext;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "TriggerText")
	UTextRenderComponent* trigger_btext;

	UPROPERTY(Category = "Trigger", VisibleAnywhere)
	UDecalComponent* trigger_indicator_decal;

	

	AWeapon* loaded_wep = nullptr;
	
};
