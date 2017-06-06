// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "FightModeBase.h"
#include "proceduralGeneration/hex/HexInstancerComponent.h"
#include "proceduralGeneration/hex/HexInstancer.h"

#include "Runtime/Engine/Classes/Engine/TriggerBox.h"
#include "SwordLock.h"

#include "PracticeMode.generated.h"

/**
 * 
 */
UCLASS()
class SWORDSWING_API APracticeMode : public AFightModeBase
{

	GENERATED_BODY()
	
public:
	APracticeMode(const FObjectInitializer& ObjectInitializer);

	virtual void InitGame(const FString &MapName, const FString &Options, FString &ErrorMessage) override;

	AActor* ChoosePlayerStart_Implementation(AController* Player) override;

	// Called every frame
	virtual void Tick(float DeltaTime) override;

	void registerDeath(APlayerController* round_loser);
	
protected:
	virtual void BeginPlay() override;

	//UPROPERTY(Category = "InstancedMesh", BlueprintReadWrite, VisibleAnywhere)
	//UHexInstancerComponent* instancer;

	AHexInstancer* instancer;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "asset")
	TSubclassOf<class AWeapon> weapon_asset;

	UPROPERTY(VisibleAnywhere)
	TSubclassOf<class ASwordLock> horiz_lock;
	ASwordLock* horiz_lock_inst;

	UPROPERTY(VisibleAnywhere)
	TSubclassOf<class ASwordLock> vert_lock;
	ASwordLock* vert_lock_inst;

	UPROPERTY(VisibleAnywhere)
	TSubclassOf<class AJointCharacterTest> hitting_dummy;

	UPROPERTY(VisibleAnywhere)
	TSubclassOf<class AJointCharacterTest> guarding_dummy;

	UBoxComponent* killzBox;
	UFUNCTION()
	void killzBeginOverlap(UPrimitiveComponent* OverlappedComp, AActor* OtherActor, UPrimitiveComponent* OtherComp, int32 OtherBodyIndex, bool bFromSweep, const FHitResult& SweepResult);


	UFUNCTION()
	void eventOne(UPrimitiveComponent* OverlappedComp, AActor* OtherActor, UPrimitiveComponent* OtherComp, int32 OtherBodyIndex, bool bFromSweep, const FHitResult& SweepResult);
	UFUNCTION()
	void eventTwo(UPrimitiveComponent* OverlappedComp, AActor* OtherActor, UPrimitiveComponent* OtherComp, int32 OtherBodyIndex, bool bFromSweep, const FHitResult& SweepResult);
	UFUNCTION()
	void eventThree(UPrimitiveComponent* OverlappedComp, AActor* OtherActor, UPrimitiveComponent* OtherComp, int32 OtherBodyIndex, bool bFromSweep, const FHitResult& SweepResult);
	UFUNCTION()
	void eventFour(UPrimitiveComponent* OverlappedComp, AActor* OtherActor, UPrimitiveComponent* OtherComp, int32 OtherBodyIndex, bool bFromSweep, const FHitResult& SweepResult);
	UFUNCTION()
	void eventFive(UPrimitiveComponent* OverlappedComp, AActor* OtherActor, UPrimitiveComponent* OtherComp, int32 OtherBodyIndex, bool bFromSweep, const FHitResult& SweepResult);
	UFUNCTION()
	void eventSex(UPrimitiveComponent* OverlappedComp, AActor* OtherActor, UPrimitiveComponent* OtherComp, int32 OtherBodyIndex, bool bFromSweep, const FHitResult& SweepResult);
	UFUNCTION()
	void eventSeven(UPrimitiveComponent* OverlappedComp, AActor* OtherActor, UPrimitiveComponent* OtherComp, int32 OtherBodyIndex, bool bFromSweep, const FHitResult& SweepResult);

	UFUNCTION()
	void delayedPlayerSpawn(APlayerController* pc);
private:

	class ATriggeredWeaponSpawner* deferred_spawner;
	void timedWeaponSpawn();

	AJointCharacterTest* practice_bot;
	void delayedDummySpawn();

	

	float hex_radius = 100.f;
	float hex_height;

	float initial_pos = -4000.f;

	// 0 == initial platform
	// 1 == jump tutorial starting platform
	// 2 == after jump completed
	// 3 == after dash jump completed
	// 4 == weapon grab platform
	// 5 == final platform with dummies
	TArray<UHexGenComponent*> platforms;
	// 0 == bridge to jump tutorial
	// 1 == bridge to weapon grab tutorial
	// 2 == bridge to first lock
	// 3 == bridge to second lock
	// 4 == bridge to final platform
	TArray<UHexGenComponent*> bridges;
	// 0 == initiates jump tutorial
	// 1 == initiates jump dash tutorial
	// 2 == initiates grab tutorial
	// 3 == initiates first lock tutorial
	// 4 == initiates second lock tutorial
	// 5 == initiates final dummy tutorial
	TArray<UBoxComponent*> event_colliders;
	
};
