// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "FightModeBase.h"
#include "proceduralGeneration/hex/HexInstancerComponent.h"
#include "proceduralGeneration/hex/HexInstancer.h"
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

	UFUNCTION(BlueprintCallable, Category = "TutorialTask")
	void completeTaskOne();

	UFUNCTION(BlueprintCallable, Category = "TutorialTask")
	void completeTaskTwo();
	
protected:
	virtual void BeginPlay() override;

	//UPROPERTY(Category = "InstancedMesh", BlueprintReadWrite, VisibleAnywhere)
	//UHexInstancerComponent* instancer;

	AHexInstancer* instancer;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "asset")
	TSubclassOf<class AWeapon> weapon_asset;

private:

	class ATriggeredWeaponSpawner* deferred_spawner;
	void timedWeaponSpawn();

	AJointCharacterTest* practice_bot;
	void delayedCharacterpawn();

	float hex_radius = 100.f;
	float hex_height;

	float initial_pos = -4000.f;

	TArray<UHexGenComponent*> platforms;
	TArray<UHexGenComponent*> bridges;
	
};
