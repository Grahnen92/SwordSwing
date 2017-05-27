// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "GameFramework/GameModeBase.h"
#include "FightModeBase.generated.h"

/**
 * 
 */
UCLASS()
class SWORDSWING_API AFightModeBase : public AGameModeBase
{
	GENERATED_BODY()

public:
	AFightModeBase(const FObjectInitializer& ObjectInitializer);

	virtual void InitGame(const FString &MapName, const FString &Options, FString &ErrorMessage) override;

	UPROPERTY(VisibleAnywhere)
	TSubclassOf<class AJointCharacterTest> fight_char;

	AActor* ChoosePlayerStart_Implementation(AController* Player) override;

	virtual void spawnPlayer(APlayerController* pc);
	virtual void despawnPlayer(APlayerController* pc);

protected:
	virtual void BeginPlay() override;

	const int MAX_PLAYERS = 4;

	TArray<UMaterial*> player_mats;
	void initPlayerMats();

	TArray<class AWeaponSpawner*> weapon_spawns;
private:
	
	
};
