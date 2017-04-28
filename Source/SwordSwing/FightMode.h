// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "GameFramework/GameModeBase.h"

#include "FightMode.generated.h"

/**
 * 
 */
UCLASS()
class SWORDSWING_API AFightMode : public AGameModeBase
{
	GENERATED_BODY()
public:
	AFightMode(const FObjectInitializer& ObjectInitializer);

	virtual void InitGame(const FString &MapName, const FString &Options, FString &ErrorMessage) override;

	UPROPERTY(VisibleAnywhere)
	TSubclassOf<class AJointCharacterTest> fight_char;

	TArray<class ARoundInfo*> round_info_displays;
	TArray<class ATextRenderActor*> fight_counters;

	AActor* ChoosePlayerStart_Implementation(AController* Player) override;

	void spawnPlayer( APlayerController* pc);
	void despawnPlayer( APlayerController* pc);

	void initStartRound();
	void startRound();

	void initEndRound(APlayerController* round_loser);
	void endRound();
	void endMatch(APlayerState* match_winner);

protected:
	virtual void BeginPlay() override;

	
private:
	const int MAX_PLAYERS = 4;
	int current_player_amount = 2;

	int current_round = 0;
	// 0 = starting round
	// 1 = round is ongoing
	// 2 = round is ending
	int round_state = 0;

	int score_to_win = 3;

	TArray<UMaterial*> player_mats;
	void initPlayerMats();
};
