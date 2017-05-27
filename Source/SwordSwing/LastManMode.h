// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "FightModeBase.h"
#include "LastManMode.generated.h"

/**
 * 
 */
UCLASS()
class SWORDSWING_API ALastManMode : public AFightModeBase
{
	GENERATED_BODY()
public:
	ALastManMode(const FObjectInitializer& ObjectInitializer);

	virtual void InitGame(const FString &MapName, const FString &Options, FString &ErrorMessage) override;

	TArray<class ARoundInfo*> round_info_displays;
	TArray<class ATextRenderActor*> fight_counters;
	void toggleFightText();

	AActor* ChoosePlayerStart_Implementation(AController* Player) override;

	virtual void spawnPlayer(APlayerController* pc);
	virtual void despawnPlayer(APlayerController* pc);

	void initStartRound();
	int count_down = 3;
	void decrementCountdown();
	void startRound();

	void registerDeath(APlayerController* round_loser);
	void initEndRound(APlayerController* round_winner);
	void endRound();
	void endMatch(APlayerState* match_winner);



protected:
	virtual void BeginPlay() override;


private:
	int current_player_amount = 2;
	int live_player_amount = 2;

	int current_round = 0;
	// 0 = starting round
	// 1 = round is ongoing
	// 2 = round is ending
	int round_state = 0;

	int score_to_win = 3;

};
