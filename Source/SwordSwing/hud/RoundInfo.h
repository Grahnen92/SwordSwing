// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "GameFramework/Actor.h"
#include "RoundInfo.generated.h"

UCLASS()
class SWORDSWING_API ARoundInfo : public AActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	ARoundInfo();

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

	UPROPERTY(Category = "Round", VisibleAnywhere)
	UTextRenderComponent* round_text;
	
	//entry 0 in this array should always be related to the pawn/controller owning this scoreboard
	TArray<class AScoreDisplay*> scores;
	int player_amount = 0;
	int score_to_win = 0;

public:	
	// Called every frame
	virtual void Tick(float DeltaTime) override;

	void Init(int _player_amount, float _score_to_win);
	void setPlayerAmount(int p_amount);

	void setPlayerScoreMaterial(int player, UMaterial* score_mat);

	void updatePlayerScores();
	void setPlayerScore(int player, float score);

	void setRound(int _round);
	
};
