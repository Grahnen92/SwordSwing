// Fill out your copyright notice in the Description page of Project Settings.

#include "SwordSwing.h"
#include "ScoreDisplay.h"

#include <sstream>
#include <string>

#include "RoundInfo.h"


// Sets default values
ARoundInfo::ARoundInfo()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

	round_text = CreateDefaultSubobject<UTextRenderComponent>(TEXT("Score"));
	round_text->SetupAttachment(RootComponent);
	round_text->SetText(FText::FromString("Round 0"));
	round_text->WorldSize = 500.f;
	round_text->HorizontalAlignment = EHorizTextAligment::EHTA_Center;
	round_text->SetOnlyOwnerSee(true);
}

// Called when the game starts or when spawned
void ARoundInfo::BeginPlay()
{
	Super::BeginPlay();
	
}

// Called every frame
void ARoundInfo::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

}

void ARoundInfo::Init(int _player_amount, float _score_to_win)
{
	player_amount = _player_amount;
	score_to_win = _score_to_win;

	std::stringstream ss;
	ss << "0/" << score_to_win;
	for (int i = 0; i < player_amount; i++)
	{
		FActorSpawnParameters SpawnInfo;
		AScoreDisplay* sd = GetWorld()->SpawnActor<AScoreDisplay>(round_text->GetComponentLocation() - FVector::UpVector*300.f,
			round_text->GetComponentRotation(), SpawnInfo);
		sd->AttachToComponent(round_text, FAttachmentTransformRules::KeepWorldTransform);
		sd->setScoreText(ss.str().c_str());
		sd->SetOwner(this);
		//FVector score_dims;
		//FVector score_origin;
		//sd->GetActorBounds(false, score_dims, score_origin);
		float score_ydim = 450.f;
		float center_offset = -(_player_amount*score_ydim) / 2.f;

		sd->AddActorLocalOffset(FVector(0.f, center_offset + i*score_ydim + score_ydim/2.f, 0.f));

		scores.Add(sd);
	}

	
}

void ARoundInfo::setPlayerAmount(int p_amount)
{
	player_amount = p_amount;
	FActorSpawnParameters SpawnInfo;
	AScoreDisplay* sd = GetWorld()->SpawnActor<AScoreDisplay>(round_text->GetComponentLocation() - FVector::UpVector*100.f,
		round_text->GetComponentRotation(), SpawnInfo);
	sd->AttachToComponent(round_text, FAttachmentTransformRules::KeepWorldTransform);
	
	scores.Add(sd);
}



void ARoundInfo::setPlayerScoreMaterial(int player, UMaterial* score_mat)
{
	scores[player]->setIconMaterial(score_mat);
}


void ARoundInfo::updatePlayerScores()
{
	APawn* player_owning_score = Cast<APawn>(this->GetOwner());
	
	if (player_owning_score)
	{
		APlayerState* owner_state = player_owning_score->GetController()->PlayerState;

		TArray<APlayerState*> * player_states = &GetWorld()->GetAuthGameMode()->GameState->PlayerArray;

		int rest_players = 1;
		for (int i = 0; i < player_states->Num(); i++)
		{
			if ((*player_states)[i] == owner_state)
			{
				setPlayerScore(0, (*player_states)[i]->Score);
			}
			else
			{
				setPlayerScore(rest_players, (*player_states)[i]->Score);
				rest_players++;
			}
		}
	}
	else
	{
		UE_LOG(LogTemp, Warning, TEXT("Scoreboard did not have a pawn owner"));
	}
}

void ARoundInfo::setPlayerScore(int player, float score)
{
	std::stringstream ss;
	ss << score <<  "/" << score_to_win;
	scores[player]->setScoreText(ss.str().c_str());
}

void ARoundInfo::setRound(int _round)
{
	std::stringstream ss;
	ss << "Round " << _round;
	round_text->SetText(FText::FromString(ss.str().c_str()));
}

