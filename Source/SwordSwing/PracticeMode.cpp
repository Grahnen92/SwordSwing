// Fill out your copyright notice in the Description page of Project Settings.

#include "SwordSwing.h"
#include "PracticeMode.h"




// Fill out your copyright notice in the Description page of Project Settings.

#include "SwordSwing.h"
#include "FightModeBase.h"

#include "EngineUtils.h"
//#include "TextRenderActor.h"

#include "InitPawn.h"
#include "jointTest/JointCharacterTest.h"
#include "ThirdPersonController.h"
#include "weapons/WeaponSpawner.h"

#include <sstream>
#include <string>


APracticeMode::APracticeMode(const FObjectInitializer& ObjectInitializer) :
	Super(ObjectInitializer)
{

}

void APracticeMode::InitGame(const FString &MapName, const FString &Options, FString &ErrorMessage)
{
	Super::InitGame(MapName, Options, ErrorMessage);
}

void APracticeMode::BeginPlay()
{
	Super::BeginPlay();
}

AActor* APracticeMode::ChoosePlayerStart_Implementation(AController* Player)
{
	int tmp_player_index;
	GameState->PlayerArray.Find(Player->PlayerState, tmp_player_index);


	for (TActorIterator<APlayerStart> ActorItr(GetWorld()); ActorItr; ++ActorItr)
	{
		if (std::stoi(*ActorItr->PlayerStartTag.ToString()) == tmp_player_index)
		{
			Player->StartSpot = *ActorItr;
			return *ActorItr;
		}
	}

	UE_LOG(LogTemp, Warning, TEXT("No spawnpoint found"));
	return nullptr;
}
