// Fill out your copyright notice in the Description page of Project Settings.

#include "SwordSwing.h"
#include "LastManMode.h"



#include "EngineUtils.h"
//#include "TextRenderActor.h"

#include "InitPawn.h"
#include "jointTest/JointCharacterTest.h"
#include "ThirdPersonController.h"
#include "weapons/WeaponSpawner.h"
#include "hud/RoundInfo.h"

#include <sstream>
#include <string>

#include "Runtime/Engine/Classes/Engine/TextRenderActor.h"

ALastManMode::ALastManMode(const FObjectInitializer& ObjectInitializer) :
	Super(ObjectInitializer)
{

}

void ALastManMode::InitGame(const FString &MapName, const FString &Options, FString &ErrorMessage)
{
	Super::InitGame(MapName, Options, ErrorMessage);

}

void ALastManMode::BeginPlay()
{
	Super::BeginPlay();

	current_round = 1;

	while (GameState->PlayerArray.Num() < 2)
	{
		UGameplayStatics::CreatePlayer(GetWorld(), -1, true);
	}

	for (int i = 0; i < GameState->PlayerArray.Num(); i++)
	{
		auto player = GameState->PlayerArray[i]->GetNetOwningPlayer()->PlayerController;

		//initialize player state info
		GameState->PlayerArray[i]->Score = 0.f;
		std::stringstream ss_player;
		ss_player << "player_" << i;
		GameState->PlayerArray[i]->SetPlayerName(ss_player.str().c_str());

		//create hud showing match info
		FActorSpawnParameters SpawnInfo;
		round_info_displays.Add(GetWorld()->SpawnActor<ARoundInfo>(FVector(0.f, 0.f, 0.f), FRotator(0.f, 0.f, 0.f), SpawnInfo));
		round_info_displays.Top()->Init(GameState->PlayerArray.Num(), score_to_win);
		round_info_displays.Top()->SetActorLocation(player->StartSpot->GetActorForwardVector()*5000.f + player->StartSpot->GetActorLocation() + FVector::UpVector * 3000);
	
		//spawnPlayer(player);

		FVector info_direction = player->StartSpot->GetActorLocation() - round_info_displays.Top()->GetActorLocation();
		FRotator look_rot = FRotationMatrix::MakeFromX(info_direction).Rotator();
		//FRotator look_mat = FLookAtMatrix(player->StartSpot->GetActorLocation(), round_info_instance.Top()->GetActorLocation(), FVector::UpVector).Rotator();
		round_info_displays.Top()->SetActorRotation(look_rot);
		//round_info_displays.Top()->SetOwner(player->GetPawn());

		//initialize player colors in match info hud
		round_info_displays.Top()->setPlayerScoreMaterial(0, player_mats[i]);
		int rest_players_count = 1;
		for (int j = 0; j < GameState->PlayerArray.Num(); j++) {
			if (j != i) {
				round_info_displays.Top()->setPlayerScoreMaterial(rest_players_count, player_mats[j]);
				rest_players_count++;
			}
		}

		SpawnInfo;
		fight_counters.Add(GetWorld()->SpawnActor<ATextRenderActor>(player->StartSpot->GetActorLocation() + player->StartSpot->GetActorForwardVector()* 200.f + player->StartSpot->GetActorRightVector()* 100.f + player->StartSpot->GetActorUpVector()* 150.f, FRotator(0.f, 0.f, 0.f), SpawnInfo));
		FVector fight_direction = player->StartSpot->GetActorLocation() - fight_counters.Top()->GetActorLocation();
		FRotator look_rot_fight = FRotationMatrix::MakeFromX(fight_direction).Rotator();
		fight_counters.Top()->SetActorRotation(look_rot_fight);

		//TODO: look into this netid 
		//if (GameState->PlayerArray[i]->UniqueId.IsValid())
	}

	toggleFightText();

	initStartRound();
}

AActor* ALastManMode::ChoosePlayerStart_Implementation(AController* Player)
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

void ALastManMode::spawnPlayer(APlayerController* pc)
{

	APawn* currentPawn = pc->GetPawn();

	if (currentPawn)
	{
		currentPawn->Destroy();
	}
	FActorSpawnParameters SpawnInfo;


	AJointCharacterTest* new_char = GetWorld()->SpawnActor<AJointCharacterTest>(fight_char, pc->StartSpot->GetActorTransform(), SpawnInfo);

	if (new_char)
	{
		if (GameState->PlayerArray.Num() < 2)
			new_char->setFOV(90);
		else
			new_char->setFOV(50);;

		int player_id;
		GameState->PlayerArray.Find(pc->PlayerState, player_id);
		new_char->setPlayerSpecificMaterial(player_mats[player_id]);

		if (round_info_displays.Num() > player_id)
		{
			round_info_displays[player_id]->SetOwner(new_char);
		}
		else
		{
			UE_LOG(LogTemp, Warning, TEXT("Tried to set owner of round_info_display before it was created"));
		}

		pc->Possess(new_char);
	}
	else
	{
		UE_LOG(LogTemp, Warning, TEXT("failed to spawn character"));
	}


}

void ALastManMode::despawnPlayer(APlayerController* pc)
{
	APawn* currentPawn = pc->GetPawn();

	if (currentPawn)
	{
		currentPawn->Destroy();
	}
	FActorSpawnParameters SpawnInfo;

	AInitPawn* new_char = GetWorld()->SpawnActor<AInitPawn>(AInitPawn::StaticClass(), pc->StartSpot->GetActorTransform(), SpawnInfo);
	if (new_char)
	{
		if (GameState->PlayerArray.Num() < 2)
			new_char->setFOV(90);
		else
			new_char->setFOV(50);;
		pc->Possess(new_char);
	}
	else
	{
		UE_LOG(LogTemp, Warning, TEXT("failed to spawn initpawn"));
	}

}

void ALastManMode::initStartRound()
{
	for (int i = 0; i < GameState->PlayerArray.Num(); i++)
	{
		//respawn characters
		APlayerController* tmp_controller = GameState->PlayerArray[i]->GetNetOwningPlayer()->PlayerController;
		//respawn characters
		if (tmp_controller->GetPawn())
		{
			tmp_controller->GetPawn()->Destroy();
		}

		spawnPlayer(tmp_controller);
		//round_info_displays[i]->SetOwner(tmp_controller->GetPawn());

		//tmp_controller->GetPawn()->DisableInput(tmp_controller);
		//tmp_controller->SetIgnoreMoveInput(true);
		Cast<AJointCharacterTest>(tmp_controller->GetPawn())->setCanMove(false);

		fight_counters[i]->GetTextRender()->SetText("Get ready!");
		round_info_displays[i]->setRound(current_round);
	}
	for (const auto& spawn : weapon_spawns)
	{
		spawn->spawnWeapon();
	}

	live_player_amount = current_player_amount;
	count_down = 3;
	toggleFightText();

	FTimerHandle unused_handle;
	GetWorldTimerManager().SetTimer(unused_handle, this, &ALastManMode::decrementCountdown, 2.0f, false);
	round_state = 0;
}

void ALastManMode::decrementCountdown()
{
	if (count_down > 0)
	{
		std::stringstream ss_count;
		ss_count << count_down;
		for (int i = 0; i < GameState->PlayerArray.Num(); i++)
		{
			fight_counters[i]->GetTextRender()->SetText(ss_count.str().c_str());
		}
		count_down--;
		FTimerHandle unused_handle;
		GetWorldTimerManager().SetTimer(unused_handle, this, &ALastManMode::decrementCountdown, 1.0f, false);
	}
	else
	{
		for (int i = 0; i < GameState->PlayerArray.Num(); i++)
		{
			fight_counters[i]->GetTextRender()->SetText("Fight!");
		}
		startRound();
		FTimerHandle unused_handle;
		GetWorldTimerManager().SetTimer(unused_handle, this, &ALastManMode::toggleFightText, 2.0f, false);
	}
}

void ALastManMode::startRound()
{
	for (int i = 0; i < GameState->PlayerArray.Num(); i++)
	{
		//respawn characters
		APlayerController* tmp_controller = GameState->PlayerArray[i]->GetNetOwningPlayer()->PlayerController;

		//tmp_controller->GetPawn()->EnableInput(tmp_controller);
		//tmp_controller->SetIgnoreMoveInput(false);
		Cast<AJointCharacterTest>(tmp_controller->GetPawn())->setCanMove(true);
	}
	round_state = 1;
}

void ALastManMode::registerDeath(APlayerController* round_loser)
{
	if (round_state != 2)
	{
		live_player_amount--;

		if (live_player_amount == 1)
		{
			round_state = 2;

			for (int i = 0; i < GameState->PlayerArray.Num(); i++)
			{
				APlayerController* tmp_controller = GameState->PlayerArray[i]->GetNetOwningPlayer()->PlayerController;
				if (Cast<AJointCharacterTest>(tmp_controller->GetPawn())->isAlive())
				{
					initEndRound(tmp_controller);
				}
			}

		}
	}

}

void ALastManMode::initEndRound(APlayerController* round_winner)
{
	round_winner->PlayerState->Score++;

	if (round_winner->PlayerState->Score >= score_to_win)
	{
		endMatch(round_winner->PlayerState);
	}

	current_round++;

	for (int i = 0; i < GameState->PlayerArray.Num(); i++)
	{
		/*round_info_displays[i]->setPlayerScore(0, GameState->PlayerArray[tmp_player_index]->Score);
		int rest_players_count = 1;
		for (int j = 0; j < GameState->PlayerArray.Num(); j++) {
		if (j != i) {
		round_info_displays[i]->setPlayerScore(rest_players_count, GameState->PlayerArray[j]->Score);
		rest_players_count++;
		}
		}*/
		round_info_displays[i]->updatePlayerScores();
	}
	FTimerHandle unused_handle;
	GetWorldTimerManager().SetTimer(unused_handle, this, &ALastManMode::endRound, 7.0f, false);

	round_state = 2;

}

void ALastManMode::endRound()
{
	//GetWorld()->ServerTravel(FString("/Game/Levels/FightArena/FightArena"));

	for (const auto& spawn : weapon_spawns)
	{
		spawn->despawnAllWeapons();
	}


	initStartRound();
}

void ALastManMode::endMatch(APlayerState* match_winner)
{
	UE_LOG(LogTemp, Warning, TEXT("The winner is: %s"), *match_winner->GetName());
	this->Reset();
	UGameplayStatics::OpenLevel(GetWorld(), "MainMenu");
}

void  ALastManMode::toggleFightText()
{
	for (int i = 0; i < GameState->PlayerArray.Num(); i++)
	{
		fight_counters[i]->GetTextRender()->ToggleVisibility();
	}
}

