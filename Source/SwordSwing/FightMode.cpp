// Fill out your copyright notice in the Description page of Project Settings.

#include "SwordSwing.h"
#include "EngineUtils.h"
//#include "TextRenderActor.h"

#include "InitPawn.h"
#include "jointTest/JointCharacterTest.h"
#include "ThirdPersonController.h"
#include "FightMode.h"
#include "hud/RoundInfo.h"

#include <sstream>
#include <string>

#include "Runtime/Engine/Classes/Engine/TextRenderActor.h"

AFightMode::AFightMode(const FObjectInitializer& ObjectInitializer) :
	Super(ObjectInitializer)
{
	static ConstructorHelpers::FObjectFinder<UBlueprint> char_obj(TEXT("/Game/JointCharacter/JointCharacter.JointCharacter"));
	if (char_obj.Succeeded())
		fight_char = (UClass*)char_obj.Object->GeneratedClass;
	else
		UE_LOG(LogTemp, Warning, TEXT("Could not find character asset"));


	initPlayerMats();
	//round_info_component = CreateDefaultSubobject<UTextRenderComponent>(TEXT("RoundInfo"));
	//round_info_component->SetupAttachment(RootComponent);
}

void AFightMode::InitGame(const FString &MapName, const FString &Options, FString &ErrorMessage)
{
	Super::InitGame(MapName, Options, ErrorMessage);

	bPauseable = true;
	//PlayerControllerClass = AThirdPersonController::StaticClass();
	DefaultPawnClass = AInitPawn::StaticClass();
}

void AFightMode::BeginPlay()
{
	Super::BeginPlay();

	current_round = 0;
	std::stringstream ss_round;
	ss_round << "Round " << current_round << "/" << (2*score_to_win)-1  << "<br>" << "0-0";

	while (GameState->PlayerArray.Num() < current_player_amount)
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
		round_info_displays.Top()->SetActorLocation(player->StartSpot->GetActorForwardVector()*5000.f + player->StartSpot->GetActorLocation() + FVector::UpVector * 4000);
		
		spawnPlayer(player);

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
		fight_counters.Add(GetWorld()->SpawnActor<ATextRenderActor>(player->StartSpot->GetActorLocation() + player->StartSpot->GetActorForwardVector()* 200.f + player->StartSpot->GetActorRightVector()* 100.f + player->StartSpot->GetActorUpVector()* 200.f, FRotator(0.f, 0.f, 0.f), SpawnInfo));
		FVector fight_direction = player->StartSpot->GetActorLocation() - fight_counters.Top()->GetActorLocation();
		FRotator look_rot_fight = FRotationMatrix::MakeFromX(fight_direction).Rotator();
		fight_counters.Top()->SetActorRotation(look_rot_fight);

		//TODO: look into this netid 
		//if (GameState->PlayerArray[i]->UniqueId.IsValid())
	}
	initStartRound();
}

AActor* AFightMode::ChoosePlayerStart_Implementation(AController* Player)
{
	int tmp_player_index;
	GameState->PlayerArray.Find(Player->PlayerState, tmp_player_index);

	switch (tmp_player_index) {
	case 0:
		for (TActorIterator<APlayerStart> ActorItr(GetWorld()); ActorItr; ++ActorItr)
		{
			if (ActorItr->PlayerStartTag == "player_0")
			{
				Player->StartSpot = *ActorItr;
				return *ActorItr;
			}
		}
		break;
	case 1:
		for (TActorIterator<APlayerStart> ActorItr(GetWorld()); ActorItr; ++ActorItr)
		{
			if (ActorItr->PlayerStartTag == "player_1")
			{
				Player->StartSpot = *ActorItr;
				return *ActorItr;
			}
		}
		break;
	}
	
	UE_LOG(LogTemp, Warning, TEXT("No spawnpoint found"));
	return nullptr;
}

void AFightMode::spawnPlayer( APlayerController* pc)
{

	APawn* currentPawn = pc->GetPawn();

	if (currentPawn)
	{
		currentPawn->Destroy();
	}
	FActorSpawnParameters SpawnInfo;
	
	
	AJointCharacterTest* new_char = GetWorld()->SpawnActor<AJointCharacterTest>(fight_char, pc->StartSpot->GetActorTransform(), SpawnInfo);
	new_char->SetActorRotation(FRotationMatrix::MakeFromX(-pc->StartSpot->GetActorRightVector()).Rotator());

	if (new_char)
	{
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

void AFightMode::despawnPlayer( APlayerController* pc)
{
	APawn* currentPawn = pc->GetPawn();

	if (currentPawn)
	{
		currentPawn->Destroy();
	}
	FActorSpawnParameters SpawnInfo;

	AInitPawn* new_char = GetWorld()->SpawnActor<AInitPawn>(AInitPawn::StaticClass(),pc->StartSpot->GetActorTransform(), SpawnInfo);
	if (new_char)
	{
		pc->Possess(new_char);
	}
	else
	{
		UE_LOG(LogTemp, Warning, TEXT("failed to spawn initpawn"));
	}
	
}

void AFightMode::initStartRound()
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

	}
	
	
	FTimerHandle unused_handle;
	GetWorldTimerManager().SetTimer(unused_handle, this, &AFightMode::startRound, 3.0f, false);
	round_state = 0;
}
void AFightMode::startRound()
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

void AFightMode::initEndRound(APlayerController* round_loser)
{
	if (round_state != 2)
	{
		int tmp_player_index;
		GameState->PlayerArray.Find(round_loser->PlayerState, tmp_player_index);
		GameState->PlayerArray[tmp_player_index]->Score++;
		if (GameState->PlayerArray[tmp_player_index]->Score >= score_to_win)
		{
			endMatch(GameState->PlayerArray[tmp_player_index]);
		}

		current_round++;


		for (int i = 0; i < GameState->PlayerArray.Num(); i++)
		{
			//Update all round displays
			round_info_displays[i]->setRound(current_round);
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
		GetWorldTimerManager().SetTimer(unused_handle, this, &AFightMode::endRound, 10.0f, false);
		
		round_state = 2;
	}
}

void AFightMode::endRound()
{
	//GetWorld()->ServerTravel(FString("/Game/Levels/FightArena/FightArena"));

	//TODO: didn't work
	for (FConstPhysicsVolumeIterator physIt = GetWorld()->GetNonDefaultPhysicsVolumeIterator(); physIt; ++physIt)
	{
		physIt->Get()->Reset();
	}

	initStartRound();
}

void AFightMode::endMatch(APlayerState* match_winner)
{
	UE_LOG(LogTemp, Warning, TEXT("The winner is: %s"), *match_winner->GetName());
	this->Reset();
	UGameplayStatics::OpenLevel(GetWorld(), "MainMenu");
}

void  AFightMode::initPlayerMats()
{
	static ConstructorHelpers::FObjectFinder<UMaterial> blue_mat(TEXT("/Game/JointCharacter/playerMaterials/blue_player_mat.blue_player_mat"));
	if (blue_mat.Succeeded())
		player_mats.Add(blue_mat.Object);
	else
		UE_LOG(LogTemp, Warning, TEXT("Could not find player material asset"));

	static ConstructorHelpers::FObjectFinder<UMaterial> orange_mat(TEXT("/Game/JointCharacter/playerMaterials/orange_player_mat.orange_player_mat"));
	if (orange_mat.Succeeded())
		player_mats.Add(orange_mat.Object);
	else
		UE_LOG(LogTemp, Warning, TEXT("Could not find player material asset"));

	static ConstructorHelpers::FObjectFinder<UMaterial> green_mat(TEXT("/Game/JointCharacter/playerMaterials/green_player_mat.green_player_mat"));
	if (green_mat.Succeeded())
		player_mats.Add(green_mat.Object);
	else
		UE_LOG(LogTemp, Warning, TEXT("Could not find player material asset"));

	static ConstructorHelpers::FObjectFinder<UMaterial> purple_mat(TEXT("/Game/JointCharacter/playerMaterials/purple_player_mat.purple_player_mat"));
	if (purple_mat.Succeeded())
		player_mats.Add(purple_mat.Object);
	else
		UE_LOG(LogTemp, Warning, TEXT("Could not find player material asset"));
}