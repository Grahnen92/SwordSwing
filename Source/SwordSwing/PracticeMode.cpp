// Fill out your copyright notice in the Description page of Project Settings.

#include "SwordSwing.h"
#include "PracticeMode.h"

#include "EngineUtils.h"
//#include "TextRenderActor.h"

#include "InitPawn.h"
#include "jointTest/JointCharacterTest.h"
#include "ThirdPersonController.h"
#include "weapons/WeaponSpawner.h"
#include "Runtime/Engine/Classes/Engine/TextRenderActor.h"


#include "jointTest/Weapon.h"
#include "weapons/TriggeredWeaponSpawner.h"

#include <sstream>
#include <string>


APracticeMode::APracticeMode(const FObjectInitializer& ObjectInitializer) :
	Super(ObjectInitializer)
{
	//instancer = CreateDefaultSubobject<UHexInstancerComponent>(TEXT("InstancedLandscape"));
	//instancer->SetWorldLocation(FVector(0.f, 0.f, 0.f));

	//instancer->addCircle(0, 100.f, 6, FVector(0.f, 0.f, 0.f));
	
	static ConstructorHelpers::FClassFinder<AWeapon> tmp_wep_asset(TEXT("/Game/JointCharacter/weapon/LongSword"));
	if(tmp_wep_asset.Succeeded())
		weapon_asset = tmp_wep_asset.Class;
}

void APracticeMode::InitGame(const FString &MapName, const FString &Options, FString &ErrorMessage)
{
	Super::InitGame(MapName, Options, ErrorMessage);
}

void APracticeMode::BeginPlay()
{
	Super::BeginPlay();
	
	if (!instancer)
	{
		FActorSpawnParameters SpawnInfo;
		FTransform tmp_trans;
		instancer = GetWorld()->SpawnActor<AHexInstancer>(AHexInstancer::StaticClass(), tmp_trans, SpawnInfo);
	}
	
	hex_height = hex_radius*FMath::Cos(PI / 6.f);
	platforms.Add(instancer->addCircle(0, hex_radius, 6, FVector(0.f, 0.f, 0.f)));
	bridges.Add( instancer->addGrid(0, hex_radius, 1, 4, FVector(0.f, 10 * 2.f*hex_height, initial_pos)));
	platforms.Add(instancer->addGrid(0, hex_radius, 4, 2, FVector(0.f, 16 * 2.f*hex_height, initial_pos)));
	platforms.Add(instancer->addGrid(0, hex_radius, 4, 2, FVector(0.f, 23 * 2.f*hex_height, initial_pos)));
	platforms.Add(instancer->addGrid(0, hex_radius, 4, 2, FVector(0.f, 31 * 2.f*hex_height, initial_pos)));
	
	bridges.Add(instancer->addGrid(0, hex_radius, 1, 4, FVector(0.f, 37 * 2.f*hex_height, initial_pos)));
	platforms.Add(instancer->addCircle(0, hex_radius, 2, FVector(0.f, 43 * 2.f*hex_height, initial_pos)));
	bridges.Add(instancer->addGrid(0, hex_radius, 0, 3, FVector(0.f, 48 * 2.f*hex_height, initial_pos)));
	platforms.Add(instancer->addCircle(0, hex_radius, 10, FVector(0.f, 61 * 2.f*hex_height, 0.f)));
	instancer->generateAll();
	TArray<FVector2D> platpaint;
	for (int i = 0; i < 1; i++)
		for (int j = 12; j < 13; j++)
			platpaint.Add(FVector2D(i, j));
	dynamic_cast<UHexCircleGenComponent*>(platforms[0])->paintHexes(platpaint);

	FActorSpawnParameters SpawnInfo;
	practice_bot = GetWorld()->SpawnActor<AJointCharacterTest>(fight_char, FVector(0.f, 61 * 2.f*hex_height, 200.f), FRotator(0.f, 0.f, 0.f), SpawnInfo);

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

// Called every frame
void APracticeMode::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
}

void APracticeMode::registerDeath(APlayerController* round_loser)
{
	if (round_loser){
		spawnPlayer(round_loser);
	}
	else {
		FTimerHandle unused_handle;
		GetWorldTimerManager().SetTimer(unused_handle, this, &APracticeMode::delayedCharacterpawn, 3.0f, false);
	}
}

void APracticeMode::completeTaskOne()
{
	dynamic_cast<UHexGridGenComponent*>(bridges[0])->animateY(700.f, true, -initial_pos, false);
	instancer->pushAnim(bridges[0]);

	dynamic_cast<UHexGridGenComponent*>(platforms[1])->animateY(500.f, true, -initial_pos, false);
	instancer->pushAnim(platforms[1]);

	dynamic_cast<UHexGridGenComponent*>(platforms[2])->animateY(500.f, true, -initial_pos, false);
	instancer->pushAnim(platforms[2]);

	dynamic_cast<UHexGridGenComponent*>(platforms[3])->animateY(500.f, true, -initial_pos, false);
	instancer->pushAnim(platforms[3]);
	
	TArray<FVector2D> bridgepaint;
	for (int j = 0; j < 19; j++)
		bridgepaint.Add(FVector2D(1, j));
	//brdige12->paintHexes(bridgepaint);
	dynamic_cast<UHexGridGenComponent*>(bridges[0])->animatePaintY(500.f, true, 0, true);

	FActorSpawnParameters SpawnInfo;
	FTransform tmp_trans; tmp_trans.SetTranslation(platforms[1]->GetComponentLocation() + FVector::UpVector*-(initial_pos - 1.f)); tmp_trans.SetRotation(FQuat(FRotator(90.f, -90.f, 0.f)));
	ATextRenderActor* tmp_text = GetWorld()->SpawnActor<ATextRenderActor>(ATextRenderActor::StaticClass(), tmp_trans, SpawnInfo);
	tmp_text->GetTextRender()->SetText("Press  X  to jump");
	tmp_text->GetTextRender()->SetWorldSize(50.f);
	tmp_text->GetTextRender()->SetTextRenderColor(FColor::Black);
	tmp_text->GetTextRender()->SetHorizontalAlignment(EHorizTextAligment::EHTA_Center);

	tmp_trans; tmp_trans.SetTranslation(platforms[2]->GetComponentLocation() + FVector::UpVector * -(initial_pos - 1.f)); tmp_trans.SetRotation(FQuat(FRotator(90.f, -90.f, 0.f)));
	ATextRenderActor* tmp_text2 = GetWorld()->SpawnActor<ATextRenderActor>(ATextRenderActor::StaticClass(), tmp_trans, SpawnInfo);
	tmp_text2->GetTextRender()->SetText("Press  L1  to dash <br> Try combining it with jump to travel further");
	tmp_text2->GetTextRender()->SetWorldSize(50.f);
	tmp_text2->GetTextRender()->SetTextRenderColor(FColor::Black);
	tmp_text2->GetTextRender()->SetHorizontalAlignment(EHorizTextAligment::EHTA_Center);
}

void APracticeMode::completeTaskTwo()
{

	FActorSpawnParameters SpawnInfo;
	FTransform tmp_trans; tmp_trans.SetTranslation(platforms[4]->GetComponentLocation() + FVector::UpVector *- (initial_pos - 1.f) - FVector::RightVector*200.f); tmp_trans.SetRotation(FQuat(FRotator(90.f, -90.f, 0.f)));
	ATextRenderActor* tmp_text = GetWorld()->SpawnActor<ATextRenderActor>(ATextRenderActor::StaticClass(), tmp_trans, SpawnInfo);
	tmp_text->GetTextRender()->SetText("Press and HOLD R1 with your <br> hand above a weapon to pick it up");
	tmp_text->GetTextRender()->SetWorldSize(50.f);
	tmp_text->GetTextRender()->SetTextRenderColor(FColor::Black);
	tmp_text->GetTextRender()->SetHorizontalAlignment(EHorizTextAligment::EHTA_Center);

	FActorSpawnParameters SpawnInfo2;
	tmp_trans.SetTranslation(platforms[4]->GetComponentLocation() + FVector::UpVector *-(initial_pos - 1.f) + FVector::RightVector*200.f); tmp_trans.SetRotation(FQuat(FRotator(0.f, 90.f, 0.f)));
	deferred_spawner = GetWorld()->SpawnActorDeferred<ATriggeredWeaponSpawner>(ATriggeredWeaponSpawner::StaticClass(), tmp_trans);
	deferred_spawner->weapon_template = weapon_asset;
	UGameplayStatics::FinishSpawningActor(deferred_spawner, tmp_trans);
	
	FTimerHandle unused_handle;
	GetWorldTimerManager().SetTimer(unused_handle, this, &APracticeMode::timedWeaponSpawn, 3.0f, false);
	
	dynamic_cast<UHexGridGenComponent*>(bridges[1])->animateY(700.f, true, -initial_pos, false);
	instancer->pushAnim(bridges[1]);

	TArray<FVector2D> bridgepaint;
	for (int j = 0; j < 19; j++)
		bridgepaint.Add(FVector2D(1, j));
	//brdige12->paintHexes(bridgepaint);
	dynamic_cast<UHexGridGenComponent*>(bridges[1])->animatePaintY(500.f, true, 0, true);

	dynamic_cast<UHexCircleGenComponent*>(platforms[4])->animateR(500.f, true, -initial_pos, false);
	instancer->pushAnim(platforms[4]);

	dynamic_cast<UHexGridGenComponent*>(bridges[2])->animateY(700.f, true, -initial_pos, false);
	instancer->pushAnim(bridges[2]);


}

void APracticeMode::timedWeaponSpawn()
{
	deferred_spawner->spawnWeapon();
}

void  APracticeMode::delayedCharacterpawn()
{
	practice_bot->Destroy();
	FActorSpawnParameters SpawnInfo;
	practice_bot = GetWorld()->SpawnActor<AJointCharacterTest>(fight_char, FVector(0.f, 61 * 2.f*hex_height, 200.f), FRotator(0.f, 0.f, 0.f), SpawnInfo);
}