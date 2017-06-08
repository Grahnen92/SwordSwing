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
	static ConstructorHelpers::FObjectFinder<UBlueprint> hit_dummy_obj(TEXT("/Game/Levels/PracticeArena/tut3/HittingJointCharacterDummy.HittingJointCharacterDummy"));
	if (hit_dummy_obj.Succeeded())
		hitting_dummy = (UClass*)hit_dummy_obj.Object->GeneratedClass;
	else
		UE_LOG(LogTemp, Warning, TEXT("Could not find dummy asset"));

	static ConstructorHelpers::FObjectFinder<UBlueprint> guard_dummy_obj(TEXT("/Game/Levels/PracticeArena/tut3/GuardingJointCharacterDummy.GuardingJointCharacterDummy"));
	if (guard_dummy_obj.Succeeded())
		guarding_dummy = (UClass*)guard_dummy_obj.Object->GeneratedClass;
	else
		UE_LOG(LogTemp, Warning, TEXT("Could not find dummy asset"));

	static ConstructorHelpers::FObjectFinder<UBlueprint> horiz_lock_obj(TEXT("/Game/Levels/HorizontalSwordLockBP.HorizontalSwordLockBP"));
	if (horiz_lock_obj.Succeeded())
		horiz_lock = (UClass*)horiz_lock_obj.Object->GeneratedClass;
	else
		UE_LOG(LogTemp, Warning, TEXT("Could not find dummy asset"));

	static ConstructorHelpers::FObjectFinder<UBlueprint> vert_lock_obj(TEXT("/Game/Levels/VerticalSwordLockBP.VerticalSwordLockBP"));
	if (vert_lock_obj.Succeeded())
		vert_lock = (UClass*)vert_lock_obj.Object->GeneratedClass;
	else
		UE_LOG(LogTemp, Warning, TEXT("Could not find dummy asset"));

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
	bridges.Add(instancer->addGrid(0, hex_radius, 0, 2, FVector(0.f, 48 * 2.f*hex_height, initial_pos)));
	bridges.Add(instancer->addGrid(0, hex_radius, 0, 2, FVector(0.f, 53 * 2.f*hex_height, initial_pos)));
	bridges.Add(instancer->addGrid(0, hex_radius, 0, 2, FVector(0.f, 58 * 2.f*hex_height, initial_pos)));

	
	platforms.Add(instancer->addCircle(0, hex_radius, 10, FVector(0.f, 71 * 2.f*hex_height, 0.f)));
	instancer->generateAll();
	TArray<FVector2D> platpaint;
	for (int i = 0; i < 1; i++)
		for (int j = 12; j < 13; j++)
			platpaint.Add(FVector2D(i, j));
	dynamic_cast<UHexCircleGenComponent*>(platforms[0])->paintHexes(platpaint);

	FActorSpawnParameters SpawnInfo;
	practice_bot = GetWorld()->SpawnActor<AJointCharacterTest>(fight_char, FVector(0.f, 71 * 2.f*hex_height, 200.f), FRotator(0.f, 0.f, 0.f), SpawnInfo);

	SpawnInfo.bNoFail = true;
	GetWorld()->SpawnActor<AJointCharacterTest>(hitting_dummy, FVector(-hex_radius*1.5f*9, 71 * 2.f*hex_height, 20.f), FRotator(0.f, -90.f, 0.f), SpawnInfo);
	GetWorld()->SpawnActor<AJointCharacterTest>(guarding_dummy, FVector(hex_radius*1.5f * 9, 71 * 2.f*hex_height, 20.f), FRotator(0.f, 90.f, 0.f), SpawnInfo);

	killzBox = Cast<UBoxComponent>(GetWorld()->SpawnActor<ATriggerBox>(ATriggerBox::StaticClass(), FVector(0.f, 0.f, initial_pos), FRotator(0.f, 0.f, 0.f), SpawnInfo)->GetCollisionComponent());
	killzBox->SetBoxExtent(FVector(2 * 71 * 2.f*hex_height, 2*71 * 2.f*hex_height, 500.f));
	killzBox->SetCollisionResponseToAllChannels(ECollisionResponse::ECR_Overlap);
	killzBox->SetCollisionResponseToChannel(ECollisionChannel::ECC_WorldStatic, ECollisionResponse::ECR_Ignore);
	killzBox->SetCollisionResponseToChannel(ECollisionChannel::ECC_WorldDynamic, ECollisionResponse::ECR_Ignore);
	killzBox->OnComponentBeginOverlap.AddDynamic(this, &APracticeMode::killzBeginOverlap);

	event_colliders.Add(Cast<UBoxComponent>(GetWorld()->SpawnActor<ATriggerBox>(ATriggerBox::StaticClass(), FVector(0.f, 5.5* 2.f*hex_height, 0.f), FRotator(0.f, 0.f, 0.f), SpawnInfo)->GetCollisionComponent()));
	event_colliders.Top()->SetBoxExtent(FVector(100.f, 100.f, 300.f));
	event_colliders.Top()->SetCollisionResponseToAllChannels(ECollisionResponse::ECR_Ignore);
	event_colliders.Top()->SetCollisionResponseToChannel(ECollisionChannel::ECC_GameTraceChannel1, ECollisionResponse::ECR_Overlap);
	event_colliders.Top()->OnComponentBeginOverlap.AddDynamic(this, &APracticeMode::eventOne);

	for (int i = 0; i < GameState->PlayerArray.Num(); i++)
	{
		auto player = GameState->PlayerArray[i]->GetNetOwningPlayer()->PlayerController;
		spawnPlayer(player);
	}
}

void APracticeMode::eventOne(UPrimitiveComponent* OverlappedComp, AActor* OtherActor, UPrimitiveComponent* OtherComp, int32 OtherBodyIndex, bool bFromSweep, const FHitResult& SweepResult)
{
	event_colliders.Top()->GetOwner()->Destroy();
	dynamic_cast<UHexGridGenComponent*>(bridges[0])->animateY(700.f, true, -initial_pos, false);
	instancer->pushAnim(bridges[0]);

	TArray<FVector2D> bridgepaint;
	for (int j = 0; j < 19; j++)
		bridgepaint.Add(FVector2D(1, j));
	//brdige12->paintHexes(bridgepaint);
	dynamic_cast<UHexGridGenComponent*>(bridges[0])->animatePaintY(500.f, true, 0, true);

	dynamic_cast<UHexGridGenComponent*>(platforms[1])->animateY(500.f, true, -initial_pos, false);
	instancer->pushAnim(platforms[1]);

	FActorSpawnParameters SpawnInfo;
	FTransform tmp_trans; tmp_trans.SetTranslation(platforms[1]->GetComponentLocation() + FVector(-700.f, 500.f, -initial_pos + 200.f)); tmp_trans.SetRotation(FQuat(FRotator(0.f, -70.f, 0.f)));
	ATextRenderActor* tmp_text = GetWorld()->SpawnActor<ATextRenderActor>(ATextRenderActor::StaticClass(), tmp_trans, SpawnInfo);
	tmp_text->GetTextRender()->SetText(TEXT("Press  A  to jump"));
	tmp_text->GetTextRender()->SetWorldSize(50.f);
	tmp_text->GetTextRender()->SetTextRenderColor(FColor::Black);
	tmp_text->GetTextRender()->SetHorizontalAlignment(EHorizTextAligment::EHTA_Center);

	event_colliders.Add(Cast<UBoxComponent>(GetWorld()->SpawnActor<ATriggerBox>(ATriggerBox::StaticClass(), FVector(0.f, 16 * 2.f*hex_height, 0.f), FRotator(0.f, 0.f, 0.f), SpawnInfo)->GetCollisionComponent()));
	event_colliders.Top()->SetBoxExtent(FVector(hex_radius*1.5f * 4, 2 * 2.f*hex_height, 300.f), true);
	event_colliders.Top()->SetCollisionResponseToAllChannels(ECollisionResponse::ECR_Ignore);
	event_colliders.Top()->SetCollisionResponseToChannel(ECollisionChannel::ECC_GameTraceChannel1, ECollisionResponse::ECR_Overlap);
	event_colliders.Top()->OnComponentBeginOverlap.AddDynamic(this, &APracticeMode::eventTwo);
}
void APracticeMode::eventTwo(UPrimitiveComponent* OverlappedComp, AActor* OtherActor, UPrimitiveComponent* OtherComp, int32 OtherBodyIndex, bool bFromSweep, const FHitResult& SweepResult)
{
	event_colliders.Top()->GetOwner()->Destroy();
	dynamic_cast<UHexGridGenComponent*>(platforms[2])->animateY(500.f, true, -initial_pos, false);
	instancer->pushAnim(platforms[2]);

	FActorSpawnParameters SpawnInfo;
	event_colliders.Add(Cast<UBoxComponent>(GetWorld()->SpawnActor<ATriggerBox>(ATriggerBox::StaticClass(), FVector(0.f, 22 * 2.f*hex_height, 0.f), FRotator(0.f, 0.f, 0.f), SpawnInfo)->GetCollisionComponent()));
	event_colliders.Top()->SetBoxExtent(FVector(hex_radius*1.5f * 4, 2 * 2.f*hex_height, 300.f));
	event_colliders.Top()->SetCollisionResponseToAllChannels(ECollisionResponse::ECR_Ignore);
	event_colliders.Top()->SetCollisionResponseToChannel(ECollisionChannel::ECC_GameTraceChannel1, ECollisionResponse::ECR_Overlap);
	event_colliders.Top()->OnComponentBeginOverlap.AddDynamic(this, &APracticeMode::eventThree);
}

void APracticeMode::eventThree(UPrimitiveComponent* OverlappedComp, AActor* OtherActor, UPrimitiveComponent* OtherComp, int32 OtherBodyIndex, bool bFromSweep, const FHitResult& SweepResult)
{
	event_colliders.Top()->GetOwner()->Destroy();

	dynamic_cast<UHexGridGenComponent*>(platforms[3])->animateY(500.f, true, -initial_pos, false);
	instancer->pushAnim(platforms[3]);

	FActorSpawnParameters SpawnInfo;
	FTransform tmp_trans; tmp_trans.SetTranslation(platforms[2]->GetComponentLocation() + FVector(-700.f, 500.f, -initial_pos + 200.f)); tmp_trans.SetRotation(FQuat(FRotator(0.f, -70.f, 0.f)));
	ATextRenderActor* tmp_text2 = GetWorld()->SpawnActor<ATextRenderActor>(ATextRenderActor::StaticClass(), tmp_trans, SpawnInfo);
	tmp_text2->GetTextRender()->SetText(TEXT("Press  L1  to dash <br> Try combining it with jump to travel further"));
	tmp_text2->GetTextRender()->SetWorldSize(50.f);
	tmp_text2->GetTextRender()->SetTextRenderColor(FColor::Black);
	tmp_text2->GetTextRender()->SetHorizontalAlignment(EHorizTextAligment::EHTA_Center);

	event_colliders.Add(Cast<UBoxComponent>(GetWorld()->SpawnActor<ATriggerBox>(ATriggerBox::StaticClass(), platforms[3]->GetComponentLocation() + FVector(0.f, 0.f, -initial_pos), FRotator(0.f, 0.f, 0.f), SpawnInfo)->GetCollisionComponent()));
	event_colliders.Top()->SetBoxExtent(FVector(hex_radius*1.5f * 4, 3 * 2.f*hex_height, 300.f));
	event_colliders.Top()->SetCollisionResponseToAllChannels(ECollisionResponse::ECR_Ignore);
	event_colliders.Top()->SetCollisionResponseToChannel(ECollisionChannel::ECC_GameTraceChannel1, ECollisionResponse::ECR_Overlap);
	event_colliders.Top()->OnComponentBeginOverlap.AddDynamic(this, &APracticeMode::eventFour);
}
void APracticeMode::eventFour(UPrimitiveComponent* OverlappedComp, AActor* OtherActor, UPrimitiveComponent* OtherComp, int32 OtherBodyIndex, bool bFromSweep, const FHitResult& SweepResult)
{
	event_colliders.Top()->GetOwner()->Destroy();

	FActorSpawnParameters SpawnInfo;
	FTransform tmp_trans; tmp_trans.SetTranslation(platforms[4]->GetComponentLocation() + FVector(-700.f, 500.f, -initial_pos + 220.f)); tmp_trans.SetRotation(FQuat(FRotator(0.f, -60.f, 0.f)));
	ATextRenderActor* tmp_text = GetWorld()->SpawnActor<ATextRenderActor>(ATextRenderActor::StaticClass(), tmp_trans, SpawnInfo);
	tmp_text->GetTextRender()->SetText(TEXT("Press and HOLD R1 with your <br> hand above a weapon to pick it up"));
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

	dynamic_cast<UHexGridGenComponent*>(bridges[1])->animatePaintY(500.f, true, 0, true);

	dynamic_cast<UHexCircleGenComponent*>(platforms[4])->animateR(500.f, true, -initial_pos, false);
	instancer->pushAnim(platforms[4]);

	//dynamic_cast<UHexGridGenComponent*>(bridges[2])->animateY(700.f, true, -initial_pos, false);
	//instancer->pushAnim(bridges[2]);

	TArray<FVector2D> bridgepaint;
	for (int j = 3; j < 4; j++)
		bridgepaint.Add(FVector2D(0, j));
	dynamic_cast<UHexGridGenComponent*>(bridges[2])->paintHexes(bridgepaint);

	event_colliders.Add(Cast<UBoxComponent>(GetWorld()->SpawnActor<ATriggerBox>(ATriggerBox::StaticClass(), bridges[2]->GetComponentLocation() + FVector(0.f, 0.f, -initial_pos), FRotator(0.f, 0.f, 0.f), SpawnInfo)->GetCollisionComponent()));
	event_colliders.Top()->SetBoxExtent(FVector(hex_radius*1.5f * 2, 2 * 2.f*hex_height, 300.f));
	event_colliders.Top()->SetCollisionResponseToAllChannels(ECollisionResponse::ECR_Ignore);
	event_colliders.Top()->SetCollisionResponseToChannel(ECollisionChannel::ECC_GameTraceChannel1, ECollisionResponse::ECR_Overlap);
	event_colliders.Top()->OnComponentBeginOverlap.AddDynamic(this, &APracticeMode::eventFive);
}
void APracticeMode::eventFive(UPrimitiveComponent* OverlappedComp, AActor* OtherActor, UPrimitiveComponent* OtherComp, int32 OtherBodyIndex, bool bFromSweep, const FHitResult& SweepResult)
{
	event_colliders.Top()->GetOwner()->Destroy();
	
	FActorSpawnParameters SpawnInfo;
	FTransform tmp_trans; tmp_trans.SetTranslation(bridges[2]->GetComponentLocation() + FVector(-700.f, 500.f, -initial_pos + 220.f)); tmp_trans.SetRotation(FQuat(FRotator(0.f, -60.f, 0.f)));
	ATextRenderActor* tmp_text = GetWorld()->SpawnActor<ATextRenderActor>(ATextRenderActor::StaticClass(), tmp_trans, SpawnInfo);
	tmp_text->GetTextRender()->SetText(TEXT("Press and HOLD L2 to enter FIGHT MODE <br> The camera will then lock to a target and <br> RIGHT STICK no longer controls the camera <br> instead it now controls the direction of your weapon <br> <br> try touching the middle of the object in front of you."));
	tmp_text->GetTextRender()->SetWorldSize(50.f);
	tmp_text->GetTextRender()->SetTextRenderColor(FColor::Black);
	tmp_text->GetTextRender()->SetHorizontalAlignment(EHorizTextAligment::EHTA_Center);
	
	vert_lock_inst = GetWorld()->SpawnActor<ASwordLock>(vert_lock, bridges[2]->GetComponentLocation() + FVector(0.f, 1.5f * 2.f*hex_height, -initial_pos + 170.f), FRotator(0.f, 0.f, 0.f), SpawnInfo);
	UStaticMeshComponent* lock_center = vert_lock_inst->getLockCenter();
	lock_center->OnComponentBeginOverlap.AddDynamic(this, &APracticeMode::eventSex);


}
void APracticeMode::eventSex(UPrimitiveComponent* OverlappedComp, AActor* OtherActor, UPrimitiveComponent* OtherComp, int32 OtherBodyIndex, bool bFromSweep, const FHitResult& SweepResult)
{
	vert_lock_inst->Destroy();
	vert_lock_inst = nullptr;
	dynamic_cast<UHexGridGenComponent*>(bridges[3])->animateY(700.f, true, -initial_pos, false);
	instancer->pushAnim(bridges[3]);
	TArray<FVector2D> bridgepaint;
	for (int j = 3; j < 4; j++)
		bridgepaint.Add(FVector2D(0, j));
	dynamic_cast<UHexGridGenComponent*>(bridges[3])->paintHexes(bridgepaint);


	FActorSpawnParameters SpawnInfo;
	horiz_lock_inst = GetWorld()->SpawnActor<ASwordLock>(horiz_lock, bridges[3]->GetComponentLocation() + FVector(0.f, 1.5f * 2.f*hex_height, -initial_pos + 170.f), FRotator(0.f, 0.f, 0.f), SpawnInfo);
	horiz_lock_inst->getLockCenter()->OnComponentBeginOverlap.AddDynamic(this, &APracticeMode::eventSeven);
}
void APracticeMode::eventSeven(UPrimitiveComponent* OverlappedComp, AActor* OtherActor, UPrimitiveComponent* OtherComp, int32 OtherBodyIndex, bool bFromSweep, const FHitResult& SweepResult)
{
	horiz_lock_inst->Destroy();
	horiz_lock_inst = nullptr;
	dynamic_cast<UHexGridGenComponent*>(bridges[4])->animateY(700.f, true, -initial_pos, false);
	instancer->pushAnim(bridges[4]);
}

void  APracticeMode::killzBeginOverlap(UPrimitiveComponent* OverlappedComp, AActor* OtherActor, UPrimitiveComponent* OtherComp, int32 OtherBodyIndex, bool bFromSweep, const FHitResult& SweepResult)
{
	AJointCharacterTest* tmp_char = Cast<AJointCharacterTest>(OtherActor);
	if (tmp_char) {
		if (tmp_char->GetNetOwningPlayer())
			registerDeath(tmp_char->GetNetOwningPlayer()->PlayerController);
		else
			registerDeath(nullptr);
	}
	else
	{
		OtherActor->Destroy();
	}
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
	FTimerDelegate TimerDel;
	TimerDel.BindUFunction(this, FName("delayedPlayerSpawn"), round_loser);
	if (round_loser){
		FTimerHandle unused_handle;
		GetWorldTimerManager().SetTimer(unused_handle, TimerDel, 3.0f, false);

	}
	else {
		FTimerHandle unused_handle;
		GetWorldTimerManager().SetTimer(unused_handle, this, &APracticeMode::delayedDummySpawn, 3.0f, false);
	}
}

void APracticeMode::timedWeaponSpawn()
{
	deferred_spawner->spawnWeapon();
}

void  APracticeMode::delayedDummySpawn()
{
	practice_bot->Destroy();
	FActorSpawnParameters SpawnInfo;
	practice_bot = GetWorld()->SpawnActor<AJointCharacterTest>(fight_char, FVector(0.f, 71 * 2.f*hex_height, 200.f), FRotator(0.f, 0.f, 0.f), SpawnInfo);
}

void  APracticeMode::delayedPlayerSpawn(APlayerController* round_loser)
{
	spawnPlayer(round_loser);
}