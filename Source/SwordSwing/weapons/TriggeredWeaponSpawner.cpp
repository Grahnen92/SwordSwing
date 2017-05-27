// Fill out your copyright notice in the Description page of Project Settings.

#include "SwordSwing.h"
#include "TriggeredWeaponSpawner.h"
#include "jointTest/Weapon.h"
#include "jointTest/JointCharacterTest.h"



// Sets default values
ATriggeredWeaponSpawner::ATriggeredWeaponSpawner() : AWeaponSpawner()
{
	
	trigger = CreateDefaultSubobject<UBoxComponent>(TEXT("Trigger"));
	trigger->SetBoxExtent(FVector(50.f, 50.f, 50.f));
	trigger->SetCollisionResponseToAllChannels(ECollisionResponse::ECR_Ignore);
	trigger->SetCollisionResponseToChannel(ECollisionChannel::ECC_GameTraceChannel1, ECollisionResponse::ECR_Overlap);
	trigger->SetCollisionResponseToChannel(ECollisionChannel::ECC_PhysicsBody, ECollisionResponse::ECR_Overlap);
	trigger->SetCollisionEnabled(ECollisionEnabled::QueryOnly);
	trigger->SetCollisionEnabled(ECollisionEnabled::QueryOnly);
	trigger->SetupAttachment(RootComponent);

	trigger_vis = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("TriggerVis"));
	//trigger_vis->SetCollisionEnabled(ECollisionEnabled::NoCollision);
	trigger_vis->SetupAttachment(trigger);

	trigger_ftext = CreateDefaultSubobject<UTextRenderComponent>(TEXT("TriggerFText"));
	trigger_ftext->SetCollisionEnabled(ECollisionEnabled::NoCollision);
	trigger_ftext->SetupAttachment(RootComponent);

	trigger_btext = CreateDefaultSubobject<UTextRenderComponent>(TEXT("TriggerBText"));
	trigger_btext->SetCollisionEnabled(ECollisionEnabled::NoCollision);
	trigger_btext->SetupAttachment(RootComponent);

}

// Called when the game starts or when spawned
void ATriggeredWeaponSpawner::BeginPlay()
{
	Super::BeginPlay();
	
	trigger_ftext->SetText(weapon_template->GetName());
	trigger_btext->SetText(weapon_template->GetName());

	trigger->OnComponentBeginOverlap.AddDynamic(this, &ATriggeredWeaponSpawner::OnOverlapBegin);
	trigger->OnComponentEndOverlap.AddDynamic(this, &ATriggeredWeaponSpawner::OnOverlapEnd);
}

// Called every frame
void ATriggeredWeaponSpawner::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

}

void ATriggeredWeaponSpawner::OnOverlapBegin(UPrimitiveComponent* OverlappedComp, AActor* OtherActor, UPrimitiveComponent* OtherComp, int32 OtherBodyIndex, bool bFromSweep, const FHitResult& SweepResult)
{
	if (!loaded_wep)
	{
		AJointCharacterTest* is_char = Cast<AJointCharacterTest>(OtherActor);
		if (is_char) {
			spawnWeapon();
			loaded_wep = weapons.Top();
		}
	}
}

void ATriggeredWeaponSpawner::OnOverlapEnd(UPrimitiveComponent* OverlappedComp, AActor* OtherActor, UPrimitiveComponent* OtherComp, int32 OtherBodyIndex)
{
	if (loaded_wep == OtherActor)
	{
		loaded_wep = nullptr;
	}
}
