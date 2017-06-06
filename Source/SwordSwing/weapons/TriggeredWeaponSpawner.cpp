// Fill out your copyright notice in the Description page of Project Settings.

#include "SwordSwing.h"
#include "TriggeredWeaponSpawner.h"
#include "jointTest/Weapon.h"
#include "jointTest/JointCharacterTest.h"



// Sets default values
ATriggeredWeaponSpawner::ATriggeredWeaponSpawner() : AWeaponSpawner()
{
	root = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));
	RootComponent = root;

	trigger = CreateDefaultSubobject<UCapsuleComponent>(TEXT("Trigger"));
	trigger->SetCapsuleHalfHeight(150.f);
	trigger->SetCapsuleRadius(50.f);
	trigger->SetCollisionResponseToAllChannels(ECollisionResponse::ECR_Ignore);
	trigger->SetCollisionResponseToChannel(ECollisionChannel::ECC_GameTraceChannel1, ECollisionResponse::ECR_Overlap);
	trigger->SetCollisionResponseToChannel(ECollisionChannel::ECC_PhysicsBody, ECollisionResponse::ECR_Overlap);
	trigger->SetCollisionEnabled(ECollisionEnabled::QueryOnly);
	trigger->SetCollisionEnabled(ECollisionEnabled::QueryOnly);
	trigger->SetupAttachment(root);
	trigger->SetRelativeLocation(FVector(0.f, 0.f, 100.f));
	trigger->SetCollisionObjectType(ECollisionChannel::ECC_GameTraceChannel3);

	trigger_indicator_decal = CreateDefaultSubobject<UDecalComponent>(TEXT("TriggerVis"));
	trigger_indicator_decal->SetupAttachment(root);
	static ConstructorHelpers::FObjectFinder<UMaterialInterface> decal_mat(TEXT("/Game/materials/redCircleDecal.redCircleDecal"));
	//instance_mats.Add(land_mat.Object);
	trigger_indicator_decal->SetMaterial(0, decal_mat.Object);
	trigger_indicator_decal->SetRelativeRotation(FRotator(90.f, 0.f, 0.f));
	trigger_indicator_decal->DecalSize = FVector(20.f, 50.f, 50.f);

	trigger_ftext = CreateDefaultSubobject<UTextRenderComponent>(TEXT("TriggerFText"));
	trigger_ftext->SetCollisionEnabled(ECollisionEnabled::NoCollision);
	trigger_ftext->SetupAttachment(root);
	trigger_ftext->SetRelativeLocation(FVector(0.f, 0.f, 200.f));
	trigger_ftext->SetWorldSize(26.f);
	trigger_ftext->SetHorizontalAlignment(EHorizTextAligment::EHTA_Center);

	trigger_btext = CreateDefaultSubobject<UTextRenderComponent>(TEXT("TriggerBText"));
	trigger_btext->SetCollisionEnabled(ECollisionEnabled::NoCollision);
	trigger_btext->SetupAttachment(root);
	trigger_btext->SetRelativeLocation(FVector(0.f, 0.f, 200.f));
	trigger_btext->SetRelativeRotation(FRotator(0.f, -180.f, 0.f));
	trigger_btext->SetWorldSize(26.f);
	trigger_btext->SetHorizontalAlignment(EHorizTextAligment::EHTA_Center);
}

// Called when the game starts or when spawned
void ATriggeredWeaponSpawner::BeginPlay()
{
	Super::BeginPlay();
	if (weapon_template)
	{
		trigger_ftext->SetText(weapon_template->GetName());
		trigger_btext->SetText(weapon_template->GetName());
	}

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
