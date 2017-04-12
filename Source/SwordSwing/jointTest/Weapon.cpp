// Fill out your copyright notice in the Description page of Project Settings.

#include "SwordSwing.h"
#include "Weapon.h"


// Sets default values
AWeapon::AWeapon()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

	initWeapon();
}

// Called when the game starts or when spawned
void AWeapon::BeginPlay()
{
	Super::BeginPlay();

	weapon_head->OnComponentHit.AddDynamic(this, &AWeapon::OnSwordHit);
}

// Called every frame
void AWeapon::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	//DrawDebugPoint(
	//	GetWorld(),
	//	weapon_handle_point->GetComponentLocation(),
	//	20,  					//size
	//	FColor(255, 0, 255),  //pink
	//	false,  				//persistent (never goes away)
	//	0.03 					//point leaves a trail on moving object
	//);

	//DrawDebugPoint(
	//	GetWorld(),
	//	GetActorLocation(),
	//	20,  					//size
	//	FColor(255, 0, 0),  //pink
	//	false,  				//persistent (never goes away)
	//	0.03 					//point leaves a trail on moving object
	//);
}

void AWeapon::OnSwordHit(UPrimitiveComponent* HitComp, AActor* OtherActor, UPrimitiveComponent* OtherComp, FVector NormalImpulse, const FHitResult& Hit)
{
	//FVector hit_vel = weapon_bi->GetUnrealWorldVelocity() + FVector::CrossProduct(weapon_bi->GetUnrealWorldAngularVelocity(), Hit.Location - weapon_handle_1_bi->GetCOMPosition());
	///*UE_LOG(LogTemp, Warning, TEXT("hit_vel: %f"), hit_vel.Size());
	//UE_LOG(LogTemp, Warning, TEXT("NormalImpulse: %f"), NormalImpulse.Size());
	//UE_LOG(LogTemp, Warning, TEXT("----------------"));*/
	//FLatentActionInfo actionInfo;
	//actionInfo.CallbackTarget = this;
	//if (hit_vel.Size() > 3500.f)
	//	GetController()->CastToPlayerController()->PlayDynamicForceFeedback(0.2f, 0.2f, false, true, false, true, EDynamicForceFeedbackAction::Start, actionInfo);

	//if (hit_vel.Size() > 11000.f && NormalImpulse.Size() > 4000.f)
	//{
	//	//UE_LOG(LogTemp, Warning, TEXT("hit_vel: %f"), hit_vel.Size());
	//	FMath::Min(weapon_wood_impact_audio->VolumeMultiplier = NormalImpulse.Size() / 40000.f, 0.6f);
	//	weapon_wood_impact_audio->Deactivate();
	//	weapon_wood_impact_audio->Activate();



	//	GetController()->CastToPlayerController()->ClientPlayForceFeedback(weapon_impact, false, FName("SwordImpact"));

	//	//GetController()->CastToPlayerController()->DynamicForceFeedbacks
	//}
}

void AWeapon::initWeapon()
{
	this->Tags.Add("weapon");
	weapon_shaft = CreateDefaultSubobject<UCapsuleComponent>(TEXT("WeaponShaft"));
	weapon_shaft->ComponentTags.Add("shaft");
	//weapon->SetupAttachment(weapon_axis);
	RootComponent = weapon_shaft;
	
	weapon_shaft->SetRelativeScale3D(FVector(1.f, 1.f, 1.f));
	weapon_shaft->SetCapsuleHalfHeight(75);
	weapon_shaft->SetCapsuleRadius(2.5f);
	weapon_shaft->SetSimulatePhysics(true);
	weapon_shaft->SetEnableGravity(false);
	weapon_shaft->SetPhysicsMaxAngularVelocity(5000.f);
	//weapon_shaft->SetMassOverrideInKg(NAME_None, 10.f, true);
	weapon_shaft_vis = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("WeaponShaftVis"));
	weapon_shaft_vis->SetupAttachment(weapon_shaft);

	weapon_handle_point = CreateDefaultSubobject<USceneComponent>(TEXT("WeaponHandle"));
	weapon_handle_point->SetupAttachment(weapon_shaft);
	weapon_handle_point->SetRelativeLocation(FVector(0.f, 0.f, -75.f));
	weapon_handle_point->ComponentTags.Add("handle_point");

	weapon_head = CreateDefaultSubobject<UBoxComponent>(TEXT("WeaponHead"));
	weapon_head->ComponentTags.Add("head");
	weapon_head->SetupAttachment(weapon_shaft);
	weapon_head->SetRelativeLocation(FVector(0.f, 0.f, 0.f));
	weapon_head->SetRelativeScale3D(FVector(1.f, 1.f, 1.f));
	weapon_head->SetBoxExtent(FVector(10.f, 2.f, 75.f));
	weapon_head->SetSimulatePhysics(false);
	weapon_head->SetEnableGravity(false);
	weapon_head->SetPhysicsMaxAngularVelocity(5000.f);
	weapon_head->SetMassOverrideInKg(NAME_None, 10.f, true);
	weapon_head_vis = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("WeaponHeadVis"));
	weapon_head_vis->SetupAttachment(weapon_head);

	weapon_trail = CreateDefaultSubobject<UParticleSystemComponent>(TEXT("WeaponHeadTrail"));
	weapon_trail->SetupAttachment(weapon_head_vis);

	

	weapon_swish_audio = CreateDefaultSubobject<UAudioComponent>(TEXT("WeaponSwishAudioTest"));
	weapon_swish_audio->SetupAttachment(weapon_shaft);
	weapon_swish_audio->bEnableLowPassFilter = true;
	weapon_swish_audio->LowPassFilterFrequency = 10.f;
	//weapon_audio->bEQFilterApplied = true;
	weapon_swish_audio->VolumeMultiplier = 0.0f;

	weapon_wood_impact_audio = CreateDefaultSubobject<UAudioComponent>(TEXT("WeaponWoodImpactAudioTest"));
	weapon_wood_impact_audio->SetupAttachment(weapon_head);
	weapon_wood_impact_audio->VolumeMultiplier = 0.5f;

	static ConstructorHelpers::FObjectFinder<UForceFeedbackEffect> SwordImpactObj(TEXT("/Game/JointCharacter/weapon/sword_impact"));

	weapon_impact = SwordImpactObj.Object;
}
