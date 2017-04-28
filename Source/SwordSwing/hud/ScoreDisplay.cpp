// Fill out your copyright notice in the Description page of Project Settings.

#include "SwordSwing.h"
#include "ScoreDisplay.h"


// Sets default values
AScoreDisplay::AScoreDisplay()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
	icon = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("Icon"));
	RootComponent = icon;
	icon->SetOnlyOwnerSee(true);

	icon->AddLocalRotation(FRotator(0.f, 180.f, 0.f));
	//icon->SetupAttachment(RootComponent);
	static ConstructorHelpers::FObjectFinder<UStaticMesh> plane(TEXT("/Engine/EditorMeshes/EditorPlane.EditorPlane"));
	if (plane.Object)
		icon->SetStaticMesh(plane.Object);
	else
		UE_LOG(LogTemp, Warning, TEXT("could not find plane asset for ScoreDisplay staticmeshcomponent"));

	static ConstructorHelpers::FObjectFinder<UMaterial> icon_mat(TEXT("/Game/hud/fightArena/player_icon_material.player_icon_material"));
	if (icon_mat.Object)
		icon->SetMaterial(0, icon_mat.Object);
	else
		UE_LOG(LogTemp, Warning, TEXT("could not find material asset for ScoreDisplay staticmeshcomponent"));

	score = CreateDefaultSubobject<UTextRenderComponent>(TEXT("Score"));
	score->SetupAttachment(RootComponent);
	score->SetRelativeLocation(FVector(0.f, 0.f, 130.f));
	score->SetRelativeRotation(FRotator(0.f, 180.f, 0.f));
	score->HorizontalAlignment = EHorizTextAligment::EHTA_Center;
	score->WorldSize = (250.f);
	score->SetOnlyOwnerSee(true);

	static ConstructorHelpers::FObjectFinder<UMaterial> text_mat(TEXT("/Engine/EngineMaterials/DefaultTextMaterialOpaque.DefaultTextMaterialOpaque"));
	if (text_mat.Object)
		score->SetMaterial(0, text_mat.Object);
	else
		UE_LOG(LogTemp, Warning, TEXT("could not find material asset for ScoreDisplay textcomponent"));
}

// Called when the game starts or when spawned
void AScoreDisplay::BeginPlay()
{
	Super::BeginPlay();
	
}

// Called every frame
void AScoreDisplay::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
}


void AScoreDisplay::setScoreText(FString score_string)
{
	score->SetText(score_string);
}

void AScoreDisplay::setIconMaterial(UMaterial* icon_mat)
{
	icon->SetMaterial(0, icon_mat);
}
