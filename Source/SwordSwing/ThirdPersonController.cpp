// Fill out your copyright notice in the Description page of Project Settings.

#include "SwordSwing.h"
#include "FightMode.h"
#include "Blueprint/UserWidget.h"
#include "ThirdPersonController.h"


AThirdPersonController::AThirdPersonController(const class FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{

}

// Called when the game starts or when spawned
void AThirdPersonController::BeginPlay()
{
	Super::BeginPlay();

	FInputModeGameOnly init_mode;
	
	SetInputMode(init_mode);
	//InputComponent
	
	SetupInputComponent();
	InputComponent->BindAction("Pause", IE_Pressed, this, &AThirdPersonController::pauseGame);
	InputComponent->BindAction("SpawnPlayer", IE_Pressed, this, &AThirdPersonController::spawnPlayer);
	InputComponent->BindAction("DespawnPlayer", IE_Pressed, this, &AThirdPersonController::despawnPlayer);

	if (pauseWidgetTemplate) // Check if the Asset is assigned in the blueprint.
	{
		// Create the widget and store it.
		PauseMenu = CreateWidget<UUserWidget>(this, pauseWidgetTemplate);
	}
	
}

void AThirdPersonController::pauseGame()
{
	if (PauseMenu)
	{
		//let add it to the view port
		PauseMenu->AddToViewport();

		FInputModeUIOnly UIMode;
		SetInputMode(UIMode);

		bShowMouseCursor = true;

	}
	else
	{
		UE_LOG(LogTemp, Warning, TEXT("No pause menu has been created"));
	}

	
}

void AThirdPersonController::spawnPlayer()
{
	Cast<AFightMode>(GetWorld()->GetAuthGameMode())->spawnPlayer(this);
}
void AThirdPersonController::despawnPlayer()
{
	Cast<AFightMode>(GetWorld()->GetAuthGameMode())->despawnPlayer(this);
}
