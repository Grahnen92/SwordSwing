// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "GameFramework/PlayerController.h"
#include "ThirdPersonController.generated.h"

/**
 * 
 */
UCLASS()
class SWORDSWING_API AThirdPersonController : public APlayerController
{
	GENERATED_BODY()

public:
	// Called every frame
	//virtual void Tick(float DeltaTime) override;
	AThirdPersonController();
	AThirdPersonController(const class FObjectInitializer& ObjectInitializer);

	void pauseGame();
	
	// Reference UMG Asset in the Editor
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Widgets")
	TSubclassOf<class UUserWidget> pauseWidgetTemplate;

	UUserWidget* PauseMenu;

	void spawnPlayer();
	void despawnPlayer();

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;
	
	
};
