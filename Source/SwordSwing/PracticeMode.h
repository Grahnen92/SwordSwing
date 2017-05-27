// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "FightModeBase.h"
#include "PracticeMode.generated.h"

/**
 * 
 */
UCLASS()
class SWORDSWING_API APracticeMode : public AFightModeBase
{

	GENERATED_BODY()
	
public:
	APracticeMode(const FObjectInitializer& ObjectInitializer);

	virtual void InitGame(const FString &MapName, const FString &Options, FString &ErrorMessage) override;

	AActor* ChoosePlayerStart_Implementation(AController* Player) override;
	
protected:
	virtual void BeginPlay() override;


private:
	
};
