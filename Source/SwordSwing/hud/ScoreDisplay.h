// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "GameFramework/Actor.h"
#include "ScoreDisplay.generated.h"

UCLASS()
class SWORDSWING_API AScoreDisplay : public AActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	AScoreDisplay();

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

	UPROPERTY(Category = "Icon", VisibleAnywhere)
	UStaticMeshComponent* icon;

	UPROPERTY(Category = "Score", VisibleAnywhere)
	UTextRenderComponent* score;

public:	
	// Called every frame
	virtual void Tick(float DeltaTime) override;

	void setScoreText(FString score_string);

	void setIconMaterial(UMaterial* icon_mat);
	
};
