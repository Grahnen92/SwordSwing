// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "proceduralGeneration/hex/HexGenComponent.h"
#include "HexGridGenComponent.generated.h"

/**
 * 
 */
UCLASS()
class SWORDSWING_API UHexGridGenComponent : public UHexGenComponent
{
	GENERATED_BODY()
	
public:
	UHexGridGenComponent();
	UHexGridGenComponent(UInstancedStaticMeshComponent* _instance, float hex_radius, int x_halfdim, int y_halfdim, FVector position);
	~UHexGridGenComponent();

	void generate();
	static void GGenerate(UInstancedStaticMeshComponent* instance, float hex_radius, int x_halfdim, int y_halfdim, FVector position);

	void animateY(float anim_speed, bool neg2pos, float target_height, bool instant_start);
	void finishAnimation();

	void paintHexes(const TArray<FVector2D> &coords);
	void animatePaintY(float anim_speed, bool neg2pos, int col, bool instant_start);

	// Called every frame
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;


	UPROPERTY(VisibleAnywhere)
	int xhd;
	UPROPERTY(VisibleAnywhere)
	int yhd;

private:
	
	float a_speed;
	bool n2p;
	float t_height;
	float start_height;
	float wave_point;
	

	float pa_speed;
	bool p_n2p;
	float p_wave_point;
	int p_col;

	
};
