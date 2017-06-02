// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "proceduralGeneration/hex/HexGenComponent.h"
#include "HexCircleGenComponent.generated.h"

/**
 * 
 */
UCLASS()
class SWORDSWING_API UHexCircleGenComponent : public UHexGenComponent
{
	GENERATED_BODY()

public:
	UHexCircleGenComponent();
	UHexCircleGenComponent(UInstancedStaticMeshComponent* _instance, float hex_radius, int circle_radius, FVector position);
	~UHexCircleGenComponent();

	void generate();

	static void CGenerate(UInstancedStaticMeshComponent* instance, float hex_radius, int circle_radius, FVector position);

	void animateR(float anim_speed, bool in2out, float target_height, bool instant_start);

	void finishAnimation();

	void paintHexes(const TArray<FVector2D> &coords);
	void animatePaintR(float anim_speed, bool in2out, int radius, bool instant_start);

	// Called every frame
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;


	UPROPERTY(VisibleAnywhere)
	int crad;

private:
	float a_speed;
	bool i2o;
	float t_height;
	float start_height;
	float wave_rad;

	float pa_speed;
	bool p_i2o;
	float p_wave_rad;
	int p_rad;
	
};
