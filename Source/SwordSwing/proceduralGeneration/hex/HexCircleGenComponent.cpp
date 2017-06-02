// Fill out your copyright notice in the Description page of Project Settings.

#include "SwordSwing.h"
#include "HexInstancer.h"

#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */

#include "HexCircleGenComponent.h"

int factorial(int n)
{
	return n == 0 || n == 1 ? 1 : n*factorial(n - 1);
}

UHexCircleGenComponent::UHexCircleGenComponent()
{
	instance = nullptr;
	PrimaryComponentTick.bCanEverTick = true;
}
UHexCircleGenComponent::UHexCircleGenComponent(UInstancedStaticMeshComponent* _instance, float hex_radius, int circle_radius, FVector position) : Super(_instance, hex_radius, position), crad(circle_radius)
{
}
UHexCircleGenComponent::~UHexCircleGenComponent()
{
}
void UHexCircleGenComponent::generate()
{
	instance_offset = (int)instance->GetInstanceCount();
	CGenerate(instance, hrad, crad, GetComponentLocation());
}

void UHexCircleGenComponent::CGenerate(UInstancedStaticMeshComponent* instance, float hex_radius, int circle_radius, FVector position)
{
	int circle_diameter = circle_radius * 2 + 1;

	FTransform tmp_trans;
	tmp_trans.SetScale3D(FVector(hex_radius / 100.f, hex_radius / 100.f, 100));

	float hex_height = hex_radius*FMath::Cos(PI / 6.f);
	float offset;

	srand(time(NULL));

	float min_rand = -10.f;
	float max_rand = 10.f;
	float r_height;
	

	for (int i = -circle_radius; i <= circle_radius; i++)
	{
		float x = i*1.5f*hex_radius;// +ox;// +w_pos.X;
		int j_diameter = circle_diameter - std::abs(i);

		if (i % 2)
			offset = 0.f;
		else
			offset = -hex_height;
		for (int j = -(j_diameter / 2); j < j_diameter - (j_diameter / 2); j++)
		{
			float y = j*2.f*hex_height + offset;// +oy;// + w_pos.Y;
			r_height = min_rand + static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / (max_rand - min_rand)));
			tmp_trans.SetTranslation(FVector(x, y, -10000 + r_height) + position);

			instance->AddInstance(tmp_trans);
		}
	}
}

void UHexCircleGenComponent::animateR(float anim_speed, bool in2out, float target_height, bool instant_start)
{
	animating = instant_start;
	a_speed = anim_speed;
	i2o = in2out;
	t_height = target_height;
	wave_rad = GetComponentLocation().Y;
	
	FTransform tmp_trans;
	instance->GetInstanceTransform(instance_offset, tmp_trans);
	start_height = tmp_trans.GetLocation().Z;
}

// Called every frame
void UHexCircleGenComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	if (animating)
	{
		float hex_height = hrad*FMath::Cos(PI / 6.f);
		FVector cpos = GetComponentLocation();

		FTransform tmp_trans;

		int curr_rad = (wave_rad - cpos.Y) / (2.f*hex_height);
		
		if (curr_rad > crad)
		{
			animating = false;
			finishAnimation();
		}

		int curr_circle_diameter = curr_rad * 2 + 1;
		int circle_diameter = crad * 2 + 1;

		int curr_outer_j_diameter = curr_circle_diameter - std::abs(curr_rad);
		int outer_j_diameter = circle_diameter - std::abs(crad);

		int first_half = (static_cast<float>(outer_j_diameter + (circle_diameter - 1)) / 2.f)*(crad);
		
		curr_rad = -curr_rad;
		int prev_j_diameter = circle_diameter - 1;
		int k;
		int curr_j_diameter = curr_circle_diameter - std::abs(curr_rad);
		int j_diameter = circle_diameter - std::abs(curr_rad);
		for (int j = (j_diameter - curr_outer_j_diameter)/2; j < circle_diameter - ((j_diameter - curr_outer_j_diameter) / 2); j++){
			k = instance_offset + (static_cast<float>(outer_j_diameter + prev_j_diameter) / 2.f)*(curr_rad + crad) + j;
			instance->GetInstanceTransform(k, tmp_trans);
			FVector trans = tmp_trans.GetTranslation();
			trans.Z = start_height + t_height;
			tmp_trans.SetTranslation(trans);
			instance->UpdateInstanceTransform(k, tmp_trans, false, true, false);
		}
		curr_rad = -curr_rad;
		prev_j_diameter = curr_rad - 1;
		for (int j = (j_diameter - curr_outer_j_diameter) / 2; j < circle_diameter - ((j_diameter - curr_outer_j_diameter) / 2); j++) {
			k = instance_offset + first_half + (static_cast<float>(circle_diameter + prev_j_diameter) / 2.f)*curr_rad + j;
			instance->GetInstanceTransform(k, tmp_trans);
			FVector trans = tmp_trans.GetTranslation();
			trans.Z = start_height + t_height;
			tmp_trans.SetTranslation(trans);
			instance->UpdateInstanceTransform(k, tmp_trans, false, true, false);
		}
		
		//all the rows inbetween
		for (int i = -(curr_rad - 1); i <= (curr_rad - 1); i++)
		{
			curr_j_diameter = curr_circle_diameter - std::abs(i);
			j_diameter = circle_diameter  -std::abs(i);
			prev_j_diameter = circle_diameter - std::abs(i - 1);
			
			//lowest hex
			int j = (j_diameter - curr_j_diameter) / 2;
			if (i < 1)
				k = instance_offset + (static_cast<float>(outer_j_diameter + prev_j_diameter) / 2.f)*(i + crad) + j;
			else
				k = instance_offset + first_half + (static_cast<float>(circle_diameter + prev_j_diameter) / 2.f)*i + j;
			instance->GetInstanceTransform(k, tmp_trans);
			FVector trans = tmp_trans.GetTranslation();
			trans.Z = start_height + t_height;
			tmp_trans.SetTranslation(trans);
			instance->UpdateInstanceTransform(k, tmp_trans, false, true, false);

			// highest hex
			j = circle_diameter - j - 1;
			if (i < 1)
				k = instance_offset + (static_cast<float>(outer_j_diameter + prev_j_diameter) / 2.f)*(i + crad) + j;
			else
				k = instance_offset + first_half + (static_cast<float>(circle_diameter + prev_j_diameter) / 2.f)*i + j;
			instance->GetInstanceTransform(k, tmp_trans);
			trans = tmp_trans.GetTranslation();
			trans.Z = start_height + t_height;
			tmp_trans.SetTranslation(trans);
			instance->UpdateInstanceTransform(k, tmp_trans, false, true, false);

		}

		
		wave_rad += DeltaTime*a_speed;
	}

	if (p_animating)
	{
		
		p_wave_rad += DeltaTime*pa_speed;
	}

}

void  UHexCircleGenComponent::finishAnimation()
{
	FTransform tmp_trans;

	int instance_index = instance_offset;

	instancer->popAnim();

}

void UHexCircleGenComponent::paintHexes(const TArray<FVector2D> &coords)
{
	float z_pos = -10000.f;
	float z_scale = 100;
	FQuat z_rot(FRotator(0.f, 60.f, 0.f));
	FTransform tmp_trans;

	FTransform rot_trans;
	rot_trans.SetScale3D(FVector(hrad / 100.f, hrad / 100.f, 100));
	rot_trans.SetRotation(z_rot);

	int circle_diameter = crad * 2 + 1;

	int outer_j_diameter = circle_diameter - std::abs(crad);

	int first_half = (static_cast<float>(outer_j_diameter + (circle_diameter - 1)) / 2.f)*(crad);


	for (const auto & co : coords) {
		int i = static_cast<int>(co.X);
		int j = static_cast<int>(co.Y);

		int prev_j_diameter = circle_diameter - std::abs(i-1);

		int k;
		if (i < 1)
			k = instance_offset + (static_cast<float>(outer_j_diameter + prev_j_diameter) / 2.f)*(i + crad) + j;
		else
			k = instance_offset + first_half+ (static_cast<float>(circle_diameter + prev_j_diameter) / 2.f)*i + j;

		instance->GetInstanceTransform(k, tmp_trans);
		rot_trans.SetTranslation(tmp_trans.GetTranslation());
		instance->UpdateInstanceTransform(k, rot_trans, false, true, false);

	}
}


void UHexCircleGenComponent::animatePaintR(float anim_speed, bool in2out, int rad, bool instant_start)
{
	p_animating = instant_start;
	pa_speed = anim_speed;
	p_i2o = in2out;
	rad = rad;

	FTransform tmp_trans;
}
