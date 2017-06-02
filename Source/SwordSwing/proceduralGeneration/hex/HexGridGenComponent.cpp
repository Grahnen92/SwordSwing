// Fill out your copyright notice in the Description page of Project Settings.

#include "SwordSwing.h"
#include "HexInstancer.h"

#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */

#include "HexGridGenComponent.h"




UHexGridGenComponent::UHexGridGenComponent() : Super()
{
	instance = nullptr;
	PrimaryComponentTick.bCanEverTick = true;
}
UHexGridGenComponent::UHexGridGenComponent(UInstancedStaticMeshComponent* _instance, float hex_radius, int x_halfdim, int y_halfdim, FVector position) : Super(_instance, hex_radius, position), xhd(x_halfdim), yhd(y_halfdim)
{
}
UHexGridGenComponent::~UHexGridGenComponent()
{
}
void UHexGridGenComponent::generate()
{
	instance_offset = (int)instance->GetInstanceCount();
	GGenerate(instance, hrad, xhd, yhd, GetComponentLocation());
}

void UHexGridGenComponent::GGenerate(UInstancedStaticMeshComponent* instance, float hex_radius, int x_halfdim, int y_halfdim, FVector position)
{
	float z_pos = -10000.f;
	float z_scale = 100;
	FQuat z_rot(FRotator(0.f, 0.f, 0.f));

	float min_rand = -10.f;
	float max_rand = 10.f;
	float r_height;

	FTransform tmp_trans;
	tmp_trans.SetScale3D(FVector(hex_radius / 100.f, hex_radius / 100.f, 100));

	float hex_height = hex_radius*FMath::Cos(PI / 6.f);
	for (int i = -x_halfdim; i <= x_halfdim; i++) {

		float x = i*1.5f*hex_radius;
		if (i % 2) {
			for (int j = -y_halfdim; j < y_halfdim; j++) {
				float y = j*2.f*hex_height;

				r_height = min_rand + static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / (max_rand - min_rand)));
				tmp_trans.SetTranslation(FVector(x, y, z_pos + r_height) + position);
				instance->AddInstance(tmp_trans);
			}
		}
		else {
			for (int j = -y_halfdim; j <= y_halfdim; j++) {
				float y = j*2.f*hex_height - hex_height;
				tmp_trans.SetTranslation(FVector(x, y, z_pos + r_height) + position);
				instance->AddInstance(tmp_trans);
			}
		}
	}
}

void UHexGridGenComponent::animateY(float anim_speed, bool neg2pos, float target_height, bool instant_start)
{
	animating = instant_start;
	a_speed = anim_speed;
	n2p = neg2pos;
	t_height = target_height;
	FTransform tmp_trans;
	
	if (neg2pos) {
		instance->GetInstanceTransform(instance_offset, tmp_trans);
		wave_point = tmp_trans.GetTranslation().Y -FMath::Sqrt(target_height);
	}
	else
	{
		if(xhd % 2)
			instance->GetInstanceTransform(instance_offset+ yhd*2 -1, tmp_trans);
		else
			instance->GetInstanceTransform(instance_offset + yhd * 2, tmp_trans);

		wave_point = tmp_trans.GetTranslation().Y + FMath::Sqrt(target_height);
	}

	start_height = tmp_trans.GetTranslation().Z;
	
}

// Called every frame
void UHexGridGenComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	if (animating)
	{
		float hex_height = hrad*FMath::Cos(PI / 6.f);
		FVector cpos = GetComponentLocation();
		
		FTransform tmp_trans;
		
		int c1_length;
		int c2_length;

		if (xhd % 2) {
			instance->GetInstanceTransform(instance_offset + yhd * 2 - 1, tmp_trans);
			c2_length = yhd * 2 + 1;
			c1_length = yhd * 2;

		}
		else {
			instance->GetInstanceTransform(instance_offset + yhd * 2, tmp_trans);
			c1_length = yhd * 2 + 1;
			c2_length = yhd * 2;
		}
		
		//if (FMath::Abs(tmp_trans.GetLocation().Z - start_height) >= t_height){
		if (tmp_trans.GetLocation().Y  < wave_point) {
			animating = false;
			finishAnimation();
		}

		int instance_index = instance_offset;

		for (int i = -xhd; i <= xhd; i++) {
			int ii = i + xhd;
			if (i % 2)
			{
				//int i = (wave_point - cpos.X) / (1.5f*hrad);
				int j = (wave_point - cpos.Y) / (2.f*hex_height);
				int jj = j + yhd;

				int k = instance_offset + (((ii + 1) / 2)*c1_length + ((ii) / 2)*c2_length) + jj;
				instance->GetInstanceTransform(k, tmp_trans);
				FVector trans = tmp_trans.GetTranslation();
				trans.Z = start_height + t_height;
				tmp_trans.SetTranslation(trans);
				instance->UpdateInstanceTransform(k, tmp_trans, false, true, false);
			}

			else
			{
			//	int i = (wave_point - cpos.X) / (1.5f*hrad);
				int j = (wave_point - cpos.Y + hex_height) / (2.f*hex_height);
				int jj = j + yhd;
				
				int k = instance_offset + (((ii + 1) / 2)*c1_length + ((ii) / 2)*c2_length) + jj;
				instance->GetInstanceTransform(k, tmp_trans);
				FVector trans = tmp_trans.GetTranslation();
				trans.Z = start_height + t_height;
				tmp_trans.SetTranslation(trans);
				instance->UpdateInstanceTransform(k, tmp_trans, false, true, false);
			}
		}		

		wave_point += DeltaTime*a_speed;
	}
	
	if (p_animating)
	{
		float hex_height = hrad*FMath::Cos(PI / 6.f);
		FVector cpos = GetComponentLocation();

		FTransform tmp_trans;
		FQuat z_rot(FRotator(0.f, 60.f, 0.f));

		int c1_length;
		int c2_length;

		if (xhd % 2) {
			instance->GetInstanceTransform(instance_offset + yhd * 2 - 1, tmp_trans);
			c2_length = yhd * 2 + 1;
			c1_length = yhd * 2;

		}
		else {
			instance->GetInstanceTransform(instance_offset + yhd * 2, tmp_trans);
			c1_length = yhd * 2 + 1;
			c2_length = yhd * 2;
		}

		//if (FMath::Abs(tmp_trans.GetLocation().Z - start_height) >= t_height){
		if (tmp_trans.GetLocation().Y  < p_wave_point) {
			p_animating = false;
			//finishAnimation();
		}

		int instance_index = instance_offset;

		int ii = p_col + xhd;
		if (p_col % 2)
		{
			//int i = (wave_point - cpos.X) / (1.5f*hrad);
			int j = (p_wave_point - cpos.Y) / (2.f*hex_height);
			int jj = j + yhd;
			float fj = (p_wave_point - cpos.Y) / (2.f*hex_height);
			UE_LOG(LogTemp, Warning, TEXT("wavepoint: %f"), p_wave_point);
			UE_LOG(LogTemp, Warning, TEXT("index: %d"), j);
			UE_LOG(LogTemp, Warning, TEXT("index: %f"), fj);

			int k = instance_offset + (((ii + 1) / 2)*c1_length + ((ii) / 2)*c2_length) + jj;
			instance->GetInstanceTransform(k, tmp_trans);
			tmp_trans.SetRotation(z_rot);
			instance->UpdateInstanceTransform(k, tmp_trans, false, true, false);
		}

		else
		{
			//	int i = (wave_point - cpos.X) / (1.5f*hrad);
			int j = (p_wave_point - cpos.Y + hex_height) / (2.f*hex_height);
			int jj = j + yhd;
			float fj = (p_wave_point - cpos.Y + hex_height) / (2.f*hex_height);
			UE_LOG(LogTemp, Warning, TEXT("wavepoint: %f"), p_wave_point);
			UE_LOG(LogTemp, Warning, TEXT("index: %d"), j);
			UE_LOG(LogTemp, Warning, TEXT("index: %f"), fj);

			int k = instance_offset + (((ii + 1) / 2)*c1_length + ((ii) / 2)*c2_length) + jj;
			instance->GetInstanceTransform(k, tmp_trans);
			tmp_trans.SetRotation(z_rot);
			instance->UpdateInstanceTransform(k, tmp_trans, false, true, false);
		}
		

		p_wave_point += DeltaTime*pa_speed;
	}

}

void  UHexGridGenComponent::finishAnimation()
{
	FTransform tmp_trans;

	int instance_index = instance_offset;

	instancer->popAnim();

	/*for (int i = -xhd; i <= xhd; i++) {
		if (i % 2) {
			for (int j = -yhd; j < yhd; j++) {
				instance->GetInstanceTransform(instance_index, tmp_trans);
				FVector trans = tmp_trans.GetTranslation();
				trans.Z = start_height + t_height;
				tmp_trans.SetTranslation(trans);
				instance->UpdateInstanceTransform(instance_index, tmp_trans, false, true, false);
				instance_index++;
			}
		}
		else {
			for (int j = -yhd; j <= yhd; j++) {
				instance->GetInstanceTransform(instance_index, tmp_trans);
				FVector trans = tmp_trans.GetTranslation();
				trans.Z = start_height + t_height;
				tmp_trans.SetTranslation(trans);
				instance->UpdateInstanceTransform(instance_index, tmp_trans, false, true, false);
				instance_index++;
			}
		}
	}*/
}

void UHexGridGenComponent::paintHexes(const TArray<FVector2D> &coords)
{
	float z_pos = -10000.f;
	float z_scale = 100;
	FQuat z_rot(FRotator(0.f, 60.f, 0.f));
	FTransform tmp_trans;

	FTransform rot_trans;
	rot_trans.SetScale3D(FVector(hrad / 100.f, hrad / 100.f, 100));
	rot_trans.SetRotation(z_rot);

	int c1_length;
	int c2_length;

	if (xhd % 2){
		c2_length = yhd * 2 + 1;
		c1_length = yhd * 2;
	} else {
		c1_length = yhd * 2 + 1;
		c2_length = yhd * 2;
	}

	for (const auto & co : coords){
		int i = static_cast<int>(co.X);
		int j = static_cast<int>(co.Y);

		int k = instance_offset + ( ((i + 1) / 2)*c1_length + ((i) / 2)*c2_length ) + j;

		instance->GetInstanceTransform(k, tmp_trans);
		rot_trans.SetTranslation(tmp_trans.GetTranslation());
		instance->UpdateInstanceTransform(k, rot_trans, false, true, false);
	}
}


void UHexGridGenComponent::animatePaintY(float anim_speed, bool neg2pos, int col, bool instant_start)
{
	p_animating = instant_start;
	pa_speed = anim_speed;
	p_n2p = neg2pos;
	p_col = col;

	FTransform tmp_trans;

	if (neg2pos) {
		instance->GetInstanceTransform(instance_offset, tmp_trans);
		p_wave_point = tmp_trans.GetTranslation().Y;
	}
	else
	{
		if (xhd % 2)
			instance->GetInstanceTransform(instance_offset + yhd * 2 - 1, tmp_trans);
		else
			instance->GetInstanceTransform(instance_offset + yhd * 2, tmp_trans);

		p_wave_point = tmp_trans.GetTranslation().Y;
	}
}

