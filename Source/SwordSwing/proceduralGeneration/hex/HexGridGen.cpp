// Fill out your copyright notice in the Description page of Project Settings.

#include "SwordSwing.h"
#include "HexGridGen.h"


HexGridGen::HexGridGen()
{
	instance = nullptr;
}
HexGridGen::HexGridGen(UInstancedStaticMeshComponent* _instance, float hex_radius, int x_halfdim, int y_halfdim, FVector position) : HexGen(_instance, hex_radius, position), xhd(x_halfdim), yhd(y_halfdim)
{
}
HexGridGen::~HexGridGen()
{
}
void HexGridGen::generate()
{
	instance_offset = (int)instance->GetInstanceCount();
	GGenerate(instance, hrad, xhd, yhd, pos);
}

void HexGridGen::GGenerate(UInstancedStaticMeshComponent* instance, float hex_radius, int x_halfdim, int y_halfdim, FVector position)
{
	FTransform tmp_trans;
	tmp_trans.SetScale3D(FVector(hex_radius / 100.f, hex_radius / 100.f, 100));

	float hex_height = hex_radius*FMath::Cos(PI / 6.f);
	for (int i = -x_halfdim; i <= x_halfdim; i++) {
		float x = i*1.5f*hex_radius;
		if (i % 2) {
			for (int j = -y_halfdim; j < y_halfdim; j++) {
				float y = j*2.f*hex_height;
				tmp_trans.SetTranslation(FVector(x, y, -10000.f) + position);
				instance->AddInstance(tmp_trans);
			}
		}
		else {
			for (int j = -y_halfdim; j <= y_halfdim; j++) {
				float y = j*2.f*hex_height - hex_height;
				tmp_trans.SetTranslation(FVector(x, y, -10000.f) + position);
				instance->AddInstance(tmp_trans);
			}
		}
	}
}

void HexGridGen::animateX(float anim_speed, bool neg2pos, float target_height, float DeltaTime)
{
	FTransform tmp_trans;

	int instance_index = instance_offset;

	for (int i = -xhd; i <= xhd; i++) {
		if (i % 2) {
			for (int j = -yhd; j < yhd; j++) {
				instance->GetInstanceTransform(instance_index, tmp_trans);
				FVector trans = tmp_trans.GetTranslation();
				trans.Z = trans.Z + DeltaTime*50.f;
				tmp_trans.SetTranslation(trans);
				instance->UpdateInstanceTransform(instance_index, tmp_trans, false, true, false);
				instance_index++;
			}
		}
		else {
			for (int j = -yhd; j <= yhd; j++) {
				instance->GetInstanceTransform(instance_index, tmp_trans);
				FVector trans = tmp_trans.GetTranslation();
				trans.Z = trans.Z + DeltaTime*50.f;
				tmp_trans.SetTranslation(trans);
				instance->UpdateInstanceTransform(instance_index, tmp_trans, false, true, false);
				instance_index++;
			}
		}
	}
}

