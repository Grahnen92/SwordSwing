// Fill out your copyright notice in the Description page of Project Settings.

#include "SwordSwing.h"
#include "HexCircleGen.h"

HexCircleGen::HexCircleGen()
{
	instance = nullptr;
}
HexCircleGen::HexCircleGen(UInstancedStaticMeshComponent* _instance, float hex_radius, int circle_radius,  FVector position): HexGen( _instance, hex_radius, position), crad(circle_radius)
{
}
HexCircleGen::~HexCircleGen()
{
}
void HexCircleGen::generate()
{
	instance_offset = (int)instance->GetInstanceCount();
	CGenerate(instance, hrad, crad, pos);
}

void HexCircleGen::CGenerate(UInstancedStaticMeshComponent* instance, float hex_radius, int circle_radius, FVector position)
{
	int circle_diameter = circle_radius * 2 + 1;

	FTransform tmp_trans;
	tmp_trans.SetScale3D(FVector(hex_radius / 100.f, hex_radius / 100.f, 100));

	float hex_height = hex_radius*FMath::Cos(PI / 6.f);
	float offset;

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
			tmp_trans.SetTranslation(FVector(x, y, -10000) + position);

			instance->AddInstance(tmp_trans);
		}
	}
}
