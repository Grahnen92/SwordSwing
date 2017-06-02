// Fill out your copyright notice in the Description page of Project Settings.

#include "SwordSwing.h"
#include <stdlib.h> 
#include "LandscapeGenerator.h"


// Sets default values
ALandscapeGenerator::ALandscapeGenerator()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

	root = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));
	//RootComponent = root;

	landscape =  CreateDefaultSubobject<UProceduralMeshComponent>(TEXT("Landscape"));

	landscape_instance = CreateDefaultSubobject<UInstancedStaticMeshComponent>(TEXT("InstancedLandscape"));
	RootComponent = landscape_instance;
	static ConstructorHelpers::FObjectFinder<UMaterialInterface> land_mat(TEXT("/Game/materials/ProceduralMaterial.ProceduralMaterial"));
	landscape_material = land_mat.Object;

	
	static ConstructorHelpers::FObjectFinder<UTexture2D> height(TEXT("/Game/DerpThruster/Max2.Max2"));
	height_map = &height.Object->PlatformData->Mips[0];
}

// Called when the game starts or when spawned
void ALandscapeGenerator::BeginPlay()
{
	Super::BeginPlay();

	//createHexGrid(FVector2D(50.f, 50.f), 100.f);
	//createInstancedHexGrid(FVector2D(5, 1), 100.f);

	float hex_radius = 100.f;
	float hex_height = hex_radius*FMath::Cos(PI / 6.f);
	createInstancedHexCircle(6, hex_radius, FVector::ZeroVector);

	createInstancedHexGrid(9, 2, hex_radius, FVector(14 * 1.5f*hex_radius, 0.f, 0.f));

	//i*1.5f*hex_radius
	//j*2.f*hex_height
	createInstancedHexCircle(6, hex_radius, FVector(30*1.5f*hex_radius, 0.f, 0.f));
	
	/*verts.SetNumUninitialized(31);
	createHexBox(100.f, FVector(0.f, 0.f, 100.f), -100.f);*/
}

// Called every frame
void ALandscapeGenerator::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
	FTransform tmp_trans;
	landscape_instance->GetInstanceTransform(50, tmp_trans);
	FVector trans = tmp_trans.GetTranslation();
	trans.Z = trans.Z + DeltaTime*50.f;
	tmp_trans.SetTranslation(trans);
	landscape_instance->UpdateInstanceTransform(50, tmp_trans, false, true, false);
}

void  ALandscapeGenerator::createInstancedHexGrid(int hx, int hy, float hex_radius, FVector grid_pos = FVector(0.f, 0.f, 0.f))
{
	landscape_instance->SetMaterial(0, landscape_material);
	
	FByteBulkData* RawImageData = &height_map->BulkData;
	FColor* FormatedImageData = static_cast<FColor*>(RawImageData->Lock(LOCK_READ_ONLY));
	uint8 PixelX = 5, PixelY = 10;
	uint32 TextureWidth = height_map->SizeX, TextureHeight = height_map->SizeY;
	FColor PixelColor;

	FTransform tmp_trans;
	tmp_trans.SetScale3D(FVector(hex_radius / 100.f, hex_radius / 100.f, 100));

	float hex_height = hex_radius*FMath::Cos(PI / 6.f);
	for (int i = -hx; i <= hx; i++){
		float x = i*1.5f*hex_radius;
		if (i % 2){
			for (int j = -hy; j < hy; j++) {
				float y = j*2.f*hex_height;
				tmp_trans.SetTranslation(FVector(x, y, -10000.f) + grid_pos);
				landscape_instance->AddInstance(tmp_trans);
			}
		}else{
			for (int j = -hy; j <= hy; j++) {
				float y = j*2.f*hex_height - hex_height;
				tmp_trans.SetTranslation(FVector(x, y, -10000.f) + grid_pos);
				landscape_instance->AddInstance(tmp_trans);
			}
		}
			

	}
	RawImageData->Unlock();
}

void  ALandscapeGenerator::createInstancedHexCircle(int circle_radius, float hex_radius, FVector circle_pos = FVector(0.f, 0.f, 0.f))
{
	//if (circle_radius % 2 == 0)
	//	circle_radius++;

	int circle_diameter = circle_radius * 2 + 1;

	landscape_instance->SetMaterial(0, landscape_material);

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
		for (int j = -(j_diameter / 2); j < j_diameter  -(j_diameter / 2); j++)
		{
			float y = j*2.f*hex_height + offset;// +oy;// + w_pos.Y;
			tmp_trans.SetTranslation(FVector(x, y, -10000) + circle_pos);

			landscape_instance->AddInstance(tmp_trans);
		}
	}
}

void ALandscapeGenerator::createHexGrid(FVector2D res, float hex_radius)
{
	verts.SetNumUninitialized(31);

	FByteBulkData* RawImageData = &height_map->BulkData;
	FColor* FormatedImageData = static_cast<FColor*>(RawImageData->Lock(LOCK_READ_ONLY));
	uint8 PixelX = 5, PixelY = 10;
	uint32 TextureWidth = height_map->SizeX, TextureHeight = height_map->SizeY;
	FColor PixelColor;

	float hex_height = hex_radius*FMath::Cos(PI / 6.f);
	float offset;
	for (int i = -res.X/2; i <= res.X/2; i++)
	{
		float x = i*1.5f*hex_radius;// +ox;// +w_pos.X;
		
		if (i % 2)
			offset = 0.f;
		else
			offset = -hex_height;
		for (int j = -res.Y / 2; j <= res.Y/2; j++)
		{
			float y = j*2.f*hex_height + offset;// +oy;// + w_pos.Y;
			createHexBox(hex_radius, FVector(x, y, FMath::Sin(i/res.X)*500.f), -2000.f);
		}
	}

	RawImageData->Unlock();
}

void ALandscapeGenerator::createHexBox(float radius, FVector pos = FVector(0.f, 0.f, 0.f), float min_z = 0.f)
{
	float x = pos.X, y = pos.Y, z = pos.Z;
	
	//top
	verts[0] = FVector(-radius*0.5f + x,	radius*c3 + y,	 z);	
	verts[1] = FVector(radius*0.5f + x,		radius*c3 + y,	 z);	
	verts[2] = FVector(radius + x,			0.0f + y,		 z);
	verts[3] = FVector(radius*0.5f + x,		-radius*c3 + y,	 z);	
	verts[4] = FVector(-radius*0.5f + x,	-radius*c3 + y,	 z);	
	verts[5] = FVector(-radius + x,		0.0f + y,			 z);
	verts[6] = FVector(0.0f + x,			0.0f + y,		 z);

	//sides+								  
	verts[7] = FVector(-radius*0.5f + x,	radius*c3 + y,	 z);	
	verts[8] = FVector(radius*0.5f + x,		radius*c3 + y,	 z);	
	verts[9] = FVector(radius + x,			y,	0.0f +		 z);
	verts[10] = FVector(radius*0.5f + x,	-radius*c3 + y,	 z);	
	verts[11] = FVector(-radius*0.5f + x,	-radius*c3 +y,	 z);	
	verts[12] = FVector(-radius + x,		0.0f + y,		 z);
	//sides-
	verts[13] = FVector(-radius*0.5f + x,	radius*c3 + y,	 min_z);	
	verts[14] = FVector(radius*0.5f + x,	radius*c3 + y,	 min_z);	
	verts[15] = FVector(radius + x,			0.0f +y,		 min_z);
	verts[16] = FVector(radius*0.5f + x,	-radius*c3 + y,	 min_z);	
	verts[17] = FVector(-radius*0.5f + x,	-radius*c3 + y,  min_z);	
	verts[18] = FVector(-radius + x,		0.0f +	y,		 min_z);
	
	//sides+
	verts[19] = FVector(-radius*0.5f + x,	radius*c3 + y,	 z);			
	verts[20] = FVector(radius*0.5f + x,	radius*c3 + y,	 z);			
	verts[21] = FVector(radius + x,			0.0f + y,		 z);		
	verts[22] = FVector(radius*0.5f + x,	-radius*c3 + y,	 z);			
	verts[23] = FVector(-radius*0.5f + x,	-radius*c3 + y,	 z);			
	verts[24] = FVector(-radius + x,		0.0f + y,		 z);	
	//sides
	verts[25] = FVector(-radius*0.5f + x,	radius*c3 + y,   min_z);		
	verts[26] = FVector(radius*0.5f + x,	radius*c3 + y,	 min_z);		
	verts[27] = FVector(radius + x,			0.0f + y,		 min_z);	
	verts[28] = FVector(radius*0.5f + x,	-radius*c3 + y,	 min_z);		
	verts[29] = FVector(-radius*0.5f + x,	-radius*c3 + y,  min_z);		
	verts[30] = FVector(-radius + x,		0.0f + y,		 min_z);	

	TArray<FVector> tmp_convex;
	tmp_convex.Add(verts[0]);
	tmp_convex.Add(verts[1]);
	tmp_convex.Add(verts[2]);
	tmp_convex.Add(verts[3]);
	tmp_convex.Add(verts[4]);
	tmp_convex.Add(verts[5]);

	tmp_convex.Add(verts[13]);
	tmp_convex.Add(verts[14]);
	tmp_convex.Add(verts[15]);
	tmp_convex.Add(verts[16]);
	tmp_convex.Add(verts[17]);
	tmp_convex.Add(verts[18]);
	TArray<TArray<FVector>> tmp_convexs;
	tmp_convexs.Add(tmp_convex);

	landscape->CreateMeshSection_LinearColor(landscape->GetNumSections(), verts, tris, normals, uvs, colors, tangents, false);
	landscape->bUseComplexAsSimpleCollision = false;
	landscape->SetCollisionResponseToAllChannels(ECollisionResponse::ECR_Block);
	landscape->SetMaterial(landscape->GetNumSections() -1 , landscape_material);
	//landscape->GetProcMeshSection(landscape->GetNumSections() - 1)->bEnableCollision = true;
}


const float ALandscapeGenerator::c3 = 0.8660254f;
const float ALandscapeGenerator::c3m1d2 = 0.0669872f;

const TArray<FVector> ALandscapeGenerator::normals = {
	FVector(0.0f, 0.0f, 1.0f),
	FVector(0.0f, 0.0f, 1.0f),
	FVector(0.0f, 0.0f, 1.0f),
	FVector(0.0f, 0.0f, 1.0f),
	FVector(0.0f, 0.0f, 1.0f),
	FVector(0.0f, 0.0f, 1.0f),
	FVector(0.0f, 0.0f, 1.0f),
	
	FVector(0.0f, 1.0f, 0.0f),
	FVector(ALandscapeGenerator::c3, 0.5f, 0.0f),
	FVector(ALandscapeGenerator::c3, -0.5f, 0.0f),
	FVector(0.0f, -1.0f, 0.0f),
	FVector(-ALandscapeGenerator::c3, -0.5f, 0.0f),
	FVector(-ALandscapeGenerator::c3, 0.5f, 0.0f),
	FVector(0.0f, 1.0f, 0.0f),
	FVector(ALandscapeGenerator::c3, 0.5f, 0.0f),
	FVector(ALandscapeGenerator::c3, -0.5f, 0.0f),
	FVector(0.0f, -1.0f, 0.0f),
	FVector(-ALandscapeGenerator::c3, -0.5f, 0.0f),
	FVector(-ALandscapeGenerator::c3, 0.5f, 0.0f),
	
	FVector(-ALandscapeGenerator::c3, 0.5f, 0.0f),
	FVector(0.0f, 1.0f, 0.0f),
	FVector(ALandscapeGenerator::c3, 0.5f, 0.0f),
	FVector(ALandscapeGenerator::c3, -0.5f, 0.0f),
	FVector(0.0f, -1.0f, 0.0f),
	FVector(-ALandscapeGenerator::c3, -0.5f, 0.0f),
	FVector(-ALandscapeGenerator::c3, 0.5f, 0.0f),
	FVector(0.0f, 1.0f, 0.0f),
	FVector(ALandscapeGenerator::c3, 0.5f, 0.0f),
	FVector(ALandscapeGenerator::c3, -0.5f, 0.0f),
	FVector(0.0f, -1.0f, 0.0f),
	FVector(-ALandscapeGenerator::c3, -0.5f, 0.0f),
};

const TArray<FProcMeshTangent> ALandscapeGenerator::tangents = {
	
	FProcMeshTangent(0.707, 0.707, 0.0f),
	FProcMeshTangent(0.707, 0.707, 0.0f),		
	FProcMeshTangent(0.707, 0.707, 0.0f),		
	FProcMeshTangent(0.707, 0.707, 0.0f),		
	FProcMeshTangent(0.707, 0.707, 0.0f),		
	FProcMeshTangent(0.707, 0.707, 0.0f),		
	FProcMeshTangent(0.707, 0.707, 0.0f),		

	FProcMeshTangent(0.0f, 0.0f, 1.0f),		
	FProcMeshTangent(0.0f, 0.0f, 1.0f),		
	FProcMeshTangent(0.0f, 0.0f, 1.0f),		
	FProcMeshTangent(0.0f, 0.0f, 1.0f),		
	FProcMeshTangent(0.0f, 0.0f, 1.0f),		
	FProcMeshTangent(0.0f, 0.0f, 1.0f),		
	FProcMeshTangent(0.0f, 0.0f, 1.0f),		
	FProcMeshTangent(0.0f, 0.0f, 1.0f),		
	FProcMeshTangent(0.0f, 0.0f, 1.0f),		
	FProcMeshTangent(0.0f, 0.0f, 1.0f),		
	FProcMeshTangent(0.0f, 0.0f, 1.0f),		
	FProcMeshTangent(0.0f, 0.0f, 1.0f),		
									 
	FProcMeshTangent(0.0f, 0.0f, 1.0f),		
	FProcMeshTangent(0.0f, 0.0f, 1.0f),		
	FProcMeshTangent(0.0f, 0.0f, 1.0f),		
	FProcMeshTangent(0.0f, 0.0f, 1.0f),		
	FProcMeshTangent(0.0f, 0.0f, 1.0f),		
	FProcMeshTangent(0.0f, 0.0f, 1.0f),		
	FProcMeshTangent(0.0f, 0.0f, 1.0f),		
	FProcMeshTangent(0.0f, 0.0f, 1.0f),		
	FProcMeshTangent(0.0f, 0.0f, 1.0f),		
	FProcMeshTangent(0.0f, 0.0f, 1.0f),		
	FProcMeshTangent(0.0f, 0.0f, 1.0f),		
	FProcMeshTangent(0.0f, 0.0f, 1.0f),		
};

const TArray<FVector2D> ALandscapeGenerator::uvs = {
		
	FVector2D(0.25f, 1 - c3m1d2),
	FVector2D(0.75f, 1 - c3m1d2),
	FVector2D(1, 0.5f),
	FVector2D(0.75f, c3m1d2),
	FVector2D(0.25f, c3m1d2),
	FVector2D(0.f, 0.5f),
	FVector2D(0.5f, 0.5f),

	FVector2D(0.0f, 1.0f),
	FVector2D(0.1f, 1.0f),
	FVector2D(0.0f, 1.0f),
	FVector2D(0.1f, 1.0f),
	FVector2D(0.0f, 1.0f),
	FVector2D(0.1f, 1.0f),	
	FVector2D(0.0f, 0.9f),
	FVector2D(0.1f, 0.9f),
	FVector2D(0.0f, 0.9f),
	FVector2D(0.1f, 0.9f),
	FVector2D(0.0f, 0.9f),
	FVector2D(0.1f, 0.9f),

	FVector2D(0.0f, 1.0f),
	FVector2D(0.1f, 1.0f),
	FVector2D(0.0f, 1.0f),
	FVector2D(0.1f, 1.0f),
	FVector2D(0.0f, 1.0f),
	FVector2D(0.1f, 1.0f),

	FVector2D(0.0f, 0.9f),
	FVector2D(0.1f, 0.9f),
	FVector2D(0.0f, 0.9f),
	FVector2D(0.1f, 0.9f),
	FVector2D(0.0f, 0.9f),
	FVector2D(0.1f, 0.9f),


	};

const TArray<FLinearColor> ALandscapeGenerator::colors = {
	FLinearColor(1.0f, 1.0f, 1.0f),
	FLinearColor(1.0f, 1.0f, 1.0f),
	FLinearColor(1.0f, 1.0f, 1.0f),
	FLinearColor(1.0f, 1.0f, 1.0f),
	FLinearColor(1.0f, 1.0f, 1.0f),
	FLinearColor(1.0f, 1.0f, 1.0f),
	FLinearColor(1.0f, 1.0f, 1.0f),
	FLinearColor(1.0f, 1.0f, 1.0f),
	FLinearColor(1.0f, 1.0f, 1.0f),
	FLinearColor(1.0f, 1.0f, 1.0f),
	FLinearColor(1.0f, 1.0f, 1.0f),
	FLinearColor(1.0f, 1.0f, 1.0f),
	FLinearColor(1.0f, 1.0f, 1.0f),
	FLinearColor(1.0f, 1.0f, 1.0f),
	FLinearColor(1.0f, 1.0f, 1.0f),
	FLinearColor(1.0f, 1.0f, 1.0f),
	FLinearColor(1.0f, 1.0f, 1.0f),
	FLinearColor(1.0f, 1.0f, 1.0f),
	FLinearColor(1.0f, 1.0f, 1.0f),
	FLinearColor(1.0f, 1.0f, 1.0f),
	FLinearColor(1.0f, 1.0f, 1.0f),
	FLinearColor(1.0f, 1.0f, 1.0f),
	FLinearColor(1.0f, 1.0f, 1.0f),
	FLinearColor(1.0f, 1.0f, 1.0f),
	FLinearColor(1.0f, 1.0f, 1.0f),
	FLinearColor(1.0f, 1.0f, 1.0f),
	FLinearColor(1.0f, 1.0f, 1.0f),
	FLinearColor(1.0f, 1.0f, 1.0f),
	FLinearColor(1.0f, 1.0f, 1.0f),
	FLinearColor(1.0f, 1.0f, 1.0f),
	FLinearColor(1.0f, 1.0f, 1.0f),
	FLinearColor(1.0f, 1.0f, 1.0f),
};

const TArray<int32> ALandscapeGenerator::tris = {
	//top
	0, 1, 6,
	1, 2, 6,
	2, 3, 6,
	3, 4, 6,
	4, 5, 6,
	5, 0, 6,
	//sides -7
	17 + 12, 11 + 12, 16,
	11 + 12, 10, 16,
	18 + 12, 12 + 12, 17,
	12 + 12, 11, 17,
	13 + 12, 7 + 12, 18, // 20, 25, 14 == 25, 14, 20, == 14, 20, 25
	7 + 12, 12, 18,
	14 + 12, 8 + 12, 13,
	8 + 12, 7, 13, // 1, 3, 0
	15 + 12, 9 + 12, 14,
	9 + 12, 8, 14,
	16 + 12, 10 + 12, 15,
	10 + 12, 9, 15,

};