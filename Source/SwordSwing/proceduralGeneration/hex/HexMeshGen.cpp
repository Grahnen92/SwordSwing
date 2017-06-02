// Fill out your copyright notice in the Description page of Project Settings.

#include "SwordSwing.h"
#include "HexMeshGen.h"


// Sets default values
AHexMeshGen::AHexMeshGen()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

	root = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));

	hexscape = CreateDefaultSubobject<UProceduralMeshComponent>(TEXT("Hexscape"));
	static ConstructorHelpers::FObjectFinder<UMaterialInterface> land_mat(TEXT("/Game/JointCharacter/ProceduralMaterial.ProceduralMaterial"));
	hexscape_material = land_mat.Object;

	static ConstructorHelpers::FObjectFinder<UTexture2D> height(TEXT("/Game/DerpThruster/Max2.Max2"));
	height_map = &height.Object->PlatformData->Mips[0];
}

// Called when the game starts or when spawned
void AHexMeshGen::BeginPlay()
{
	Super::BeginPlay();
}

// Called every frame
void AHexMeshGen::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
}


void AHexMeshGen::createHexGrid(FVector2D res, float hex_radius)
{
	verts.SetNumUninitialized(31);

	FByteBulkData* RawImageData = &height_map->BulkData;
	FColor* FormatedImageData = static_cast<FColor*>(RawImageData->Lock(LOCK_READ_ONLY));
	uint8 PixelX = 5, PixelY = 10;
	uint32 TextureWidth = height_map->SizeX, TextureHeight = height_map->SizeY;
	FColor PixelColor;

	float hex_height = hex_radius*FMath::Cos(PI / 6.f);
	float offset;
	for (int i = -res.X / 2; i <= res.X / 2; i++)
	{
		float x = i*1.5f*hex_radius;// +ox;// +w_pos.X;

		if (i % 2)
			offset = 0.f;
		else
			offset = -hex_height;
		for (int j = -res.Y / 2; j <= res.Y / 2; j++)
		{
			float y = j*2.f*hex_height + offset;// +oy;// + w_pos.Y;
			createHexBox(hex_radius, FVector(x, y, FMath::Sin(i / res.X)*500.f), -2000.f);
		}
	}

	RawImageData->Unlock();
}

void AHexMeshGen::createHexBox(float radius, FVector pos = FVector(0.f, 0.f, 0.f), float min_z = 0.f)
{
	float x = pos.X, y = pos.Y, z = pos.Z;

	//top
	verts[0] = FVector(-radius*0.5f + x, radius*c3 + y, z);
	verts[1] = FVector(radius*0.5f + x, radius*c3 + y, z);
	verts[2] = FVector(radius + x, 0.0f + y, z);
	verts[3] = FVector(radius*0.5f + x, -radius*c3 + y, z);
	verts[4] = FVector(-radius*0.5f + x, -radius*c3 + y, z);
	verts[5] = FVector(-radius + x, 0.0f + y, z);
	verts[6] = FVector(0.0f + x, 0.0f + y, z);

	//sides+								  
	verts[7] = FVector(-radius*0.5f + x, radius*c3 + y, z);
	verts[8] = FVector(radius*0.5f + x, radius*c3 + y, z);
	verts[9] = FVector(radius + x, y, 0.0f + z);
	verts[10] = FVector(radius*0.5f + x, -radius*c3 + y, z);
	verts[11] = FVector(-radius*0.5f + x, -radius*c3 + y, z);
	verts[12] = FVector(-radius + x, 0.0f + y, z);
	//sides-
	verts[13] = FVector(-radius*0.5f + x, radius*c3 + y, min_z);
	verts[14] = FVector(radius*0.5f + x, radius*c3 + y, min_z);
	verts[15] = FVector(radius + x, 0.0f + y, min_z);
	verts[16] = FVector(radius*0.5f + x, -radius*c3 + y, min_z);
	verts[17] = FVector(-radius*0.5f + x, -radius*c3 + y, min_z);
	verts[18] = FVector(-radius + x, 0.0f + y, min_z);

	//sides+
	verts[19] = FVector(-radius*0.5f + x, radius*c3 + y, z);
	verts[20] = FVector(radius*0.5f + x, radius*c3 + y, z);
	verts[21] = FVector(radius + x, 0.0f + y, z);
	verts[22] = FVector(radius*0.5f + x, -radius*c3 + y, z);
	verts[23] = FVector(-radius*0.5f + x, -radius*c3 + y, z);
	verts[24] = FVector(-radius + x, 0.0f + y, z);
	//sides
	verts[25] = FVector(-radius*0.5f + x, radius*c3 + y, min_z);
	verts[26] = FVector(radius*0.5f + x, radius*c3 + y, min_z);
	verts[27] = FVector(radius + x, 0.0f + y, min_z);
	verts[28] = FVector(radius*0.5f + x, -radius*c3 + y, min_z);
	verts[29] = FVector(-radius*0.5f + x, -radius*c3 + y, min_z);
	verts[30] = FVector(-radius + x, 0.0f + y, min_z);


	hexscape->CreateMeshSection_LinearColor(hexscape->GetNumSections(), verts, tris, normals, uvs, colors, tangents, false);
	hexscape->bUseComplexAsSimpleCollision = false;
	hexscape->SetCollisionResponseToAllChannels(ECollisionResponse::ECR_Block);
	hexscape->SetMaterial(hexscape->GetNumSections() - 1, hexscape_material);
	//landscape->GetProcMeshSection(landscape->GetNumSections() - 1)->bEnableCollision = true;
}


const float AHexMeshGen::c3 = 0.8660254f;
const float AHexMeshGen::c3m1d2 = 0.0669872f;

const TArray<FVector> AHexMeshGen::normals = {
	FVector(0.0f, 0.0f, 1.0f),
	FVector(0.0f, 0.0f, 1.0f),
	FVector(0.0f, 0.0f, 1.0f),
	FVector(0.0f, 0.0f, 1.0f),
	FVector(0.0f, 0.0f, 1.0f),
	FVector(0.0f, 0.0f, 1.0f),
	FVector(0.0f, 0.0f, 1.0f),

	FVector(0.0f, 1.0f, 0.0f),
	FVector(AHexMeshGen::c3, 0.5f, 0.0f),
	FVector(AHexMeshGen::c3, -0.5f, 0.0f),
	FVector(0.0f, -1.0f, 0.0f),
	FVector(-AHexMeshGen::c3, -0.5f, 0.0f),
	FVector(-AHexMeshGen::c3, 0.5f, 0.0f),
	FVector(0.0f, 1.0f, 0.0f),
	FVector(AHexMeshGen::c3, 0.5f, 0.0f),
	FVector(AHexMeshGen::c3, -0.5f, 0.0f),
	FVector(0.0f, -1.0f, 0.0f),
	FVector(-AHexMeshGen::c3, -0.5f, 0.0f),
	FVector(-AHexMeshGen::c3, 0.5f, 0.0f),

	FVector(-AHexMeshGen::c3, 0.5f, 0.0f),
	FVector(0.0f, 1.0f, 0.0f),
	FVector(AHexMeshGen::c3, 0.5f, 0.0f),
	FVector(AHexMeshGen::c3, -0.5f, 0.0f),
	FVector(0.0f, -1.0f, 0.0f),
	FVector(-AHexMeshGen::c3, -0.5f, 0.0f),
	FVector(-AHexMeshGen::c3, 0.5f, 0.0f),
	FVector(0.0f, 1.0f, 0.0f),
	FVector(AHexMeshGen::c3, 0.5f, 0.0f),
	FVector(AHexMeshGen::c3, -0.5f, 0.0f),
	FVector(0.0f, -1.0f, 0.0f),
	FVector(-AHexMeshGen::c3, -0.5f, 0.0f),
};

const TArray<FProcMeshTangent> AHexMeshGen::tangents = {

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

const TArray<FVector2D> AHexMeshGen::uvs = {

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

const TArray<FLinearColor> AHexMeshGen::colors = {
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

const TArray<int32> AHexMeshGen::tris = {
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
