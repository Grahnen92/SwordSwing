// Fill out your copyright notice in the Description page of Project Settings.

#include "SwordSwing.h"
#include <string>

#include "HexCircleGen.h"

#include "HexInstancerComponent.h"


// Sets default values for this component's properties
UHexInstancerComponent::UHexInstancerComponent()
{
	// Set this component to be initialized when the game starts, and to be ticked every frame.  You can turn these features
	// off to improve performance if you don't need them.
	PrimaryComponentTick.bCanEverTick = true;

	int i = instances.Num();
	std::string comp_name = "Instance" + std::to_string(i);
	instances.Add(CreateDefaultSubobject<UInstancedStaticMeshComponent>(TEXT("InstancedLandscape")));
	static ConstructorHelpers::FObjectFinder<UStaticMesh> hex_mesh(TEXT("/Game/Meshes/hex_box.hex_box"));
	instances[i]->SetStaticMesh(hex_mesh.Object);

	static ConstructorHelpers::FObjectFinder<UMaterialInterface> hex_mat(TEXT("/Game/materials/ProceduralMaterial.ProceduralMaterial"));
	//instance_mats.Add(land_mat.Object);
	instances[i]->SetMaterial(0, hex_mat.Object);
	// ...
}


// Called when the game starts
void UHexInstancerComponent::BeginPlay()
{
	Super::BeginPlay();
	// ...
}


// Called every frame
void UHexInstancerComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
	// ...
}



int UHexInstancerComponent::addInstance()
{
	int i = instances.Num();
	std::string comp_name = "Instance" + std::to_string(i);

	instances.Add(ConstructObject<UInstancedStaticMeshComponent>(UInstancedStaticMeshComponent::StaticClass(), this, FName(&comp_name[0])));
	instances[i]->SetMaterial(0, instances[0]->GetMaterial(0));
	return i;
}


int  UHexInstancerComponent::addInstance(UMaterialInterface* mat)
{
	int i = instances.Num();
	std::string comp_name = "Instance" + std::to_string(i);

	instances.Add(ConstructObject<UInstancedStaticMeshComponent>(UInstancedStaticMeshComponent::StaticClass(), this, FName(&comp_name[0])));
	instances[i]->SetMaterial(0, mat);
	return i;
}


void  UHexInstancerComponent::setInstanceMat(int index, UMaterialInterface* mat)
{
	instances[index]->SetMaterial(0, mat);
}


int  UHexInstancerComponent::addCircle(int instance_index, float hex_rad, int circle_radius, FVector position)
{
	int i = generators.Num();
	HexCircleGen* gen = new HexCircleGen(instances[instance_index], hex_rad, circle_radius, position);
	generators.Add(gen);
	return i;
}

void  UHexInstancerComponent::generateAll() {
	for (const auto & gen : generators)
	{
		gen->generate();
	}
}