// Fill out your copyright notice in the Description page of Project Settings.

#include "SwordSwing.h"
#include <string>

#include "HexInstancer.h"


// Sets default values
AHexInstancer::AHexInstancer()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

	root = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));
	RootComponent = root;

	int i = instances.Num();
	std::string comp_name = "Instance" + std::to_string(i);
	instances.Add (CreateDefaultSubobject<UInstancedStaticMeshComponent>(TEXT("InstancedLandscape")));
	static ConstructorHelpers::FObjectFinder<UStaticMesh> hex_mesh(TEXT("/Game/Meshes/hex_box.hex_box"));
	instances[i]->SetStaticMesh(hex_mesh.Object);

	static ConstructorHelpers::FObjectFinder<UMaterialInterface> hex_mat(TEXT("/Game/materials/ProceduralHexMat.ProceduralHexMat"));
	//instance_mats.Add(land_mat.Object);
	instances[i]->SetMaterial(0, hex_mat.Object);


}

// Called when the game starts or when spawned
void AHexInstancer::BeginPlay()
{
	Super::BeginPlay();
	SetActorScale3D(FVector(1.f, 1.f, 2.f));
	//instances[0]->SetWorldScale3D(FVector(1.f, 1.f, 2.f));
}

// Called every frame
void AHexInstancer::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

}

int AHexInstancer::addInstance()
{
	int i = instances.Num();
	std::string comp_name = "Instance" + std::to_string(i);

	instances.Add(ConstructObject<UInstancedStaticMeshComponent>(UInstancedStaticMeshComponent::StaticClass(), this, FName(&comp_name[0])));
	instances[i]->SetMaterial(0, instances[0]->GetMaterial(0));
	return i;
}


int  AHexInstancer::addInstance(UMaterialInterface* mat)
{
	int i = instances.Num();
	std::string comp_name = "Instance" + std::to_string(i);

	instances.Add(ConstructObject<UInstancedStaticMeshComponent>(UInstancedStaticMeshComponent::StaticClass(), this, FName(&comp_name[0])));
	instances[i]->SetMaterial(0, mat);
	return i;
}


void  AHexInstancer::setInstanceMat(int index, UMaterialInterface* mat)
{
	instances[index]->SetMaterial(0, mat);
}


UHexCircleGenComponent* AHexInstancer::addCircle(int instance_index, float hex_rad, int circle_radius, FVector position)
{
	int i = generators.Num();
	std::string comp_name = "generator" + std::to_string(i);
	UHexCircleGenComponent* gen = ConstructObject<UHexCircleGenComponent>(UHexCircleGenComponent::StaticClass(), this, FName(&comp_name[0]));
	gen->crad = circle_radius;
	gen->hrad = hex_rad;
	gen->instance = instances[instance_index];
	gen->instancer = this;
	gen->SetWorldLocation(position);

	gen->RegisterComponent();
	//gen->AttachTo(instances[instance_index]);
	gen->InitializeComponent();

	generators.Add(gen);
	return gen;
}

UHexGridGenComponent* AHexInstancer::addGrid(int instance_index, float hex_rad, int x_halfdim, int y_halfdim, FVector position)
{
	int i = generators.Num();
	std::string comp_name = "generator" + std::to_string(i);
	UHexGridGenComponent* gen = ConstructObject<UHexGridGenComponent>(UHexGridGenComponent::StaticClass(), this, FName(&comp_name[0]));
	gen->hrad = hex_rad;
	gen->xhd = x_halfdim;
	gen->yhd = y_halfdim;
	gen->instance = instances[instance_index];
	gen->instancer = this;
	gen->SetWorldLocation(position);

	gen->RegisterComponent();
//	gen->AttachTo(instances[instance_index]);
	gen->InitializeComponent();

	generators.Add(gen);
	return gen;
}

int  AHexInstancer::addGen(UHexGenComponent* gen)
{
	int i = generators.Num();
	generators.Add(gen);
	return i;
}

void  AHexInstancer::generateAll() {
	for (const auto & gen : generators)
	{
		gen->generate();
	}
}

void  AHexInstancer::pushAnim(UHexGenComponent* a_gen)
{
	animating_generators.push(a_gen);
	if (animating_generators.size() == 1)
		animating_generators.front()->animating = true;

}

void  AHexInstancer::popAnim()
{
	if (!animating_generators.front()->animating)
	{
		animating_generators.pop();
		if (animating_generators.size() > 0)
			animating_generators.front()->animating = true;
	}
		
}

void AHexInstancer::animTick(float DeltaTime)
{

}