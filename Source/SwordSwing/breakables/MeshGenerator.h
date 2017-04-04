// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "GameFramework/Actor.h"
#include "ScalarField.h"
#include "ProceduralMeshComponent.h"
#include "MeshGenerator.generated.h"

UCLASS()
class SWORDSWING_API AMeshGenerator : public AActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	AMeshGenerator();

	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

	UPROPERTY(Category = "Mesh", BlueprintReadWrite, VisibleAnywhere)
	class USceneComponent * root;
	
	UPROPERTY(Category = "Mesh", BlueprintReadWrite, VisibleAnywhere)
	class UProceduralMeshComponent * mesh_frag_1;
	UPROPERTY(Category = "Mesh", BlueprintReadWrite, VisibleAnywhere)
	class UProceduralMeshComponent * mesh_frag_2;
	UPROPERTY(Category = "Mesh", BlueprintReadWrite, VisibleAnywhere)
	class UProceduralMeshComponent * mesh_frag_3;
	UPROPERTY(Category = "Mesh", BlueprintReadWrite, VisibleAnywhere)
	class UProceduralMeshComponent * mesh_frag_4;
	UPROPERTY(Category = "Mesh", BlueprintReadWrite, VisibleAnywhere)
	class UProceduralMeshComponent * mesh_frag_5;
	UPROPERTY(Category = "Mesh", BlueprintReadWrite, VisibleAnywhere)
	class UProceduralMeshComponent * mesh_frag_6;
	UPROPERTY(Category = "Mesh", BlueprintReadWrite, VisibleAnywhere)
	class UProceduralMeshComponent * mesh_frag_7;
	UPROPERTY(Category = "Mesh", BlueprintReadWrite, VisibleAnywhere)
	class UProceduralMeshComponent * mesh_frag_8;

	UPROPERTY(Category = "Mesh", BlueprintReadWrite, VisibleAnywhere)
	TArray< class UProceduralMeshComponent*> mesh_frags;


	UPROPERTY(Category = "Mesh", BlueprintReadWrite, VisibleAnywhere)
	UStaticMeshComponent* baseModel;
	ScalarField<float>* base_model_sf;
	FVector mid_point;
	FVector increased_extent;
	

	UPROPERTY(Category = "Mesh", BlueprintReadWrite, EditAnywhere)
	int32 resolution = 30;
	// Called every frame
	virtual void Tick( float DeltaSeconds ) override;

	/** called when projectile hits something */
	UFUNCTION()
	void OnOriginalModelHit(UPrimitiveComponent* HitComp, AActor* OtherActor, UPrimitiveComponent* OtherComp, FVector NormalImpulse, const FHitResult& Hit);

private:
	
	void CreateFragment(FMatrix _collision_rot, FVector _collision_loc, FVector _frag_offset);

	TArray<ScalarField<float>*> frag_sf;
	
};
