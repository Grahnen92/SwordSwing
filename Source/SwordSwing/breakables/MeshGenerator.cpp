// Fill out your copyright notice in the Description page of Project Settings.


#include "SwordSwing.h"

#include <string>
#include <list>

#include "MeshGenerator.h"
#include "Triangulation.h"

#include "voronoi/Voronoi.h"

#include "RawMesh.h"
#include "Components/StaticMeshComponent.h"


FMatrix axisRotMatrix(float _a, FVector _axis) {
	FMatrix result;

	_axis.Normalize();
	float c = FMath::Cos(_a);
	float s = FMath::Sin(_a);

	float u = _axis.X;
	float v = _axis.Y;
	float w = _axis.Z;
	float u2 = u*u;
	float v2 = v*v;
	float w2 = w*w;

	FVector a1 = FVector(u2 + (1 - u2)*c, u*v*(1 - c) + w*s, u*w*(1 - c) - v*s);
	FVector a2 = FVector(u*v*(1 - c) - w*s, v2 + (1 - v2)*c, v*w*(1 - c) + u*s);
	FVector a3 = FVector(u*w*(1 - c) + v*s, v*w*(1 - c) - u*s, w2 + (1 - w2)*c);
	FVector a4 = FVector(0.f, 0.f, 0.f);
	result.SetAxes(&a1,&a2,&a3,&a4);

	return result;
}

// Sets default values
AMeshGenerator::AMeshGenerator()
{
	root = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));
	RootComponent = root;

	baseModel = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("Base Model"));
	baseModel->SetupAttachment(root);
	
	baseModel->SetSimulatePhysics(true);

 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
	
	baseModel->OnComponentHit.AddDynamic(this, &AMeshGenerator::OnOriginalModelHit);
	

}

// Called when the game starts or when spawned
void AMeshGenerator::BeginPlay()
{
	Super::BeginPlay();

	//TODO: this 
	Voronoi v;
	TArray<FVector2D> tmp_voro_sites;
	tmp_voro_sites.Add(FVector2D(50.f, 95.f));
	tmp_voro_sites.Add(FVector2D(5.f, 55.f));
	tmp_voro_sites.Add(FVector2D(95.f, 50.f));
	tmp_voro_sites.Add(FVector2D(50.f, 30.f));
	//tmp_voro_sites.Add(FVector2D(60.f, 30.f));
	v.setDims(100.f, 100.f);
	v.CalculateDiagram(&tmp_voro_sites);

	{
		DrawDebugLine(
			GetWorld(),
			FVector(0.f, 0.f, 300.f),
			FVector(0.f, v.getDims().Y, 300.f), 					//size
			FColor(255, 255, 255),  //pink
			true,  				//persistent (never goes away)
			0.0, 					//point leaves a trail on moving object
			10,
			1.f
		);

		DrawDebugLine(
			GetWorld(),
			FVector(0.f, v.getDims().Y, 300.f),
			FVector(v.getDims().X, v.getDims().Y, 300.f), 					//size
			FColor(255, 255, 255),  //pink
			true,  				//persistent (never goes away)
			0.0, 					//point leaves a trail on moving object
			10,
			1.f
		);

		DrawDebugLine(
			GetWorld(),
			FVector(v.getDims().X, v.getDims().Y, 300.f),
			FVector(v.getDims().X, 0.f, 300.f), 					//size
			FColor(255, 255, 255),  //pink
			true,  				//persistent (never goes away)
			0.0, 					//point leaves a trail on moving object
			10,
			1.f
		);

		DrawDebugLine(
			GetWorld(),
			FVector(v.getDims().X, 0.f, 300.f),
			FVector(0.f, 0.f, 300.f), 					//size
			FColor(255, 255, 255),  //pink
			true,  				//persistent (never goes away)
			0.0, 					//point leaves a trail on moving object
			10,
			1.f
		);
	}

	std::vector<VSite>* v_sites = v.getSites();
	int counter = 0;
	for (const auto &v_site : *v_sites)
	{
		FVector site3D = FVector(v_site.pos.X, v_site.pos.Y, 300.f);
		DrawDebugPoint(
			GetWorld(),
			site3D,
			10,  					//size
			FColor(counter*50, 0, (v_sites->size() -counter)*50),  //pink
			true,  				//persistent (never goes away)
			0.0 					//point leaves a trail on moving object
		);
		if (counter == 2)
		{
			for (const auto &edge : v_site.edges)
			{
				FVector start3D = FVector(edge->start.X, edge->start.Y, 300.f);
				FVector end3D = FVector(edge->end.X, edge->end.Y, 300.f);
				FVector dir3D = FVector(edge->direction.X, edge->direction.Y, 0.f)*100.f;
				FVector right3D = FVector(edge->right->pos, 300.f) - start3D;
				DrawDebugLine(
					GetWorld(),
					start3D,
					start3D + dir3D, 					//size
					FColor(counter * 50, 0, (v_sites->size() - counter) * 50),  //pink
					true,  				//persistent (never goes away)
					0.0, 					//point leaves a trail on moving object
					10,
					1.f
				);

				DrawDebugLine(
					GetWorld(),
					start3D,
					start3D + right3D, 					//size
					FColor(255, 255, 255),  //pink
					true,  				//persistent (never goes away)
					0.0, 					//point leaves a trail on moving object
					10,
					1.f
				);
			}
		}
		
		counter++;
	}

	ScalarField<float> voronoi_sf(32, v.getDims());
	voronoi_sf.voronoiSignedDist(&v, -v.getDims()/2.0f);

	//Create signed distance function and level set from a model ===============================================================================
	FStaticMeshSourceModel* sourceM = &baseModel->GetStaticMesh()->SourceModels[0];
	FRawMesh rawMesh;
	sourceM->RawMeshBulkData->LoadRawMesh(rawMesh);

	float max_dim = 2.0f*FMath::Max(FMath::Max(baseModel->Bounds.BoxExtent.X, baseModel->Bounds.BoxExtent.Y), baseModel->Bounds.BoxExtent.Z);
	FVector min_dims;
	FVector max_dims;
	triangulation::findMaxMinExtent(rawMesh.VertexPositions, min_dims, max_dims);
	FVector extent = max_dims - min_dims;
	//extent = extent*baseModel->GetComponentScale();

	//<-- Used to create a scalar field that always have the same amount of scalar values but distributed 
	// at cubic intervals along boxes of different dimensions
	float k = std::cbrt((extent.Z*extent.Z) / (extent.X*extent.Y));
	float j = (extent.Y / extent.Z)*k;
	float i = (extent.X / extent.Z)*k;
	FVector res(std::ceilf(resolution*i), std::ceilf(resolution*j), std::ceilf(resolution*k));
	FVector point_interval = extent / res;
	//FVector increased_extent(extent.X + (extent.X / res.X) * 2, extent.Y + (extent.Y / res.Y) * 2, extent.Z + (extent.Z / res.Z) * 2);
	increased_extent = extent + 2 * point_interval;
	mid_point = (min_dims)+(extent / 2.0f);

	//<-- create the scalar field with the determined resolution and with a somewhat increased dimension so that scalar values exist outside and around the original model
	base_model_sf = new ScalarField<float>(res, increased_extent);
	base_model_sf->setIsoValue(0.0f);
	base_model_sf->meshToLeveSet(&rawMesh, mid_point);

	base_material = baseModel->GetMaterial(0);
}

// Called every frame
void AMeshGenerator::Tick( float DeltaTime )
{
	Super::Tick( DeltaTime );
}

void AMeshGenerator::OnOriginalModelHit(UPrimitiveComponent* HitComp, AActor* OtherActor, UPrimitiveComponent* OtherComp, FVector NormalImpulse, const FHitResult& Hit)
{	
	if (NormalImpulse.Size() > 10000.f)
	{

		FVector test_mid = baseModel->GetCenterOfMass();

		//Create signed distance function and level set for fragments ===============================================================================
		frag_sf.Add( new ScalarField<float>(resolution, increased_extent.Z + (increased_extent.Z / resolution)));
		frag_sf[0]->setIsoValue(0.0f);
		frag_sf[0]->cubeSignedDistance(FVector::ZeroVector);

		//Transform impulsnormal and hit location to model space
		FVector localImpulseNormal = baseModel->ComponentToWorld.Inverse().TransformVector(NormalImpulse.GetSafeNormal());
		FVector local_hit_location = baseModel->ComponentToWorld.InverseTransformPositionNoScale(Hit.Location);
		
		//create the rotation matrix that align the fragments to the impulsenormal
		float rot_angle = FMath::Acos(FVector::DotProduct(FVector::UpVector, localImpulseNormal));// *180.f / PI;
		FVector rot_axis = FVector::CrossProduct( FVector::UpVector, localImpulseNormal);
		FMatrix rot_mat;
		rot_mat = axisRotMatrix(rot_angle, rot_axis);

		//calculate the individual positions of the fragments and then merge them with the model level set and the triangulate the merged level set
		float frag_offset = increased_extent.Z / 2.0f;
		FVector corner1 = FVector(frag_offset, -frag_offset, -frag_offset);
		CreateFragment(rot_mat, local_hit_location, corner1);

		FVector corner2 = FVector(frag_offset, frag_offset, -frag_offset);
		CreateFragment(rot_mat, local_hit_location, corner2);

		FVector corner3 =  FVector(frag_offset, -frag_offset, frag_offset);
		CreateFragment(rot_mat, local_hit_location, corner3);

		FVector corner4 =  FVector(frag_offset, frag_offset, frag_offset);
		CreateFragment(rot_mat, local_hit_location, corner4);

		FVector corner5 =  FVector(-frag_offset, -frag_offset, -frag_offset);
		CreateFragment(rot_mat, local_hit_location, corner5);

		FVector corner6 =  FVector(-frag_offset, frag_offset, -frag_offset);
		CreateFragment(rot_mat, local_hit_location, corner6);

		FVector corner7 = FVector(-frag_offset, -frag_offset, frag_offset);
		CreateFragment(rot_mat, local_hit_location, corner7);

		FVector corner8 = FVector(-frag_offset, frag_offset, frag_offset);
		CreateFragment(rot_mat, local_hit_location, corner8);

		baseModel->UnregisterComponent();
		baseModel->DestroyComponent();
		//baseModel->SetCollisionResponseToAllChannels(ECollisionResponse::ECR_Ignore);
	}

	
}

void AMeshGenerator::CreateFragment(FMatrix _collision_rot, FVector _collision_loc, FVector _frag_offset)
{
	int mf_i = mesh_frags.Num();
	std::string comp_name = "ProceduralMesh" + std::to_string(mf_i);
	mesh_frags.Add(ConstructObject<UProceduralMeshComponent>(UProceduralMeshComponent::StaticClass(), this, FName(&comp_name[0])));
	mesh_frags[mf_i]->RegisterComponent();
	mesh_frags[mf_i]->AttachTo(baseModel);
	mesh_frags[mf_i]->InitializeComponent();
	mesh_frags[mf_i]->bUseComplexAsSimpleCollision = false;

	mesh_frags[mf_i]->SetMaterial(0, base_material);

	ScalarField<float> merging_sf;
	ScalarField<float>::mergeLevelSets(base_model_sf, frag_sf[0], _collision_rot, _collision_loc, _frag_offset, &merging_sf);
	FVector tmp_zero = FVector::ZeroVector;
	triangulation::marchingCubes(mesh_frags[mf_i], &merging_sf, tmp_zero);
	mesh_frags[mf_i]->AddRelativeLocation(_collision_rot.TransformPosition(_frag_offset), false, nullptr, ETeleportType::TeleportPhysics);
	mesh_frags[mf_i]->AddRelativeRotation(_collision_rot.Rotator(), false, nullptr, ETeleportType::TeleportPhysics);
	mesh_frags[mf_i]->AddRelativeLocation(_collision_loc, false, nullptr, ETeleportType::TeleportPhysics);
	
	mesh_frags[mf_i]->SetCollisionObjectType(ECollisionChannel::ECC_PhysicsBody);

	mesh_frags[mf_i]->SetSimulatePhysics(true);
	mesh_frags[mf_i]->ComponentVelocity = baseModel->ComponentVelocity;
}
