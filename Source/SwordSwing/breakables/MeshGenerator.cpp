// Fill out your copyright notice in the Description page of Project Settings.


#include "SwordSwing.h"
#include <string>
#include "MeshGenerator.h"
#include "Triangulation.h"
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
	/*root = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));
	RootComponent = root;
*/
	baseModel = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("Base Model"));
	baseModel->SetupAttachment(RootComponent);
	//static ConstructorHelpers::FObjectFinder<UStaticMesh> baseModelObj(TEXT("/Game/StarterContent/Props/SM_Statue"));
	//baseModel->SetStaticMesh(baseModelObj.Object);
	RootComponent = baseModel;
	baseModel->SetSimulatePhysics(true);

 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
	mesh_frag_1 = CreateDefaultSubobject<UProceduralMeshComponent>(TEXT("ProceduralMesh1"));
	mesh_frag_1->SetupAttachment(RootComponent);
	mesh_frag_2 = CreateDefaultSubobject<UProceduralMeshComponent>(TEXT("ProceduralMesh2"));
	mesh_frag_2->SetupAttachment(RootComponent);
	mesh_frag_3 = CreateDefaultSubobject<UProceduralMeshComponent>(TEXT("ProceduralMesh3"));
	mesh_frag_3->SetupAttachment(RootComponent);
	mesh_frag_4 = CreateDefaultSubobject<UProceduralMeshComponent>(TEXT("ProceduralMesh4"));
	mesh_frag_4->SetupAttachment(RootComponent);
	mesh_frag_5 = CreateDefaultSubobject<UProceduralMeshComponent>(TEXT("ProceduralMesh5"));
	mesh_frag_5->SetupAttachment(RootComponent);
	mesh_frag_6 = CreateDefaultSubobject<UProceduralMeshComponent>(TEXT("ProceduralMesh6"));
	mesh_frag_6->SetupAttachment(RootComponent);
	mesh_frag_7 = CreateDefaultSubobject<UProceduralMeshComponent>(TEXT("ProceduralMesh7"));
	mesh_frag_7->SetupAttachment(RootComponent);
	mesh_frag_8 = CreateDefaultSubobject<UProceduralMeshComponent>(TEXT("ProceduralMesh8"));
	mesh_frag_8->SetupAttachment(RootComponent);
	
	baseModel->OnComponentHit.AddDynamic(this, &AMeshGenerator::OnOriginalModelHit);
}

// Called when the game starts or when spawned
void AMeshGenerator::BeginPlay()
{
	Super::BeginPlay();


	/*mesh_frags.Add(ConstructObject<UProceduralMeshComponent>(UProceduralMeshComponent::StaticClass(), this, FName("ProceduralMesh9")));
	mesh_frags[0]->RegisterComponent();
	mesh_frags[0]->AttachTo(baseModel);*/
	//mesh_frags[0]->SetWorldLocation(baseModel->GetComponentTransform().GetLocation());
	//mesh_frags[0]->SetWorldRotation(baseModel->GetComponentTransform().GetRotation());
	
	//mesh_frags[0]->SetRelativeLocation(FVector::ZeroVector);
	//mesh_frags[0]->SetRelativeRotation(FRotator(0.f, 0.f, 0.f));
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
	//ScalarField<float> sf(res, increased_extent);
	base_model_sf->setIsoValue(0.0f);
	base_model_sf->meshToLeveSet(&rawMesh, mid_point);
	//triangulation::marchingCubes(mesh, &sf, mid_point);
	//baseModel->SetRelativeLocation(FVector(0.f, 0.f, 100.f));
	//baseModel->UnregisterComponent();
	//baseModel->DestroyComponent();
	//triangulation::marchingCubes(mesh, base_model_sf, mid_point);
}

// Called every frame
void AMeshGenerator::Tick( float DeltaTime )
{
	Super::Tick( DeltaTime );
	//mesh->AddForce(FVector(0.0f, 0.0f, 1000.0f));

	//triangulation::marchingCubes(mesh, base_model_sf, mid_point);
}

void AMeshGenerator::OnOriginalModelHit(UPrimitiveComponent* HitComp, AActor* OtherActor, UPrimitiveComponent* OtherComp, FVector NormalImpulse, const FHitResult& Hit)
{
	DrawDebugPoint(
		GetWorld(),
		Hit.Location,
		10,  					//size
		FColor(255, 0, 255),  //pink
		false,  				//persistent (never goes away)
		0.03 					//point leaves a trail on moving object
	);
	DrawDebugPoint(
		GetWorld(),
		baseModel->ComponentToWorld.TransformPosition(FVector(0.f, 0.f, 0.f)),
		10,  					//size
		FColor(0, 0, 255),  //pink
		false,  				//persistent (never goes away)
		0.03 					//point leaves a trail on moving object
	);
	
	if (NormalImpulse.Size() > 10000.f)
	{

		FVector test_mid = baseModel->GetCenterOfMass();

		//Create signed distance function and level set for fragments ===============================================================================
		frag_sf.Add( new ScalarField<float>(resolution, increased_extent.Z / 1.0f));
		frag_sf[0]->setIsoValue(0.0f);
		frag_sf[0]->cubeSignedDistance(FVector::ZeroVector);
		ScalarField<float> sf_frag_1(resolution, increased_extent.Z / 1.0f);
		sf_frag_1.setIsoValue(0.0f);
		sf_frag_1.cubeSignedDistance(FVector::ZeroVector);

		FVector localImpulseNormal = baseModel->ComponentToWorld.Inverse().TransformVector(NormalImpulse.GetSafeNormal());
		UE_LOG(LogTemp, Warning, TEXT("localImpulseNormal: %s"), *localImpulseNormal.ToString());
		UE_LOG(LogTemp, Warning, TEXT("NormalImpulse: %s"), *NormalImpulse.ToString());
		
		
		//mid_point = FVector(0.0f, 0.0f, 0.0f);
		//triangulation::marchingCubes(mesh, &sf2, mid_point);
		test_mid = test_mid - mid_point;
		
		FVector tmp_zero = FVector::ZeroVector;
		FVector local_hit_location = baseModel->ComponentToWorld.InverseTransformPositionNoScale(Hit.Location);
		//local_hit_location = FVector::ZeroVector;
		//FVector translation_offset = test_mid - Hit.Location;
		//FVector translation_offset = mid_point;
		
		float rot_angle = FMath::Acos(FVector::DotProduct(FVector::UpVector,
			localImpulseNormal));// *180.f / PI;
		FVector rot_axis = FVector::CrossProduct( FVector::UpVector, localImpulseNormal);

		axisRotMatrix(rot_angle, rot_axis);

		FMatrix rot_mat;
		//FRotator test_rot = (-localImpulseNormal).Rotation();
		//FTransform test_transform((-localImpulseNormal).Rotation());
		//rot_mat = test_transform.ToMatrixWithScale();
		rot_mat = axisRotMatrix(rot_angle, rot_axis);
		FMatrix tmp_identity = FMatrix::Identity;

		ScalarField<float> sf3(resolution, increased_extent.Z / 1.0f);
		float frag_offset = increased_extent.Z / 2.0f;
		FVector corner1 = FVector(frag_offset, -frag_offset, -frag_offset);
		CreateFragment(rot_mat, local_hit_location, corner1);
		/*ScalarField<float>::mergeLevelSets(base_model_sf, &sf_frag_1, rot_mat, local_hit_location, corner1, &sf3);
		triangulation::marchingCubes(mesh_frag_1, &sf3, tmp_zero);
		mesh_frag_1->AddRelativeLocation(rot_mat.TransformPosition(corner1), false, nullptr, ETeleportType::TeleportPhysics);
		mesh_frag_1->AddRelativeRotation(rot_mat.Rotator(), false, nullptr, ETeleportType::TeleportPhysics);
		mesh_frag_1->AddRelativeLocation(local_hit_location, false, nullptr, ETeleportType::TeleportPhysics);
		*/
		FVector corner2 = FVector(frag_offset, frag_offset, -frag_offset);
		CreateFragment(rot_mat, local_hit_location, corner2);
		/*ScalarField<float>::mergeLevelSets(base_model_sf, &sf_frag_1, rot_mat, local_hit_location,( corner2), &sf3);
		triangulation::marchingCubes(mesh_frag_2, &sf3, tmp_zero);
		mesh_frag_2->AddRelativeLocation(rot_mat.TransformPosition(corner2), false, nullptr, ETeleportType::TeleportPhysics);
		mesh_frag_2->AddRelativeRotation(rot_mat.Rotator(), false, nullptr, ETeleportType::TeleportPhysics);
		mesh_frag_2->AddRelativeLocation(local_hit_location, false, nullptr, ETeleportType::TeleportPhysics);
*/

		FVector corner3 =  FVector(frag_offset, -frag_offset, frag_offset);
		CreateFragment(rot_mat, local_hit_location, corner3);
		/*ScalarField<float>::mergeLevelSets(base_model_sf, &sf_frag_1, rot_mat, local_hit_location,( corner3), &sf3);
		triangulation::marchingCubes(mesh_frag_3, &sf3, tmp_zero);
		mesh_frag_3->AddRelativeLocation(rot_mat.TransformPosition(corner3), false, nullptr, ETeleportType::TeleportPhysics);
		mesh_frag_3->AddRelativeRotation(rot_mat.Rotator(), false, nullptr, ETeleportType::TeleportPhysics);
		mesh_frag_3->AddRelativeLocation(local_hit_location, false, nullptr, ETeleportType::TeleportPhysics);
*/

		FVector corner4 =  FVector(frag_offset, frag_offset, frag_offset);
		CreateFragment(rot_mat, local_hit_location, corner4);
		/*ScalarField<float>::mergeLevelSets(base_model_sf, &sf_frag_1, rot_mat, local_hit_location,( corner4), &sf3);
		triangulation::marchingCubes(mesh_frag_4, &sf3, tmp_zero);
		mesh_frag_4->AddRelativeLocation(rot_mat.TransformPosition(corner4), false, nullptr, ETeleportType::TeleportPhysics);
		mesh_frag_4->AddRelativeRotation(rot_mat.Rotator(), false, nullptr, ETeleportType::TeleportPhysics);
		mesh_frag_4->AddRelativeLocation(local_hit_location, false, nullptr, ETeleportType::TeleportPhysics);
		*/

		FVector corner5 =  FVector(-frag_offset, -frag_offset, -frag_offset);
		CreateFragment(rot_mat, local_hit_location, corner5);
		/*ScalarField<float>::mergeLevelSets(base_model_sf, &sf_frag_1, rot_mat, local_hit_location,( corner5), &sf3);
		triangulation::marchingCubes(mesh_frag_5, &sf3, tmp_zero);
		mesh_frag_5->AddRelativeLocation(rot_mat.TransformPosition(corner5), false, nullptr, ETeleportType::TeleportPhysics);
		mesh_frag_5->AddRelativeRotation(rot_mat.Rotator(), false, nullptr, ETeleportType::TeleportPhysics);
		mesh_frag_5->AddRelativeLocation(local_hit_location, false, nullptr, ETeleportType::TeleportPhysics);
		*/

		FVector corner6 =  FVector(-frag_offset, frag_offset, -frag_offset);
		CreateFragment(rot_mat, local_hit_location, corner6);
		/*ScalarField<float>::mergeLevelSets(base_model_sf, &sf_frag_1, rot_mat, local_hit_location,( corner6), &sf3);
		triangulation::marchingCubes(mesh_frag_6, &sf3, tmp_zero);
		mesh_frag_6->AddRelativeLocation(rot_mat.TransformPosition(corner6), false, nullptr, ETeleportType::TeleportPhysics);
		mesh_frag_6->AddRelativeRotation(rot_mat.Rotator(), false, nullptr, ETeleportType::TeleportPhysics);
		mesh_frag_6->AddRelativeLocation(local_hit_location, false, nullptr, ETeleportType::TeleportPhysics);
		*/

		FVector corner7 = FVector(-frag_offset, -frag_offset, frag_offset);
		CreateFragment(rot_mat, local_hit_location, corner7);
		/*ScalarField<float>::mergeLevelSets(base_model_sf, &sf_frag_1, rot_mat, local_hit_location, ( corner7), &sf3);
		triangulation::marchingCubes(mesh_frag_7, &sf3, tmp_zero);
		mesh_frag_7->AddRelativeLocation(rot_mat.TransformPosition(corner7), false, nullptr, ETeleportType::TeleportPhysics);
		mesh_frag_7->AddRelativeRotation(rot_mat.Rotator(), false, nullptr, ETeleportType::TeleportPhysics);
		mesh_frag_7->AddRelativeLocation(local_hit_location, false, nullptr, ETeleportType::TeleportPhysics);
		*/

		FVector corner8 = FVector(-frag_offset, frag_offset, frag_offset);
		CreateFragment(rot_mat, local_hit_location, corner8);
		/*ScalarField<float>::mergeLevelSets(base_model_sf, &sf_frag_1, rot_mat, local_hit_location, ( corner8), &sf3);
		triangulation::marchingCubes(mesh_frag_8, &sf3, tmp_zero);
		mesh_frag_8->AddRelativeLocation(rot_mat.TransformPosition(corner8), false, nullptr, ETeleportType::TeleportPhysics);
		mesh_frag_8->AddRelativeRotation(rot_mat.Rotator(), false, nullptr, ETeleportType::TeleportPhysics);
		mesh_frag_8->AddRelativeLocation(local_hit_location, false, nullptr, ETeleportType::TeleportPhysics);
		*/

		/*mesh_frag_1->SetSimulatePhysics(true);
		mesh_frag_2->SetSimulatePhysics(true);
		mesh_frag_3->SetSimulatePhysics(true);
		mesh_frag_4->SetSimulatePhysics(true);
		mesh_frag_5->SetSimulatePhysics(true);
		mesh_frag_6->SetSimulatePhysics(true);
		mesh_frag_7->SetSimulatePhysics(true);
		mesh_frag_8->SetSimulatePhysics(true);

		mesh_frag_1->ComponentVelocity = baseModel->ComponentVelocity;
		mesh_frag_2->ComponentVelocity = baseModel->ComponentVelocity;
		mesh_frag_3->ComponentVelocity = baseModel->ComponentVelocity;
		mesh_frag_4->ComponentVelocity = baseModel->ComponentVelocity;
		mesh_frag_5->ComponentVelocity = baseModel->ComponentVelocity;
		mesh_frag_6->ComponentVelocity = baseModel->ComponentVelocity;
		mesh_frag_7->ComponentVelocity = baseModel->ComponentVelocity;
		mesh_frag_8->ComponentVelocity = baseModel->ComponentVelocity;*/

		//mesh_frag_1->AddImpulse(-NormalImpulse*0.125f);
		//mesh_frag_2->AddImpulse(-NormalImpulse*0.125f);
		//mesh_frag_3->AddImpulse(-NormalImpulse*0.125f);
		//mesh_frag_4->AddImpulse(-NormalImpulse*0.125f);
		//mesh_frag_5->AddImpulse(-NormalImpulse*0.125f);
		//mesh_frag_6->AddImpulse(-NormalImpulse*0.125f);
		//mesh_frag_7->AddImpulse(-NormalImpulse*0.125f);
		//mesh_frag_8->AddImpulse(-NormalImpulse*0.125f);


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
	mesh_frags[mf_i]->InitializeComponent();
	mesh_frags[mf_i]->AttachTo(baseModel);
	

	ScalarField<float> merging_sf;
	ScalarField<float>::mergeLevelSets(base_model_sf, frag_sf[0], _collision_rot, _collision_loc, _frag_offset, &merging_sf);
	FVector tmp_zero = FVector::ZeroVector;
	triangulation::marchingCubes(mesh_frags[mf_i], &merging_sf, tmp_zero);
	mesh_frags[mf_i]->AddRelativeLocation(_collision_rot.TransformPosition(_frag_offset), false, nullptr, ETeleportType::TeleportPhysics);
	mesh_frags[mf_i]->AddRelativeRotation(_collision_rot.Rotator(), false, nullptr, ETeleportType::TeleportPhysics);
	mesh_frags[mf_i]->AddRelativeLocation(_collision_loc, false, nullptr, ETeleportType::TeleportPhysics);
	
	mesh_frags[mf_i]->SetSimulatePhysics(true);
	mesh_frags[mf_i]->ComponentVelocity = baseModel->ComponentVelocity;
}
