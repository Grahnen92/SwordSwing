// Fill out your copyright notice in the Description page of Project Settings.
#pragma once

using namespace std;


#include "SwordSwing.h"

#include "C:/Program Files (x86)/Epic Games/projects/SwordSwing/ThirdParty/voro++/includes/voro++.hh"
//#include "voro++.cc"

#include <string>
#include <list>

#include "MeshGenerator.h"
#include "Triangulation.h"
#include "MCTriangulator.h"
#include "ScalarField.h"
#include "LevelSet.h"

//#include "utilities/OMath.h"


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

	//container test(-100, 100, -100, 100, -100, 100, 6, 6, 6, false, false, false, 8);
	FVector con_dims(50.f, 50.f, 50.f);
	voro::container con(-con_dims.X, con_dims.X, -con_dims.Y, con_dims.Y, -con_dims.Z, con_dims.Z, 6, 6, 6, false, false, false, 8);
	std::vector<FVector> v_particles;
	v_particles.push_back(FVector(40, 0.f, 0.f));
	v_particles.push_back(FVector(0.f, 40.f, 0.f));
	v_particles.push_back(FVector(-40.f, 0.f, 0.f));
	//v_particles.push_back(FVector(50.f, 30.f, 0.f));
	//v_particles.push_back(FVector(50.f, 10.f, 0.f));
	//v_particles.push_back(FVector(90.f, 10.f, 0.f));
	for (int i = 0; i < v_particles.size(); i++)
	{
		DrawDebugPoint(
			GetWorld(),
			v_particles[i],
			10,  					//size
			FColor(255, 0, 255),  //pink
			true,  				//persistent (never goes away)
			0.0 					//point leaves a trail on moving object
		);
		con.put(i, v_particles[i].X, v_particles[i].Y, v_particles[i].Z);
	}
		

	std::vector<voro::voronoicell_neighbor> v_cells(v_particles.size());
	voro::voronoicell_neighbor tmp_cell;
	int loop_counter = 0;
	voro::c_loop_all cl(con);
	if (cl.start())
	{
		double x, y, z;
		//do if(con.compute_cell(tmp_cell, cl)){
		do if (con.compute_cell(v_cells[loop_counter], cl)) {
			cl.pos(x, y, z);
			v_particles[loop_counter] = FVector(x, y, z);
			loop_counter++;
		} while (cl.inc());
	}
	
	//{
	//	Voronoi v;
	//	TArray<FVector2D> tmp_voro_sites;
	//	tmp_voro_sites.Add(FVector2D(50.f, 95.f));
	//	tmp_voro_sites.Add(FVector2D(5.f, 55.f));
	//	tmp_voro_sites.Add(FVector2D(95.f, 50.f));
	//	tmp_voro_sites.Add(FVector2D(50.f, 30.f));
	//	tmp_voro_sites.Add(FVector2D(50.f, 10.f));
	//	tmp_voro_sites.Add(FVector2D(90.f, 10.f));

	//	//tmp_voro_sites.Add(FVector2D(50.f, 95.f));
	//	//tmp_voro_sites.Add(FVector2D(55.f, 90.f));
	//	//tmp_voro_sites.Add(FVector2D(45.f, 65.f));

	//
	//	//tmp_voro_sites.Add(FVector2D(50.f, 30.f));

	//	v.setDims(100.f, 100.f);
	//	v.CalculateDiagram(&tmp_voro_sites);

	//	{
	//		DrawDebugLine(
	//			GetWorld(),
	//			FVector(0.f, 0.f, 300.f),
	//			FVector(0.f, v.getDims().Y, 300.f), 					//size
	//			FColor(255, 255, 255),  //pink
	//			true,  				//persistent (never goes away)
	//			0.0, 					//point leaves a trail on moving object
	//			10,
	//			1.f
	//		);

	//		DrawDebugLine(
	//			GetWorld(),
	//			FVector(0.f, v.getDims().Y, 300.f),
	//			FVector(v.getDims().X, v.getDims().Y, 300.f), 					//size
	//			FColor(255, 255, 255),  //pink
	//			true,  				//persistent (never goes away)
	//			0.0, 					//point leaves a trail on moving object
	//			10,
	//			1.f
	//		);

	//		DrawDebugLine(
	//			GetWorld(),
	//			FVector(v.getDims().X, v.getDims().Y, 300.f),
	//			FVector(v.getDims().X, 0.f, 300.f), 					//size
	//			FColor(255, 255, 255),  //pink
	//			true,  				//persistent (never goes away)
	//			0.0, 					//point leaves a trail on moving object
	//			10,
	//			1.f
	//		);

	//		DrawDebugLine(
	//			GetWorld(),
	//			FVector(v.getDims().X, 0.f, 300.f),
	//			FVector(0.f, 0.f, 300.f), 					//size
	//			FColor(255, 255, 255),  //pink
	//			true,  				//persistent (never goes away)
	//			0.0, 					//point leaves a trail on moving object
	//			10,
	//			1.f
	//		);
	//	}

	//	std::vector<VSite>* v_sites = v.getSites();
	//	int counter = 0;
	//	for (const auto &v_site : *v_sites)
	//	{
	//		FVector site3D = FVector(v_site.pos.X, v_site.pos.Y, 300.f);
	//		DrawDebugPoint(
	//			GetWorld(),
	//			site3D,
	//			10,  					//size
	//			FColor(counter*50, 0, (v_sites->size() -counter)*50),  //pink
	//			true,  				//persistent (never goes away)
	//			0.0 					//point leaves a trail on moving object
	//		);
	//		//if (counter == 2)
	//		{
	//			for (const auto &edge : v_site.edges)
	//			{
	//				FVector start3D = FVector(edge->start.X, edge->start.Y, 300.f);
	//				FVector end3D = FVector(edge->end.X, edge->end.Y, 300.f);
	//				FVector dir3D = FVector(edge->direction.X, edge->direction.Y, 0.f)*100.f;
	//				FVector right3D = FVector(edge->right->pos, 300.f) - start3D;
	//				DrawDebugLine(
	//					GetWorld(),
	//					start3D,
	//					end3D,
	//					//start3D + dir3D, 					//size
	//					FColor(counter * 50, 0, (v_sites->size() - counter) * 50),  //pink
	//					true,  				//persistent (never goes away)
	//					0.0, 					//point leaves a trail on moving object
	//					10,
	//					0.5f
	//				);

	//				//DrawDebugLine(
	//				//	GetWorld(),
	//				//	start3D,
	//				//	start3D + right3D, 					//size
	//				//	FColor(255, 255, 255),  //pink
	//				//	true,  				//persistent (never goes away)
	//				//	0.0, 					//point leaves a trail on moving object
	//				//	10,
	//				//	1.f
	//				//);
	//			}
	//		}
	//	
	//		counter++;
	//	}
	//}
	
	//int i, j, k, l, m;
	//bool e_found = false;
	//for (i = 1; i < v_cells[0].p; i++) {
	//	for (j = 0; j < v_cells[0].nu[i]; j++) {
	//		k = v_cells[0].ed[i][j];
	//		if (k >= 0) {
	//			FVector v1 = FVector(0.5*v_cells[0].pts[3 * i], 0.5*v_cells[0].pts[3 * i + 1], 0.5*v_cells[0].pts[3 * i + 2]);
	//			l = i; m = j;
	//			do {
	//				v_cells[0].ed[k][v_cells[0].ed[l][v_cells[0].nu[l] + m]] = -1 - l;
	//				v_cells[0].ed[l][m] = -1 - k;
	//				l = k;
	//				FVector v2 = FVector(0.5*v_cells[0].pts[3 * k], 0.5*v_cells[0].pts[3 * k + 1], 0.5*v_cells[0].pts[3 * k + 2]);

	//				if(i == 9 )
	//				DrawDebugLine(
	//					GetWorld(),
	//					v1,
	//					v2, 					//size
	//					FColor(255, 255, 255),  //pink
	//					true,  				//persistent (never goes away)
	//					0.0, 					//point leaves a trail on moving object
	//					10,
	//					1.f
	//				);

	//				v1 = v2;
	//				e_found = false;
	//				for (m = 0; m < v_cells[0].nu[l]; m++) {
	//					k = v_cells[0].ed[l][m];
	//					if (k >= 0) {
	//						e_found = true;
	//						break;
	//					}
	//				}
	//			} while (e_found);
	//		}
	//	}
	//}
	//int i2, j2;
	//for (i2 = 0; i2 < v_cells[0].p; i2++){
	//	for (j2 = 0; j2<v_cells[0].nu[i2]; j2++) {
	//		if (v_cells[0].ed[i2][j2] >= 0)
	//			UE_LOG(LogTemp, Warning, TEXT("oh no"));

	//		v_cells[0].ed[i2][j2] = -1 - v_cells[0].ed[i2][j2];
	//	}
	//}

	DrawDebugPoint(
		GetWorld(),
		FVector(0.f, 0.f, 0.f),
		20,  					//size
		FColor(255, 255, 0),  //pink
		true,  				//persistent (never goes away)
		0.0 					//point leaves a trail on moving object
	);

	for (int i = 0; i < v_cells.size(); i++)
	{
		std::vector<double> vert_test;
		v_cells[i].vertices(v_particles[i].X, v_particles[i].Y, v_particles[i].Z, vert_test);
		//v_cells[i].vertices(vert_test);

		std::vector<int> edge_test;
		int nroe = v_cells[i].number_of_edges();
		v_cells[i].edges(edge_test);

		std::vector<int> face_test;
		v_cells[i].face_vertices(face_test);

		std::vector<int> face_orders_test;
		v_cells[i].face_orders(face_orders_test);
		//for (int j = 0; j < face_orders_test[0]; j = j + 1)
		//{
		//	FVector v1 = FVector(vert_test[face_test[j] * 3], vert_test[face_test[j] * 3 + 1], vert_test[face_test[j] * 3 + 2]);
		//	DrawDebugPoint(GetWorld(),v1,10,FColor(255, 0, 0),true,0.0);
		//}


		//if (i == 2)
		{
			for (int j = 0; j < edge_test.size(); j = j + 2)
			{
				float x1 = vert_test[edge_test[j] * 3];
				float y1 = vert_test[edge_test[j] * 3 + 1];
				float z1 = vert_test[edge_test[j] * 3 + 2];
				FVector v1 = FVector(x1, y1, z1);
				float x2 = vert_test[edge_test[j + 1] * 3];
				float y2 = vert_test[edge_test[j + 1] * 3 + 1];
				float z2 = vert_test[edge_test[j + 1] * 3 + 2];
				FVector v2 = FVector(x2, y2, z2);
				DrawDebugLine(GetWorld(), v1, v2, FColor(255, 255, 255), true, 0.0, 10, 1.f);
			}
		}

	}
	

	float v_point_interval = FMath::Sqrt(v_cells[0].max_radius_squared()) / 32;
	ScalarField<float> v_sf(32, FMath::Sqrt(v_cells[0].max_radius_squared()) + 2* v_point_interval);
	v_sf.voronoiCellSignedDist(&v_cells[0], &con, 0, v_particles, con_dims, v_particles[0]);
	//v_sf.drawBounds(GetWorld());
	//v_sf.drawScalars(GetWorld());

	LevelSet v_ls(32, FMath::Sqrt(v_cells[0].max_radius_squared()) + 2 * v_point_interval);
	v_ls.voronoiCellSignedDist(&v_cells[0], &con, 0, v_particles, con_dims, v_particles[0]);
	//v_ls.drawBounds(GetWorld());
	//v_ls.drawScalars(GetWorld());

	mesh_frags.Add(ConstructObject<UProceduralMeshComponent>(UProceduralMeshComponent::StaticClass(), this, FName("test")));
	mesh_frags[0]->RegisterComponent();
	mesh_frags[0]->AttachTo(baseModel);
	mesh_frags[0]->InitializeComponent();
	mesh_frags[0]->bUseComplexAsSimpleCollision = false;
	mesh_frags[0]->SetMaterial(0, base_material);
	//triangulation::marchingCubes(mesh_frags[0], &v_sf);
	MCTriangulator mc_tri;
	mc_tri.marchingCubes(mesh_frags[0], v_ls.getScalarField(), v_ls.getIsoVal());

	ScalarField<float> vd_sf(32, con_dims*2.f);
	vd_sf.voronoiDiagramSignedDist(&v_cells, v_particles, con_dims);
	//vd_sf.drawBounds(GetWorld());
	//vd_sf.drawScalarSections(GetWorld());
	//Triangulator mc_tri;
	//mc_tri.marchingCubes(mesh_frags[0], &v_sf);
	LevelSet vd_ls(32, con_dims.X*2.f);
	vd_ls.voronoiDiagramSignedDist(&v_cells, v_particles, con_dims);
	vd_ls.drawBounds(GetWorld());
	vd_ls.drawScalarSections(GetWorld());

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
	baseModel->ToggleVisibility();
}

// Called every frame
void AMeshGenerator::Tick( float DeltaTime )
{
	Super::Tick( DeltaTime );
}

void AMeshGenerator::OnOriginalModelHit(UPrimitiveComponent* HitComp, AActor* OtherActor, UPrimitiveComponent* OtherComp, FVector NormalImpulse, const FHitResult& Hit)
{	
	return;
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
