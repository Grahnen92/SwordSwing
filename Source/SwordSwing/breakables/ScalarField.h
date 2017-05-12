// Fill out your copyright notice in the Description page of Project Settings.
#pragma once

#ifndef SCALARFIELD_H
#define SCALARFIELD_H

#include "SwordSwing.h"
#include <memory>
#include "math.h"
#include <vector>
#include "ProceduralMeshComponent.h"

#include "voronoi/Voronoi.h"
#include "C:/Program Files (x86)/Epic Games/projects/SwordSwing/ThirdParty/voro++/includes/voro++.hh"

#include "utilities/OMath.h"
#include "RawMesh.h"
#include <limits>
#include <iostream>

/**
 * 
 */


template <typename scalar_value>
class SWORDSWING_API ScalarField
{

public:
	ScalarField()
	{
		res = FVector(0.f, 0.f, 0.f);
		dim = FVector(0.f, 0.f, 0.f);
		offset = FVector(0.f, 0.f, 0.f);
	}
	ScalarField(int _cubed_res)
	{
		res = FVector(_cubed_res, _cubed_res, _cubed_res);
		dim = FVector(1.f, 1.f, 1.f);
		offset = FVector(0.f, 0.f, 0.f);
		allocateData();
	}

	ScalarField(int _cubed_res, float _cubed_dim)
	{
		res = FVector(_cubed_res, _cubed_res, _cubed_res);
		dim = FVector(_cubed_dim, _cubed_dim, _cubed_dim);
		offset = FVector(0.f, 0.f, 0.f);
		allocateData();
	}
	ScalarField(FVector _res, float _cubed_dim)
	{
		res = _res;
		dim = FVector(_cubed_dim, _cubed_dim, _cubed_dim);
		offset = FVector(0.f, 0.f, 0.f);
		allocateData();
	}
	ScalarField(int _cubed_res, FVector _dim)
	{
		res = FVector(_cubed_res, _cubed_res, _cubed_res);
		dim = _dim;
		offset = FVector(0.f, 0.f, 0.f);
		allocateData();
	}
	ScalarField(FVector _res, FVector _dim)
	{
		res = _res;
		dim = _dim;
		offset = FVector(0.f, 0.f, 0.f);
		allocateData();
	}




	~ScalarField()
	{
		deAllocateData();
	}

	FVector getRes()
	{
		return res;
	}
	FVector* getResPtr()
	{
		return &res;
	}
	void setRes(FVector _res)
	{
		res = _res;
		allocateData();
	}
	FVector getDims()
	{
		return dim;
	}
	FVector* getDimsPtr()
	{
		return &dim;
	}
	void setDims(FVector _dims)
	{
		dim = _dims;
		//allocateData();
	}
	scalar_value getIsoValue()
	{
		return iso_value;
	}
	void setIsoValue(scalar_value _iv)
	{
		iso_value = _iv;
	}

	void setLargestValue(scalar_value _v)
	{
		largest_value = _v;
	}

	scalar_value getLargestValue()
	{
		return largest_value;
	}

	scalar_value* getLargestValuePtr()
	{
		return &largest_value;
	}

	scalar_value*** getDataPtr()
	{
		return data;
	}
	scalar_value** operator[](int i)
	{
		return data[i];
	}
	scalar_value getValue(int _x, int _y, int _z)
	{
		return data[_x][_y][_z];
	}

	unsigned char*** getSectionPtr()
	{
		return section;
	}


	void setAllValues(scalar_value _v)
	{
		for (int i = 0; i < res.X; i++)
		{
			for (int j = 0; j < res.Y; j++)
			{
				for (int k = 0; k < res.Z; k++) {
					if (i == 0 || j == 0 || k == 0 || i == res.X - 1 || j == res.Y - 1 || k == res.Z - 1) {
						data[i][j][k] = 0.0f;
					}
					else {
						data[i][j][k] = 255.0f;
					}
				}
			}
		}
	}

	void setHalfOfAllValues(scalar_value _v)
	{
		for (int i = 0; i < res.X; i++)
		{
			for (int j = 0; j < res.Y; j++)
			{
				for (int k = 0; k < res.Z; k++) {
					if (i == 0 || j == 0 || k == 0 || i == res.X - 1 || j == res.Y - 1 || k == res.Z - 1) {
						data[i][j][k] = 0.0f;
					}
					else if (k > res.Z / 2) {
						data[i][j][k] = 255.0f;
					}
					else
					{
						data[i][j][k] = 0.0f;
					}
				}
			}
		}
	}

	void setAsCosFun(scalar_value _v)
	{
		for (int i = 0; i < res.X; i++)
		{
			for (int j = 0; j < res.Y; j++)
			{
				for (int k = 0; k < res.Z; k++) {
					if (i == 0 || j == 0 || k == 0 || i == res.X - 1 || j == res.Y - 1 || k == res.Z - 1) {
						data[i][j][k] = 0.0f;
					}
					else if (k > std::cos(i)*res.X / 2) {
						data[i][j][k] = 255.0f;
					}
					else
					{
						data[i][j][k] = 0.0f;
					}
				}
			}
		}
	}

	void setAsWedge(scalar_value _v)
	{
		for (int i = 0; i < res.X; i++)
		{
			for (int j = 0; j < res.Y; j++)
			{
				for (int k = 0; k < res.Z; k++) {
					x = ((i - (res.X / 2)) / res.X)*dim.X + (dim.X / res.X)*0.5f + _mid_point.X;
					y = ((j - (res.Y / 2)) / res.Y)*dim.Y + (dim.Y / res.Y)*0.5f + _mid_point.Y;
					z = ((k - (res.Z / 2)) / res.Z)*dim.Z + (dim.Z / res.Z)*0.5f + _mid_point.Z;
				}
			}
		}
	}

	void sphereSignedDistance(FVector _offset)
	{
		offset = _offset;
		//TODO: this is unsafe if the resolution or dimension are not cubed
		float radius = dim.X / 2.0f - (dim.X / res.X);
		FVector origin(0.0f, 0.0f, 0.0f);
		FVector p1;
		FVector p2;
		FVector dist_vec;
		float dist;

		float x, y, z;
		for (int i = 0; i < res.X; i++)
		{
			for (int j = 0; j < res.Y; j++)
			{
				for (int k = 0; k < res.Z; k++) 
				{
					x = ((i - (res.X / 2)) / res.X)*dim.X + (dim.X / res.X)*0.5f;
					y = ((j - (res.Y / 2)) / res.Y)*dim.Y + (dim.Y / res.Y)*0.5f;
					z = ((k - (res.Z / 2)) / res.Z)*dim.Z + (dim.Z / res.Z)*0.5f;

					p1 = FVector(x, y, z);
					p2 = p1;
					p2.Normalize();
					p2 = p2*radius;

					dist_vec = p2 - p1;
					dist = dist_vec.Size();
					dist_vec.Normalize();

					if (FVector::DotProduct(p2, dist_vec) < 0)
					{
						data[i][j][k] = -dist;
					}
					else
					{
						data[i][j][k] = dist;
					}



				}
			}
		}
	}

	void cubeSignedDistance(FVector _offset)
	{
		offset = _offset;
		//TODO: this is unsafe if the resolution or dimension are not cubed
		//cube_half_length
		float chl = dim.X / 2.0f - (dim.X / res.X);
		FVector origin(0.0f, 0.0f, 0.0f);
		FVector p1;
		FVector p2;
		FVector dist_vec;

		float x, y, z;
		for (int i = 0; i < res.X; i++)
		{
			for (int j = 0; j < res.Y; j++)
			{
				for (int k = 0; k < res.Z; k++)
				{
					x = ((i - (res.X / 2)) / res.X)*dim.X + (dim.X / res.X)*0.5f;
					y = ((j - (res.Y / 2)) / res.Y)*dim.Y + (dim.Y / res.Y)*0.5f;
					z = ((k - (res.Z / 2)) / res.Z)*dim.Z + (dim.Z / res.Z)*0.5f;

					float x_dist_max = x - chl;
					float x_dist_min = x - (-chl);
					float x_dist;
					if (FMath::Abs(x_dist_max) < FMath::Abs(x_dist_min))
					{
						x_dist = x_dist_max;
					}
					else 
					{
						x_dist = -x_dist_min;
					}
					x_dist = -x_dist;

					float y_dist_max = y - chl;
					float y_dist_min = y - (-chl);
					float y_dist;
					if (FMath::Abs(y_dist_max) < FMath::Abs(y_dist_min))
					{
						y_dist = y_dist_max;
					}
					else
					{
						y_dist = -y_dist_min;
					}
					y_dist = -y_dist;

					float z_dist_max = z - chl;
					float z_dist_min = z - (-chl);
					float z_dist;
					if (FMath::Abs(z_dist_max) < FMath::Abs(z_dist_min))
					{
						z_dist = z_dist_max;
					}
					else
					{
						z_dist = -z_dist_min;
					}
					z_dist = -z_dist;

					data[i][j][k] = FMath::Min3(x_dist, y_dist, z_dist);
				}
			}
		}
	}

	void voronoiCellSignedDist(voro::voronoicell_neighbor* _v_cell, voro::container* _vcon, const int& _cell_id, const std::vector<FVector> &_v_particles, FVector &_vd_dims,  const  FVector& _offset = FVector(0.f, 0.f, 0.f))
	{
		float x, y, z;
		float ox = (dim.X / res.X)*0.5f;
		float oy = (dim.Y / res.Y)*0.5f;
		float oz = (dim.Z / res.Z)*0.5f;
		float resxm1d2 = ((res.X - 1) / 2.f);
		float resym1d2 = ((res.Y - 1) / 2.f);
		float reszm1d2 = ((res.Z - 1) / 2.f);
		float resxm1 = (res.X - 1);
		float resym1 = (res.Y - 1);
		float reszm1 = (res.Z - 1);
		
		//Verts
		std::vector<double> cell_verts;
		//_v_cell->vertices(_offset.X, _offset.Y, _offset.Z, cell_verts);
		_v_cell->vertices(cell_verts);
		//edges
		std::vector<int> cell_edges;
		_v_cell->edges(cell_edges);
		//faces
		std::vector<int> cell_faces;
		_v_cell->face_vertices(cell_faces);
		std::vector<int> cell_face_orders;
		_v_cell->face_orders(cell_face_orders);
		std::vector<double> cell_face_normals;
		_v_cell->normals(cell_face_normals);


		offset = _offset;
		//int tmp_cell_id;
		FVector curr_p;
		
		largest_value = 0.f;

		for (int i = 0; i < res.X; i++)
		{
			for (int j = 0; j < res.Y; j++)
			{
				for (int k = 0; k < res.Z; k++)
				{
					x = ((i - resxm1d2) / resxm1)*dim.X;// +offset.X;
					y = ((j - resym1d2) / resym1)*dim.Y;// + offset.Y;
					z = ((k - reszm1d2) / reszm1)*dim.Z;// +offset.Z;
					curr_p = FVector(x, y, z);

					data[i][j][k] = OMath::distToVoronoiCell(curr_p, cell_verts, cell_edges, cell_faces, cell_face_orders, cell_face_normals);
					if (FMath::Abs(data[i][j][k]) > largest_value)
						largest_value = data[i][j][k];

					curr_p = curr_p + offset;
					/*double rx, ry, rz;
					if (!_vcon->find_voronoi_cell(curr_p.X, curr_p.Y, curr_p.Z, rx, ry, rz, tmp_cell_id))
						data[i][j][k] = -data[i][j][k];
					else if (tmp_cell_id != _cell_id)
						data[i][j][k] = -data[i][j][k];*/
					if(!OMath::pointIsInsideCube(curr_p, FVector::ZeroVector, _vd_dims))
						data[i][j][k] = -data[i][j][k];
					else if (OMath::closestPoint(curr_p, _v_particles) != _cell_id)
						data[i][j][k] = -data[i][j][k];
					
					//UE_LOG(LogTemp, Warning, TEXT("aweawe %d"), tmp_cell_id);

				}
			}
		}
	}

	void voronoiDiagramSignedDist(std::vector<voro::voronoicell_neighbor> *_v_cells, const std::vector<FVector> &_v_particles, FVector &_vd_dims, const  FVector& _offset = FVector(0.f, 0.f, 0.f))
	{
		float x, y, z;
		float ox = (dim.X / res.X)*0.5f;
		float oy = (dim.Y / res.Y)*0.5f;
		float oz = (dim.Z / res.Z)*0.5f;
		float resxm1d2 = ((res.X - 1) / 2.f);
		float resym1d2 = ((res.Y - 1) / 2.f);
		float reszm1d2 = ((res.Z - 1) / 2.f);
		float resxm1 = (res.X - 1);
		float resym1 = (res.Y - 1);
		float reszm1 = (res.Z - 1);

		allocateSectionData();

		//Verts
		std::vector<std::vector<double>> cell_verts(_v_cells->size());
		//edges
		std::vector<std::vector<int>> cell_edges(_v_cells->size());
		//faces
		std::vector<std::vector<int>> cell_faces(_v_cells->size());
		std::vector<std::vector<int>> cell_face_orders(_v_cells->size());
		std::vector<std::vector<double>> cell_face_normals(_v_cells->size());
		

		for (int i = 0; i < _v_cells->size(); i++)
		{
			(*_v_cells)[i].vertices(_v_particles[i].X, _v_particles[i].Y, _v_particles[i].Z, cell_verts[i]);
			(*_v_cells)[i].edges(cell_edges[i]);
			(*_v_cells)[i].face_vertices(cell_faces[i]);
			(*_v_cells)[i].face_orders(cell_face_orders[i]);
			(*_v_cells)[i].normals(cell_face_normals[i]);
		}
		


		offset = _offset;
		//int tmp_cell_id;
		FVector curr_p;
		int cell_id;

		largest_value = 0.f;

		for (int i = 0; i < res.X; i++)
		{
			for (int j = 0; j < res.Y; j++)
			{
				for (int k = 0; k < res.Z; k++)
				{
					x = ((i - resxm1d2) / resxm1)*dim.X;// +offset.X;
					y = ((j - resym1d2) / resym1)*dim.Y;// + offset.Y;
					z = ((k - reszm1d2) / reszm1)*dim.Z;// +offset.Z;

					curr_p = FVector(x, y, z);
					cell_id = OMath::closestPoint(curr_p, _v_particles);
					section[i][j][k] = cell_id;
					
					data[i][j][k] = OMath::distToVoronoiCell(curr_p, cell_verts[cell_id], cell_edges[cell_id], cell_faces[cell_id], cell_face_orders[cell_id], cell_face_normals[cell_id]);
					if (FMath::Abs(data[i][j][k]) > largest_value)
						largest_value = data[i][j][k];
					//UE_LOG(LogTemp, Warning, TEXT("aweawe %d"), tmp_cell_id);

				}
			}
		}
	}

	void meshToLeveSet(FRawMesh* _rm, FVector& _offset)
	{
		offset = _offset;
		float x, y, z;
		float ox = (dim.X / res.X)*0.5f;
		float oy = (dim.Y / res.Y)*0.5f;
		float oz = (dim.Z / res.Z)*0.5f;
		float resxm1d2 = ((res.X - 1) / 2.f);
		float resym1d2 = ((res.Y - 1) / 2.f);
		float reszm1d2 = ((res.Z - 1) / 2.f);
		float resxm1 = (res.X - 1);
		float resym1 = (res.Y - 1);
		float reszm1 = (res.Z - 1);

		/*scalar_value max = -std::numeric_limits<scalar_value>::infinity();
		scalar_value min = -max;*/

		for (int i = 0; i < res.X; i++)
		{
			for (int j = 0; j < res.Y; j++)
			{
				for (int k = 0; k < res.Z; k++) {
					//if (i == 0 || j == 0 || k == 0 || i == res.X - 1 || j == res.Y - 1 || k == res.Z - 1) 
					//{
					//	//data[i][j][k] = -1000.0f;
					//}
					//else
					//{
					x = ((i - resxm1d2) / resxm1)*dim.X + offset.X;
					y = ((j - resym1d2) / resym1)*dim.Y + offset.Y;
					z = ((k - reszm1d2) / reszm1)*dim.Z + offset.Z;
					//z = ((k ) / res.Z)*dim.Z + (dim.Z / res.Z)*0.5f;
					data[i][j][k] = -distanceToMesh(_rm, FVector(x, y, z));


					/*if (data[i][j][k] > max)
					{
					max = data[i][j][k];
					}
					else if (data[i][j][k] < min)
					{
					min = data[i][j][k];
					}*/

					/*if(data[i][j][k] > 0)
					{
					data[i][j][k] = 0.0f;
					}
					else
					{
					data[i][j][k] = 255.0f;
					}*/
					//}
				}
			}
		}
	}

	//_rel_transform is the transform that puts the origin of sf2 at a certain point relative to the origin of sf1
	static void mergeLevelSets(ScalarField* _sf1, ScalarField* _sf2, FMatrix _rel_rotation, FVector rel_position, FVector frag_offset, ScalarField* _sf_out)
	{
		
		_sf_out->res = _sf2->res;
		_sf_out->dim = _sf2->dim;
		_sf_out->offset = _sf2->offset;
		_sf_out->iso_value = _sf2->iso_value;
		_sf_out->reAllocateData();

		FVector xyz1;
		//int i1, j1, k1;
		float resxm1d2_1 = ((_sf1->res.X - 1) / 2.f);
		float resym1d2_1 = ((_sf1->res.Y - 1) / 2.f);
		float reszm1d2_1 = ((_sf1->res.Z - 1) / 2.f);
		float resxm1_1 = (_sf1->res.X - 1);
		float resym1_1 = (_sf1->res.Y - 1);
		float reszm1_1 = (_sf1->res.Z - 1);
		float interp_val_1;


		FVector xyz2;
		float resxm1d2_2 = ((_sf2->res.X - 1) / 2.f);
		float resym1d2_2 = ((_sf2->res.Y - 1) / 2.f);
		float reszm1d2_2 = ((_sf2->res.Z - 1) / 2.f);
		float resxm1_2 = (_sf2->res.X - 1);
		float resym1_2 = (_sf2->res.Y - 1);
		float reszm1_2 = (_sf2->res.Z - 1);


		float x, y, z;
		for (int i2 = 0; i2 < _sf2->res.X; i2++)
		{
			for (int j2 = 0; j2 < _sf2->res.Y; j2++)
			{
				for (int k2 = 0; k2 < _sf2->res.Z; k2++)
				{
					//inside fragment
					if (_sf2->data[i2][j2][k2] >= _sf2->iso_value) 
					{
						x = ((i2 - resxm1d2_2) / resxm1_2)*_sf2->dim.X + _sf2->offset.X;
						y = ((j2 - resym1d2_2) / resym1_2)*_sf2->dim.Y + _sf2->offset.Y;
						z = ((k2 - reszm1d2_2) / reszm1_2)*_sf2->dim.Z + _sf2->offset.Z;



						xyz2 = FVector(x, y, z) + frag_offset;

						xyz1 = _rel_rotation.TransformPosition(xyz2) + rel_position;
						interp_val_1 = _sf1->getTLIValue(xyz1);
							//inside original model
						if (interp_val_1 >= _sf1->iso_value) 
						{
							_sf_out->data[i2][j2][k2] = std::fmin(interp_val_1, _sf2->data[i2][j2][k2]);
						}
						else
						{
							_sf_out->data[i2][j2][k2] = interp_val_1;
						}
		
					}
					else
					{
						_sf_out->data[i2][j2][k2] = _sf2->data[i2][j2][k2];
					}
				}
			}
		}
	}


	scalar_value getTLIValue(FVector _pos)
	{
		FVector data_pos = transform2BoundedDataPos(_pos);
		//integer positions
		int x0 = FMath::Floor(data_pos.X);
		int x1 = FMath::Ceil(data_pos.X);
		int y0 = FMath::Floor(data_pos.Y);
		int y1 = FMath::Ceil(data_pos.Y);
		int z0 = FMath::Floor(data_pos.Z);
		int z1 = FMath::Ceil(data_pos.Z);
		//differentials
		float xd = data_pos.X - x0;
		float yd = data_pos.Y - y0;
		float zd = data_pos.Z - z0;
		//interpolate along x
		float v00 = data[x0][y0][z0] *(1 - xd) + data[x1][y0][z0] *xd;
		float v01 = data[x0][y0][z1] *(1 - xd) + data[x1][y0][z1] *xd;
		float v10 = data[x0][y1][z0] *(1 - xd) + data[x1][y1][z0] *xd;
		float v11 = data[x0][y1][z1] *(1 - xd) + data[x1][y1][z1] *xd;
		//interpolate along y
		float v0 = v00*(1 - yd) + v10*yd;
		float v1 = v01*(1 - yd) + v11*yd;
		//interpolate along z
		float v = v0*(1 - zd) + v1*zd;
		
		return v;
	}

	

	void drawBounds(UWorld * _world, const FVector& _offset = FVector(0.f, 0.f, 0.f)) {

		FVector v1 = FVector(dim.X, dim.Y, dim.Z)*0.5f + _offset;
		FVector v2 = FVector(-dim.X, dim.Y, dim.Z)*0.5f + _offset;
		FVector v3 = FVector(-dim.X, -dim.Y, dim.Z)*0.5f + _offset;
		FVector v4 = FVector(dim.X, -dim.Y, dim.Z)*0.5f + _offset;

		FVector v5 = FVector(dim.X, dim.Y, -dim.Z)*0.5f + _offset;
		FVector v6 = FVector(-dim.X, dim.Y, -dim.Z)*0.5f + _offset;
		FVector v7 = FVector(-dim.X, -dim.Y, -dim.Z)*0.5f + _offset;
		FVector v8 = FVector(dim.X, -dim.Y, -dim.Z)*0.5f + _offset;

		DrawDebugLine(_world, v1, v2, FColor(255, 255, 255), true,0.0,10,1.f);
		DrawDebugLine(_world, v2, v3, FColor(255, 255, 255), true, 0.0, 10, 1.f);
		DrawDebugLine(_world, v3, v4, FColor(255, 255, 255), true, 0.0, 10, 1.f);
		DrawDebugLine(_world, v4, v1, FColor(255, 255, 255), true, 0.0, 10, 1.f);

		DrawDebugLine(_world, v1, v5, FColor(255, 255, 255), true, 0.0, 10, 1.f);
		DrawDebugLine(_world, v2, v6, FColor(255, 255, 255), true, 0.0, 10, 1.f);
		DrawDebugLine(_world, v3, v7, FColor(255, 255, 255), true, 0.0, 10, 1.f);
		DrawDebugLine(_world, v4, v8, FColor(255, 255, 255), true, 0.0, 10, 1.f);

		DrawDebugLine(_world, v5, v6, FColor(255, 255, 255), true, 0.0, 10, 1.f);
		DrawDebugLine(_world, v6, v7, FColor(255, 255, 255), true, 0.0, 10, 1.f);
		DrawDebugLine(_world, v7, v8, FColor(255, 255, 255), true, 0.0, 10, 1.f);
		DrawDebugLine(_world, v8, v5, FColor(255, 255, 255), true, 0.0, 10, 1.f);

	}

	void drawScalars(UWorld * _world, const FVector& _offset = FVector(0.f, 0.f, 0.f)) {
		float x, y, z;
		float ox = (dim.X / res.X)*0.5f;
		float oy = (dim.Y / res.Y)*0.5f;
		float oz = (dim.Z / res.Z)*0.5f;
		float resxm1d2 = ((res.X - 1) / 2.f);
		float resym1d2 = ((res.Y - 1) / 2.f);
		float reszm1d2 = ((res.Z - 1) / 2.f);
		float resxm1 = (res.X - 1);
		float resym1 = (res.Y - 1);
		float reszm1 = (res.Z - 1);
		for (int i = 0; i < res.X; i++)
		{
			for (int j = 0; j < res.Y; j++)
			{
				for (int k = 0; k < res.Z; k++) {
					x = ((i - resxm1d2) / resxm1)*dim.X + _offset.X;
					y = ((j - resym1d2) / resym1)*dim.Y + _offset.Y;
					z = ((k - reszm1d2) / reszm1)*dim.Z + _offset.Z;
					FColor col;
					

					float scaled_val = FMath::Abs(data[i][j][k]) / largest_value;
					/*col = FColor(255 * scaled_val, 0, 255 * (1 - scaled_val));*/
					if(data[i][j][k]> 0.f)
						col = FColor(255* scaled_val, 0, 0);
					else
						col = FColor(0, 0, 255*scaled_val);


					DrawDebugPoint(
						_world,
						FVector(x,y,z),
						2,//size
						col,
						true, //persistent (never goes away)
						0.0 //point leaves a trail on moving object
					);
				}
			}
		}

	}

	void drawScalarSections(UWorld * _world, const FVector& _offset = FVector(0.f, 0.f, 0.f)) {
		float x, y, z;
		float ox = (dim.X / res.X)*0.5f;
		float oy = (dim.Y / res.Y)*0.5f;
		float oz = (dim.Z / res.Z)*0.5f;
		float resxm1d2 = ((res.X - 1) / 2.f);
		float resym1d2 = ((res.Y - 1) / 2.f);
		float reszm1d2 = ((res.Z - 1) / 2.f);
		float resxm1 = (res.X - 1);
		float resym1 = (res.Y - 1);
		float reszm1 = (res.Z - 1);
		for (int i = 0; i < res.X; i++)
		{
			for (int j = 0; j < res.Y; j++)
			{
				for (int k = 0; k < res.Z; k++) {
					x = ((i - resxm1d2) / resxm1)*dim.X + _offset.X;
					y = ((j - resym1d2) / resym1)*dim.Y + _offset.Y;
					z = ((k - reszm1d2) / reszm1)*dim.Z + _offset.Z;
					FColor col;


					float scaled_val = FMath::Abs(data[i][j][k]) / largest_value;
					/*col = FColor(255 * scaled_val, 0, 255 * (1 - scaled_val));*/
					if (section[i][j][k] == (char)0)
						col = FColor(255 * scaled_val, 0, 0);
					else if(section[i][j][k] == (char)1)
						col = FColor(0, 0, 255 * scaled_val);
					else if(section[i][j][k] == (char)2)
						col = FColor(0, 255 * scaled_val, 0);


					DrawDebugPoint(
						_world,
						FVector(x, y, z),
						2,//size
						col,
						true, //persistent (never goes away)
						0.0 //point leaves a trail on moving object
					);
				}
			}
		}

	}
	void allocateData() {
		if (data)
		{
			UE_LOG(LogTemp, Warning, TEXT("data already allocated, returning."));
			std::cout << "data already allocated, returning.";
			return;
		}
		//data = scalar_u_ptr_3d<scalar_value>(new scalar_u_ptr_2d<scalar_value>[res.X]);
		data = new scalar_value**[res.X];
		for (int i = 0; i < res.X; i++)
		{
			//data[i] = scalar_u_ptr_2d<scalar_value>(new scalar_u_ptr_1d<scalar_value>[res.Y]);
			data[i] = new scalar_value*[res.Y];
			for (int j = 0; j < res.Y; j++)
			{
				//data[i][j] = scalar_u_ptr_1d<scalar_value>(new scalar_value[res.Z]);
				data[i][j] = new scalar_value[res.Z];
			}
		}
	}

	void deAllocateData() {

		if (!data)
		{
			UE_LOG(LogTemp, Warning, TEXT("data already deallocated, returning."));
			std::cout << "data already deallocated, returning.";
			return;
		}

		for (int i = 0; i < res.X; i++)
		{
			for (int j = 0; j < res.Y; j++)
			{
				delete[] data[i][j];
			}
			delete[] data[i];
		}
		delete[] data;

		data = nullptr;
	}

	void reAllocateData() {
		deAllocateData();
		allocateData();
	}

	void allocateSectionData() {
		if (section)
		{
			UE_LOG(LogTemp, Warning, TEXT("sectiondata already allocated, returning."));
			std::cout << "sectiondata already allocated, returning.";
			return;
		}
		//data = scalar_u_ptr_3d<scalar_value>(new scalar_u_ptr_2d<scalar_value>[res.X]);
		section = new unsigned char**[res.X];
		for (int i = 0; i < res.X; i++)
		{
			//data[i] = scalar_u_ptr_2d<scalar_value>(new scalar_u_ptr_1d<scalar_value>[res.Y]);
			section[i] = new unsigned char*[res.Y];
			for (int j = 0; j < res.Y; j++)
			{
				//data[i][j] = scalar_u_ptr_1d<scalar_value>(new scalar_value[res.Z]);
				section[i][j] = new unsigned char[res.Z];
			}
		}
	}

	void deAllocateSectionData() {

		if (!section)
		{
			UE_LOG(LogTemp, Warning, TEXT("sectiondata already deallocated, returning."));
			std::cout << "sectiondata already deallocated, returning.";
			return;
		}

		for (int i = 0; i < res.X; i++)
		{
			for (int j = 0; j < res.Y; j++)
			{
				delete[] section[i][j];
			}
			delete[] section[i];
		}
		delete[] section;

		data = nullptr;
	}

	void reAllocateSectionData() {
		deAllocateSectionData();
		allocateSectionData();
	}

private:
	//scalar_u_ptr_3d<scalar_value> data;
	scalar_value*** data = nullptr;
	unsigned char*** section = nullptr;
	FVector res;
	FVector dim;
	FVector offset;

	scalar_value iso_value;
	scalar_value largest_value;


	float distanceToMesh(FRawMesh* _rm, FVector _p) 
	{
		FVector closest_point_on_tri;
		FVector closest_point_on_mesh;

		float tmp_dist;
		float min_dist;

		int closest_tri_index;

		closest_point_on_tri = FMath::ClosestPointOnTriangleToPoint(_p,
			_rm->VertexPositions[_rm->WedgeIndices[0]],
			_rm->VertexPositions[_rm->WedgeIndices[0 + 1]],
			_rm->VertexPositions[_rm->WedgeIndices[0 + 2]]);

		min_dist = FVector::DistSquared(closest_point_on_tri, _p);
		closest_point_on_mesh = closest_point_on_tri;
		closest_tri_index = 0;


		for (int i = 3; i < _rm->WedgeIndices.Num(); i = i + 3)
		{
			closest_point_on_tri = FMath::ClosestPointOnTriangleToPoint(_p,
				_rm->VertexPositions[_rm->WedgeIndices[i]],
				_rm->VertexPositions[_rm->WedgeIndices[i + 1]],
				_rm->VertexPositions[_rm->WedgeIndices[i + 2]]);

			tmp_dist = FVector::DistSquared(closest_point_on_tri, _p);
			if (tmp_dist < min_dist)
			{
				min_dist = tmp_dist;
				closest_point_on_mesh = closest_point_on_tri;
				closest_tri_index = i;
			}
		}


		FVector closest_vector = closest_point_on_mesh - _p;
		FVector e1 = _rm->VertexPositions[_rm->WedgeIndices[closest_tri_index + 1]] - _rm->VertexPositions[_rm->WedgeIndices[closest_tri_index]];
		FVector e2 = _rm->VertexPositions[_rm->WedgeIndices[closest_tri_index + 2]] - _rm->VertexPositions[_rm->WedgeIndices[closest_tri_index]];
		closest_vector.Normalize();
		if (FVector::DotProduct(closest_vector, FVector::CrossProduct(e1, e2)) < 0)
		{
			return -min_dist;
		}
		else
		{
			return min_dist;
		}

	}

	// Transforms the positition to the integer space of  the data 3D array. Note that the returned 
	// vector still contains floats and these are not rounded to integer values.
	// This particular function will also bound the values, meaning that if the provided position
	// is outside of the data integer space it will set the transformed vector to the closest point 
	// within the data space.
	FVector transform2BoundedDataPos(FVector& _pos)
	{
		//X
		float x = ((_pos.X - offset.X) / dim.X)*(res.X - 1) + (res.X - 1) / 2.0f;
		if (x < 0)
			x = 0;
		else if (x > (res.X -1))
			x = (res.X - 1);
		//Y
		float y = ((_pos.Y - offset.Y) / dim.Y)*(res.Y - 1) + (res.Y - 1) / 2.0f;
		if (y < 0)
			y = 0;
		else if (y >(res.Y - 1))
			y = (res.Y - 1);
		//Z
		float z = ((_pos.Z - offset.Z) / dim.Z)*(res.Z - 1) + (res.Z - 1) / 2.0f;
		if (z < 0)
			z = 0;
		else if (z >(res.Z - 1))
			z = (res.Z - 1);

		return FVector(x, y, z);
	}
};

//namespace ScalarField
//{
//	void mergeLevelSets(ScalarField* _sf1, ScalarField* _sf2, FVector _offset, ScalarField* _sf_out)
//	{
//
//	}
//}
#endif