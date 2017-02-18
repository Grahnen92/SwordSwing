// Fill out your copyright notice in the Description page of Project Settings.
#pragma once

#include "SwordSwing.h"
#include <memory>
#include "math.h"
#include <vector>
#include "ProceduralMeshComponent.h"
#include "RawMesh.h"
#include <limits>

template <typename scalar_value>
using scalar_u_ptr_3d = std::unique_ptr < std::unique_ptr<std::unique_ptr<scalar_value[]>[]>[]>;
template <typename scalar_value>
using scalar_u_ptr_2d = std::unique_ptr < std::unique_ptr<scalar_value[]>[]>;
template <typename scalar_value>
using scalar_u_ptr_1d = std::unique_ptr < scalar_value[]>;

/**
 * 
 */
template <typename scalar_value>
class SWORDSWING_API ScalarField
{

public:
	ScalarField() 
	{
		res = FVector(10.f, 10.f, 10.f);
		dim = FVector(1.f, 1.f, 1.f);
		allocateData();
	}
	ScalarField(int _cubed_res)
	{
		res = FVector(_cubed_res, _cubed_res, _cubed_res);
		dim = FVector(1.f, 1.f, 1.f);
		allocateData();
	}

	ScalarField(int _cubed_res, float _cubed_dim)
	{
		res = FVector(_cubed_res, _cubed_res, _cubed_res);
		dim = FVector(_cubed_dim, _cubed_dim, _cubed_dim);
		allocateData();
	}
	ScalarField(FVector _res, float _cubed_dim)
	{
		res = _res;
		dim = FVector(_cubed_dim, _cubed_dim, _cubed_dim);
		allocateData();
	}
	ScalarField(int _cubed_res, FVector _dim)
	{
		res = FVector(_cubed_res, _cubed_res, _cubed_res);
		dim = _dim;
		allocateData();
	}
	ScalarField(FVector _res, FVector _dim)
	{
		res = _res;
		dim = _dim;
		allocateData();
	}


	

	~ScalarField() 
	{
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

	void meshToLeveSet(FRawMesh* _rm, FVector& _mid_point)
	{
		float x, y, z;

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
						x = ((i - (res.X / 2)) / res.X)*dim.X + (dim.X/ res.X)*0.5f + _mid_point.X;
						y = ((j - (res.Y / 2)) / res.Y)*dim.Y + (dim.Y / res.Y)*0.5f + _mid_point.Y;
						z = ((k - (res.Z / 2)) / res.Z)*dim.Z + (dim.Z / res.Z)*0.5f + _mid_point.Z;
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

	scalar_value*** getData()
	{
		return data;
	}
	scalar_value getValue(int _x, int _y, int _z)
	{
		return data[_x][_y][_z];
	}
	FVector getRes() 
	{
		return res;
	}
	FVector getDim() 
	{
		return dim;
	}
	scalar_value getIsoValue() 
	{
		return iso_value;
	}
	void setIsoValue(scalar_value _iv)
	{
		 iso_value = _iv;
	}

private:
		//scalar_u_ptr_3d<scalar_value> data;
		scalar_value*** data;
		FVector res;
		FVector dim;

		scalar_value iso_value;

		void allocateData() {
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

		float distanceToMesh(FRawMesh* _rm, FVector _p) {
			
			
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
		
};
