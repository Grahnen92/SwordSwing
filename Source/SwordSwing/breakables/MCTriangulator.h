// Fill out your copyright notice in the Description page of Project Settings.

#pragma once
#ifndef MCTRIANGULATOR_H
#define MCTRIANGULATOR_H
#include <vector>
#include "ProceduralMeshComponent.h"
#include "ScalarField.h"
#include <string>
//template <typename scalar_value>
/**
 * 
 */
//TODO make subclasses for each type of triangulation

class SWORDSWING_API MCTriangulator
{
public:

	MCTriangulator();
	~MCTriangulator();

	void marchingCubes(UProceduralMeshComponent* _mesh, ScalarField<float>* _sf, float _iso_val = 0.f, const FVector& _mid_point = FVector(0.0f, 0.0f, 0.0f));
	//void marchingCubes(UProceduralMeshComponent* _mesh, LevelSet* _ls, const FVector& _mid_point = FVector(0.0f, 0.0f, 0.0f));
	void calcNormals();
private:

	//MC cell iterator data structures =======================
	int cubeIndex;

	float xyz[8][3];
	int vertList[12];
	float val[8];
	bool cellIsoBool[8];
	double dVal;

	int vertexCap = 0;
	int triangleCap = 0;

	int x, y, z;
	int layerIndex = 0;

	/*int edgeTable[256];
	int triTable[256][16];
	void initTables(int* _edgeTable, int** _triTable);*/
	const static int edgeTable[];// [256];
	const static int triTable[][16];// [256][16];

	struct mcCacheCell {
		bool isoBool;
		float cornerPoint[3];
		int vertexIndex[3];
	};

	mcCacheCell*** isoCache;//[2][resy][resz]
	std::vector<int> y0Cache;
	int z0Cache;

	//arrays to store generated data =============================

	TArray<FVector> vertexArray;
	TArray<int32> triangleArray;
	TArray<FVector> normalArray;
	TArray<FProcMeshTangent> tangentArray;


};
#endif