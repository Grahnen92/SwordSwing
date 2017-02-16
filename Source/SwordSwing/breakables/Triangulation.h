#pragma once

#include "SwordSwing.h"
#include "ScalarField.h"
#include "ProceduralMeshComponent.h"
#include <vector>

namespace triangulation {

	void test(){
		int t = 1;
	}

	struct cacheCell {
		bool isoBool;
		float cornerPoint[3];
		int vertexIndex[3];
	};

	template <typename scalar_value>
	void marchingCubes(UProceduralMeshComponent* _mesh, ScalarField<scalar_value>* _sf) {
		UE_LOG(LogTemp, Warning, TEXT("generating mesh..."));
		int cubeIndex;

		float xyz[8][3];
		int vertList[12];
		unsigned char val[8];
		bool cellIsoBool[8];
		double dVal;

		int vertexCap = 0;
		int triangleCap = 0;

		int x, y, z;
		int layerIndex = 0;

		TArray<FVector> vertexArray;
		TArray<int32> triangleArray;
		TArray<FVector> normalArray;
		TArray<FProcMeshTangent> tangentArray;

		int edgeTable[256];
		int triTable[256][16];

		initTables(edgeTable, triTable);


		FVector _res = _sf->getRes();
		FVector _dim = _sf->getDim();
		scalar_value _iso_value = _sf->getIsoValue();
		scalar_value*** _data = _sf->getData();

		cacheCell*** isoCache = new cacheCell**[2];
		for (int i = 0; i < 2; i++)
		{
			isoCache[i] = new cacheCell*[_res.Y];
			for (int j = 0; j < _res.Y; j++)
			{
				isoCache[i][j] = new cacheCell[_res.Z];
			}
		}

		std::vector<int> y0Cache;
		y0Cache.resize(_res.Z);
		int z0Cache;

		//create the first layer
		x = 0;
		y = 0;
		z = 0;

		// create the first voxel ============================================================

		//inherit corner values from local variable

		//inherit corner values from isoCache

		//calculate corner values that could not be inherited
		xyz[0][0] = (float)((x - (_res.X / 2)) / ((float)(_res.X / 2)))*_dim.X;
		xyz[0][1] = (float)((y - (_res.Y / 2)) / ((float)(_res.Y / 2)))*_dim.Y;
		xyz[0][2] = (float)((z - (_res.Z / 2)) / ((float)(_res.Z / 2)))*_dim.Z;
		val[0] = _data[x][y][z];
		if (_data[x][y][z] < _iso_value)			// cubeIndex |= 1;
			cellIsoBool[0] = true;
		else
			cellIsoBool[0] = false;



		xyz[1][0] = (float)((x + 1 - (_res.X / 2)) / ((float)(_res.X / 2)))*_dim.X;
		xyz[1][1] = (float)((y - (_res.Y / 2)) / ((float)(_res.Y / 2)))*_dim.Y;
		xyz[1][2] = (float)((z - (_res.Z / 2)) / ((float)(_res.Z / 2)))*_dim.Z;
		val[1] = _data[x + 1][y][z];
		if (_data[x + 1][y][z] < _iso_value)		// cubeIndex |= 2;
			cellIsoBool[1] = true;
		else
			cellIsoBool[1] = false;

		xyz[2][0] = (float)((x + 1 - (_res.X / 2)) / ((float)(_res.X / 2)))*_dim.X;
		xyz[2][1] = (float)((y - (_res.Y / 2)) / ((float)(_res.Y / 2)))*_dim.Y;
		xyz[2][2] = (float)((z + 1 - (_res.Z / 2)) / ((float)(_res.Z / 2)))*_dim.Z;
		val[2] = _data[x + 1][y][z + 1];
		if (_data[x + 1][y][z + 1] < _iso_value)	// cubeIndex |= 4;
			cellIsoBool[2] = true;
		else
			cellIsoBool[2] = false;

		xyz[3][0] = (float)((x - (_res.X / 2)) / ((float)(_res.X / 2)))*_dim.X;
		xyz[3][1] = (float)((y - (_res.Y / 2)) / ((float)(_res.Y / 2)))*_dim.Y;
		xyz[3][2] = (float)((z + 1 - (_res.Z / 2)) / ((float)(_res.Z / 2)))*_dim.Z;
		val[3] = _data[x][y][z + 1];
		if (_data[x][y][z + 1] < _iso_value)		// cubeIndex |= 8;
			cellIsoBool[3] = true;
		else
			cellIsoBool[3] = false;

		xyz[4][0] = (float)((x - (_res.X / 2)) / ((float)(_res.X / 2)))*_dim.X;
		xyz[4][1] = (float)((y + 1 - (_res.Y / 2)) / ((float)(_res.Y / 2)))*_dim.Y;
		xyz[4][2] = (float)((z - (_res.Z / 2)) / ((float)(_res.Z / 2)))*_dim.Z;
		val[4] = _data[x][y + 1][z];
		if (_data[x][y + 1][z] < _iso_value)		//cubeIndex |= 16;
			cellIsoBool[4] = true;
		else
			cellIsoBool[4] = false;

		xyz[5][0] = (float)((x + 1 - (_res.X / 2)) / ((float)(_res.X / 2)))*_dim.X;
		xyz[5][1] = (float)((y + 1 - (_res.Y / 2)) / ((float)(_res.Y / 2)))*_dim.Y;
		xyz[5][2] = (float)((z - (_res.Z / 2)) / ((float)(_res.Z / 2)))*_dim.Z;
		val[5] = _data[x + 1][y + 1][z];
		if (_data[x + 1][y + 1][z] < _iso_value)	// cubeIndex |= 32;
			cellIsoBool[5] = true;
		else
			cellIsoBool[5] = false;

		//save the sixth corner values to isoCache
		xyz[6][0] = isoCache[layerIndex][y][z].cornerPoint[0] = (float)((x + 1 - (_res.X / 2)) / ((float)(_res.X / 2)))*_dim.X;
		xyz[6][1] = isoCache[layerIndex][y][z].cornerPoint[1] = (float)((y + 1 - (_res.Y / 2)) / ((float)(_res.Y / 2)))*_dim.Y;
		xyz[6][2] = isoCache[layerIndex][y][z].cornerPoint[2] = (float)((z + 1 - (_res.Z / 2)) / ((float)(_res.Z / 2)))*_dim.Z;
		val[6] = _data[x + 1][y + 1][z + 1];
		if (_data[x + 1][y + 1][z + 1] < _iso_value)// cubeIndex |= 64;
			cellIsoBool[6] = isoCache[layerIndex][y][z].isoBool = true;
		else
			cellIsoBool[6] = isoCache[layerIndex][y][z].isoBool = false;

		xyz[7][0] = (float)((x - (_res.X / 2)) / ((float)(_res.X / 2)))*_dim.X;
		xyz[7][1] = (float)((y + 1 - (_res.Y / 2)) / ((float)(_res.Y / 2)))*_dim.Y;
		xyz[7][2] = (float)((z + 1 - (_res.Z / 2)) / ((float)(_res.Z / 2)))*_dim.Z;
		val[7] = _data[x][y + 1][z + 1];
		if (_data[x][y + 1][z + 1] < _iso_value)	// cubeIndex |= 128;
			cellIsoBool[7] = true;
		else
			cellIsoBool[7] = false;

		// get the case index
		cubeIndex = cellIsoBool[0] * 1 + cellIsoBool[1] * 2 + cellIsoBool[2] * 4 + cellIsoBool[3] * 8 +
			cellIsoBool[4] * 16 + cellIsoBool[5] * 32 + cellIsoBool[6] * 64 + cellIsoBool[7] * 128;

		// check if cube is entirely in or out of the surface ----------------------------
		if (edgeTable[cubeIndex] != 0) {

			// Find the vertices where the surface intersects the cube--------------------

			//inherit vertex indices from local variable ---------------------------------

			//inherit vertex indices from isoCache ---------------------------------------

			//calculate indices that could not be inherited ------------------------------
			if (edgeTable[cubeIndex] & 1) {
				dVal = (double)(_iso_value - val[0]) / (double)(val[1] - val[0]);
				/*vertexArray[vertexCap][0] = xyz[0][0] + dVal*(xyz[1][0] - xyz[0][0]);
				vertexArray[vertexCap][1] = xyz[0][1] + dVal*(xyz[1][1] - xyz[0][1]);
				vertexArray[vertexCap][2] = xyz[0][2] + dVal*(xyz[1][2] - xyz[0][2]);*/
				vertexArray.Add(FVector(xyz[0][0] + dVal*(xyz[1][0] - xyz[0][0]), xyz[0][1] + dVal*(xyz[1][1] - xyz[0][1]), xyz[0][2] + dVal*(xyz[1][2] - xyz[0][2])));
				vertList[0] = vertexCap;
				vertexCap++;
			}

			if (edgeTable[cubeIndex] & 2) {
				dVal = (double)(_iso_value - val[1]) / (double)(val[2] - val[1]);
				/*vertexArray[vertexCap][0] = xyz[1][0] + dVal*(xyz[2][0] - xyz[1][0]);
				vertexArray[vertexCap][1] = xyz[1][1] + dVal*(xyz[2][1] - xyz[1][1]);
				vertexArray[vertexCap][2] = xyz[1][2] + dVal*(xyz[2][2] - xyz[1][2]);*/
				vertexArray.Add(FVector(xyz[1][0] + dVal*(xyz[2][0] - xyz[1][0]), xyz[1][1] + dVal*(xyz[2][1] - xyz[1][1]), xyz[1][2] + dVal*(xyz[2][2] - xyz[1][2])));
				vertList[1] = vertexCap;
				vertexCap++;
			}
			if (edgeTable[cubeIndex] & 4) {
				dVal = (double)(_iso_value - val[2]) / (double)(val[3] - val[2]);
				//vertexArray[vertexCap][0] = xyz[2][0] + dVal*(xyz[3][0] - xyz[2][0]);
				//vertexArray[vertexCap][1] = xyz[2][1] + dVal*(xyz[3][1] - xyz[2][1]);
				//vertexArray[vertexCap][2] = xyz[2][2] + dVal*(xyz[3][2] - xyz[2][2]);
				vertexArray.Add(FVector(xyz[2][0] + dVal*(xyz[3][0] - xyz[2][0]), xyz[2][1] + dVal*(xyz[3][1] - xyz[2][1]), xyz[2][2] + dVal*(xyz[3][2] - xyz[2][2])));
				vertList[2] = vertexCap;
				vertexCap++;
			}
			if (edgeTable[cubeIndex] & 8) {
				dVal = (double)(_iso_value - val[3]) / (double)(val[0] - val[3]);
				//vertexArray[vertexCap][0] = xyz[3][0] + dVal*(xyz[0][0] - xyz[3][0]);
				//vertexArray[vertexCap][1] = xyz[3][1] + dVal*(xyz[0][1] - xyz[3][1]);
				//vertexArray[vertexCap][2] = xyz[3][2] + dVal*(xyz[0][2] - xyz[3][2]);
				vertexArray.Add(FVector(xyz[3][0] + dVal*(xyz[0][0] - xyz[3][0]), xyz[3][1] + dVal*(xyz[0][1] - xyz[3][1]), xyz[3][2] + dVal*(xyz[0][2] - xyz[3][2])));
				vertList[3] = vertexCap;
				vertexCap++;
			}
			if (edgeTable[cubeIndex] & 16) {
				dVal = (double)(_iso_value - val[4]) / (double)(val[5] - val[4]);
				/*vertexArray[vertexCap][0] = xyz[4][0] + dVal*(xyz[5][0] - xyz[4][0]);
				vertexArray[vertexCap][1] = xyz[4][1] + dVal*(xyz[5][1] - xyz[4][1]);
				vertexArray[vertexCap][2] = xyz[4][2] + dVal*(xyz[5][2] - xyz[4][2]);*/
				vertexArray.Add(FVector(xyz[4][0] + dVal*(xyz[5][0] - xyz[4][0]), xyz[4][1] + dVal*(xyz[5][1] - xyz[4][1]), xyz[4][2] + dVal*(xyz[5][2] - xyz[4][2])));
				vertList[4] = z0Cache = vertexCap;
				vertexCap++;
			}
			if (edgeTable[cubeIndex] & 32) {
				dVal = (double)(_iso_value - val[5]) / (double)(val[6] - val[5]);
				//vertexArray[vertexCap][0] = xyz[5][0] + dVal*(xyz[6][0] - xyz[5][0]);
				//vertexArray[vertexCap][1] = xyz[5][1] + dVal*(xyz[6][1] - xyz[5][1]);
				//vertexArray[vertexCap][2] = xyz[5][2] + dVal*(xyz[6][2] - xyz[5][2]);
				vertexArray.Add(FVector(xyz[5][0] + dVal*(xyz[6][0] - xyz[5][0]), xyz[5][1] + dVal*(xyz[6][1] - xyz[5][1]), xyz[5][2] + dVal*(xyz[6][2] - xyz[5][2])));
				isoCache[layerIndex][y][z].vertexIndex[0] = vertList[5] = vertexCap;
				vertexCap++;
			}
			if (edgeTable[cubeIndex] & 64) {
				dVal = (double)(_iso_value - val[6]) / (double)(val[7] - val[6]);
				/*vertexArray[vertexCap][0] = xyz[6][0] + dVal*(xyz[7][0] - xyz[6][0]);
				vertexArray[vertexCap][1] = xyz[6][1] + dVal*(xyz[7][1] - xyz[6][1]);
				vertexArray[vertexCap][2] = xyz[6][2] + dVal*(xyz[7][2] - xyz[6][2]);*/
				vertexArray.Add(FVector(xyz[6][0] + dVal*(xyz[7][0] - xyz[6][0]), xyz[6][1] + dVal*(xyz[7][1] - xyz[6][1]), xyz[6][2] + dVal*(xyz[7][2] - xyz[6][2])));
				isoCache[layerIndex][y][z].vertexIndex[1] = vertList[6] = vertexCap;
				vertexCap++;
			}
			if (edgeTable[cubeIndex] & 128) {
				dVal = (double)(_iso_value - val[7]) / (double)(val[4] - val[7]);
				/*vertexArray[vertexCap][0] = xyz[7][0] + dVal*(xyz[4][0] - xyz[7][0]);
				vertexArray[vertexCap][1] = xyz[7][1] + dVal*(xyz[4][1] - xyz[7][1]);
				vertexArray[vertexCap][2] = xyz[7][2] + dVal*(xyz[4][2] - xyz[7][2]);*/
				vertexArray.Add(FVector(xyz[7][0] + dVal*(xyz[4][0] - xyz[7][0]), xyz[7][1] + dVal*(xyz[4][1] - xyz[7][1]), xyz[7][2] + dVal*(xyz[4][2] - xyz[7][2])));
				vertList[7] = y0Cache[z] = vertexCap;
				vertexCap++;
			}
			if (edgeTable[cubeIndex] & 256) {
				dVal = (double)(_iso_value - val[0]) / (double)(val[4] - val[0]);
				/*vertexArray[vertexCap][0] = xyz[0][0] + dVal*(xyz[4][0] - xyz[0][0]);
				vertexArray[vertexCap][1] = xyz[0][1] + dVal*(xyz[4][1] - xyz[0][1]);
				vertexArray[vertexCap][2] = xyz[0][2] + dVal*(xyz[4][2] - xyz[0][2]);*/
				vertexArray.Add(FVector(xyz[0][0] + dVal*(xyz[4][0] - xyz[0][0]), xyz[0][1] + dVal*(xyz[4][1] - xyz[0][1]), xyz[0][2] + dVal*(xyz[4][2] - xyz[0][2])));
				vertList[8] = vertexCap;
				vertexCap++;
			}
			if (edgeTable[cubeIndex] & 512) {
				dVal = (double)(_iso_value - val[1]) / (double)(val[5] - val[1]);
				/*vertexArray[vertexCap][0] = xyz[1][0] + dVal*(xyz[5][0] - xyz[1][0]);
				vertexArray[vertexCap][1] = xyz[1][1] + dVal*(xyz[5][1] - xyz[1][1]);
				vertexArray[vertexCap][2] = xyz[1][2] + dVal*(xyz[5][2] - xyz[1][2]);*/
				vertexArray.Add(FVector(xyz[1][0] + dVal*(xyz[5][0] - xyz[1][0]), xyz[1][1] + dVal*(xyz[5][1] - xyz[1][1]), xyz[1][2] + dVal*(xyz[5][2] - xyz[1][2])));
				vertList[9] = vertexCap;
				vertexCap++;
			}
			if (edgeTable[cubeIndex] & 1024) {
				dVal = (double)(_iso_value - val[2]) / (double)(val[6] - val[2]);
				/*vertexArray[vertexCap][0] = xyz[2][0] + dVal*(xyz[6][0] - xyz[2][0]);
				vertexArray[vertexCap][1] = xyz[2][1] + dVal*(xyz[6][1] - xyz[2][1]);
				vertexArray[vertexCap][2] = xyz[2][2] + dVal*(xyz[6][2] - xyz[2][2]);*/
				vertexArray.Add(FVector(xyz[2][0] + dVal*(xyz[6][0] - xyz[2][0]), xyz[2][1] + dVal*(xyz[6][1] - xyz[2][1]), xyz[2][2] + dVal*(xyz[6][2] - xyz[2][2])));
				isoCache[layerIndex][y][z].vertexIndex[2] = vertList[10] = vertexCap;
				vertexCap++;
			}
			if (edgeTable[cubeIndex] & 2048) {
				dVal = (double)(_iso_value - val[3]) / (double)(val[7] - val[3]);
				/*vertexArray[vertexCap][0] = xyz[3][0] + dVal*(xyz[7][0] - xyz[3][0]);
				vertexArray[vertexCap][1] = xyz[3][1] + dVal*(xyz[7][1] - xyz[3][1]);
				vertexArray[vertexCap][2] = xyz[3][2] + dVal*(xyz[7][2] - xyz[3][2]);*/
				vertexArray.Add(FVector(xyz[3][0] + dVal*(xyz[7][0] - xyz[3][0]), xyz[3][1] + dVal*(xyz[7][1] - xyz[3][1]), xyz[3][2] + dVal*(xyz[7][2] - xyz[3][2])));
				vertList[11] = vertexCap;
				vertexCap++;
			}
			int test = triTable[cubeIndex][3];
			// bind triangle indecies
			for (int i = 0; triTable[cubeIndex][i] != -1; i += 3) {
				/*triangleArray[triangleCap*3 + 2] = vertList[triTable[cubeIndex][i]];
				triangleArray[triangleCap*3 + 1] = vertList[triTable[cubeIndex][i + 1]];
				triangleArray[triangleCap*3] = vertList[triTable[cubeIndex][i + 2]];*/
				triangleArray.Add(vertList[triTable[cubeIndex][i]]);
				triangleArray.Add(vertList[triTable[cubeIndex][i + 1]]);
				triangleArray.Add(vertList[triTable[cubeIndex][i + 2]]);

				triangleCap++;
			}
		}

		//create the remaining voxels of first row of first layer ======================================
		for (z = 1; z < _res.Z - 1; z++) {

			//inherit corner values from local variable
			xyz[0][0] = xyz[3][0];
			xyz[0][1] = xyz[3][1];
			xyz[0][2] = xyz[3][2];
			val[0] = _data[x][y][z];
			cellIsoBool[0] = cellIsoBool[3];

			xyz[1][0] = xyz[2][0];
			xyz[1][1] = xyz[2][1];
			xyz[1][2] = xyz[2][2];
			val[1] = _data[x + 1][y][z];
			cellIsoBool[1] = cellIsoBool[2];

			xyz[4][0] = xyz[7][0];
			xyz[4][1] = xyz[7][1];
			xyz[4][2] = xyz[7][2];
			val[4] = _data[x][y + 1][z];
			cellIsoBool[4] = cellIsoBool[7];

			//inherit corner values from isoCache
			xyz[5][0] = isoCache[layerIndex][y][z - 1].cornerPoint[0];
			xyz[5][1] = isoCache[layerIndex][y][z - 1].cornerPoint[1];
			xyz[5][2] = isoCache[layerIndex][y][z - 1].cornerPoint[2];
			val[5] = _data[x + 1][y + 1][z];
			cellIsoBool[5] = isoCache[layerIndex][y][z - 1].isoBool;

			//calculate corner values that could not be inherited
			xyz[2][0] = (float)((x + 1 - (_res.X / 2)) / ((float)(_res.X / 2)))*_dim.X;
			xyz[2][1] = (float)((y - (_res.Y / 2)) / ((float)(_res.Y / 2)))*_dim.Y;
			xyz[2][2] = (float)((z + 1 - (_res.Z / 2)) / ((float)(_res.Z / 2)))*_dim.Z;
			val[2] = _data[x + 1][y][z + 1];
			if (_data[x + 1][y][z + 1] < _iso_value)	// cubeIndex |= 4;
				cellIsoBool[2] = true;
			else
				cellIsoBool[2] = false;

			xyz[3][0] = (float)((x - (_res.X / 2)) / ((float)(_res.X / 2)))*_dim.X;
			xyz[3][1] = (float)((y - (_res.Y / 2)) / ((float)(_res.Y / 2)))*_dim.Y;
			xyz[3][2] = (float)((z + 1 - (_res.Z / 2)) / ((float)(_res.Z / 2)))*_dim.Z;
			val[3] = _data[x][y][z + 1];
			if (_data[x][y][z + 1] < _iso_value)		// cubeIndex |= 8;
				cellIsoBool[3] = true;
			else
				cellIsoBool[3] = false;

			//save the sixth corner values to isoCache
			xyz[6][0] = isoCache[layerIndex][y][z].cornerPoint[0] = (float)((x + 1 - (_res.X / 2)) / ((float)(_res.X / 2)))*_dim.X;
			xyz[6][1] = isoCache[layerIndex][y][z].cornerPoint[1] = (float)((y + 1 - (_res.Y / 2)) / ((float)(_res.Y / 2)))*_dim.Y;
			xyz[6][2] = isoCache[layerIndex][y][z].cornerPoint[2] = (float)((z + 1 - (_res.Z / 2)) / ((float)(_res.Z / 2)))*_dim.Z;
			val[6] = _data[x + 1][y + 1][z + 1];
			if (_data[x + 1][y + 1][z + 1] < _iso_value)// cubeIndex |= 64;
				cellIsoBool[6] = isoCache[layerIndex][y][z].isoBool = true;
			else
				cellIsoBool[6] = isoCache[layerIndex][y][z].isoBool = false;

			xyz[7][0] = (float)((x - (_res.X / 2)) / ((float)(_res.X / 2)))*_dim.X;
			xyz[7][1] = (float)((y + 1 - (_res.Y / 2)) / ((float)(_res.Y / 2)))*_dim.Y;
			xyz[7][2] = (float)((z + 1 - (_res.Z / 2)) / ((float)(_res.Z / 2)))*_dim.Z;
			val[7] = _data[x][y + 1][z + 1];
			if (_data[x][y + 1][z + 1] < _iso_value)	// cubeIndex |= 128;
				cellIsoBool[7] = true;
			else
				cellIsoBool[7] = false;

			// get the case index
			cubeIndex = cellIsoBool[0] * 1 + cellIsoBool[1] * 2 + cellIsoBool[2] * 4 + cellIsoBool[3] * 8 +
				cellIsoBool[4] * 16 + cellIsoBool[5] * 32 + cellIsoBool[6] * 64 + cellIsoBool[7] * 128;

			// check if cube is entirely in or out of the surface ----------------------------
			if (edgeTable[cubeIndex] != 0) {

				// Find the vertices where the surface intersects the cube--------------------

				//inherit vertex indices from local variable ---------------------------------
				if (edgeTable[cubeIndex] & 1) {
					vertList[0] = vertList[2];
				}
				if (edgeTable[cubeIndex] & 256) {
					vertList[8] = vertList[11];
				}

				//inherit vertex indices from isoCache ---------------------------------------
				if (edgeTable[cubeIndex] & 16) {
					vertList[4] = isoCache[layerIndex][y][z - 1].vertexIndex[1];
				}
				if (edgeTable[cubeIndex] & 512) {
					vertList[9] = isoCache[layerIndex][y][z - 1].vertexIndex[2];
				}
				//calculate indices that could not be inherited ------------------------------
				if (edgeTable[cubeIndex] & 2) {
					dVal = (double)(_iso_value - val[1]) / (double)(val[2] - val[1]);
					/*vertexArray[vertexCap][0] = xyz[1][0] + dVal*(xyz[2][0] - xyz[1][0]);
					vertexArray[vertexCap][1] = xyz[1][1] + dVal*(xyz[2][1] - xyz[1][1]);
					vertexArray[vertexCap][2] = xyz[1][2] + dVal*(xyz[2][2] - xyz[1][2]);*/
					vertexArray.Add(FVector(xyz[1][0] + dVal*(xyz[2][0] - xyz[1][0]), xyz[1][1] + dVal*(xyz[2][1] - xyz[1][1]), xyz[1][2] + dVal*(xyz[2][2] - xyz[1][2])));
					vertList[1] = vertexCap;
					vertexCap++;
				}
				if (edgeTable[cubeIndex] & 4) {
					dVal = (double)(_iso_value - val[2]) / (double)(val[3] - val[2]);
					/*vertexArray[vertexCap][0] = xyz[2][0] + dVal*(xyz[3][0] - xyz[2][0]);
					vertexArray[vertexCap][1] = xyz[2][1] + dVal*(xyz[3][1] - xyz[2][1]);
					vertexArray[vertexCap][2] = xyz[2][2] + dVal*(xyz[3][2] - xyz[2][2]);*/
					vertexArray.Add(FVector(xyz[2][0] + dVal*(xyz[3][0] - xyz[2][0]), xyz[2][1] + dVal*(xyz[3][1] - xyz[2][1]), xyz[2][2] + dVal*(xyz[3][2] - xyz[2][2])));
					vertList[2] = vertexCap;
					vertexCap++;
				}
				if (edgeTable[cubeIndex] & 8) {
					dVal = (double)(_iso_value - val[3]) / (double)(val[0] - val[3]);
					/*vertexArray[vertexCap][0] = xyz[3][0] + dVal*(xyz[0][0] - xyz[3][0]);
					vertexArray[vertexCap][1] = xyz[3][1] + dVal*(xyz[0][1] - xyz[3][1]);
					vertexArray[vertexCap][2] = xyz[3][2] + dVal*(xyz[0][2] - xyz[3][2]);*/
					vertexArray.Add(FVector(xyz[3][0] + dVal*(xyz[0][0] - xyz[3][0]), xyz[3][1] + dVal*(xyz[0][1] - xyz[3][1]), xyz[3][2] + dVal*(xyz[0][2] - xyz[3][2])));
					vertList[3] = vertexCap;
					vertexCap++;
				}

				if (edgeTable[cubeIndex] & 32) {
					dVal = (double)(_iso_value - val[5]) / (double)(val[6] - val[5]);
					//vertexArray[vertexCap][0] = xyz[5][0] + dVal*(xyz[6][0] - xyz[5][0]);
					//vertexArray[vertexCap][1] = xyz[5][1] + dVal*(xyz[6][1] - xyz[5][1]);
					//vertexArray[vertexCap][2] = xyz[5][2] + dVal*(xyz[6][2] - xyz[5][2]);
					vertexArray.Add(FVector(xyz[5][0] + dVal*(xyz[6][0] - xyz[5][0]), xyz[5][1] + dVal*(xyz[6][1] - xyz[5][1]), xyz[5][2] + dVal*(xyz[6][2] - xyz[5][2])));
					isoCache[layerIndex][y][z].vertexIndex[0] = vertList[5] = vertexCap;
					vertexCap++;
				}
				if (edgeTable[cubeIndex] & 64) {
					dVal = (double)(_iso_value - val[6]) / (double)(val[7] - val[6]);
					/*vertexArray[vertexCap][0] = xyz[6][0] + dVal*(xyz[7][0] - xyz[6][0]);
					vertexArray[vertexCap][1] = xyz[6][1] + dVal*(xyz[7][1] - xyz[6][1]);
					vertexArray[vertexCap][2] = xyz[6][2] + dVal*(xyz[7][2] - xyz[6][2]);*/
					vertexArray.Add(FVector(xyz[6][0] + dVal*(xyz[7][0] - xyz[6][0]), xyz[6][1] + dVal*(xyz[7][1] - xyz[6][1]), xyz[6][2] + dVal*(xyz[7][2] - xyz[6][2])));
					isoCache[layerIndex][y][z].vertexIndex[1] = vertList[6] = vertexCap;
					vertexCap++;
				}
				if (edgeTable[cubeIndex] & 128) {
					dVal = (double)(_iso_value - val[7]) / (double)(val[4] - val[7]);
					//vertexArray[vertexCap][0] = xyz[7][0] + dVal*(xyz[4][0] - xyz[7][0]);
					//vertexArray[vertexCap][1] = xyz[7][1] + dVal*(xyz[4][1] - xyz[7][1]);
					//vertexArray[vertexCap][2] = xyz[7][2] + dVal*(xyz[4][2] - xyz[7][2]);
					vertexArray.Add(FVector(xyz[7][0] + dVal*(xyz[4][0] - xyz[7][0]), xyz[7][1] + dVal*(xyz[4][1] - xyz[7][1]), xyz[7][2] + dVal*(xyz[4][2] - xyz[7][2])));
					vertList[7] = y0Cache[z] = vertexCap;
					vertexCap++;
				}
				if (edgeTable[cubeIndex] & 1024) {
					dVal = (double)(_iso_value - val[2]) / (double)(val[6] - val[2]);
					//vertexArray[vertexCap][0] = xyz[2][0] + dVal*(xyz[6][0] - xyz[2][0]);
					//vertexArray[vertexCap][1] = xyz[2][1] + dVal*(xyz[6][1] - xyz[2][1]);
					//vertexArray[vertexCap][2] = xyz[2][2] + dVal*(xyz[6][2] - xyz[2][2]);
					vertexArray.Add(FVector(xyz[2][0] + dVal*(xyz[6][0] - xyz[2][0]), xyz[2][1] + dVal*(xyz[6][1] - xyz[2][1]), xyz[2][2] + dVal*(xyz[6][2] - xyz[2][2])));
					isoCache[layerIndex][y][z].vertexIndex[2] = vertList[10] = vertexCap;
					vertexCap++;
				}
				if (edgeTable[cubeIndex] & 2048) {
					dVal = (double)(_iso_value - val[3]) / (double)(val[7] - val[3]);
					/*vertexArray[vertexCap][0] = xyz[3][0] + dVal*(xyz[7][0] - xyz[3][0]);
					vertexArray[vertexCap][1] = xyz[3][1] + dVal*(xyz[7][1] - xyz[3][1]);
					vertexArray[vertexCap][2] = xyz[3][2] + dVal*(xyz[7][2] - xyz[3][2]);*/
					vertexArray.Add(FVector(xyz[3][0] + dVal*(xyz[7][0] - xyz[3][0]), xyz[3][1] + dVal*(xyz[7][1] - xyz[3][1]), xyz[3][2] + dVal*(xyz[7][2] - xyz[3][2])));
					vertList[11] = vertexCap;
					vertexCap++;
				}

				// bind triangle indecies
				for (int i = 0; triTable[cubeIndex][i] != -1; i += 3) {
					/*triangleArray[triangleCap*3 + 2] = vertList[triTable[cubeIndex][i]];
					triangleArray[triangleCap*3 + 1] = vertList[triTable[cubeIndex][i + 1]];
					triangleArray[triangleCap*3] = vertList[triTable[cubeIndex][i + 2]];*/
					triangleArray.Add(vertList[triTable[cubeIndex][i]]);
					triangleArray.Add(vertList[triTable[cubeIndex][i + 1]]);
					triangleArray.Add(vertList[triTable[cubeIndex][i + 2]]);

					triangleCap++;
				}
			}
		}

		//create the remaining rows of the first layer=============================================
		for (y = 1; y < _res.Y - 1; y++) {

			z = 0;
			//create the first voxel of remaining rows of first layer=============================

			//inherit corner values from local variable -----------------------------------------

			//inherit corner values from isoCache -----------------------------------------------
			xyz[2][0] = isoCache[layerIndex][y - 1][z].cornerPoint[0];
			xyz[2][1] = isoCache[layerIndex][y - 1][z].cornerPoint[1];
			xyz[2][2] = isoCache[layerIndex][y - 1][z].cornerPoint[2];
			val[2] = _data[x + 1][y][z + 1];
			cellIsoBool[2] = isoCache[layerIndex][y - 1][z].isoBool;

			//calculate corner values that could not be inherited ---------------------------------
			xyz[0][0] = (float)((x - (_res.X / 2)) / ((float)(_res.X / 2)))*_dim.X;
			xyz[0][1] = (float)((y - (_res.Y / 2)) / ((float)(_res.Y / 2)))*_dim.Y;
			xyz[0][2] = (float)((z - (_res.Z / 2)) / ((float)(_res.Z / 2)))*_dim.Z;
			val[0] = _data[x][y][z];
			if (_data[x][y][z] < _iso_value)			// cubeIndex |= 1;
				cellIsoBool[0] = true;
			else
				cellIsoBool[0] = false;

			xyz[1][0] = (float)((x + 1 - (_res.X / 2)) / ((float)(_res.X / 2)))*_dim.X;
			xyz[1][1] = (float)((y - (_res.Y / 2)) / ((float)(_res.Y / 2)))*_dim.Y;
			xyz[1][2] = (float)((z - (_res.Z / 2)) / ((float)(_res.Z / 2)))*_dim.Z;
			val[1] = _data[x + 1][y][z];
			if (_data[x + 1][y][z] < _iso_value)		// cubeIndex |= 2;
				cellIsoBool[1] = true;
			else
				cellIsoBool[1] = false;

			xyz[3][0] = (float)((x - (_res.X / 2)) / ((float)(_res.X / 2)))*_dim.X;
			xyz[3][1] = (float)((y - (_res.Y / 2)) / ((float)(_res.Y / 2)))*_dim.Y;
			xyz[3][2] = (float)((z + 1 - (_res.Z / 2)) / ((float)(_res.Z / 2)))*_dim.Z;
			val[3] = _data[x][y][z + 1];
			if (_data[x][y][z + 1] < _iso_value)		// cubeIndex |= 8;
				cellIsoBool[3] = true;
			else
				cellIsoBool[3] = false;

			xyz[4][0] = (float)((x - (_res.X / 2)) / ((float)(_res.X / 2)))*_dim.X;
			xyz[4][1] = (float)((y + 1 - (_res.Y / 2)) / ((float)(_res.Y / 2)))*_dim.Y;
			xyz[4][2] = (float)((z - (_res.Z / 2)) / ((float)(_res.Z / 2)))*_dim.Z;
			val[4] = _data[x][y + 1][z];
			if (_data[x][y + 1][z] < _iso_value)		//cubeIndex |= 16;
				cellIsoBool[4] = true;
			else
				cellIsoBool[4] = false;

			xyz[5][0] = (float)((x + 1 - (_res.X / 2)) / ((float)(_res.X / 2)))*_dim.X;
			xyz[5][1] = (float)((y + 1 - (_res.Y / 2)) / ((float)(_res.Y / 2)))*_dim.Y;
			xyz[5][2] = (float)((z - (_res.Z / 2)) / ((float)(_res.Z / 2)))*_dim.Z;
			val[5] = _data[x + 1][y + 1][z];
			if (_data[x + 1][y + 1][z] < _iso_value)	// cubeIndex |= 32;
				cellIsoBool[5] = true;
			else
				cellIsoBool[5] = false;

			//save the sixth corner values to isoCache
			xyz[6][0] = isoCache[layerIndex][y][z].cornerPoint[0] = (float)((x + 1 - (_res.X / 2)) / ((float)(_res.X / 2)))*_dim.X;
			xyz[6][1] = isoCache[layerIndex][y][z].cornerPoint[1] = (float)((y + 1 - (_res.Y / 2)) / ((float)(_res.Y / 2)))*_dim.Y;
			xyz[6][2] = isoCache[layerIndex][y][z].cornerPoint[2] = (float)((z + 1 - (_res.Z / 2)) / ((float)(_res.Z / 2)))*_dim.Z;
			val[6] = _data[x + 1][y + 1][z + 1];
			if (_data[x + 1][y + 1][z + 1] < _iso_value)// cubeIndex |= 64;
				cellIsoBool[6] = isoCache[layerIndex][y][z].isoBool = true;
			else
				cellIsoBool[6] = isoCache[layerIndex][y][z].isoBool = false;

			xyz[7][0] = (float)((x - (_res.X / 2)) / ((float)(_res.X / 2)))*_dim.X;
			xyz[7][1] = (float)((y + 1 - (_res.Y / 2)) / ((float)(_res.Y / 2)))*_dim.Y;
			xyz[7][2] = (float)((z + 1 - (_res.Z / 2)) / ((float)(_res.Z / 2)))*_dim.Z;
			val[7] = _data[x][y + 1][z + 1];
			if (_data[x][y + 1][z + 1] < _iso_value)	// cubeIndex |= 128;
				cellIsoBool[7] = true;
			else
				cellIsoBool[7] = false;

			// get the case index
			cubeIndex = cellIsoBool[0] * 1 + cellIsoBool[1] * 2 + cellIsoBool[2] * 4 + cellIsoBool[3] * 8 +
				cellIsoBool[4] * 16 + cellIsoBool[5] * 32 + cellIsoBool[6] * 64 + cellIsoBool[7] * 128;

			// check if cube is entirely in or out of the surface ----------------------------
			if (edgeTable[cubeIndex] != 0) {

				// Find the vertices where the surface intersects the cube--------------------

				//inherit vertex indices from local variable ---------------------------------
				if (edgeTable[cubeIndex] & 1) {
					vertList[0] = z0Cache;
				}
				if (edgeTable[cubeIndex] & 8) {
					vertList[3] = y0Cache[z];
				}

				//inherit vertex indices from isoCache ---------------------------------------
				if (edgeTable[cubeIndex] & 2) {
					vertList[1] = isoCache[layerIndex][y - 1][z].vertexIndex[0];
				}
				if (edgeTable[cubeIndex] & 4) {
					vertList[2] = isoCache[layerIndex][y - 1][z].vertexIndex[1];
				}

				//calculate indices that could not be inherited ------------------------------

				if (edgeTable[cubeIndex] & 16) {
					dVal = (double)(_iso_value - val[4]) / (double)(val[5] - val[4]);
					//vertexArray[vertexCap][0] = xyz[4][0] + dVal*(xyz[5][0] - xyz[4][0]);
					//vertexArray[vertexCap][1] = xyz[4][1] + dVal*(xyz[5][1] - xyz[4][1]);
					//vertexArray[vertexCap][2] = xyz[4][2] + dVal*(xyz[5][2] - xyz[4][2]);
					vertexArray.Add(FVector(xyz[4][0] + dVal*(xyz[5][0] - xyz[4][0]), xyz[4][1] + dVal*(xyz[5][1] - xyz[4][1]), xyz[4][2] + dVal*(xyz[5][2] - xyz[4][2])));
					vertList[4] = z0Cache = vertexCap;
					vertexCap++;
				}
				if (edgeTable[cubeIndex] & 32) {
					dVal = (double)(_iso_value - val[5]) / (double)(val[6] - val[5]);
					//vertexArray[vertexCap][0] = xyz[5][0] + dVal*(xyz[6][0] - xyz[5][0]);
					//vertexArray[vertexCap][1] = xyz[5][1] + dVal*(xyz[6][1] - xyz[5][1]);
					//vertexArray[vertexCap][2] = xyz[5][2] + dVal*(xyz[6][2] - xyz[5][2]);
					vertexArray.Add(FVector(xyz[5][0] + dVal*(xyz[6][0] - xyz[5][0]), xyz[5][1] + dVal*(xyz[6][1] - xyz[5][1]), xyz[5][2] + dVal*(xyz[6][2] - xyz[5][2])));
					isoCache[layerIndex][y][z].vertexIndex[0] = vertList[5] = vertexCap;
					vertexCap++;
				}
				if (edgeTable[cubeIndex] & 64) {
					dVal = (double)(_iso_value - val[6]) / (double)(val[7] - val[6]);
					//vertexArray[vertexCap][0] = xyz[6][0] + dVal*(xyz[7][0] - xyz[6][0]);
					//vertexArray[vertexCap][1] = xyz[6][1] + dVal*(xyz[7][1] - xyz[6][1]);
					//vertexArray[vertexCap][2] = xyz[6][2] + dVal*(xyz[7][2] - xyz[6][2]);
					vertexArray.Add(FVector(xyz[6][0] + dVal*(xyz[7][0] - xyz[6][0]), xyz[6][1] + dVal*(xyz[7][1] - xyz[6][1]), xyz[6][2] + dVal*(xyz[7][2] - xyz[6][2])));
					isoCache[layerIndex][y][z].vertexIndex[1] = vertList[6] = vertexCap;
					vertexCap++;
				}
				if (edgeTable[cubeIndex] & 128) {
					dVal = (double)(_iso_value - val[7]) / (double)(val[4] - val[7]);
					//vertexArray[vertexCap][0] = xyz[7][0] + dVal*(xyz[4][0] - xyz[7][0]);
					//vertexArray[vertexCap][1] = xyz[7][1] + dVal*(xyz[4][1] - xyz[7][1]);
					//vertexArray[vertexCap][2] = xyz[7][2] + dVal*(xyz[4][2] - xyz[7][2]);
					vertexArray.Add(FVector(xyz[7][0] + dVal*(xyz[4][0] - xyz[7][0]), xyz[7][1] + dVal*(xyz[4][1] - xyz[7][1]), xyz[7][2] + dVal*(xyz[4][2] - xyz[7][2])));
					vertList[7] = y0Cache[z] = vertexCap;
					vertexCap++;
				}
				if (edgeTable[cubeIndex] & 256) {
					dVal = (double)(_iso_value - val[0]) / (double)(val[4] - val[0]);
					//vertexArray[vertexCap][0] = xyz[0][0] + dVal*(xyz[4][0] - xyz[0][0]);
					//vertexArray[vertexCap][1] = xyz[0][1] + dVal*(xyz[4][1] - xyz[0][1]);
					//vertexArray[vertexCap][2] = xyz[0][2] + dVal*(xyz[4][2] - xyz[0][2]);
					vertexArray.Add(FVector(xyz[0][0] + dVal*(xyz[4][0] - xyz[0][0]), xyz[0][1] + dVal*(xyz[4][1] - xyz[0][1]), xyz[0][2] + dVal*(xyz[4][2] - xyz[0][2])));
					vertList[8] = vertexCap;
					vertexCap++;
				}
				if (edgeTable[cubeIndex] & 512) {
					dVal = (double)(_iso_value - val[1]) / (double)(val[5] - val[1]);
					//vertexArray[vertexCap][0] = xyz[1][0] + dVal*(xyz[5][0] - xyz[1][0]);
					//vertexArray[vertexCap][1] = xyz[1][1] + dVal*(xyz[5][1] - xyz[1][1]);
					//vertexArray[vertexCap][2] = xyz[1][2] + dVal*(xyz[5][2] - xyz[1][2]);
					vertexArray.Add(FVector(xyz[1][0] + dVal*(xyz[5][0] - xyz[1][0]), xyz[1][1] + dVal*(xyz[5][1] - xyz[1][1]), xyz[1][2] + dVal*(xyz[5][2] - xyz[1][2])));
					vertList[9] = vertexCap;
					vertexCap++;
				}
				if (edgeTable[cubeIndex] & 1024) {
					dVal = (double)(_iso_value - val[2]) / (double)(val[6] - val[2]);
					//vertexArray[vertexCap][0] = xyz[2][0] + dVal*(xyz[6][0] - xyz[2][0]);
					//vertexArray[vertexCap][1] = xyz[2][1] + dVal*(xyz[6][1] - xyz[2][1]);
					//vertexArray[vertexCap][2] = xyz[2][2] + dVal*(xyz[6][2] - xyz[2][2]);
					vertexArray.Add(FVector(xyz[2][0] + dVal*(xyz[6][0] - xyz[2][0]), xyz[2][1] + dVal*(xyz[6][1] - xyz[2][1]), xyz[2][2] + dVal*(xyz[6][2] - xyz[2][2])));
					isoCache[layerIndex][y][z].vertexIndex[2] = vertList[10] = vertexCap;
					vertexCap++;
				}
				if (edgeTable[cubeIndex] & 2048) {
					dVal = (double)(_iso_value - val[3]) / (double)(val[7] - val[3]);
					//vertexArray[vertexCap][0] = xyz[3][0] + dVal*(xyz[7][0] - xyz[3][0]);
					//vertexArray[vertexCap][1] = xyz[3][1] + dVal*(xyz[7][1] - xyz[3][1]);
					//vertexArray[vertexCap][2] = xyz[3][2] + dVal*(xyz[7][2] - xyz[3][2]);
					vertexArray.Add(FVector(xyz[3][0] + dVal*(xyz[7][0] - xyz[3][0]), xyz[3][1] + dVal*(xyz[7][1] - xyz[3][1]), xyz[3][2] + dVal*(xyz[7][2] - xyz[3][2])));
					vertList[11] = vertexCap;
					vertexCap++;
				}

				// bind triangle indecies
				for (int i = 0; triTable[cubeIndex][i] != -1; i += 3) {
					/*triangleArray[triangleCap*3 + 2] = vertList[triTable[cubeIndex][i]];
					triangleArray[triangleCap*3 + 1] = vertList[triTable[cubeIndex][i + 1]];
					triangleArray[triangleCap*3] = vertList[triTable[cubeIndex][i + 2]];*/
					triangleArray.Add(vertList[triTable[cubeIndex][i]]);
					triangleArray.Add(vertList[triTable[cubeIndex][i + 1]]);
					triangleArray.Add(vertList[triTable[cubeIndex][i + 2]]);

					triangleCap++;
				}
			}

			//create the remaining voxels of remaining rows of first layer=============================
			for (z = 1; z < _res.Z - 1; z++) {
				//inherit corner values from local variable
				xyz[0][0] = xyz[3][0];
				xyz[0][1] = xyz[3][1];
				xyz[0][2] = xyz[3][2];
				val[0] = _data[x][y][z];
				cellIsoBool[0] = cellIsoBool[3];

				xyz[4][0] = xyz[7][0];
				xyz[4][1] = xyz[7][1];
				xyz[4][2] = xyz[7][2];
				val[4] = _data[x][y + 1][z];
				cellIsoBool[4] = cellIsoBool[7];

				//inherit corner values from isoCache
				xyz[1][0] = isoCache[layerIndex][y - 1][z - 1].cornerPoint[0];
				xyz[1][1] = isoCache[layerIndex][y - 1][z - 1].cornerPoint[1];
				xyz[1][2] = isoCache[layerIndex][y - 1][z - 1].cornerPoint[2];
				val[1] = _data[x + 1][y][z];
				cellIsoBool[1] = isoCache[layerIndex][y - 1][z - 1].isoBool;

				xyz[2][0] = isoCache[layerIndex][y - 1][z].cornerPoint[0];
				xyz[2][1] = isoCache[layerIndex][y - 1][z].cornerPoint[1];
				xyz[2][2] = isoCache[layerIndex][y - 1][z].cornerPoint[2];
				val[2] = _data[x + 1][y][z + 1];
				cellIsoBool[2] = isoCache[layerIndex][y - 1][z].isoBool;

				xyz[5][0] = isoCache[layerIndex][y][z - 1].cornerPoint[0];
				xyz[5][1] = isoCache[layerIndex][y][z - 1].cornerPoint[1];
				xyz[5][2] = isoCache[layerIndex][y][z - 1].cornerPoint[2];
				val[5] = _data[x + 1][y + 1][z];
				cellIsoBool[5] = isoCache[layerIndex][y][z - 1].isoBool;

				//calculate corner values that could not be inherited
				xyz[3][0] = (float)((x - (_res.X / 2)) / ((float)(_res.X / 2)))*_dim.X;
				xyz[3][1] = (float)((y - (_res.Y / 2)) / ((float)(_res.Y / 2)))*_dim.Y;
				xyz[3][2] = (float)((z + 1 - (_res.Z / 2)) / ((float)(_res.Z / 2)))*_dim.Z;
				val[3] = _data[x][y][z + 1];
				if (_data[x][y][z + 1] < _iso_value)		// cubeIndex |= 8;
					cellIsoBool[3] = true;
				else
					cellIsoBool[3] = false;

				//save the sixth corner values to isoCache
				xyz[6][0] = isoCache[layerIndex][y][z].cornerPoint[0] = (float)((x + 1 - (_res.X / 2)) / ((float)(_res.X / 2)))*_dim.X;
				xyz[6][1] = isoCache[layerIndex][y][z].cornerPoint[1] = (float)((y + 1 - (_res.Y / 2)) / ((float)(_res.Y / 2)))*_dim.Y;
				xyz[6][2] = isoCache[layerIndex][y][z].cornerPoint[2] = (float)((z + 1 - (_res.Z / 2)) / ((float)(_res.Z / 2)))*_dim.Z;
				val[6] = _data[x + 1][y + 1][z + 1];
				if (_data[x + 1][y + 1][z + 1] < _iso_value)// cubeIndex |= 64;
					cellIsoBool[6] = isoCache[layerIndex][y][z].isoBool = true;
				else
					cellIsoBool[6] = isoCache[layerIndex][y][z].isoBool = false;

				xyz[7][0] = (float)((x - (_res.X / 2)) / ((float)(_res.X / 2)))*_dim.X;
				xyz[7][1] = (float)((y + 1 - (_res.Y / 2)) / ((float)(_res.Y / 2)))*_dim.Y;
				xyz[7][2] = (float)((z + 1 - (_res.Z / 2)) / ((float)(_res.Z / 2)))*_dim.Z;
				val[7] = _data[x][y + 1][z + 1];
				if (_data[x][y + 1][z + 1] < _iso_value)	// cubeIndex |= 128;
					cellIsoBool[7] = true;
				else
					cellIsoBool[7] = false;

				// get the case index
				cubeIndex = cellIsoBool[0] * 1 + cellIsoBool[1] * 2 + cellIsoBool[2] * 4 + cellIsoBool[3] * 8 +
					cellIsoBool[4] * 16 + cellIsoBool[5] * 32 + cellIsoBool[6] * 64 + cellIsoBool[7] * 128;

				// check if cube is entirely in or out of the surface ----------------------------
				if (edgeTable[cubeIndex] != 0) {

					// Find the vertices where the surface intersects the cube--------------------

					//inherit vertex indices from local variable ---------------------------------
					if (edgeTable[cubeIndex] & 8) {
						vertList[3] = y0Cache[z];
					}
					if (edgeTable[cubeIndex] & 256) {
						vertList[8] = vertList[11];
					}

					//inherit vertex indices from isoCache ---------------------------------------
					if (edgeTable[cubeIndex] & 1) {
						vertList[0] = isoCache[layerIndex][y - 1][z - 1].vertexIndex[1];
					}
					if (edgeTable[cubeIndex] & 16) {
						vertList[4] = isoCache[layerIndex][y][z - 1].vertexIndex[1];
					}
					if (edgeTable[cubeIndex] & 512) {
						vertList[9] = isoCache[layerIndex][y][z - 1].vertexIndex[2];
					}
					if (edgeTable[cubeIndex] & 2) {
						vertList[1] = isoCache[layerIndex][y - 1][z].vertexIndex[0];
					}
					if (edgeTable[cubeIndex] & 4) {
						vertList[2] = isoCache[layerIndex][y - 1][z].vertexIndex[1];
					}

					//calculate indices that could not be inherited ------------------------------

					if (edgeTable[cubeIndex] & 32) {
						dVal = (double)(_iso_value - val[5]) / (double)(val[6] - val[5]);
						//vertexArray[vertexCap][0] = xyz[5][0] + dVal*(xyz[6][0] - xyz[5][0]);
						//vertexArray[vertexCap][1] = xyz[5][1] + dVal*(xyz[6][1] - xyz[5][1]);
						//vertexArray[vertexCap][2] = xyz[5][2] + dVal*(xyz[6][2] - xyz[5][2]);
						vertexArray.Add(FVector(xyz[5][0] + dVal*(xyz[6][0] - xyz[5][0]), xyz[5][1] + dVal*(xyz[6][1] - xyz[5][1]), xyz[5][2] + dVal*(xyz[6][2] - xyz[5][2])));
						isoCache[layerIndex][y][z].vertexIndex[0] = vertList[5] = vertexCap;
						vertexCap++;
					}
					if (edgeTable[cubeIndex] & 64) {
						dVal = (double)(_iso_value - val[6]) / (double)(val[7] - val[6]);
						//vertexArray[vertexCap][0] = xyz[6][0] + dVal*(xyz[7][0] - xyz[6][0]);
						//vertexArray[vertexCap][1] = xyz[6][1] + dVal*(xyz[7][1] - xyz[6][1]);
						//vertexArray[vertexCap][2] = xyz[6][2] + dVal*(xyz[7][2] - xyz[6][2]);
						vertexArray.Add(FVector(xyz[6][0] + dVal*(xyz[7][0] - xyz[6][0]), xyz[6][1] + dVal*(xyz[7][1] - xyz[6][1]), xyz[6][2] + dVal*(xyz[7][2] - xyz[6][2])));
						isoCache[layerIndex][y][z].vertexIndex[1] = vertList[6] = vertexCap;
						vertexCap++;
					}
					if (edgeTable[cubeIndex] & 128) {
						dVal = (double)(_iso_value - val[7]) / (double)(val[4] - val[7]);
						//vertexArray[vertexCap][0] = xyz[7][0] + dVal*(xyz[4][0] - xyz[7][0]);
						//vertexArray[vertexCap][1] = xyz[7][1] + dVal*(xyz[4][1] - xyz[7][1]);
						//vertexArray[vertexCap][2] = xyz[7][2] + dVal*(xyz[4][2] - xyz[7][2]);
						vertexArray.Add(FVector(xyz[7][0] + dVal*(xyz[4][0] - xyz[7][0]), xyz[7][1] + dVal*(xyz[4][1] - xyz[7][1]), xyz[7][2] + dVal*(xyz[4][2] - xyz[7][2])));
						vertList[7] = y0Cache[z] = vertexCap;
						vertexCap++;
					}
					if (edgeTable[cubeIndex] & 1024) {
						dVal = (double)(_iso_value - val[2]) / (double)(val[6] - val[2]);
						//vertexArray[vertexCap][0] = xyz[2][0] + dVal*(xyz[6][0] - xyz[2][0]);
						//vertexArray[vertexCap][1] = xyz[2][1] + dVal*(xyz[6][1] - xyz[2][1]);
						//vertexArray[vertexCap][2] = xyz[2][2] + dVal*(xyz[6][2] - xyz[2][2]);
						vertexArray.Add(FVector(xyz[2][0] + dVal*(xyz[6][0] - xyz[2][0]), xyz[2][1] + dVal*(xyz[6][1] - xyz[2][1]), xyz[2][2] + dVal*(xyz[6][2] - xyz[2][2])));
						isoCache[layerIndex][y][z].vertexIndex[2] = vertList[10] = vertexCap;
						vertexCap++;
					}
					if (edgeTable[cubeIndex] & 2048) {
						dVal = (double)(_iso_value - val[3]) / (double)(val[7] - val[3]);
						//vertexArray[vertexCap][0] = xyz[3][0] + dVal*(xyz[7][0] - xyz[3][0]);
						//vertexArray[vertexCap][1] = xyz[3][1] + dVal*(xyz[7][1] - xyz[3][1]);
						//vertexArray[vertexCap][2] = xyz[3][2] + dVal*(xyz[7][2] - xyz[3][2]);
						vertexArray.Add(FVector(xyz[3][0] + dVal*(xyz[7][0] - xyz[3][0]), xyz[3][1] + dVal*(xyz[7][1] - xyz[3][1]), xyz[3][2] + dVal*(xyz[7][2] - xyz[3][2])));
						vertList[11] = vertexCap;
						vertexCap++;
					}

					// bind triangle indecies
					for (int i = 0; triTable[cubeIndex][i] != -1; i += 3) {
						/*triangleArray[triangleCap*3 + 2] = vertList[triTable[cubeIndex][i]];
						triangleArray[triangleCap*3 + 1] = vertList[triTable[cubeIndex][i + 1]];
						triangleArray[triangleCap*3] = vertList[triTable[cubeIndex][i + 2]];*/
						triangleArray.Add(vertList[triTable[cubeIndex][i]]);
						triangleArray.Add(vertList[triTable[cubeIndex][i + 1]]);
						triangleArray.Add(vertList[triTable[cubeIndex][i + 2]]);

						triangleCap++;
					}
				}
			}
		}

		layerIndex++; // move to next layer
					  // create remaining layers ============================================================================
		for (x = 1; x < _res.X - 1; x++) {
			y = 0;
			z = 0;
			//create first voxel of first row of remaining layers ---------------------------------------------


			//inherit corner values from local variable -------------------------------------------------------


			//inherit corner values from isoCache -------------------------------------------------------------
			xyz[7][0] = isoCache[(layerIndex + 1) % 2][y][z].cornerPoint[0];
			xyz[7][1] = isoCache[(layerIndex + 1) % 2][y][z].cornerPoint[1];
			xyz[7][2] = isoCache[(layerIndex + 1) % 2][y][z].cornerPoint[2];
			val[7] = _data[x][y + 1][z + 1];
			cellIsoBool[7] = isoCache[(layerIndex + 1) % 2][y][z].isoBool;

			//calculate corner values that could not be inherited
			xyz[0][0] = (float)((x - (_res.X / 2)) / ((float)(_res.X / 2)))*_dim.X;
			xyz[0][1] = (float)((y - (_res.Y / 2)) / ((float)(_res.Y / 2)))*_dim.Y;
			xyz[0][2] = (float)((z - (_res.Z / 2)) / ((float)(_res.Z / 2)))*_dim.Z;
			val[0] = _data[x][y][z];
			if (_data[x][y][z] < _iso_value)			// cubeIndex |= 1;
				cellIsoBool[0] = true;
			else
				cellIsoBool[0] = false;

			xyz[1][0] = (float)((x + 1 - (_res.X / 2)) / ((float)(_res.X / 2)))*_dim.X;
			xyz[1][1] = (float)((y - (_res.Y / 2)) / ((float)(_res.Y / 2)))*_dim.Y;
			xyz[1][2] = (float)((z - (_res.Z / 2)) / ((float)(_res.Z / 2)))*_dim.Z;
			val[1] = _data[x + 1][y][z];
			if (_data[x + 1][y][z] < _iso_value)		// cubeIndex |= 2;
				cellIsoBool[1] = true;
			else
				cellIsoBool[1] = false;

			xyz[2][0] = (float)((x + 1 - (_res.X / 2)) / ((float)(_res.X / 2)))*_dim.X;
			xyz[2][1] = (float)((y - (_res.Y / 2)) / ((float)(_res.Y / 2)))*_dim.Y;
			xyz[2][2] = (float)((z + 1 - (_res.Z / 2)) / ((float)(_res.Z / 2)))*_dim.Z;
			val[2] = _data[x + 1][y][z + 1];
			if (_data[x + 1][y][z + 1] < _iso_value)	// cubeIndex |= 4;
				cellIsoBool[2] = true;
			else
				cellIsoBool[2] = false;

			xyz[3][0] = (float)((x - (_res.X / 2)) / ((float)(_res.X / 2)))*_dim.X;
			xyz[3][1] = (float)((y - (_res.Y / 2)) / ((float)(_res.Y / 2)))*_dim.Y;
			xyz[3][2] = (float)((z + 1 - (_res.Z / 2)) / ((float)(_res.Z / 2)))*_dim.Z;
			val[3] = _data[x][y][z + 1];
			if (_data[x][y][z + 1] < _iso_value)		// cubeIndex |= 8;
				cellIsoBool[3] = true;
			else
				cellIsoBool[3] = false;

			xyz[4][0] = (float)((x - (_res.X / 2)) / ((float)(_res.X / 2)))*_dim.X;
			xyz[4][1] = (float)((y + 1 - (_res.Y / 2)) / ((float)(_res.Y / 2)))*_dim.Y;
			xyz[4][2] = (float)((z - (_res.Z / 2)) / ((float)(_res.Z / 2)))*_dim.Z;
			val[4] = _data[x][y + 1][z];
			if (_data[x][y + 1][z] < _iso_value)		//cubeIndex |= 16;
				cellIsoBool[4] = true;
			else
				cellIsoBool[4] = false;

			xyz[5][0] = (float)((x + 1 - (_res.X / 2)) / ((float)(_res.X / 2)))*_dim.X;
			xyz[5][1] = (float)((y + 1 - (_res.Y / 2)) / ((float)(_res.Y / 2)))*_dim.Y;
			xyz[5][2] = (float)((z - (_res.Z / 2)) / ((float)(_res.Z / 2)))*_dim.Z;
			val[5] = _data[x + 1][y + 1][z];
			if (_data[x + 1][y + 1][z] < _iso_value)	// cubeIndex |= 32;
				cellIsoBool[5] = true;
			else
				cellIsoBool[5] = false;

			//save the sixth corner values to isoCache
			xyz[6][0] = isoCache[layerIndex][y][z].cornerPoint[0] = (float)((x + 1 - (_res.X / 2)) / ((float)(_res.X / 2)))*_dim.X;
			xyz[6][1] = isoCache[layerIndex][y][z].cornerPoint[1] = (float)((y + 1 - (_res.Y / 2)) / ((float)(_res.Y / 2)))*_dim.Y;
			xyz[6][2] = isoCache[layerIndex][y][z].cornerPoint[2] = (float)((z + 1 - (_res.Z / 2)) / ((float)(_res.Z / 2)))*_dim.Z;
			val[6] = _data[x + 1][y + 1][z + 1];
			if (_data[x + 1][y + 1][z + 1] < _iso_value)// cubeIndex |= 64;
				cellIsoBool[6] = isoCache[layerIndex][y][z].isoBool = true;
			else
				cellIsoBool[6] = isoCache[layerIndex][y][z].isoBool = false;

			// get the case index
			cubeIndex = cellIsoBool[0] * 1 + cellIsoBool[1] * 2 + cellIsoBool[2] * 4 + cellIsoBool[3] * 8 +
				cellIsoBool[4] * 16 + cellIsoBool[5] * 32 + cellIsoBool[6] * 64 + cellIsoBool[7] * 128;

			// check if cube is entirely in or out of the surface -----------------------------------------------
			if (edgeTable[cubeIndex] != 0) {

				// Find the vertices where the surface intersects the cube---------------------------------------

				//inherit vertex indices from local variable ----------------------------------------------------

				//inherit vertex indices from isoCache ----------------------------------------------------------
				if (edgeTable[cubeIndex] & 128) {
					vertList[7] = isoCache[(layerIndex + 1) % 2][y][z].vertexIndex[0];
				}

				if (edgeTable[cubeIndex] & 2048) {
					vertList[11] = isoCache[(layerIndex + 1) % 2][y][z].vertexIndex[2];
				}

				//calculate indices that could not be inherited ------------------------------
				if (edgeTable[cubeIndex] & 1) {
					dVal = (double)(_iso_value - val[0]) / (double)(val[1] - val[0]);
					/*vertexArray[vertexCap][0] = xyz[0][0] + dVal*(xyz[1][0] - xyz[0][0]);
					vertexArray[vertexCap][1] = xyz[0][1] + dVal*(xyz[1][1] - xyz[0][1]);
					vertexArray[vertexCap][2] = xyz[0][2] + dVal*(xyz[1][2] - xyz[0][2]);*/
					vertexArray.Add(FVector(xyz[0][0] + dVal*(xyz[1][0] - xyz[0][0]), xyz[0][1] + dVal*(xyz[1][1] - xyz[0][1]), xyz[0][2] + dVal*(xyz[1][2] - xyz[0][2])));
					vertList[0] = vertexCap;
					vertexCap++;
				}

				if (edgeTable[cubeIndex] & 2) {
					dVal = (double)(_iso_value - val[1]) / (double)(val[2] - val[1]);
					/*vertexArray[vertexCap][0] = xyz[1][0] + dVal*(xyz[2][0] - xyz[1][0]);
					vertexArray[vertexCap][1] = xyz[1][1] + dVal*(xyz[2][1] - xyz[1][1]);
					vertexArray[vertexCap][2] = xyz[1][2] + dVal*(xyz[2][2] - xyz[1][2]);*/
					vertexArray.Add(FVector(xyz[1][0] + dVal*(xyz[2][0] - xyz[1][0]), xyz[1][1] + dVal*(xyz[2][1] - xyz[1][1]), xyz[1][2] + dVal*(xyz[2][2] - xyz[1][2])));
					vertList[1] = vertexCap;
					vertexCap++;
				}
				if (edgeTable[cubeIndex] & 4) {
					dVal = (double)(_iso_value - val[2]) / (double)(val[3] - val[2]);
					//vertexArray[vertexCap][0] = xyz[2][0] + dVal*(xyz[3][0] - xyz[2][0]);
					//vertexArray[vertexCap][1] = xyz[2][1] + dVal*(xyz[3][1] - xyz[2][1]);
					//vertexArray[vertexCap][2] = xyz[2][2] + dVal*(xyz[3][2] - xyz[2][2]);
					vertexArray.Add(FVector(xyz[2][0] + dVal*(xyz[3][0] - xyz[2][0]), xyz[2][1] + dVal*(xyz[3][1] - xyz[2][1]), xyz[2][2] + dVal*(xyz[3][2] - xyz[2][2])));
					vertList[2] = vertexCap;
					vertexCap++;
				}
				if (edgeTable[cubeIndex] & 8) {
					dVal = (double)(_iso_value - val[3]) / (double)(val[0] - val[3]);
					/*	vertexArray[vertexCap][0] = xyz[3][0] + dVal*(xyz[0][0] - xyz[3][0]);
					vertexArray[vertexCap][1] = xyz[3][1] + dVal*(xyz[0][1] - xyz[3][1]);
					vertexArray[vertexCap][2] = xyz[3][2] + dVal*(xyz[0][2] - xyz[3][2]);*/
					vertexArray.Add(FVector(xyz[3][0] + dVal*(xyz[0][0] - xyz[3][0]), xyz[3][1] + dVal*(xyz[0][1] - xyz[3][1]), xyz[3][2] + dVal*(xyz[0][2] - xyz[3][2])));
					vertList[3] = vertexCap;
					vertexCap++;
				}
				if (edgeTable[cubeIndex] & 16) {
					dVal = (double)(_iso_value - val[4]) / (double)(val[5] - val[4]);
					//vertexArray[vertexCap][0] = xyz[4][0] + dVal*(xyz[5][0] - xyz[4][0]);
					//vertexArray[vertexCap][1] = xyz[4][1] + dVal*(xyz[5][1] - xyz[4][1]);
					//vertexArray[vertexCap][2] = xyz[4][2] + dVal*(xyz[5][2] - xyz[4][2]);
					vertexArray.Add(FVector(xyz[4][0] + dVal*(xyz[5][0] - xyz[4][0]), xyz[4][1] + dVal*(xyz[5][1] - xyz[4][1]), xyz[4][2] + dVal*(xyz[5][2] - xyz[4][2])));
					vertList[4] = z0Cache = vertexCap;
					vertexCap++;
				}
				if (edgeTable[cubeIndex] & 32) {
					dVal = (double)(_iso_value - val[5]) / (double)(val[6] - val[5]);
					//vertexArray[vertexCap][0] = xyz[5][0] + dVal*(xyz[6][0] - xyz[5][0]);
					//vertexArray[vertexCap][1] = xyz[5][1] + dVal*(xyz[6][1] - xyz[5][1]);
					//vertexArray[vertexCap][2] = xyz[5][2] + dVal*(xyz[6][2] - xyz[5][2]);
					vertexArray.Add(FVector(xyz[5][0] + dVal*(xyz[6][0] - xyz[5][0]), xyz[5][1] + dVal*(xyz[6][1] - xyz[5][1]), xyz[5][2] + dVal*(xyz[6][2] - xyz[5][2])));
					isoCache[layerIndex][y][z].vertexIndex[0] = vertList[5] = vertexCap;
					vertexCap++;
				}
				if (edgeTable[cubeIndex] & 64) {
					dVal = (double)(_iso_value - val[6]) / (double)(val[7] - val[6]);
					//vertexArray[vertexCap][0] = xyz[6][0] + dVal*(xyz[7][0] - xyz[6][0]);
					//vertexArray[vertexCap][1] = xyz[6][1] + dVal*(xyz[7][1] - xyz[6][1]);
					//vertexArray[vertexCap][2] = xyz[6][2] + dVal*(xyz[7][2] - xyz[6][2]);
					vertexArray.Add(FVector(xyz[6][0] + dVal*(xyz[7][0] - xyz[6][0]), xyz[6][1] + dVal*(xyz[7][1] - xyz[6][1]), xyz[6][2] + dVal*(xyz[7][2] - xyz[6][2])));
					isoCache[layerIndex][y][z].vertexIndex[1] = vertList[6] = vertexCap;
					vertexCap++;
				}

				if (edgeTable[cubeIndex] & 256) {
					dVal = (double)(_iso_value - val[0]) / (double)(val[4] - val[0]);
					//vertexArray[vertexCap][0] = xyz[0][0] + dVal*(xyz[4][0] - xyz[0][0]);
					//vertexArray[vertexCap][1] = xyz[0][1] + dVal*(xyz[4][1] - xyz[0][1]);
					//vertexArray[vertexCap][2] = xyz[0][2] + dVal*(xyz[4][2] - xyz[0][2]);
					vertexArray.Add(FVector(xyz[0][0] + dVal*(xyz[4][0] - xyz[0][0]), xyz[0][1] + dVal*(xyz[4][1] - xyz[0][1]), xyz[0][2] + dVal*(xyz[4][2] - xyz[0][2])));
					vertList[8] = vertexCap;
					vertexCap++;
				}
				if (edgeTable[cubeIndex] & 512) {
					dVal = (double)(_iso_value - val[1]) / (double)(val[5] - val[1]);
					//vertexArray[vertexCap][0] = xyz[1][0] + dVal*(xyz[5][0] - xyz[1][0]);
					//vertexArray[vertexCap][1] = xyz[1][1] + dVal*(xyz[5][1] - xyz[1][1]);
					//vertexArray[vertexCap][2] = xyz[1][2] + dVal*(xyz[5][2] - xyz[1][2]);
					vertexArray.Add(FVector(xyz[1][0] + dVal*(xyz[5][0] - xyz[1][0]), xyz[1][1] + dVal*(xyz[5][1] - xyz[1][1]), xyz[1][2] + dVal*(xyz[5][2] - xyz[1][2])));
					vertList[9] = vertexCap;
					vertexCap++;
				}
				if (edgeTable[cubeIndex] & 1024) {
					dVal = (double)(_iso_value - val[2]) / (double)(val[6] - val[2]);
					//vertexArray[vertexCap][0] = xyz[2][0] + dVal*(xyz[6][0] - xyz[2][0]);
					//vertexArray[vertexCap][1] = xyz[2][1] + dVal*(xyz[6][1] - xyz[2][1]);
					//vertexArray[vertexCap][2] = xyz[2][2] + dVal*(xyz[6][2] - xyz[2][2]);
					vertexArray.Add(FVector(xyz[2][0] + dVal*(xyz[6][0] - xyz[2][0]), xyz[2][1] + dVal*(xyz[6][1] - xyz[2][1]), xyz[2][2] + dVal*(xyz[6][2] - xyz[2][2])));
					isoCache[layerIndex][y][z].vertexIndex[2] = vertList[10] = vertexCap;
					vertexCap++;
				}

				// bind triangle indecies
				for (int i = 0; triTable[cubeIndex][i] != -1; i += 3) {
					/*triangleArray[triangleCap*3 + 2] = vertList[triTable[cubeIndex][i]];
					triangleArray[triangleCap*3 + 1] = vertList[triTable[cubeIndex][i + 1]];
					triangleArray[triangleCap*3] = vertList[triTable[cubeIndex][i + 2]];*/
					triangleArray.Add(vertList[triTable[cubeIndex][i]]);
					triangleArray.Add(vertList[triTable[cubeIndex][i + 1]]);
					triangleArray.Add(vertList[triTable[cubeIndex][i + 2]]);

					triangleCap++;
				}
			}

			//create remaining voxels of first row of remaining layers ===========================
			for (z = 1; z < _res.Z - 1; z++) {

				//inherit corner values from local variable --------------------------------------
				xyz[0][0] = xyz[3][0];
				xyz[0][1] = xyz[3][1];
				xyz[0][2] = xyz[3][2];
				val[0] = _data[x][y][z];
				cellIsoBool[0] = cellIsoBool[3];

				xyz[1][0] = xyz[2][0];
				xyz[1][1] = xyz[2][1];
				xyz[1][2] = xyz[2][2];
				val[1] = _data[x + 1][y][z];
				cellIsoBool[1] = cellIsoBool[2];

				//inherit corner values from isoCache --------------------------------------------
				xyz[4][0] = isoCache[(layerIndex + 1) % 2][y][z - 1].cornerPoint[0];
				xyz[4][1] = isoCache[(layerIndex + 1) % 2][y][z - 1].cornerPoint[1];
				xyz[4][2] = isoCache[(layerIndex + 1) % 2][y][z - 1].cornerPoint[2];
				val[4] = _data[x][y + 1][z];
				cellIsoBool[4] = isoCache[(layerIndex + 1) % 2][y][z - 1].isoBool;

				xyz[5][0] = isoCache[layerIndex][y][z - 1].cornerPoint[0];
				xyz[5][1] = isoCache[layerIndex][y][z - 1].cornerPoint[1];
				xyz[5][2] = isoCache[layerIndex][y][z - 1].cornerPoint[2];
				val[5] = _data[x + 1][y + 1][z];
				cellIsoBool[5] = isoCache[layerIndex][y][z - 1].isoBool;

				xyz[7][0] = isoCache[(layerIndex + 1) % 2][y][z].cornerPoint[0];
				xyz[7][1] = isoCache[(layerIndex + 1) % 2][y][z].cornerPoint[1];
				xyz[7][2] = isoCache[(layerIndex + 1) % 2][y][z].cornerPoint[2];
				val[7] = _data[x][y + 1][z + 1];
				cellIsoBool[7] = isoCache[(layerIndex + 1) % 2][y][z].isoBool;

				//calculate corner values that could not be inherited ---------------------------
				xyz[2][0] = (float)((x + 1 - (_res.X / 2)) / ((float)(_res.X / 2)))*_dim.X;
				xyz[2][1] = (float)((y - (_res.Y / 2)) / ((float)(_res.Y / 2)))*_dim.Y;
				xyz[2][2] = (float)((z + 1 - (_res.Z / 2)) / ((float)(_res.Z / 2)))*_dim.Z;
				val[2] = _data[x + 1][y][z + 1];
				if (_data[x + 1][y][z + 1] < _iso_value)	// cubeIndex |= 4;
					cellIsoBool[2] = true;
				else
					cellIsoBool[2] = false;

				xyz[3][0] = (float)((x - (_res.X / 2)) / ((float)(_res.X / 2)))*_dim.X;
				xyz[3][1] = (float)((y - (_res.Y / 2)) / ((float)(_res.Y / 2)))*_dim.Y;
				xyz[3][2] = (float)((z + 1 - (_res.Z / 2)) / ((float)(_res.Z / 2)))*_dim.Z;
				val[3] = _data[x][y][z + 1];
				if (_data[x][y][z + 1] < _iso_value)		// cubeIndex |= 8;
					cellIsoBool[3] = true;
				else
					cellIsoBool[3] = false;

				//save the sixth corner values to isoCache
				xyz[6][0] = isoCache[layerIndex][y][z].cornerPoint[0] = (float)((x + 1 - (_res.X / 2)) / ((float)(_res.X / 2)))*_dim.X;
				xyz[6][1] = isoCache[layerIndex][y][z].cornerPoint[1] = (float)((y + 1 - (_res.Y / 2)) / ((float)(_res.Y / 2)))*_dim.Y;
				xyz[6][2] = isoCache[layerIndex][y][z].cornerPoint[2] = (float)((z + 1 - (_res.Z / 2)) / ((float)(_res.Z / 2)))*_dim.Z;
				val[6] = _data[x + 1][y + 1][z + 1];
				if (_data[x + 1][y + 1][z + 1] < _iso_value)// cubeIndex |= 64;
					cellIsoBool[6] = isoCache[layerIndex][y][z].isoBool = true;
				else
					cellIsoBool[6] = isoCache[layerIndex][y][z].isoBool = false;

				// get the case index
				cubeIndex = cellIsoBool[0] * 1 + cellIsoBool[1] * 2 + cellIsoBool[2] * 4 + cellIsoBool[3] * 8 +
					cellIsoBool[4] * 16 + cellIsoBool[5] * 32 + cellIsoBool[6] * 64 + cellIsoBool[7] * 128;

				// check if cube is entirely in or out of the surface ----------------------------
				if (edgeTable[cubeIndex] != 0) {

					// Find the vertices where the surface intersects the cube--------------------

					//inherit vertex indices from local variable ---------------------------------
					if (edgeTable[cubeIndex] & 1) {
						vertList[0] = vertList[2];
					}


					//inherit vertex indices from isoCache ---------------------------------------
					if (edgeTable[cubeIndex] & 16) {
						vertList[4] = isoCache[layerIndex][y][z - 1].vertexIndex[1];
					}
					if (edgeTable[cubeIndex] & 128) {
						vertList[7] = isoCache[(layerIndex + 1) % 2][y][z].vertexIndex[0];
					}
					if (edgeTable[cubeIndex] & 256) {
						vertList[8] = isoCache[(layerIndex + 1) % 2][y][z - 1].vertexIndex[2];
					}
					if (edgeTable[cubeIndex] & 512) {
						vertList[9] = isoCache[layerIndex][y][z - 1].vertexIndex[2];
					}
					if (edgeTable[cubeIndex] & 2048) {
						vertList[11] = isoCache[(layerIndex + 1) % 2][y][z].vertexIndex[2];
					}

					//calculate indices that could not be inherited ------------------------------
					if (edgeTable[cubeIndex] & 2) {
						dVal = (double)(_iso_value - val[1]) / (double)(val[2] - val[1]);
						/*vertexArray[vertexCap][0] = xyz[1][0] + dVal*(xyz[2][0] - xyz[1][0]);
						vertexArray[vertexCap][1] = xyz[1][1] + dVal*(xyz[2][1] - xyz[1][1]);
						vertexArray[vertexCap][2] = xyz[1][2] + dVal*(xyz[2][2] - xyz[1][2]);*/
						vertexArray.Add(FVector(xyz[1][0] + dVal*(xyz[2][0] - xyz[1][0]), xyz[1][1] + dVal*(xyz[2][1] - xyz[1][1]), xyz[1][2] + dVal*(xyz[2][2] - xyz[1][2])));
						vertList[1] = vertexCap;
						vertexCap++;
					}
					if (edgeTable[cubeIndex] & 4) {
						dVal = (double)(_iso_value - val[2]) / (double)(val[3] - val[2]);
						//vertexArray[vertexCap][0] = xyz[2][0] + dVal*(xyz[3][0] - xyz[2][0]);
						//vertexArray[vertexCap][1] = xyz[2][1] + dVal*(xyz[3][1] - xyz[2][1]);
						//vertexArray[vertexCap][2] = xyz[2][2] + dVal*(xyz[3][2] - xyz[2][2]);
						vertexArray.Add(FVector(xyz[2][0] + dVal*(xyz[3][0] - xyz[2][0]), xyz[2][1] + dVal*(xyz[3][1] - xyz[2][1]), xyz[2][2] + dVal*(xyz[3][2] - xyz[2][2])));
						vertList[2] = vertexCap;
						vertexCap++;
					}
					if (edgeTable[cubeIndex] & 8) {
						dVal = (double)(_iso_value - val[3]) / (double)(val[0] - val[3]);
						/*			vertexArray[vertexCap][0] = xyz[3][0] + dVal*(xyz[0][0] - xyz[3][0]);
						vertexArray[vertexCap][1] = xyz[3][1] + dVal*(xyz[0][1] - xyz[3][1]);
						vertexArray[vertexCap][2] = xyz[3][2] + dVal*(xyz[0][2] - xyz[3][2]);*/
						vertexArray.Add(FVector(xyz[3][0] + dVal*(xyz[0][0] - xyz[3][0]), xyz[3][1] + dVal*(xyz[0][1] - xyz[3][1]), xyz[3][2] + dVal*(xyz[0][2] - xyz[3][2])));
						vertList[3] = vertexCap;
						vertexCap++;
					}

					if (edgeTable[cubeIndex] & 32) {
						dVal = (double)(_iso_value - val[5]) / (double)(val[6] - val[5]);
						//vertexArray[vertexCap][0] = xyz[5][0] + dVal*(xyz[6][0] - xyz[5][0]);
						//vertexArray[vertexCap][1] = xyz[5][1] + dVal*(xyz[6][1] - xyz[5][1]);
						//vertexArray[vertexCap][2] = xyz[5][2] + dVal*(xyz[6][2] - xyz[5][2]);
						vertexArray.Add(FVector(xyz[5][0] + dVal*(xyz[6][0] - xyz[5][0]), xyz[5][1] + dVal*(xyz[6][1] - xyz[5][1]), xyz[5][2] + dVal*(xyz[6][2] - xyz[5][2])));
						isoCache[layerIndex][y][z].vertexIndex[0] = vertList[5] = vertexCap;
						vertexCap++;
					}
					if (edgeTable[cubeIndex] & 64) {
						dVal = (double)(_iso_value - val[6]) / (double)(val[7] - val[6]);
						//vertexArray[vertexCap][0] = xyz[6][0] + dVal*(xyz[7][0] - xyz[6][0]);
						//vertexArray[vertexCap][1] = xyz[6][1] + dVal*(xyz[7][1] - xyz[6][1]);
						//vertexArray[vertexCap][2] = xyz[6][2] + dVal*(xyz[7][2] - xyz[6][2]);
						vertexArray.Add(FVector(xyz[6][0] + dVal*(xyz[7][0] - xyz[6][0]), xyz[6][1] + dVal*(xyz[7][1] - xyz[6][1]), xyz[6][2] + dVal*(xyz[7][2] - xyz[6][2])));
						isoCache[layerIndex][y][z].vertexIndex[1] = vertList[6] = vertexCap;
						vertexCap++;
					}
					if (edgeTable[cubeIndex] & 1024) {
						dVal = (double)(_iso_value - val[2]) / (double)(val[6] - val[2]);
						//vertexArray[vertexCap][0] = xyz[2][0] + dVal*(xyz[6][0] - xyz[2][0]);
						//vertexArray[vertexCap][1] = xyz[2][1] + dVal*(xyz[6][1] - xyz[2][1]);
						//vertexArray[vertexCap][2] = xyz[2][2] + dVal*(xyz[6][2] - xyz[2][2]);
						vertexArray.Add(FVector(xyz[2][0] + dVal*(xyz[6][0] - xyz[2][0]), xyz[2][1] + dVal*(xyz[6][1] - xyz[2][1]), xyz[2][2] + dVal*(xyz[6][2] - xyz[2][2])));
						isoCache[layerIndex][y][z].vertexIndex[2] = vertList[10] = vertexCap;
						vertexCap++;
					}

					// bind triangle indecies
					for (int i = 0; triTable[cubeIndex][i] != -1; i += 3) {
						/*triangleArray[triangleCap*3 + 2] = vertList[triTable[cubeIndex][i]];
						triangleArray[triangleCap*3 + 1] = vertList[triTable[cubeIndex][i + 1]];
						triangleArray[triangleCap*3] = vertList[triTable[cubeIndex][i + 2]];*/
						triangleArray.Add(vertList[triTable[cubeIndex][i]]);
						triangleArray.Add(vertList[triTable[cubeIndex][i + 1]]);
						triangleArray.Add(vertList[triTable[cubeIndex][i + 2]]);

						triangleCap++;
					}
				}
			}

			//create remaining rows of remaining layers ==============================================
			for (y = 1; y < _res.Y - 1; y++) {

				//create first voxel of remaining rows of remaining layers
				z = 0;

				//create the first voxel of remaining rows of first layer=============================

				//inherit corner values from local variable -----------------------------------------

				//inherit corner values from isoCache -----------------------------------------------
				xyz[2][0] = isoCache[layerIndex][y - 1][z].cornerPoint[0];
				xyz[2][1] = isoCache[layerIndex][y - 1][z].cornerPoint[1];
				xyz[2][2] = isoCache[layerIndex][y - 1][z].cornerPoint[2];
				val[2] = _data[x + 1][y][z + 1];
				cellIsoBool[2] = isoCache[layerIndex][y - 1][z].isoBool;

				xyz[3][0] = isoCache[(layerIndex + 1) % 2][y - 1][z].cornerPoint[0];
				xyz[3][1] = isoCache[(layerIndex + 1) % 2][y - 1][z].cornerPoint[1];
				xyz[3][2] = isoCache[(layerIndex + 1) % 2][y - 1][z].cornerPoint[2];
				val[3] = _data[x][y][z + 1];
				cellIsoBool[3] = isoCache[(layerIndex + 1) % 2][y - 1][z].isoBool;

				xyz[7][0] = isoCache[(layerIndex + 1) % 2][y][z].cornerPoint[0];
				xyz[7][1] = isoCache[(layerIndex + 1) % 2][y][z].cornerPoint[1];
				xyz[7][2] = isoCache[(layerIndex + 1) % 2][y][z].cornerPoint[2];
				val[7] = _data[x][y + 1][z + 1];
				cellIsoBool[7] = isoCache[(layerIndex + 1) % 2][y][z].isoBool;

				//calculate corner values that could not be inherited ---------------------------------
				xyz[0][0] = (float)((x - (_res.X / 2)) / ((float)(_res.X / 2)))*_dim.X;
				xyz[0][1] = (float)((y - (_res.Y / 2)) / ((float)(_res.Y / 2)))*_dim.Y;
				xyz[0][2] = (float)((z - (_res.Z / 2)) / ((float)(_res.Z / 2)))*_dim.Z;
				val[0] = _data[x][y][z];
				if (_data[x][y][z] < _iso_value)			// cubeIndex |= 1;
					cellIsoBool[0] = true;
				else
					cellIsoBool[0] = false;

				xyz[1][0] = (float)((x + 1 - (_res.X / 2)) / ((float)(_res.X / 2)))*_dim.X;
				xyz[1][1] = (float)((y - (_res.Y / 2)) / ((float)(_res.Y / 2)))*_dim.Y;
				xyz[1][2] = (float)((z - (_res.Z / 2)) / ((float)(_res.Z / 2)))*_dim.Z;
				val[1] = _data[x + 1][y][z];
				if (_data[x + 1][y][z] < _iso_value)		// cubeIndex |= 2;
					cellIsoBool[1] = true;
				else
					cellIsoBool[1] = false;

				xyz[4][0] = (float)((x - (_res.X / 2)) / ((float)(_res.X / 2)))*_dim.X;
				xyz[4][1] = (float)((y + 1 - (_res.Y / 2)) / ((float)(_res.Y / 2)))*_dim.Y;
				xyz[4][2] = (float)((z - (_res.Z / 2)) / ((float)(_res.Z / 2)))*_dim.Z;
				val[4] = _data[x][y + 1][z];
				if (_data[x][y + 1][z] < _iso_value)		//cubeIndex |= 16;
					cellIsoBool[4] = true;
				else
					cellIsoBool[4] = false;

				xyz[5][0] = (float)((x + 1 - (_res.X / 2)) / ((float)(_res.X / 2)))*_dim.X;
				xyz[5][1] = (float)((y + 1 - (_res.Y / 2)) / ((float)(_res.Y / 2)))*_dim.Y;
				xyz[5][2] = (float)((z - (_res.Z / 2)) / ((float)(_res.Z / 2)))*_dim.Z;
				val[5] = _data[x + 1][y + 1][z];
				if (_data[x + 1][y + 1][z] < _iso_value)	// cubeIndex |= 32;
					cellIsoBool[5] = true;
				else
					cellIsoBool[5] = false;

				//save the sixth corner values to isoCache
				xyz[6][0] = isoCache[layerIndex][y][z].cornerPoint[0] = (float)((x + 1 - (_res.X / 2)) / ((float)(_res.X / 2)))*_dim.X;
				xyz[6][1] = isoCache[layerIndex][y][z].cornerPoint[1] = (float)((y + 1 - (_res.Y / 2)) / ((float)(_res.Y / 2)))*_dim.Y;
				xyz[6][2] = isoCache[layerIndex][y][z].cornerPoint[2] = (float)((z + 1 - (_res.Z / 2)) / ((float)(_res.Z / 2)))*_dim.Z;
				val[6] = _data[x + 1][y + 1][z + 1];
				if (_data[x + 1][y + 1][z + 1] < _iso_value)// cubeIndex |= 64;
					cellIsoBool[6] = isoCache[layerIndex][y][z].isoBool = true;
				else
					cellIsoBool[6] = isoCache[layerIndex][y][z].isoBool = false;

				// get the case index
				cubeIndex = cellIsoBool[0] * 1 + cellIsoBool[1] * 2 + cellIsoBool[2] * 4 + cellIsoBool[3] * 8 +
					cellIsoBool[4] * 16 + cellIsoBool[5] * 32 + cellIsoBool[6] * 64 + cellIsoBool[7] * 128;

				// check if cube is entirely in or out of the surface ----------------------------
				if (edgeTable[cubeIndex] != 0) {

					// Find the vertices where the surface intersects the cube--------------------

					//inherit vertex indices from local variable ---------------------------------
					if (edgeTable[cubeIndex] & 1) {
						vertList[0] = z0Cache;
					}

					//inherit vertex indices from isoCache ---------------------------------------
					if (edgeTable[cubeIndex] & 2) {
						vertList[1] = isoCache[layerIndex][y - 1][z].vertexIndex[0];
					}
					if (edgeTable[cubeIndex] & 4) {
						vertList[2] = isoCache[layerIndex][y - 1][z].vertexIndex[1];
					}
					if (edgeTable[cubeIndex] & 8) {
						vertList[3] = isoCache[(layerIndex + 1) % 2][y - 1][z].vertexIndex[0];
					}
					if (edgeTable[cubeIndex] & 128) {
						vertList[7] = isoCache[(layerIndex + 1) % 2][y][z].vertexIndex[0];
					}
					if (edgeTable[cubeIndex] & 2048) {
						vertList[11] = isoCache[(layerIndex + 1) % 2][y][z].vertexIndex[2];
					}

					//calculate indices that could not be inherited ------------------------------


					if (edgeTable[cubeIndex] & 16) {
						dVal = (double)(_iso_value - val[4]) / (double)(val[5] - val[4]);
						//vertexArray[vertexCap][0] = xyz[4][0] + dVal*(xyz[5][0] - xyz[4][0]);
						//vertexArray[vertexCap][1] = xyz[4][1] + dVal*(xyz[5][1] - xyz[4][1]);
						//vertexArray[vertexCap][2] = xyz[4][2] + dVal*(xyz[5][2] - xyz[4][2]);
						vertexArray.Add(FVector(xyz[4][0] + dVal*(xyz[5][0] - xyz[4][0]), xyz[4][1] + dVal*(xyz[5][1] - xyz[4][1]), xyz[4][2] + dVal*(xyz[5][2] - xyz[4][2])));
						vertList[4] = z0Cache = vertexCap;
						vertexCap++;
					}
					if (edgeTable[cubeIndex] & 32) {
						dVal = (double)(_iso_value - val[5]) / (double)(val[6] - val[5]);
						//vertexArray[vertexCap][0] = xyz[5][0] + dVal*(xyz[6][0] - xyz[5][0]);
						//vertexArray[vertexCap][1] = xyz[5][1] + dVal*(xyz[6][1] - xyz[5][1]);
						//vertexArray[vertexCap][2] = xyz[5][2] + dVal*(xyz[6][2] - xyz[5][2]);
						vertexArray.Add(FVector(xyz[5][0] + dVal*(xyz[6][0] - xyz[5][0]), xyz[5][1] + dVal*(xyz[6][1] - xyz[5][1]), xyz[5][2] + dVal*(xyz[6][2] - xyz[5][2])));
						isoCache[layerIndex][y][z].vertexIndex[0] = vertList[5] = vertexCap;
						vertexCap++;
					}
					if (edgeTable[cubeIndex] & 64) {
						dVal = (double)(_iso_value - val[6]) / (double)(val[7] - val[6]);
						//vertexArray[vertexCap][0] = xyz[6][0] + dVal*(xyz[7][0] - xyz[6][0]);
						//vertexArray[vertexCap][1] = xyz[6][1] + dVal*(xyz[7][1] - xyz[6][1]);
						//vertexArray[vertexCap][2] = xyz[6][2] + dVal*(xyz[7][2] - xyz[6][2]);
						vertexArray.Add(FVector(xyz[6][0] + dVal*(xyz[7][0] - xyz[6][0]), xyz[6][1] + dVal*(xyz[7][1] - xyz[6][1]), xyz[6][2] + dVal*(xyz[7][2] - xyz[6][2])));
						isoCache[layerIndex][y][z].vertexIndex[1] = vertList[6] = vertexCap;
						vertexCap++;
					}

					if (edgeTable[cubeIndex] & 256) {
						dVal = (double)(_iso_value - val[0]) / (double)(val[4] - val[0]);
						//vertexArray[vertexCap][0] = xyz[0][0] + dVal*(xyz[4][0] - xyz[0][0]);
						//vertexArray[vertexCap][1] = xyz[0][1] + dVal*(xyz[4][1] - xyz[0][1]);
						//vertexArray[vertexCap][2] = xyz[0][2] + dVal*(xyz[4][2] - xyz[0][2]);
						vertexArray.Add(FVector(xyz[0][0] + dVal*(xyz[4][0] - xyz[0][0]), xyz[0][1] + dVal*(xyz[4][1] - xyz[0][1]), xyz[0][2] + dVal*(xyz[4][2] - xyz[0][2])));
						vertList[8] = vertexCap;
						vertexCap++;
					}
					if (edgeTable[cubeIndex] & 512) {
						dVal = (double)(_iso_value - val[1]) / (double)(val[5] - val[1]);
						//vertexArray[vertexCap][0] = xyz[1][0] + dVal*(xyz[5][0] - xyz[1][0]);
						//vertexArray[vertexCap][1] = xyz[1][1] + dVal*(xyz[5][1] - xyz[1][1]);
						//vertexArray[vertexCap][2] = xyz[1][2] + dVal*(xyz[5][2] - xyz[1][2]);
						vertexArray.Add(FVector(xyz[1][0] + dVal*(xyz[5][0] - xyz[1][0]), xyz[1][1] + dVal*(xyz[5][1] - xyz[1][1]), xyz[1][2] + dVal*(xyz[5][2] - xyz[1][2])));
						vertList[9] = vertexCap;
						vertexCap++;
					}
					if (edgeTable[cubeIndex] & 1024) {
						dVal = (double)(_iso_value - val[2]) / (double)(val[6] - val[2]);
						//vertexArray[vertexCap][0] = xyz[2][0] + dVal*(xyz[6][0] - xyz[2][0]);
						//vertexArray[vertexCap][1] = xyz[2][1] + dVal*(xyz[6][1] - xyz[2][1]);
						//vertexArray[vertexCap][2] = xyz[2][2] + dVal*(xyz[6][2] - xyz[2][2]);
						vertexArray.Add(FVector(xyz[2][0] + dVal*(xyz[6][0] - xyz[2][0]), xyz[2][1] + dVal*(xyz[6][1] - xyz[2][1]), xyz[2][2] + dVal*(xyz[6][2] - xyz[2][2])));
						isoCache[layerIndex][y][z].vertexIndex[2] = vertList[10] = vertexCap;
						vertexCap++;
					}

					// bind triangle indecies
					for (int i = 0; triTable[cubeIndex][i] != -1; i += 3) {
						/*triangleArray[triangleCap*3 + 2] = vertList[triTable[cubeIndex][i]];
						triangleArray[triangleCap*3 + 1] = vertList[triTable[cubeIndex][i + 1]];
						triangleArray[triangleCap*3] = vertList[triTable[cubeIndex][i + 2]];*/
						triangleArray.Add(vertList[triTable[cubeIndex][i]]);
						triangleArray.Add(vertList[triTable[cubeIndex][i + 1]]);
						triangleArray.Add(vertList[triTable[cubeIndex][i + 2]]);

						triangleCap++;
					}
				}

				//create remaining voxels of remaining rows of remaining layers ==================
				for (z = 1; z < _res.Z - 1; z++) {
					//inherit corner values from local variable -----------------------------------

					//inherit corner values from isoCache -----------------------------------------
					xyz[0][0] = isoCache[(layerIndex + 1) % 2][y - 1][z - 1].cornerPoint[0];
					xyz[0][1] = isoCache[(layerIndex + 1) % 2][y - 1][z - 1].cornerPoint[1];
					xyz[0][2] = isoCache[(layerIndex + 1) % 2][y - 1][z - 1].cornerPoint[2];
					val[0] = _data[x][y][z];
					cellIsoBool[0] = isoCache[(layerIndex + 1) % 2][y - 1][z - 1].isoBool;

					xyz[1][0] = isoCache[layerIndex][y - 1][z - 1].cornerPoint[0];
					xyz[1][1] = isoCache[layerIndex][y - 1][z - 1].cornerPoint[1];
					xyz[1][2] = isoCache[layerIndex][y - 1][z - 1].cornerPoint[2];
					val[1] = _data[x + 1][y][z];
					cellIsoBool[1] = isoCache[layerIndex][y - 1][z - 1].isoBool;

					xyz[2][0] = isoCache[layerIndex][y - 1][z].cornerPoint[0];
					xyz[2][1] = isoCache[layerIndex][y - 1][z].cornerPoint[1];
					xyz[2][2] = isoCache[layerIndex][y - 1][z].cornerPoint[2];
					val[2] = _data[x + 1][y][z + 1];
					cellIsoBool[2] = isoCache[layerIndex][y - 1][z].isoBool;

					xyz[3][0] = isoCache[(layerIndex + 1) % 2][y - 1][z].cornerPoint[0];
					xyz[3][1] = isoCache[(layerIndex + 1) % 2][y - 1][z].cornerPoint[1];
					xyz[3][2] = isoCache[(layerIndex + 1) % 2][y - 1][z].cornerPoint[2];
					val[3] = _data[x][y][z + 1];
					cellIsoBool[3] = isoCache[(layerIndex + 1) % 2][y - 1][z].isoBool;

					xyz[4][0] = isoCache[(layerIndex + 1) % 2][y][z - 1].cornerPoint[0];
					xyz[4][1] = isoCache[(layerIndex + 1) % 2][y][z - 1].cornerPoint[1];
					xyz[4][2] = isoCache[(layerIndex + 1) % 2][y][z - 1].cornerPoint[2];
					val[4] = _data[x][y + 1][z];
					cellIsoBool[4] = isoCache[(layerIndex + 1) % 2][y][z - 1].isoBool;

					xyz[5][0] = isoCache[layerIndex][y][z - 1].cornerPoint[0];
					xyz[5][1] = isoCache[layerIndex][y][z - 1].cornerPoint[1];
					xyz[5][2] = isoCache[layerIndex][y][z - 1].cornerPoint[2];
					val[5] = _data[x + 1][y + 1][z];
					cellIsoBool[5] = isoCache[layerIndex][y][z - 1].isoBool;

					xyz[7][0] = isoCache[(layerIndex + 1) % 2][y][z].cornerPoint[0];
					xyz[7][1] = isoCache[(layerIndex + 1) % 2][y][z].cornerPoint[1];
					xyz[7][2] = isoCache[(layerIndex + 1) % 2][y][z].cornerPoint[2];
					val[7] = _data[x][y + 1][z + 1];
					cellIsoBool[7] = isoCache[(layerIndex + 1) % 2][y][z].isoBool;

					//calculate corner values that could not be inherited -------------------------------------------------
					//save the sixth corner values to isoCache
					xyz[6][0] = isoCache[layerIndex][y][z].cornerPoint[0] = (float)((x + 1 - (_res.X / 2)) / ((float)(_res.X / 2)))*_dim.X;
					xyz[6][1] = isoCache[layerIndex][y][z].cornerPoint[1] = (float)((y + 1 - (_res.Y / 2)) / ((float)(_res.Y / 2)))*_dim.Y;
					xyz[6][2] = isoCache[layerIndex][y][z].cornerPoint[2] = (float)((z + 1 - (_res.Z / 2)) / ((float)(_res.Z / 2)))*_dim.Z;
					val[6] = _data[x + 1][y + 1][z + 1];
					if (_data[x + 1][y + 1][z + 1] < _iso_value)// cubeIndex |= 64;
						cellIsoBool[6] = isoCache[layerIndex][y][z].isoBool = true;
					else
						cellIsoBool[6] = isoCache[layerIndex][y][z].isoBool = false;

					// get the case index
					cubeIndex = cellIsoBool[0] * 1 + cellIsoBool[1] * 2 + cellIsoBool[2] * 4 + cellIsoBool[3] * 8 +
						cellIsoBool[4] * 16 + cellIsoBool[5] * 32 + cellIsoBool[6] * 64 + cellIsoBool[7] * 128;

					// check if cube is entirely in or out of the surface ----------------------------
					if (edgeTable[cubeIndex] != 0) {

						// Find the vertices where the surface intersects the cube--------------------

						//inherit vertex indices from local variable ---------------------------------


						//inherit vertex indices from isoCache ---------------------------------------
						if (edgeTable[cubeIndex] & 1) {
							vertList[0] = isoCache[layerIndex][y - 1][z - 1].vertexIndex[1];
						}
						if (edgeTable[cubeIndex] & 2) {
							vertList[1] = isoCache[layerIndex][y - 1][z].vertexIndex[0];
						}
						if (edgeTable[cubeIndex] & 4) {
							vertList[2] = isoCache[layerIndex][y - 1][z].vertexIndex[1];
						}
						if (edgeTable[cubeIndex] & 8) {
							vertList[3] = isoCache[(layerIndex + 1) % 2][y - 1][z].vertexIndex[0];
						}
						if (edgeTable[cubeIndex] & 16) {
							vertList[4] = isoCache[layerIndex][y][z - 1].vertexIndex[1];
						}
						if (edgeTable[cubeIndex] & 128) {
							vertList[7] = isoCache[(layerIndex + 1) % 2][y][z].vertexIndex[0];
						}
						if (edgeTable[cubeIndex] & 256) {
							vertList[8] = isoCache[(layerIndex + 1) % 2][y][z - 1].vertexIndex[2];
						}
						if (edgeTable[cubeIndex] & 512) {
							vertList[9] = isoCache[layerIndex][y][z - 1].vertexIndex[2];
						}
						if (edgeTable[cubeIndex] & 2048) {
							vertList[11] = isoCache[(layerIndex + 1) % 2][y][z].vertexIndex[2];
						}

						//calculate indices that could not be inherited ------------------------------			


						if (edgeTable[cubeIndex] & 32) {
							dVal = (double)(_iso_value - val[5]) / (double)(val[6] - val[5]);
							//vertexArray[vertexCap][0] = xyz[5][0] + dVal*(xyz[6][0] - xyz[5][0]);
							//vertexArray[vertexCap][1] = xyz[5][1] + dVal*(xyz[6][1] - xyz[5][1]);
							//vertexArray[vertexCap][2] = xyz[5][2] + dVal*(xyz[6][2] - xyz[5][2]);
							vertexArray.Add(FVector(xyz[5][0] + dVal*(xyz[6][0] - xyz[5][0]), xyz[5][1] + dVal*(xyz[6][1] - xyz[5][1]), xyz[5][2] + dVal*(xyz[6][2] - xyz[5][2])));
							isoCache[layerIndex][y][z].vertexIndex[0] = vertList[5] = vertexCap;
							vertexCap++;
						}
						if (edgeTable[cubeIndex] & 64) {
							dVal = (double)(_iso_value - val[6]) / (double)(val[7] - val[6]);
							/*				vertexArray[vertexCap][0] = xyz[6][0] + dVal*(xyz[7][0] - xyz[6][0]);
							vertexArray[vertexCap][1] = xyz[6][1] + dVal*(xyz[7][1] - xyz[6][1]);
							vertexArray[vertexCap][2] = xyz[6][2] + dVal*(xyz[7][2] - xyz[6][2]);*/
							vertexArray.Add(FVector(xyz[6][0] + dVal*(xyz[7][0] - xyz[6][0]), xyz[6][1] + dVal*(xyz[7][1] - xyz[6][1]), xyz[6][2] + dVal*(xyz[7][2] - xyz[6][2])));
							isoCache[layerIndex][y][z].vertexIndex[1] = vertList[6] = vertexCap;
							vertexCap++;
						}
						if (edgeTable[cubeIndex] & 1024) {
							dVal = (double)(_iso_value - val[2]) / (double)(val[6] - val[2]);
							//vertexArray[vertexCap][0] = xyz[2][0] + dVal*(xyz[6][0] - xyz[2][0]);
							//vertexArray[vertexCap][1] = xyz[2][1] + dVal*(xyz[6][1] - xyz[2][1]);
							//vertexArray[vertexCap][2] = xyz[2][2] + dVal*(xyz[6][2] - xyz[2][2]);
							vertexArray.Add(FVector(xyz[2][0] + dVal*(xyz[6][0] - xyz[2][0]), xyz[2][1] + dVal*(xyz[6][1] - xyz[2][1]), xyz[2][2] + dVal*(xyz[6][2] - xyz[2][2])));
							isoCache[layerIndex][y][z].vertexIndex[2] = vertList[10] = vertexCap;
							vertexCap++;
						}

						// bind triangle indecies
						for (int i = 0; triTable[cubeIndex][i] != -1; i += 3) {
							/*triangleArray[triangleCap*3 + 2] = vertList[triTable[cubeIndex][i]];
							triangleArray[triangleCap*3 + 1] = vertList[triTable[cubeIndex][i + 1]];
							triangleArray[triangleCap*3] = vertList[triTable[cubeIndex][i + 2]];*/
							triangleArray.Add(vertList[triTable[cubeIndex][i]]);
							triangleArray.Add(vertList[triTable[cubeIndex][i + 1]]);
							triangleArray.Add(vertList[triTable[cubeIndex][i + 2]]);

							triangleCap++;
						}
					}
				}
			}
			layerIndex = (layerIndex + 1) % 2;
		}


		auto UV0 = TArray<FVector2D>();
		auto Colors = TArray<FColor>();
		for (int i = 0; i < vertexArray.Num(); i++)
			Colors.Add(FColor(1.0f, 1.0f, 1.0f));


		calcNormals(&vertexArray, &triangleArray, &normalArray, &tangentArray);

		_mesh->CreateMeshSection(0, vertexArray, triangleArray, normalArray, UV0, Colors, tangentArray, true);
		//debugpoint
		UE_LOG(LogTemp, Warning, TEXT("finished..."));
		UE_LOG(LogTemp, Warning, TEXT("tris %f"), triangleArray.Num());
		UE_LOG(LogTemp, Warning, TEXT("verts %f"), vertexArray.Num());

		UE_LOG(LogTemp, Warning, TEXT("edgeTable %f"), edgeTable[130]);
		UE_LOG(LogTemp, Warning, TEXT("triTable %f"), triTable[0][0]);

		//cleanup
		for (int i = 0; i < 2; i++)
		{
			for (int j = 0; j < _res.Y; j++)
			{
				delete[] isoCache[i][j];
			}
		}

		for (int i = 0; i < 2; i++)
		{
			delete[] isoCache[i];
		}
		delete[] isoCache;
	}

	void calcNormals(TArray<FVector>* vertexArray, TArray<int32>* triangleArray, TArray<FVector>* normalArray, TArray<FProcMeshTangent>* tangentArray) {

		FVector v1;
		FVector v2;
		FVector v3;
		FVector e1;
		FVector	e2;
		FVector tangentX;
		FVector tangentZ;

		normalArray->SetNumZeroed(vertexArray->Num());
		//normalArray._reserve(vertexArray.Num());
		tangentArray->SetNumZeroed(vertexArray->Num());
		/*tangentArray._reserve(vertexArray.Num());*/

		for (int i = 0; i < triangleArray->Num(); i = i + 3) {
			v1 = (*vertexArray)[(*triangleArray)[i]];
			v2 = (*vertexArray)[(*triangleArray)[i + 1]];
			v3 = (*vertexArray)[(*triangleArray)[i + 2]];

			e1 = v2 - v1;
			e2 = v3 - v1;

			tangentX = e1.GetSafeNormal();

			tangentZ = (e2 ^ e1).GetSafeNormal();


			(*tangentArray)[(*triangleArray)[i]] = FProcMeshTangent(tangentX, false);
			(*normalArray)[(*triangleArray)[i]] = tangentZ;

			(*tangentArray)[(*triangleArray)[i + 1]] = FProcMeshTangent(tangentX, false);
			(*normalArray)[(*triangleArray)[i + 1]] = tangentZ;

			(*tangentArray)[(*triangleArray)[i + 2]] = FProcMeshTangent(tangentX, false);
			(*normalArray)[(*triangleArray)[i + 2]] = tangentZ;

		}
	}

	void initTables(int edgeTable[], int triTable[][16]) {

		int tempEdgeTable[256] = {
			0x0, 0x109, 0x203, 0x30a, 0x406, 0x50f, 0x605, 0x70c,
			0x80c, 0x905, 0xa0f, 0xb06, 0xc0a, 0xd03, 0xe09, 0xf00,
			0x190, 0x99, 0x393, 0x29a, 0x596, 0x49f, 0x795, 0x69c,
			0x99c, 0x895, 0xb9f, 0xa96, 0xd9a, 0xc93, 0xf99, 0xe90,
			0x230, 0x339, 0x33, 0x13a, 0x636, 0x73f, 0x435, 0x53c,
			0xa3c, 0xb35, 0x83f, 0x936, 0xe3a, 0xf33, 0xc39, 0xd30,
			0x3a0, 0x2a9, 0x1a3, 0xaa, 0x7a6, 0x6af, 0x5a5, 0x4ac,
			0xbac, 0xaa5, 0x9af, 0x8a6, 0xfaa, 0xea3, 0xda9, 0xca0,
			0x460, 0x569, 0x663, 0x76a, 0x66, 0x16f, 0x265, 0x36c,
			0xc6c, 0xd65, 0xe6f, 0xf66, 0x86a, 0x963, 0xa69, 0xb60,
			0x5f0, 0x4f9, 0x7f3, 0x6fa, 0x1f6, 0xff, 0x3f5, 0x2fc,
			0xdfc, 0xcf5, 0xfff, 0xef6, 0x9fa, 0x8f3, 0xbf9, 0xaf0,
			0x650, 0x759, 0x453, 0x55a, 0x256, 0x35f, 0x55, 0x15c,
			0xe5c, 0xf55, 0xc5f, 0xd56, 0xa5a, 0xb53, 0x859, 0x950,
			0x7c0, 0x6c9, 0x5c3, 0x4ca, 0x3c6, 0x2cf, 0x1c5, 0xcc,
			0xfcc, 0xec5, 0xdcf, 0xcc6, 0xbca, 0xac3, 0x9c9, 0x8c0,
			0x8c0, 0x9c9, 0xac3, 0xbca, 0xcc6, 0xdcf, 0xec5, 0xfcc,
			0xcc, 0x1c5, 0x2cf, 0x3c6, 0x4ca, 0x5c3, 0x6c9, 0x7c0,
			0x950, 0x859, 0xb53, 0xa5a, 0xd56, 0xc5f, 0xf55, 0xe5c,
			0x15c, 0x55, 0x35f, 0x256, 0x55a, 0x453, 0x759, 0x650,
			0xaf0, 0xbf9, 0x8f3, 0x9fa, 0xef6, 0xfff, 0xcf5, 0xdfc,
			0x2fc, 0x3f5, 0xff, 0x1f6, 0x6fa, 0x7f3, 0x4f9, 0x5f0,
			0xb60, 0xa69, 0x963, 0x86a, 0xf66, 0xe6f, 0xd65, 0xc6c,
			0x36c, 0x265, 0x16f, 0x66, 0x76a, 0x663, 0x569, 0x460,
			0xca0, 0xda9, 0xea3, 0xfaa, 0x8a6, 0x9af, 0xaa5, 0xbac,
			0x4ac, 0x5a5, 0x6af, 0x7a6, 0xaa, 0x1a3, 0x2a9, 0x3a0,
			0xd30, 0xc39, 0xf33, 0xe3a, 0x936, 0x83f, 0xb35, 0xa3c,
			0x53c, 0x435, 0x73f, 0x636, 0x13a, 0x33, 0x339, 0x230,
			0xe90, 0xf99, 0xc93, 0xd9a, 0xa96, 0xb9f, 0x895, 0x99c,
			0x69c, 0x795, 0x49f, 0x596, 0x29a, 0x393, 0x99, 0x190,
			0xf00, 0xe09, 0xd03, 0xc0a, 0xb06, 0xa0f, 0x905, 0x80c,
			0x70c, 0x605, 0x50f, 0x406, 0x30a, 0x203, 0x109, 0x0 };
		int tempTriTable[256][16] =
		{ { -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
		{ 0, 8, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
		{ 0, 1, 9, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
		{ 1, 8, 3, 9, 8, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
		{ 1, 2, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
		{ 0, 8, 3, 1, 2, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
		{ 9, 2, 10, 0, 2, 9, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
		{ 2, 8, 3, 2, 10, 8, 10, 9, 8, -1, -1, -1, -1, -1, -1, -1 },
		{ 3, 11, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
		{ 0, 11, 2, 8, 11, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
		{ 1, 9, 0, 2, 3, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
		{ 1, 11, 2, 1, 9, 11, 9, 8, 11, -1, -1, -1, -1, -1, -1, -1 },
		{ 3, 10, 1, 11, 10, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
		{ 0, 10, 1, 0, 8, 10, 8, 11, 10, -1, -1, -1, -1, -1, -1, -1 },
		{ 3, 9, 0, 3, 11, 9, 11, 10, 9, -1, -1, -1, -1, -1, -1, -1 },
		{ 9, 8, 10, 10, 8, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
		{ 4, 7, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
		{ 4, 3, 0, 7, 3, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
		{ 0, 1, 9, 8, 4, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
		{ 4, 1, 9, 4, 7, 1, 7, 3, 1, -1, -1, -1, -1, -1, -1, -1 },
		{ 1, 2, 10, 8, 4, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
		{ 3, 4, 7, 3, 0, 4, 1, 2, 10, -1, -1, -1, -1, -1, -1, -1 },
		{ 9, 2, 10, 9, 0, 2, 8, 4, 7, -1, -1, -1, -1, -1, -1, -1 },
		{ 2, 10, 9, 2, 9, 7, 2, 7, 3, 7, 9, 4, -1, -1, -1, -1 },
		{ 8, 4, 7, 3, 11, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
		{ 11, 4, 7, 11, 2, 4, 2, 0, 4, -1, -1, -1, -1, -1, -1, -1 },
		{ 9, 0, 1, 8, 4, 7, 2, 3, 11, -1, -1, -1, -1, -1, -1, -1 },
		{ 4, 7, 11, 9, 4, 11, 9, 11, 2, 9, 2, 1, -1, -1, -1, -1 },
		{ 3, 10, 1, 3, 11, 10, 7, 8, 4, -1, -1, -1, -1, -1, -1, -1 },
		{ 1, 11, 10, 1, 4, 11, 1, 0, 4, 7, 11, 4, -1, -1, -1, -1 },
		{ 4, 7, 8, 9, 0, 11, 9, 11, 10, 11, 0, 3, -1, -1, -1, -1 },
		{ 4, 7, 11, 4, 11, 9, 9, 11, 10, -1, -1, -1, -1, -1, -1, -1 },
		{ 9, 5, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
		{ 9, 5, 4, 0, 8, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
		{ 0, 5, 4, 1, 5, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
		{ 8, 5, 4, 8, 3, 5, 3, 1, 5, -1, -1, -1, -1, -1, -1, -1 },
		{ 1, 2, 10, 9, 5, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
		{ 3, 0, 8, 1, 2, 10, 4, 9, 5, -1, -1, -1, -1, -1, -1, -1 },
		{ 5, 2, 10, 5, 4, 2, 4, 0, 2, -1, -1, -1, -1, -1, -1, -1 },
		{ 2, 10, 5, 3, 2, 5, 3, 5, 4, 3, 4, 8, -1, -1, -1, -1 },
		{ 9, 5, 4, 2, 3, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
		{ 0, 11, 2, 0, 8, 11, 4, 9, 5, -1, -1, -1, -1, -1, -1, -1 },
		{ 0, 5, 4, 0, 1, 5, 2, 3, 11, -1, -1, -1, -1, -1, -1, -1 },
		{ 2, 1, 5, 2, 5, 8, 2, 8, 11, 4, 8, 5, -1, -1, -1, -1 },
		{ 10, 3, 11, 10, 1, 3, 9, 5, 4, -1, -1, -1, -1, -1, -1, -1 },
		{ 4, 9, 5, 0, 8, 1, 8, 10, 1, 8, 11, 10, -1, -1, -1, -1 },
		{ 5, 4, 0, 5, 0, 11, 5, 11, 10, 11, 0, 3, -1, -1, -1, -1 },
		{ 5, 4, 8, 5, 8, 10, 10, 8, 11, -1, -1, -1, -1, -1, -1, -1 },
		{ 9, 7, 8, 5, 7, 9, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
		{ 9, 3, 0, 9, 5, 3, 5, 7, 3, -1, -1, -1, -1, -1, -1, -1 },
		{ 0, 7, 8, 0, 1, 7, 1, 5, 7, -1, -1, -1, -1, -1, -1, -1 },
		{ 1, 5, 3, 3, 5, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
		{ 9, 7, 8, 9, 5, 7, 10, 1, 2, -1, -1, -1, -1, -1, -1, -1 },
		{ 10, 1, 2, 9, 5, 0, 5, 3, 0, 5, 7, 3, -1, -1, -1, -1 },
		{ 8, 0, 2, 8, 2, 5, 8, 5, 7, 10, 5, 2, -1, -1, -1, -1 },
		{ 2, 10, 5, 2, 5, 3, 3, 5, 7, -1, -1, -1, -1, -1, -1, -1 },
		{ 7, 9, 5, 7, 8, 9, 3, 11, 2, -1, -1, -1, -1, -1, -1, -1 },
		{ 9, 5, 7, 9, 7, 2, 9, 2, 0, 2, 7, 11, -1, -1, -1, -1 },
		{ 2, 3, 11, 0, 1, 8, 1, 7, 8, 1, 5, 7, -1, -1, -1, -1 },
		{ 11, 2, 1, 11, 1, 7, 7, 1, 5, -1, -1, -1, -1, -1, -1, -1 },
		{ 9, 5, 8, 8, 5, 7, 10, 1, 3, 10, 3, 11, -1, -1, -1, -1 },
		{ 5, 7, 0, 5, 0, 9, 7, 11, 0, 1, 0, 10, 11, 10, 0, -1 },
		{ 11, 10, 0, 11, 0, 3, 10, 5, 0, 8, 0, 7, 5, 7, 0, -1 },
		{ 11, 10, 5, 7, 11, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
		{ 10, 6, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
		{ 0, 8, 3, 5, 10, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
		{ 9, 0, 1, 5, 10, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
		{ 1, 8, 3, 1, 9, 8, 5, 10, 6, -1, -1, -1, -1, -1, -1, -1 },
		{ 1, 6, 5, 2, 6, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
		{ 1, 6, 5, 1, 2, 6, 3, 0, 8, -1, -1, -1, -1, -1, -1, -1 },
		{ 9, 6, 5, 9, 0, 6, 0, 2, 6, -1, -1, -1, -1, -1, -1, -1 },
		{ 5, 9, 8, 5, 8, 2, 5, 2, 6, 3, 2, 8, -1, -1, -1, -1 },
		{ 2, 3, 11, 10, 6, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
		{ 11, 0, 8, 11, 2, 0, 10, 6, 5, -1, -1, -1, -1, -1, -1, -1 },
		{ 0, 1, 9, 2, 3, 11, 5, 10, 6, -1, -1, -1, -1, -1, -1, -1 },
		{ 5, 10, 6, 1, 9, 2, 9, 11, 2, 9, 8, 11, -1, -1, -1, -1 },
		{ 6, 3, 11, 6, 5, 3, 5, 1, 3, -1, -1, -1, -1, -1, -1, -1 },
		{ 0, 8, 11, 0, 11, 5, 0, 5, 1, 5, 11, 6, -1, -1, -1, -1 },
		{ 3, 11, 6, 0, 3, 6, 0, 6, 5, 0, 5, 9, -1, -1, -1, -1 },
		{ 6, 5, 9, 6, 9, 11, 11, 9, 8, -1, -1, -1, -1, -1, -1, -1 },
		{ 5, 10, 6, 4, 7, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
		{ 4, 3, 0, 4, 7, 3, 6, 5, 10, -1, -1, -1, -1, -1, -1, -1 },
		{ 1, 9, 0, 5, 10, 6, 8, 4, 7, -1, -1, -1, -1, -1, -1, -1 },
		{ 10, 6, 5, 1, 9, 7, 1, 7, 3, 7, 9, 4, -1, -1, -1, -1 },
		{ 6, 1, 2, 6, 5, 1, 4, 7, 8, -1, -1, -1, -1, -1, -1, -1 },
		{ 1, 2, 5, 5, 2, 6, 3, 0, 4, 3, 4, 7, -1, -1, -1, -1 },
		{ 8, 4, 7, 9, 0, 5, 0, 6, 5, 0, 2, 6, -1, -1, -1, -1 },
		{ 7, 3, 9, 7, 9, 4, 3, 2, 9, 5, 9, 6, 2, 6, 9, -1 },
		{ 3, 11, 2, 7, 8, 4, 10, 6, 5, -1, -1, -1, -1, -1, -1, -1 },
		{ 5, 10, 6, 4, 7, 2, 4, 2, 0, 2, 7, 11, -1, -1, -1, -1 },
		{ 0, 1, 9, 4, 7, 8, 2, 3, 11, 5, 10, 6, -1, -1, -1, -1 },
		{ 9, 2, 1, 9, 11, 2, 9, 4, 11, 7, 11, 4, 5, 10, 6, -1 },
		{ 8, 4, 7, 3, 11, 5, 3, 5, 1, 5, 11, 6, -1, -1, -1, -1 },
		{ 5, 1, 11, 5, 11, 6, 1, 0, 11, 7, 11, 4, 0, 4, 11, -1 },
		{ 0, 5, 9, 0, 6, 5, 0, 3, 6, 11, 6, 3, 8, 4, 7, -1 },
		{ 6, 5, 9, 6, 9, 11, 4, 7, 9, 7, 11, 9, -1, -1, -1, -1 },
		{ 10, 4, 9, 6, 4, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
		{ 4, 10, 6, 4, 9, 10, 0, 8, 3, -1, -1, -1, -1, -1, -1, -1 },
		{ 10, 0, 1, 10, 6, 0, 6, 4, 0, -1, -1, -1, -1, -1, -1, -1 },
		{ 8, 3, 1, 8, 1, 6, 8, 6, 4, 6, 1, 10, -1, -1, -1, -1 },
		{ 1, 4, 9, 1, 2, 4, 2, 6, 4, -1, -1, -1, -1, -1, -1, -1 },
		{ 3, 0, 8, 1, 2, 9, 2, 4, 9, 2, 6, 4, -1, -1, -1, -1 },
		{ 0, 2, 4, 4, 2, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
		{ 8, 3, 2, 8, 2, 4, 4, 2, 6, -1, -1, -1, -1, -1, -1, -1 },
		{ 10, 4, 9, 10, 6, 4, 11, 2, 3, -1, -1, -1, -1, -1, -1, -1 },
		{ 0, 8, 2, 2, 8, 11, 4, 9, 10, 4, 10, 6, -1, -1, -1, -1 },
		{ 3, 11, 2, 0, 1, 6, 0, 6, 4, 6, 1, 10, -1, -1, -1, -1 },
		{ 6, 4, 1, 6, 1, 10, 4, 8, 1, 2, 1, 11, 8, 11, 1, -1 },
		{ 9, 6, 4, 9, 3, 6, 9, 1, 3, 11, 6, 3, -1, -1, -1, -1 },
		{ 8, 11, 1, 8, 1, 0, 11, 6, 1, 9, 1, 4, 6, 4, 1, -1 },
		{ 3, 11, 6, 3, 6, 0, 0, 6, 4, -1, -1, -1, -1, -1, -1, -1 },
		{ 6, 4, 8, 11, 6, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
		{ 7, 10, 6, 7, 8, 10, 8, 9, 10, -1, -1, -1, -1, -1, -1, -1 },
		{ 0, 7, 3, 0, 10, 7, 0, 9, 10, 6, 7, 10, -1, -1, -1, -1 },
		{ 10, 6, 7, 1, 10, 7, 1, 7, 8, 1, 8, 0, -1, -1, -1, -1 },
		{ 10, 6, 7, 10, 7, 1, 1, 7, 3, -1, -1, -1, -1, -1, -1, -1 },
		{ 1, 2, 6, 1, 6, 8, 1, 8, 9, 8, 6, 7, -1, -1, -1, -1 },
		{ 2, 6, 9, 2, 9, 1, 6, 7, 9, 0, 9, 3, 7, 3, 9, -1 },
		{ 7, 8, 0, 7, 0, 6, 6, 0, 2, -1, -1, -1, -1, -1, -1, -1 },
		{ 7, 3, 2, 6, 7, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
		{ 2, 3, 11, 10, 6, 8, 10, 8, 9, 8, 6, 7, -1, -1, -1, -1 },
		{ 2, 0, 7, 2, 7, 11, 0, 9, 7, 6, 7, 10, 9, 10, 7, -1 },
		{ 1, 8, 0, 1, 7, 8, 1, 10, 7, 6, 7, 10, 2, 3, 11, -1 },
		{ 11, 2, 1, 11, 1, 7, 10, 6, 1, 6, 7, 1, -1, -1, -1, -1 },
		{ 8, 9, 6, 8, 6, 7, 9, 1, 6, 11, 6, 3, 1, 3, 6, -1 },
		{ 0, 9, 1, 11, 6, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
		{ 7, 8, 0, 7, 0, 6, 3, 11, 0, 11, 6, 0, -1, -1, -1, -1 },
		{ 7, 11, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
		{ 7, 6, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
		{ 3, 0, 8, 11, 7, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
		{ 0, 1, 9, 11, 7, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
		{ 8, 1, 9, 8, 3, 1, 11, 7, 6, -1, -1, -1, -1, -1, -1, -1 },
		{ 10, 1, 2, 6, 11, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
		{ 1, 2, 10, 3, 0, 8, 6, 11, 7, -1, -1, -1, -1, -1, -1, -1 },
		{ 2, 9, 0, 2, 10, 9, 6, 11, 7, -1, -1, -1, -1, -1, -1, -1 },
		{ 6, 11, 7, 2, 10, 3, 10, 8, 3, 10, 9, 8, -1, -1, -1, -1 },
		{ 7, 2, 3, 6, 2, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
		{ 7, 0, 8, 7, 6, 0, 6, 2, 0, -1, -1, -1, -1, -1, -1, -1 },
		{ 2, 7, 6, 2, 3, 7, 0, 1, 9, -1, -1, -1, -1, -1, -1, -1 },
		{ 1, 6, 2, 1, 8, 6, 1, 9, 8, 8, 7, 6, -1, -1, -1, -1 },
		{ 10, 7, 6, 10, 1, 7, 1, 3, 7, -1, -1, -1, -1, -1, -1, -1 },
		{ 10, 7, 6, 1, 7, 10, 1, 8, 7, 1, 0, 8, -1, -1, -1, -1 },
		{ 0, 3, 7, 0, 7, 10, 0, 10, 9, 6, 10, 7, -1, -1, -1, -1 },
		{ 7, 6, 10, 7, 10, 8, 8, 10, 9, -1, -1, -1, -1, -1, -1, -1 },
		{ 6, 8, 4, 11, 8, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
		{ 3, 6, 11, 3, 0, 6, 0, 4, 6, -1, -1, -1, -1, -1, -1, -1 },
		{ 8, 6, 11, 8, 4, 6, 9, 0, 1, -1, -1, -1, -1, -1, -1, -1 },
		{ 9, 4, 6, 9, 6, 3, 9, 3, 1, 11, 3, 6, -1, -1, -1, -1 },
		{ 6, 8, 4, 6, 11, 8, 2, 10, 1, -1, -1, -1, -1, -1, -1, -1 },
		{ 1, 2, 10, 3, 0, 11, 0, 6, 11, 0, 4, 6, -1, -1, -1, -1 },
		{ 4, 11, 8, 4, 6, 11, 0, 2, 9, 2, 10, 9, -1, -1, -1, -1 },
		{ 10, 9, 3, 10, 3, 2, 9, 4, 3, 11, 3, 6, 4, 6, 3, -1 },
		{ 8, 2, 3, 8, 4, 2, 4, 6, 2, -1, -1, -1, -1, -1, -1, -1 },
		{ 0, 4, 2, 4, 6, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
		{ 1, 9, 0, 2, 3, 4, 2, 4, 6, 4, 3, 8, -1, -1, -1, -1 },
		{ 1, 9, 4, 1, 4, 2, 2, 4, 6, -1, -1, -1, -1, -1, -1, -1 },
		{ 8, 1, 3, 8, 6, 1, 8, 4, 6, 6, 10, 1, -1, -1, -1, -1 },
		{ 10, 1, 0, 10, 0, 6, 6, 0, 4, -1, -1, -1, -1, -1, -1, -1 },
		{ 4, 6, 3, 4, 3, 8, 6, 10, 3, 0, 3, 9, 10, 9, 3, -1 },
		{ 10, 9, 4, 6, 10, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
		{ 4, 9, 5, 7, 6, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
		{ 0, 8, 3, 4, 9, 5, 11, 7, 6, -1, -1, -1, -1, -1, -1, -1 },
		{ 5, 0, 1, 5, 4, 0, 7, 6, 11, -1, -1, -1, -1, -1, -1, -1 },
		{ 11, 7, 6, 8, 3, 4, 3, 5, 4, 3, 1, 5, -1, -1, -1, -1 },
		{ 9, 5, 4, 10, 1, 2, 7, 6, 11, -1, -1, -1, -1, -1, -1, -1 },
		{ 6, 11, 7, 1, 2, 10, 0, 8, 3, 4, 9, 5, -1, -1, -1, -1 },
		{ 7, 6, 11, 5, 4, 10, 4, 2, 10, 4, 0, 2, -1, -1, -1, -1 },
		{ 3, 4, 8, 3, 5, 4, 3, 2, 5, 10, 5, 2, 11, 7, 6, -1 },
		{ 7, 2, 3, 7, 6, 2, 5, 4, 9, -1, -1, -1, -1, -1, -1, -1 },
		{ 9, 5, 4, 0, 8, 6, 0, 6, 2, 6, 8, 7, -1, -1, -1, -1 },
		{ 3, 6, 2, 3, 7, 6, 1, 5, 0, 5, 4, 0, -1, -1, -1, -1 },
		{ 6, 2, 8, 6, 8, 7, 2, 1, 8, 4, 8, 5, 1, 5, 8, -1 },
		{ 9, 5, 4, 10, 1, 6, 1, 7, 6, 1, 3, 7, -1, -1, -1, -1 },
		{ 1, 6, 10, 1, 7, 6, 1, 0, 7, 8, 7, 0, 9, 5, 4, -1 },
		{ 4, 0, 10, 4, 10, 5, 0, 3, 10, 6, 10, 7, 3, 7, 10, -1 },
		{ 7, 6, 10, 7, 10, 8, 5, 4, 10, 4, 8, 10, -1, -1, -1, -1 },
		{ 6, 9, 5, 6, 11, 9, 11, 8, 9, -1, -1, -1, -1, -1, -1, -1 },
		{ 3, 6, 11, 0, 6, 3, 0, 5, 6, 0, 9, 5, -1, -1, -1, -1 },
		{ 0, 11, 8, 0, 5, 11, 0, 1, 5, 5, 6, 11, -1, -1, -1, -1 },
		{ 6, 11, 3, 6, 3, 5, 5, 3, 1, -1, -1, -1, -1, -1, -1, -1 },
		{ 1, 2, 10, 9, 5, 11, 9, 11, 8, 11, 5, 6, -1, -1, -1, -1 },
		{ 0, 11, 3, 0, 6, 11, 0, 9, 6, 5, 6, 9, 1, 2, 10, -1 },
		{ 11, 8, 5, 11, 5, 6, 8, 0, 5, 10, 5, 2, 0, 2, 5, -1 },
		{ 6, 11, 3, 6, 3, 5, 2, 10, 3, 10, 5, 3, -1, -1, -1, -1 },
		{ 5, 8, 9, 5, 2, 8, 5, 6, 2, 3, 8, 2, -1, -1, -1, -1 },
		{ 9, 5, 6, 9, 6, 0, 0, 6, 2, -1, -1, -1, -1, -1, -1, -1 },
		{ 1, 5, 8, 1, 8, 0, 5, 6, 8, 3, 8, 2, 6, 2, 8, -1 },
		{ 1, 5, 6, 2, 1, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
		{ 1, 3, 6, 1, 6, 10, 3, 8, 6, 5, 6, 9, 8, 9, 6, -1 },
		{ 10, 1, 0, 10, 0, 6, 9, 5, 0, 5, 6, 0, -1, -1, -1, -1 },
		{ 0, 3, 8, 5, 6, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
		{ 10, 5, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
		{ 11, 5, 10, 7, 5, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
		{ 11, 5, 10, 11, 7, 5, 8, 3, 0, -1, -1, -1, -1, -1, -1, -1 },
		{ 5, 11, 7, 5, 10, 11, 1, 9, 0, -1, -1, -1, -1, -1, -1, -1 },
		{ 10, 7, 5, 10, 11, 7, 9, 8, 1, 8, 3, 1, -1, -1, -1, -1 },
		{ 11, 1, 2, 11, 7, 1, 7, 5, 1, -1, -1, -1, -1, -1, -1, -1 },
		{ 0, 8, 3, 1, 2, 7, 1, 7, 5, 7, 2, 11, -1, -1, -1, -1 },
		{ 9, 7, 5, 9, 2, 7, 9, 0, 2, 2, 11, 7, -1, -1, -1, -1 },
		{ 7, 5, 2, 7, 2, 11, 5, 9, 2, 3, 2, 8, 9, 8, 2, -1 },
		{ 2, 5, 10, 2, 3, 5, 3, 7, 5, -1, -1, -1, -1, -1, -1, -1 },
		{ 8, 2, 0, 8, 5, 2, 8, 7, 5, 10, 2, 5, -1, -1, -1, -1 },
		{ 9, 0, 1, 5, 10, 3, 5, 3, 7, 3, 10, 2, -1, -1, -1, -1 },
		{ 9, 8, 2, 9, 2, 1, 8, 7, 2, 10, 2, 5, 7, 5, 2, -1 },
		{ 1, 3, 5, 3, 7, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
		{ 0, 8, 7, 0, 7, 1, 1, 7, 5, -1, -1, -1, -1, -1, -1, -1 },
		{ 9, 0, 3, 9, 3, 5, 5, 3, 7, -1, -1, -1, -1, -1, -1, -1 },
		{ 9, 8, 7, 5, 9, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
		{ 5, 8, 4, 5, 10, 8, 10, 11, 8, -1, -1, -1, -1, -1, -1, -1 },
		{ 5, 0, 4, 5, 11, 0, 5, 10, 11, 11, 3, 0, -1, -1, -1, -1 },
		{ 0, 1, 9, 8, 4, 10, 8, 10, 11, 10, 4, 5, -1, -1, -1, -1 },
		{ 10, 11, 4, 10, 4, 5, 11, 3, 4, 9, 4, 1, 3, 1, 4, -1 },
		{ 2, 5, 1, 2, 8, 5, 2, 11, 8, 4, 5, 8, -1, -1, -1, -1 },
		{ 0, 4, 11, 0, 11, 3, 4, 5, 11, 2, 11, 1, 5, 1, 11, -1 },
		{ 0, 2, 5, 0, 5, 9, 2, 11, 5, 4, 5, 8, 11, 8, 5, -1 },
		{ 9, 4, 5, 2, 11, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
		{ 2, 5, 10, 3, 5, 2, 3, 4, 5, 3, 8, 4, -1, -1, -1, -1 },
		{ 5, 10, 2, 5, 2, 4, 4, 2, 0, -1, -1, -1, -1, -1, -1, -1 },
		{ 3, 10, 2, 3, 5, 10, 3, 8, 5, 4, 5, 8, 0, 1, 9, -1 },
		{ 5, 10, 2, 5, 2, 4, 1, 9, 2, 9, 4, 2, -1, -1, -1, -1 },
		{ 8, 4, 5, 8, 5, 3, 3, 5, 1, -1, -1, -1, -1, -1, -1, -1 },
		{ 0, 4, 5, 1, 0, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
		{ 8, 4, 5, 8, 5, 3, 9, 0, 5, 0, 3, 5, -1, -1, -1, -1 },
		{ 9, 4, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
		{ 4, 11, 7, 4, 9, 11, 9, 10, 11, -1, -1, -1, -1, -1, -1, -1 },
		{ 0, 8, 3, 4, 9, 7, 9, 11, 7, 9, 10, 11, -1, -1, -1, -1 },
		{ 1, 10, 11, 1, 11, 4, 1, 4, 0, 7, 4, 11, -1, -1, -1, -1 },
		{ 3, 1, 4, 3, 4, 8, 1, 10, 4, 7, 4, 11, 10, 11, 4, -1 },
		{ 4, 11, 7, 9, 11, 4, 9, 2, 11, 9, 1, 2, -1, -1, -1, -1 },
		{ 9, 7, 4, 9, 11, 7, 9, 1, 11, 2, 11, 1, 0, 8, 3, -1 },
		{ 11, 7, 4, 11, 4, 2, 2, 4, 0, -1, -1, -1, -1, -1, -1, -1 },
		{ 11, 7, 4, 11, 4, 2, 8, 3, 4, 3, 2, 4, -1, -1, -1, -1 },
		{ 2, 9, 10, 2, 7, 9, 2, 3, 7, 7, 4, 9, -1, -1, -1, -1 },
		{ 9, 10, 7, 9, 7, 4, 10, 2, 7, 8, 7, 0, 2, 0, 7, -1 },
		{ 3, 7, 10, 3, 10, 2, 7, 4, 10, 1, 10, 0, 4, 0, 10, -1 },
		{ 1, 10, 2, 8, 7, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
		{ 4, 9, 1, 4, 1, 7, 7, 1, 3, -1, -1, -1, -1, -1, -1, -1 },
		{ 4, 9, 1, 4, 1, 7, 0, 8, 1, 8, 7, 1, -1, -1, -1, -1 },
		{ 4, 0, 3, 7, 4, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
		{ 4, 8, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
		{ 9, 10, 8, 10, 11, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
		{ 3, 0, 9, 3, 9, 11, 11, 9, 10, -1, -1, -1, -1, -1, -1, -1 },
		{ 0, 1, 10, 0, 10, 8, 8, 10, 11, -1, -1, -1, -1, -1, -1, -1 },
		{ 3, 1, 10, 11, 3, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
		{ 1, 2, 11, 1, 11, 9, 9, 11, 8, -1, -1, -1, -1, -1, -1, -1 },
		{ 3, 0, 9, 3, 9, 11, 1, 2, 9, 2, 11, 9, -1, -1, -1, -1 },
		{ 0, 2, 11, 8, 0, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
		{ 3, 2, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
		{ 2, 3, 8, 2, 8, 10, 10, 8, 9, -1, -1, -1, -1, -1, -1, -1 },
		{ 9, 10, 2, 0, 9, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
		{ 2, 3, 8, 2, 8, 10, 0, 1, 8, 1, 10, 8, -1, -1, -1, -1 },
		{ 1, 10, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
		{ 1, 3, 8, 9, 1, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
		{ 0, 9, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
		{ 0, 3, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
		{ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 } };

		for (int i = 0; i < 256; i++)
		{
			edgeTable[i] = tempEdgeTable[i];

			for (int j = 0; j < 16; j++)
				triTable[i][j] = tempTriTable[i][j];
		}
	}

}