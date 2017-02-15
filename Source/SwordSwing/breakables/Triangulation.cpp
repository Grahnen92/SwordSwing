
//#include "ProceduralMeshComponent.h"
#include "SwordSwing.h"
#include "ScalarField.h"

namespace triangulation {

	//struct cacheCell {
	//	bool isoBool;
	//	float cornerPoint[3];
	//	int vertexIndex[3];
	//};

	//template <typename scalar_value>
	//void marchingCubes(UProceduralMeshComponent* _mesh, ScalarField<scalar_value>* _sf) {
	//	UE_LOG(LogTemp, Warning, TEXT("generating mesh..."));
	//	int cubeIndex;

	//	float xyz[8][3];
	//	int vertList[12];
	//	unsigned char val[8];
	//	bool cellIsoBool[8];
	//	double dVal;

	//	int vertexCap = 0;
	//	int triangleCap = 0;

	//	int x, y, z;
	//	int layerIndex = 0;

	//	std::vector<int> y0Cache;
	//	int z0Cache;

	//	//const static MCTables mct;

	//	TArray<FVector> vertexArray;
	//	TArray<int32> triangleArray;
	//	TArray<FVector> normalArray;
	//	TArray<FProcMeshTangent> tangentArray;

	//	int edgeTable[256];
	//	int triTable[256][16];

	//	cacheCell*** isoCache = new cacheCell**[2];
	//	for (int i = 0; i < 2; i++)
	//	{
	//		isoCache[i] = new cacheCell*[res.Y];
	//		for (int j = 0; j < res.Y; j++)
	//		{
	//			isoCache[i][j] = new cacheCell[res.Z];
	//		}
	//	}

	//	y0Cache.resize(res.Z);

	//	//create the first layer
	//	x = 0;
	//	y = 0;
	//	z = 0;

	//	// create the first voxel ============================================================

	//	//inherit corner values from local variable

	//	//inherit corner values from isoCache

	//	//calculate corner values that could not be inherited
	//	xyz[0][0] = (float)((x - (res.X / 2)) / ((float)(res.X / 2)))*dim.X;
	//	xyz[0][1] = (float)((y - (res.Y / 2)) / ((float)(res.Y / 2)))*dim.Y;
	//	xyz[0][2] = (float)((z - (res.Z / 2)) / ((float)(res.Z / 2)))*dim.Z;
	//	val[0] = data[x][y][z];
	//	if (data[x][y][z] < iso_value)			// cubeIndex |= 1;
	//		cellIsoBool[0] = true;
	//	else
	//		cellIsoBool[0] = false;



	//	xyz[1][0] = (float)((x + 1 - (res.X / 2)) / ((float)(res.X / 2)))*dim.X;
	//	xyz[1][1] = (float)((y - (res.Y / 2)) / ((float)(res.Y / 2)))*dim.Y;
	//	xyz[1][2] = (float)((z - (res.Z / 2)) / ((float)(res.Z / 2)))*dim.Z;
	//	val[1] = data[x + 1][y][z];
	//	if (data[x + 1][y][z] < iso_value)		// cubeIndex |= 2;
	//		cellIsoBool[1] = true;
	//	else
	//		cellIsoBool[1] = false;

	//	xyz[2][0] = (float)((x + 1 - (res.X / 2)) / ((float)(res.X / 2)))*dim.X;
	//	xyz[2][1] = (float)((y - (res.Y / 2)) / ((float)(res.Y / 2)))*dim.Y;
	//	xyz[2][2] = (float)((z + 1 - (res.Z / 2)) / ((float)(res.Z / 2)))*dim.Z;
	//	val[2] = data[x + 1][y][z + 1];
	//	if (data[x + 1][y][z + 1] < iso_value)	// cubeIndex |= 4;
	//		cellIsoBool[2] = true;
	//	else
	//		cellIsoBool[2] = false;

	//	xyz[3][0] = (float)((x - (res.X / 2)) / ((float)(res.X / 2)))*dim.X;
	//	xyz[3][1] = (float)((y - (res.Y / 2)) / ((float)(res.Y / 2)))*dim.Y;
	//	xyz[3][2] = (float)((z + 1 - (res.Z / 2)) / ((float)(res.Z / 2)))*dim.Z;
	//	val[3] = data[x][y][z + 1];
	//	if (data[x][y][z + 1] < iso_value)		// cubeIndex |= 8;
	//		cellIsoBool[3] = true;
	//	else
	//		cellIsoBool[3] = false;

	//	xyz[4][0] = (float)((x - (res.X / 2)) / ((float)(res.X / 2)))*dim.X;
	//	xyz[4][1] = (float)((y + 1 - (res.Y / 2)) / ((float)(res.Y / 2)))*dim.Y;
	//	xyz[4][2] = (float)((z - (res.Z / 2)) / ((float)(res.Z / 2)))*dim.Z;
	//	val[4] = data[x][y + 1][z];
	//	if (data[x][y + 1][z] < iso_value)		//cubeIndex |= 16;
	//		cellIsoBool[4] = true;
	//	else
	//		cellIsoBool[4] = false;

	//	xyz[5][0] = (float)((x + 1 - (res.X / 2)) / ((float)(res.X / 2)))*dim.X;
	//	xyz[5][1] = (float)((y + 1 - (res.Y / 2)) / ((float)(res.Y / 2)))*dim.Y;
	//	xyz[5][2] = (float)((z - (res.Z / 2)) / ((float)(res.Z / 2)))*dim.Z;
	//	val[5] = data[x + 1][y + 1][z];
	//	if (data[x + 1][y + 1][z] < iso_value)	// cubeIndex |= 32;
	//		cellIsoBool[5] = true;
	//	else
	//		cellIsoBool[5] = false;

	//	//save the sixth corner values to isoCache
	//	xyz[6][0] = isoCache[layerIndex][y][z].cornerPoint[0] = (float)((x + 1 - (res.X / 2)) / ((float)(res.X / 2)))*dim.X;
	//	xyz[6][1] = isoCache[layerIndex][y][z].cornerPoint[1] = (float)((y + 1 - (res.Y / 2)) / ((float)(res.Y / 2)))*dim.Y;
	//	xyz[6][2] = isoCache[layerIndex][y][z].cornerPoint[2] = (float)((z + 1 - (res.Z / 2)) / ((float)(res.Z / 2)))*dim.Z;
	//	val[6] = data[x + 1][y + 1][z + 1];
	//	if (data[x + 1][y + 1][z + 1] < iso_value)// cubeIndex |= 64;
	//		cellIsoBool[6] = isoCache[layerIndex][y][z].isoBool = true;
	//	else
	//		cellIsoBool[6] = isoCache[layerIndex][y][z].isoBool = false;

	//	xyz[7][0] = (float)((x - (res.X / 2)) / ((float)(res.X / 2)))*dim.X;
	//	xyz[7][1] = (float)((y + 1 - (res.Y / 2)) / ((float)(res.Y / 2)))*dim.Y;
	//	xyz[7][2] = (float)((z + 1 - (res.Z / 2)) / ((float)(res.Z / 2)))*dim.Z;
	//	val[7] = data[x][y + 1][z + 1];
	//	if (data[x][y + 1][z + 1] < iso_value)	// cubeIndex |= 128;
	//		cellIsoBool[7] = true;
	//	else
	//		cellIsoBool[7] = false;

	//	// get the case index
	//	cubeIndex = cellIsoBool[0] * 1 + cellIsoBool[1] * 2 + cellIsoBool[2] * 4 + cellIsoBool[3] * 8 +
	//		cellIsoBool[4] * 16 + cellIsoBool[5] * 32 + cellIsoBool[6] * 64 + cellIsoBool[7] * 128;

	//	// check if cube is entirely in or out of the surface ----------------------------
	//	if (edgeTable[cubeIndex] != 0) {

	//		// Find the vertices where the surface intersects the cube--------------------

	//		//inherit vertex indices from local variable ---------------------------------

	//		//inherit vertex indices from isoCache ---------------------------------------

	//		//calculate indices that could not be inherited ------------------------------
	//		if (edgeTable[cubeIndex] & 1) {
	//			dVal = (double)(iso_value - val[0]) / (double)(val[1] - val[0]);
	//			/*vertexArray[vertexCap][0] = xyz[0][0] + dVal*(xyz[1][0] - xyz[0][0]);
	//			vertexArray[vertexCap][1] = xyz[0][1] + dVal*(xyz[1][1] - xyz[0][1]);
	//			vertexArray[vertexCap][2] = xyz[0][2] + dVal*(xyz[1][2] - xyz[0][2]);*/
	//			vertexArray.Add(FVector(xyz[0][0] + dVal*(xyz[1][0] - xyz[0][0]), xyz[0][1] + dVal*(xyz[1][1] - xyz[0][1]), xyz[0][2] + dVal*(xyz[1][2] - xyz[0][2])));
	//			vertList[0] = vertexCap;
	//			vertexCap++;
	//		}

	//		if (edgeTable[cubeIndex] & 2) {
	//			dVal = (double)(iso_value - val[1]) / (double)(val[2] - val[1]);
	//			/*vertexArray[vertexCap][0] = xyz[1][0] + dVal*(xyz[2][0] - xyz[1][0]);
	//			vertexArray[vertexCap][1] = xyz[1][1] + dVal*(xyz[2][1] - xyz[1][1]);
	//			vertexArray[vertexCap][2] = xyz[1][2] + dVal*(xyz[2][2] - xyz[1][2]);*/
	//			vertexArray.Add(FVector(xyz[1][0] + dVal*(xyz[2][0] - xyz[1][0]), xyz[1][1] + dVal*(xyz[2][1] - xyz[1][1]), xyz[1][2] + dVal*(xyz[2][2] - xyz[1][2])));
	//			vertList[1] = vertexCap;
	//			vertexCap++;
	//		}
	//		if (edgeTable[cubeIndex] & 4) {
	//			dVal = (double)(iso_value - val[2]) / (double)(val[3] - val[2]);
	//			//vertexArray[vertexCap][0] = xyz[2][0] + dVal*(xyz[3][0] - xyz[2][0]);
	//			//vertexArray[vertexCap][1] = xyz[2][1] + dVal*(xyz[3][1] - xyz[2][1]);
	//			//vertexArray[vertexCap][2] = xyz[2][2] + dVal*(xyz[3][2] - xyz[2][2]);
	//			vertexArray.Add(FVector(xyz[2][0] + dVal*(xyz[3][0] - xyz[2][0]), xyz[2][1] + dVal*(xyz[3][1] - xyz[2][1]), xyz[2][2] + dVal*(xyz[3][2] - xyz[2][2])));
	//			vertList[2] = vertexCap;
	//			vertexCap++;
	//		}
	//		if (edgeTable[cubeIndex] & 8) {
	//			dVal = (double)(iso_value - val[3]) / (double)(val[0] - val[3]);
	//			//vertexArray[vertexCap][0] = xyz[3][0] + dVal*(xyz[0][0] - xyz[3][0]);
	//			//vertexArray[vertexCap][1] = xyz[3][1] + dVal*(xyz[0][1] - xyz[3][1]);
	//			//vertexArray[vertexCap][2] = xyz[3][2] + dVal*(xyz[0][2] - xyz[3][2]);
	//			vertexArray.Add(FVector(xyz[3][0] + dVal*(xyz[0][0] - xyz[3][0]), xyz[3][1] + dVal*(xyz[0][1] - xyz[3][1]), xyz[3][2] + dVal*(xyz[0][2] - xyz[3][2])));
	//			vertList[3] = vertexCap;
	//			vertexCap++;
	//		}
	//		if (edgeTable[cubeIndex] & 16) {
	//			dVal = (double)(iso_value - val[4]) / (double)(val[5] - val[4]);
	//			/*vertexArray[vertexCap][0] = xyz[4][0] + dVal*(xyz[5][0] - xyz[4][0]);
	//			vertexArray[vertexCap][1] = xyz[4][1] + dVal*(xyz[5][1] - xyz[4][1]);
	//			vertexArray[vertexCap][2] = xyz[4][2] + dVal*(xyz[5][2] - xyz[4][2]);*/
	//			vertexArray.Add(FVector(xyz[4][0] + dVal*(xyz[5][0] - xyz[4][0]), xyz[4][1] + dVal*(xyz[5][1] - xyz[4][1]), xyz[4][2] + dVal*(xyz[5][2] - xyz[4][2])));
	//			vertList[4] = z0Cache = vertexCap;
	//			vertexCap++;
	//		}
	//		if (edgeTable[cubeIndex] & 32) {
	//			dVal = (double)(iso_value - val[5]) / (double)(val[6] - val[5]);
	//			//vertexArray[vertexCap][0] = xyz[5][0] + dVal*(xyz[6][0] - xyz[5][0]);
	//			//vertexArray[vertexCap][1] = xyz[5][1] + dVal*(xyz[6][1] - xyz[5][1]);
	//			//vertexArray[vertexCap][2] = xyz[5][2] + dVal*(xyz[6][2] - xyz[5][2]);
	//			vertexArray.Add(FVector(xyz[5][0] + dVal*(xyz[6][0] - xyz[5][0]), xyz[5][1] + dVal*(xyz[6][1] - xyz[5][1]), xyz[5][2] + dVal*(xyz[6][2] - xyz[5][2])));
	//			isoCache[layerIndex][y][z].vertexIndex[0] = vertList[5] = vertexCap;
	//			vertexCap++;
	//		}
	//		if (edgeTable[cubeIndex] & 64) {
	//			dVal = (double)(iso_value - val[6]) / (double)(val[7] - val[6]);
	//			/*vertexArray[vertexCap][0] = xyz[6][0] + dVal*(xyz[7][0] - xyz[6][0]);
	//			vertexArray[vertexCap][1] = xyz[6][1] + dVal*(xyz[7][1] - xyz[6][1]);
	//			vertexArray[vertexCap][2] = xyz[6][2] + dVal*(xyz[7][2] - xyz[6][2]);*/
	//			vertexArray.Add(FVector(xyz[6][0] + dVal*(xyz[7][0] - xyz[6][0]), xyz[6][1] + dVal*(xyz[7][1] - xyz[6][1]), xyz[6][2] + dVal*(xyz[7][2] - xyz[6][2])));
	//			isoCache[layerIndex][y][z].vertexIndex[1] = vertList[6] = vertexCap;
	//			vertexCap++;
	//		}
	//		if (edgeTable[cubeIndex] & 128) {
	//			dVal = (double)(iso_value - val[7]) / (double)(val[4] - val[7]);
	//			/*vertexArray[vertexCap][0] = xyz[7][0] + dVal*(xyz[4][0] - xyz[7][0]);
	//			vertexArray[vertexCap][1] = xyz[7][1] + dVal*(xyz[4][1] - xyz[7][1]);
	//			vertexArray[vertexCap][2] = xyz[7][2] + dVal*(xyz[4][2] - xyz[7][2]);*/
	//			vertexArray.Add(FVector(xyz[7][0] + dVal*(xyz[4][0] - xyz[7][0]), xyz[7][1] + dVal*(xyz[4][1] - xyz[7][1]), xyz[7][2] + dVal*(xyz[4][2] - xyz[7][2])));
	//			vertList[7] = y0Cache[z] = vertexCap;
	//			vertexCap++;
	//		}
	//		if (edgeTable[cubeIndex] & 256) {
	//			dVal = (double)(iso_value - val[0]) / (double)(val[4] - val[0]);
	//			/*vertexArray[vertexCap][0] = xyz[0][0] + dVal*(xyz[4][0] - xyz[0][0]);
	//			vertexArray[vertexCap][1] = xyz[0][1] + dVal*(xyz[4][1] - xyz[0][1]);
	//			vertexArray[vertexCap][2] = xyz[0][2] + dVal*(xyz[4][2] - xyz[0][2]);*/
	//			vertexArray.Add(FVector(xyz[0][0] + dVal*(xyz[4][0] - xyz[0][0]), xyz[0][1] + dVal*(xyz[4][1] - xyz[0][1]), xyz[0][2] + dVal*(xyz[4][2] - xyz[0][2])));
	//			vertList[8] = vertexCap;
	//			vertexCap++;
	//		}
	//		if (edgeTable[cubeIndex] & 512) {
	//			dVal = (double)(iso_value - val[1]) / (double)(val[5] - val[1]);
	//			/*vertexArray[vertexCap][0] = xyz[1][0] + dVal*(xyz[5][0] - xyz[1][0]);
	//			vertexArray[vertexCap][1] = xyz[1][1] + dVal*(xyz[5][1] - xyz[1][1]);
	//			vertexArray[vertexCap][2] = xyz[1][2] + dVal*(xyz[5][2] - xyz[1][2]);*/
	//			vertexArray.Add(FVector(xyz[1][0] + dVal*(xyz[5][0] - xyz[1][0]), xyz[1][1] + dVal*(xyz[5][1] - xyz[1][1]), xyz[1][2] + dVal*(xyz[5][2] - xyz[1][2])));
	//			vertList[9] = vertexCap;
	//			vertexCap++;
	//		}
	//		if (edgeTable[cubeIndex] & 1024) {
	//			dVal = (double)(iso_value - val[2]) / (double)(val[6] - val[2]);
	//			/*vertexArray[vertexCap][0] = xyz[2][0] + dVal*(xyz[6][0] - xyz[2][0]);
	//			vertexArray[vertexCap][1] = xyz[2][1] + dVal*(xyz[6][1] - xyz[2][1]);
	//			vertexArray[vertexCap][2] = xyz[2][2] + dVal*(xyz[6][2] - xyz[2][2]);*/
	//			vertexArray.Add(FVector(xyz[2][0] + dVal*(xyz[6][0] - xyz[2][0]), xyz[2][1] + dVal*(xyz[6][1] - xyz[2][1]), xyz[2][2] + dVal*(xyz[6][2] - xyz[2][2])));
	//			isoCache[layerIndex][y][z].vertexIndex[2] = vertList[10] = vertexCap;
	//			vertexCap++;
	//		}
	//		if (edgeTable[cubeIndex] & 2048) {
	//			dVal = (double)(iso_value - val[3]) / (double)(val[7] - val[3]);
	//			/*vertexArray[vertexCap][0] = xyz[3][0] + dVal*(xyz[7][0] - xyz[3][0]);
	//			vertexArray[vertexCap][1] = xyz[3][1] + dVal*(xyz[7][1] - xyz[3][1]);
	//			vertexArray[vertexCap][2] = xyz[3][2] + dVal*(xyz[7][2] - xyz[3][2]);*/
	//			vertexArray.Add(FVector(xyz[3][0] + dVal*(xyz[7][0] - xyz[3][0]), xyz[3][1] + dVal*(xyz[7][1] - xyz[3][1]), xyz[3][2] + dVal*(xyz[7][2] - xyz[3][2])));
	//			vertList[11] = vertexCap;
	//			vertexCap++;
	//		}
	//		int test = triTable[cubeIndex][3];
	//		// bind triangle indecies
	//		for (int i = 0; triTable[cubeIndex][i] != -1; i += 3) {
	//			/*triangleArray[triangleCap*3 + 2] = vertList[triTable[cubeIndex][i]];
	//			triangleArray[triangleCap*3 + 1] = vertList[triTable[cubeIndex][i + 1]];
	//			triangleArray[triangleCap*3] = vertList[triTable[cubeIndex][i + 2]];*/
	//			triangleArray.Add(vertList[triTable[cubeIndex][i]]);
	//			triangleArray.Add(vertList[triTable[cubeIndex][i + 1]]);
	//			triangleArray.Add(vertList[triTable[cubeIndex][i + 2]]);

	//			triangleCap++;
	//		}
	//	}

	//	//create the remaining voxels of first row of first layer ======================================
	//	for (z = 1; z < res.Z - 1; z++) {

	//		//inherit corner values from local variable
	//		xyz[0][0] = xyz[3][0];
	//		xyz[0][1] = xyz[3][1];
	//		xyz[0][2] = xyz[3][2];
	//		val[0] = data[x][y][z];
	//		cellIsoBool[0] = cellIsoBool[3];

	//		xyz[1][0] = xyz[2][0];
	//		xyz[1][1] = xyz[2][1];
	//		xyz[1][2] = xyz[2][2];
	//		val[1] = data[x + 1][y][z];
	//		cellIsoBool[1] = cellIsoBool[2];

	//		xyz[4][0] = xyz[7][0];
	//		xyz[4][1] = xyz[7][1];
	//		xyz[4][2] = xyz[7][2];
	//		val[4] = data[x][y + 1][z];
	//		cellIsoBool[4] = cellIsoBool[7];

	//		//inherit corner values from isoCache
	//		xyz[5][0] = isoCache[layerIndex][y][z - 1].cornerPoint[0];
	//		xyz[5][1] = isoCache[layerIndex][y][z - 1].cornerPoint[1];
	//		xyz[5][2] = isoCache[layerIndex][y][z - 1].cornerPoint[2];
	//		val[5] = data[x + 1][y + 1][z];
	//		cellIsoBool[5] = isoCache[layerIndex][y][z - 1].isoBool;

	//		//calculate corner values that could not be inherited
	//		xyz[2][0] = (float)((x + 1 - (res.X / 2)) / ((float)(res.X / 2)))*dim.X;
	//		xyz[2][1] = (float)((y - (res.Y / 2)) / ((float)(res.Y / 2)))*dim.Y;
	//		xyz[2][2] = (float)((z + 1 - (res.Z / 2)) / ((float)(res.Z / 2)))*dim.Z;
	//		val[2] = data[x + 1][y][z + 1];
	//		if (data[x + 1][y][z + 1] < iso_value)	// cubeIndex |= 4;
	//			cellIsoBool[2] = true;
	//		else
	//			cellIsoBool[2] = false;

	//		xyz[3][0] = (float)((x - (res.X / 2)) / ((float)(res.X / 2)))*dim.X;
	//		xyz[3][1] = (float)((y - (res.Y / 2)) / ((float)(res.Y / 2)))*dim.Y;
	//		xyz[3][2] = (float)((z + 1 - (res.Z / 2)) / ((float)(res.Z / 2)))*dim.Z;
	//		val[3] = data[x][y][z + 1];
	//		if (data[x][y][z + 1] < iso_value)		// cubeIndex |= 8;
	//			cellIsoBool[3] = true;
	//		else
	//			cellIsoBool[3] = false;

	//		//save the sixth corner values to isoCache
	//		xyz[6][0] = isoCache[layerIndex][y][z].cornerPoint[0] = (float)((x + 1 - (res.X / 2)) / ((float)(res.X / 2)))*dim.X;
	//		xyz[6][1] = isoCache[layerIndex][y][z].cornerPoint[1] = (float)((y + 1 - (res.Y / 2)) / ((float)(res.Y / 2)))*dim.Y;
	//		xyz[6][2] = isoCache[layerIndex][y][z].cornerPoint[2] = (float)((z + 1 - (res.Z / 2)) / ((float)(res.Z / 2)))*dim.Z;
	//		val[6] = data[x + 1][y + 1][z + 1];
	//		if (data[x + 1][y + 1][z + 1] < iso_value)// cubeIndex |= 64;
	//			cellIsoBool[6] = isoCache[layerIndex][y][z].isoBool = true;
	//		else
	//			cellIsoBool[6] = isoCache[layerIndex][y][z].isoBool = false;

	//		xyz[7][0] = (float)((x - (res.X / 2)) / ((float)(res.X / 2)))*dim.X;
	//		xyz[7][1] = (float)((y + 1 - (res.Y / 2)) / ((float)(res.Y / 2)))*dim.Y;
	//		xyz[7][2] = (float)((z + 1 - (res.Z / 2)) / ((float)(res.Z / 2)))*dim.Z;
	//		val[7] = data[x][y + 1][z + 1];
	//		if (data[x][y + 1][z + 1] < iso_value)	// cubeIndex |= 128;
	//			cellIsoBool[7] = true;
	//		else
	//			cellIsoBool[7] = false;

	//		// get the case index
	//		cubeIndex = cellIsoBool[0] * 1 + cellIsoBool[1] * 2 + cellIsoBool[2] * 4 + cellIsoBool[3] * 8 +
	//			cellIsoBool[4] * 16 + cellIsoBool[5] * 32 + cellIsoBool[6] * 64 + cellIsoBool[7] * 128;

	//		// check if cube is entirely in or out of the surface ----------------------------
	//		if (edgeTable[cubeIndex] != 0) {

	//			// Find the vertices where the surface intersects the cube--------------------

	//			//inherit vertex indices from local variable ---------------------------------
	//			if (edgeTable[cubeIndex] & 1) {
	//				vertList[0] = vertList[2];
	//			}
	//			if (edgeTable[cubeIndex] & 256) {
	//				vertList[8] = vertList[11];
	//			}

	//			//inherit vertex indices from isoCache ---------------------------------------
	//			if (edgeTable[cubeIndex] & 16) {
	//				vertList[4] = isoCache[layerIndex][y][z - 1].vertexIndex[1];
	//			}
	//			if (edgeTable[cubeIndex] & 512) {
	//				vertList[9] = isoCache[layerIndex][y][z - 1].vertexIndex[2];
	//			}
	//			//calculate indices that could not be inherited ------------------------------
	//			if (edgeTable[cubeIndex] & 2) {
	//				dVal = (double)(iso_value - val[1]) / (double)(val[2] - val[1]);
	//				/*vertexArray[vertexCap][0] = xyz[1][0] + dVal*(xyz[2][0] - xyz[1][0]);
	//				vertexArray[vertexCap][1] = xyz[1][1] + dVal*(xyz[2][1] - xyz[1][1]);
	//				vertexArray[vertexCap][2] = xyz[1][2] + dVal*(xyz[2][2] - xyz[1][2]);*/
	//				vertexArray.Add(FVector(xyz[1][0] + dVal*(xyz[2][0] - xyz[1][0]), xyz[1][1] + dVal*(xyz[2][1] - xyz[1][1]), xyz[1][2] + dVal*(xyz[2][2] - xyz[1][2])));
	//				vertList[1] = vertexCap;
	//				vertexCap++;
	//			}
	//			if (edgeTable[cubeIndex] & 4) {
	//				dVal = (double)(iso_value - val[2]) / (double)(val[3] - val[2]);
	//				/*vertexArray[vertexCap][0] = xyz[2][0] + dVal*(xyz[3][0] - xyz[2][0]);
	//				vertexArray[vertexCap][1] = xyz[2][1] + dVal*(xyz[3][1] - xyz[2][1]);
	//				vertexArray[vertexCap][2] = xyz[2][2] + dVal*(xyz[3][2] - xyz[2][2]);*/
	//				vertexArray.Add(FVector(xyz[2][0] + dVal*(xyz[3][0] - xyz[2][0]), xyz[2][1] + dVal*(xyz[3][1] - xyz[2][1]), xyz[2][2] + dVal*(xyz[3][2] - xyz[2][2])));
	//				vertList[2] = vertexCap;
	//				vertexCap++;
	//			}
	//			if (edgeTable[cubeIndex] & 8) {
	//				dVal = (double)(iso_value - val[3]) / (double)(val[0] - val[3]);
	//				/*vertexArray[vertexCap][0] = xyz[3][0] + dVal*(xyz[0][0] - xyz[3][0]);
	//				vertexArray[vertexCap][1] = xyz[3][1] + dVal*(xyz[0][1] - xyz[3][1]);
	//				vertexArray[vertexCap][2] = xyz[3][2] + dVal*(xyz[0][2] - xyz[3][2]);*/
	//				vertexArray.Add(FVector(xyz[3][0] + dVal*(xyz[0][0] - xyz[3][0]), xyz[3][1] + dVal*(xyz[0][1] - xyz[3][1]), xyz[3][2] + dVal*(xyz[0][2] - xyz[3][2])));
	//				vertList[3] = vertexCap;
	//				vertexCap++;
	//			}

	//			if (edgeTable[cubeIndex] & 32) {
	//				dVal = (double)(iso_value - val[5]) / (double)(val[6] - val[5]);
	//				//vertexArray[vertexCap][0] = xyz[5][0] + dVal*(xyz[6][0] - xyz[5][0]);
	//				//vertexArray[vertexCap][1] = xyz[5][1] + dVal*(xyz[6][1] - xyz[5][1]);
	//				//vertexArray[vertexCap][2] = xyz[5][2] + dVal*(xyz[6][2] - xyz[5][2]);
	//				vertexArray.Add(FVector(xyz[5][0] + dVal*(xyz[6][0] - xyz[5][0]), xyz[5][1] + dVal*(xyz[6][1] - xyz[5][1]), xyz[5][2] + dVal*(xyz[6][2] - xyz[5][2])));
	//				isoCache[layerIndex][y][z].vertexIndex[0] = vertList[5] = vertexCap;
	//				vertexCap++;
	//			}
	//			if (edgeTable[cubeIndex] & 64) {
	//				dVal = (double)(iso_value - val[6]) / (double)(val[7] - val[6]);
	//				/*vertexArray[vertexCap][0] = xyz[6][0] + dVal*(xyz[7][0] - xyz[6][0]);
	//				vertexArray[vertexCap][1] = xyz[6][1] + dVal*(xyz[7][1] - xyz[6][1]);
	//				vertexArray[vertexCap][2] = xyz[6][2] + dVal*(xyz[7][2] - xyz[6][2]);*/
	//				vertexArray.Add(FVector(xyz[6][0] + dVal*(xyz[7][0] - xyz[6][0]), xyz[6][1] + dVal*(xyz[7][1] - xyz[6][1]), xyz[6][2] + dVal*(xyz[7][2] - xyz[6][2])));
	//				isoCache[layerIndex][y][z].vertexIndex[1] = vertList[6] = vertexCap;
	//				vertexCap++;
	//			}
	//			if (edgeTable[cubeIndex] & 128) {
	//				dVal = (double)(iso_value - val[7]) / (double)(val[4] - val[7]);
	//				//vertexArray[vertexCap][0] = xyz[7][0] + dVal*(xyz[4][0] - xyz[7][0]);
	//				//vertexArray[vertexCap][1] = xyz[7][1] + dVal*(xyz[4][1] - xyz[7][1]);
	//				//vertexArray[vertexCap][2] = xyz[7][2] + dVal*(xyz[4][2] - xyz[7][2]);
	//				vertexArray.Add(FVector(xyz[7][0] + dVal*(xyz[4][0] - xyz[7][0]), xyz[7][1] + dVal*(xyz[4][1] - xyz[7][1]), xyz[7][2] + dVal*(xyz[4][2] - xyz[7][2])));
	//				vertList[7] = y0Cache[z] = vertexCap;
	//				vertexCap++;
	//			}
	//			if (edgeTable[cubeIndex] & 1024) {
	//				dVal = (double)(iso_value - val[2]) / (double)(val[6] - val[2]);
	//				//vertexArray[vertexCap][0] = xyz[2][0] + dVal*(xyz[6][0] - xyz[2][0]);
	//				//vertexArray[vertexCap][1] = xyz[2][1] + dVal*(xyz[6][1] - xyz[2][1]);
	//				//vertexArray[vertexCap][2] = xyz[2][2] + dVal*(xyz[6][2] - xyz[2][2]);
	//				vertexArray.Add(FVector(xyz[2][0] + dVal*(xyz[6][0] - xyz[2][0]), xyz[2][1] + dVal*(xyz[6][1] - xyz[2][1]), xyz[2][2] + dVal*(xyz[6][2] - xyz[2][2])));
	//				isoCache[layerIndex][y][z].vertexIndex[2] = vertList[10] = vertexCap;
	//				vertexCap++;
	//			}
	//			if (edgeTable[cubeIndex] & 2048) {
	//				dVal = (double)(iso_value - val[3]) / (double)(val[7] - val[3]);
	//				/*vertexArray[vertexCap][0] = xyz[3][0] + dVal*(xyz[7][0] - xyz[3][0]);
	//				vertexArray[vertexCap][1] = xyz[3][1] + dVal*(xyz[7][1] - xyz[3][1]);
	//				vertexArray[vertexCap][2] = xyz[3][2] + dVal*(xyz[7][2] - xyz[3][2]);*/
	//				vertexArray.Add(FVector(xyz[3][0] + dVal*(xyz[7][0] - xyz[3][0]), xyz[3][1] + dVal*(xyz[7][1] - xyz[3][1]), xyz[3][2] + dVal*(xyz[7][2] - xyz[3][2])));
	//				vertList[11] = vertexCap;
	//				vertexCap++;
	//			}

	//			// bind triangle indecies
	//			for (int i = 0; triTable[cubeIndex][i] != -1; i += 3) {
	//				/*triangleArray[triangleCap*3 + 2] = vertList[triTable[cubeIndex][i]];
	//				triangleArray[triangleCap*3 + 1] = vertList[triTable[cubeIndex][i + 1]];
	//				triangleArray[triangleCap*3] = vertList[triTable[cubeIndex][i + 2]];*/
	//				triangleArray.Add(vertList[triTable[cubeIndex][i]]);
	//				triangleArray.Add(vertList[triTable[cubeIndex][i + 1]]);
	//				triangleArray.Add(vertList[triTable[cubeIndex][i + 2]]);

	//				triangleCap++;
	//			}
	//		}
	//	}

	//	//create the remaining rows of the first layer=============================================
	//	for (y = 1; y < res.Y - 1; y++) {

	//		z = 0;
	//		//create the first voxel of remaining rows of first layer=============================

	//		//inherit corner values from local variable -----------------------------------------

	//		//inherit corner values from isoCache -----------------------------------------------
	//		xyz[2][0] = isoCache[layerIndex][y - 1][z].cornerPoint[0];
	//		xyz[2][1] = isoCache[layerIndex][y - 1][z].cornerPoint[1];
	//		xyz[2][2] = isoCache[layerIndex][y - 1][z].cornerPoint[2];
	//		val[2] = data[x + 1][y][z + 1];
	//		cellIsoBool[2] = isoCache[layerIndex][y - 1][z].isoBool;

	//		//calculate corner values that could not be inherited ---------------------------------
	//		xyz[0][0] = (float)((x - (res.X / 2)) / ((float)(res.X / 2)))*dim.X;
	//		xyz[0][1] = (float)((y - (res.Y / 2)) / ((float)(res.Y / 2)))*dim.Y;
	//		xyz[0][2] = (float)((z - (res.Z / 2)) / ((float)(res.Z / 2)))*dim.Z;
	//		val[0] = data[x][y][z];
	//		if (data[x][y][z] < iso_value)			// cubeIndex |= 1;
	//			cellIsoBool[0] = true;
	//		else
	//			cellIsoBool[0] = false;

	//		xyz[1][0] = (float)((x + 1 - (res.X / 2)) / ((float)(res.X / 2)))*dim.X;
	//		xyz[1][1] = (float)((y - (res.Y / 2)) / ((float)(res.Y / 2)))*dim.Y;
	//		xyz[1][2] = (float)((z - (res.Z / 2)) / ((float)(res.Z / 2)))*dim.Z;
	//		val[1] = data[x + 1][y][z];
	//		if (data[x + 1][y][z] < iso_value)		// cubeIndex |= 2;
	//			cellIsoBool[1] = true;
	//		else
	//			cellIsoBool[1] = false;

	//		xyz[3][0] = (float)((x - (res.X / 2)) / ((float)(res.X / 2)))*dim.X;
	//		xyz[3][1] = (float)((y - (res.Y / 2)) / ((float)(res.Y / 2)))*dim.Y;
	//		xyz[3][2] = (float)((z + 1 - (res.Z / 2)) / ((float)(res.Z / 2)))*dim.Z;
	//		val[3] = data[x][y][z + 1];
	//		if (data[x][y][z + 1] < iso_value)		// cubeIndex |= 8;
	//			cellIsoBool[3] = true;
	//		else
	//			cellIsoBool[3] = false;

	//		xyz[4][0] = (float)((x - (res.X / 2)) / ((float)(res.X / 2)))*dim.X;
	//		xyz[4][1] = (float)((y + 1 - (res.Y / 2)) / ((float)(res.Y / 2)))*dim.Y;
	//		xyz[4][2] = (float)((z - (res.Z / 2)) / ((float)(res.Z / 2)))*dim.Z;
	//		val[4] = data[x][y + 1][z];
	//		if (data[x][y + 1][z] < iso_value)		//cubeIndex |= 16;
	//			cellIsoBool[4] = true;
	//		else
	//			cellIsoBool[4] = false;

	//		xyz[5][0] = (float)((x + 1 - (res.X / 2)) / ((float)(res.X / 2)))*dim.X;
	//		xyz[5][1] = (float)((y + 1 - (res.Y / 2)) / ((float)(res.Y / 2)))*dim.Y;
	//		xyz[5][2] = (float)((z - (res.Z / 2)) / ((float)(res.Z / 2)))*dim.Z;
	//		val[5] = data[x + 1][y + 1][z];
	//		if (data[x + 1][y + 1][z] < iso_value)	// cubeIndex |= 32;
	//			cellIsoBool[5] = true;
	//		else
	//			cellIsoBool[5] = false;

	//		//save the sixth corner values to isoCache
	//		xyz[6][0] = isoCache[layerIndex][y][z].cornerPoint[0] = (float)((x + 1 - (res.X / 2)) / ((float)(res.X / 2)))*dim.X;
	//		xyz[6][1] = isoCache[layerIndex][y][z].cornerPoint[1] = (float)((y + 1 - (res.Y / 2)) / ((float)(res.Y / 2)))*dim.Y;
	//		xyz[6][2] = isoCache[layerIndex][y][z].cornerPoint[2] = (float)((z + 1 - (res.Z / 2)) / ((float)(res.Z / 2)))*dim.Z;
	//		val[6] = data[x + 1][y + 1][z + 1];
	//		if (data[x + 1][y + 1][z + 1] < iso_value)// cubeIndex |= 64;
	//			cellIsoBool[6] = isoCache[layerIndex][y][z].isoBool = true;
	//		else
	//			cellIsoBool[6] = isoCache[layerIndex][y][z].isoBool = false;

	//		xyz[7][0] = (float)((x - (res.X / 2)) / ((float)(res.X / 2)))*dim.X;
	//		xyz[7][1] = (float)((y + 1 - (res.Y / 2)) / ((float)(res.Y / 2)))*dim.Y;
	//		xyz[7][2] = (float)((z + 1 - (res.Z / 2)) / ((float)(res.Z / 2)))*dim.Z;
	//		val[7] = data[x][y + 1][z + 1];
	//		if (data[x][y + 1][z + 1] < iso_value)	// cubeIndex |= 128;
	//			cellIsoBool[7] = true;
	//		else
	//			cellIsoBool[7] = false;

	//		// get the case index
	//		cubeIndex = cellIsoBool[0] * 1 + cellIsoBool[1] * 2 + cellIsoBool[2] * 4 + cellIsoBool[3] * 8 +
	//			cellIsoBool[4] * 16 + cellIsoBool[5] * 32 + cellIsoBool[6] * 64 + cellIsoBool[7] * 128;

	//		// check if cube is entirely in or out of the surface ----------------------------
	//		if (edgeTable[cubeIndex] != 0) {

	//			// Find the vertices where the surface intersects the cube--------------------

	//			//inherit vertex indices from local variable ---------------------------------
	//			if (edgeTable[cubeIndex] & 1) {
	//				vertList[0] = z0Cache;
	//			}
	//			if (edgeTable[cubeIndex] & 8) {
	//				vertList[3] = y0Cache[z];
	//			}

	//			//inherit vertex indices from isoCache ---------------------------------------
	//			if (edgeTable[cubeIndex] & 2) {
	//				vertList[1] = isoCache[layerIndex][y - 1][z].vertexIndex[0];
	//			}
	//			if (edgeTable[cubeIndex] & 4) {
	//				vertList[2] = isoCache[layerIndex][y - 1][z].vertexIndex[1];
	//			}

	//			//calculate indices that could not be inherited ------------------------------

	//			if (edgeTable[cubeIndex] & 16) {
	//				dVal = (double)(iso_value - val[4]) / (double)(val[5] - val[4]);
	//				//vertexArray[vertexCap][0] = xyz[4][0] + dVal*(xyz[5][0] - xyz[4][0]);
	//				//vertexArray[vertexCap][1] = xyz[4][1] + dVal*(xyz[5][1] - xyz[4][1]);
	//				//vertexArray[vertexCap][2] = xyz[4][2] + dVal*(xyz[5][2] - xyz[4][2]);
	//				vertexArray.Add(FVector(xyz[4][0] + dVal*(xyz[5][0] - xyz[4][0]), xyz[4][1] + dVal*(xyz[5][1] - xyz[4][1]), xyz[4][2] + dVal*(xyz[5][2] - xyz[4][2])));
	//				vertList[4] = z0Cache = vertexCap;
	//				vertexCap++;
	//			}
	//			if (edgeTable[cubeIndex] & 32) {
	//				dVal = (double)(iso_value - val[5]) / (double)(val[6] - val[5]);
	//				//vertexArray[vertexCap][0] = xyz[5][0] + dVal*(xyz[6][0] - xyz[5][0]);
	//				//vertexArray[vertexCap][1] = xyz[5][1] + dVal*(xyz[6][1] - xyz[5][1]);
	//				//vertexArray[vertexCap][2] = xyz[5][2] + dVal*(xyz[6][2] - xyz[5][2]);
	//				vertexArray.Add(FVector(xyz[5][0] + dVal*(xyz[6][0] - xyz[5][0]), xyz[5][1] + dVal*(xyz[6][1] - xyz[5][1]), xyz[5][2] + dVal*(xyz[6][2] - xyz[5][2])));
	//				isoCache[layerIndex][y][z].vertexIndex[0] = vertList[5] = vertexCap;
	//				vertexCap++;
	//			}
	//			if (edgeTable[cubeIndex] & 64) {
	//				dVal = (double)(iso_value - val[6]) / (double)(val[7] - val[6]);
	//				//vertexArray[vertexCap][0] = xyz[6][0] + dVal*(xyz[7][0] - xyz[6][0]);
	//				//vertexArray[vertexCap][1] = xyz[6][1] + dVal*(xyz[7][1] - xyz[6][1]);
	//				//vertexArray[vertexCap][2] = xyz[6][2] + dVal*(xyz[7][2] - xyz[6][2]);
	//				vertexArray.Add(FVector(xyz[6][0] + dVal*(xyz[7][0] - xyz[6][0]), xyz[6][1] + dVal*(xyz[7][1] - xyz[6][1]), xyz[6][2] + dVal*(xyz[7][2] - xyz[6][2])));
	//				isoCache[layerIndex][y][z].vertexIndex[1] = vertList[6] = vertexCap;
	//				vertexCap++;
	//			}
	//			if (edgeTable[cubeIndex] & 128) {
	//				dVal = (double)(iso_value - val[7]) / (double)(val[4] - val[7]);
	//				//vertexArray[vertexCap][0] = xyz[7][0] + dVal*(xyz[4][0] - xyz[7][0]);
	//				//vertexArray[vertexCap][1] = xyz[7][1] + dVal*(xyz[4][1] - xyz[7][1]);
	//				//vertexArray[vertexCap][2] = xyz[7][2] + dVal*(xyz[4][2] - xyz[7][2]);
	//				vertexArray.Add(FVector(xyz[7][0] + dVal*(xyz[4][0] - xyz[7][0]), xyz[7][1] + dVal*(xyz[4][1] - xyz[7][1]), xyz[7][2] + dVal*(xyz[4][2] - xyz[7][2])));
	//				vertList[7] = y0Cache[z] = vertexCap;
	//				vertexCap++;
	//			}
	//			if (edgeTable[cubeIndex] & 256) {
	//				dVal = (double)(iso_value - val[0]) / (double)(val[4] - val[0]);
	//				//vertexArray[vertexCap][0] = xyz[0][0] + dVal*(xyz[4][0] - xyz[0][0]);
	//				//vertexArray[vertexCap][1] = xyz[0][1] + dVal*(xyz[4][1] - xyz[0][1]);
	//				//vertexArray[vertexCap][2] = xyz[0][2] + dVal*(xyz[4][2] - xyz[0][2]);
	//				vertexArray.Add(FVector(xyz[0][0] + dVal*(xyz[4][0] - xyz[0][0]), xyz[0][1] + dVal*(xyz[4][1] - xyz[0][1]), xyz[0][2] + dVal*(xyz[4][2] - xyz[0][2])));
	//				vertList[8] = vertexCap;
	//				vertexCap++;
	//			}
	//			if (edgeTable[cubeIndex] & 512) {
	//				dVal = (double)(iso_value - val[1]) / (double)(val[5] - val[1]);
	//				//vertexArray[vertexCap][0] = xyz[1][0] + dVal*(xyz[5][0] - xyz[1][0]);
	//				//vertexArray[vertexCap][1] = xyz[1][1] + dVal*(xyz[5][1] - xyz[1][1]);
	//				//vertexArray[vertexCap][2] = xyz[1][2] + dVal*(xyz[5][2] - xyz[1][2]);
	//				vertexArray.Add(FVector(xyz[1][0] + dVal*(xyz[5][0] - xyz[1][0]), xyz[1][1] + dVal*(xyz[5][1] - xyz[1][1]), xyz[1][2] + dVal*(xyz[5][2] - xyz[1][2])));
	//				vertList[9] = vertexCap;
	//				vertexCap++;
	//			}
	//			if (edgeTable[cubeIndex] & 1024) {
	//				dVal = (double)(iso_value - val[2]) / (double)(val[6] - val[2]);
	//				//vertexArray[vertexCap][0] = xyz[2][0] + dVal*(xyz[6][0] - xyz[2][0]);
	//				//vertexArray[vertexCap][1] = xyz[2][1] + dVal*(xyz[6][1] - xyz[2][1]);
	//				//vertexArray[vertexCap][2] = xyz[2][2] + dVal*(xyz[6][2] - xyz[2][2]);
	//				vertexArray.Add(FVector(xyz[2][0] + dVal*(xyz[6][0] - xyz[2][0]), xyz[2][1] + dVal*(xyz[6][1] - xyz[2][1]), xyz[2][2] + dVal*(xyz[6][2] - xyz[2][2])));
	//				isoCache[layerIndex][y][z].vertexIndex[2] = vertList[10] = vertexCap;
	//				vertexCap++;
	//			}
	//			if (edgeTable[cubeIndex] & 2048) {
	//				dVal = (double)(iso_value - val[3]) / (double)(val[7] - val[3]);
	//				//vertexArray[vertexCap][0] = xyz[3][0] + dVal*(xyz[7][0] - xyz[3][0]);
	//				//vertexArray[vertexCap][1] = xyz[3][1] + dVal*(xyz[7][1] - xyz[3][1]);
	//				//vertexArray[vertexCap][2] = xyz[3][2] + dVal*(xyz[7][2] - xyz[3][2]);
	//				vertexArray.Add(FVector(xyz[3][0] + dVal*(xyz[7][0] - xyz[3][0]), xyz[3][1] + dVal*(xyz[7][1] - xyz[3][1]), xyz[3][2] + dVal*(xyz[7][2] - xyz[3][2])));
	//				vertList[11] = vertexCap;
	//				vertexCap++;
	//			}

	//			// bind triangle indecies
	//			for (int i = 0; triTable[cubeIndex][i] != -1; i += 3) {
	//				/*triangleArray[triangleCap*3 + 2] = vertList[triTable[cubeIndex][i]];
	//				triangleArray[triangleCap*3 + 1] = vertList[triTable[cubeIndex][i + 1]];
	//				triangleArray[triangleCap*3] = vertList[triTable[cubeIndex][i + 2]];*/
	//				triangleArray.Add(vertList[triTable[cubeIndex][i]]);
	//				triangleArray.Add(vertList[triTable[cubeIndex][i + 1]]);
	//				triangleArray.Add(vertList[triTable[cubeIndex][i + 2]]);

	//				triangleCap++;
	//			}
	//		}

	//		//create the remaining voxels of remaining rows of first layer=============================
	//		for (z = 1; z < res.Z - 1; z++) {
	//			//inherit corner values from local variable
	//			xyz[0][0] = xyz[3][0];
	//			xyz[0][1] = xyz[3][1];
	//			xyz[0][2] = xyz[3][2];
	//			val[0] = data[x][y][z];
	//			cellIsoBool[0] = cellIsoBool[3];

	//			xyz[4][0] = xyz[7][0];
	//			xyz[4][1] = xyz[7][1];
	//			xyz[4][2] = xyz[7][2];
	//			val[4] = data[x][y + 1][z];
	//			cellIsoBool[4] = cellIsoBool[7];

	//			//inherit corner values from isoCache
	//			xyz[1][0] = isoCache[layerIndex][y - 1][z - 1].cornerPoint[0];
	//			xyz[1][1] = isoCache[layerIndex][y - 1][z - 1].cornerPoint[1];
	//			xyz[1][2] = isoCache[layerIndex][y - 1][z - 1].cornerPoint[2];
	//			val[1] = data[x + 1][y][z];
	//			cellIsoBool[1] = isoCache[layerIndex][y - 1][z - 1].isoBool;

	//			xyz[2][0] = isoCache[layerIndex][y - 1][z].cornerPoint[0];
	//			xyz[2][1] = isoCache[layerIndex][y - 1][z].cornerPoint[1];
	//			xyz[2][2] = isoCache[layerIndex][y - 1][z].cornerPoint[2];
	//			val[2] = data[x + 1][y][z + 1];
	//			cellIsoBool[2] = isoCache[layerIndex][y - 1][z].isoBool;

	//			xyz[5][0] = isoCache[layerIndex][y][z - 1].cornerPoint[0];
	//			xyz[5][1] = isoCache[layerIndex][y][z - 1].cornerPoint[1];
	//			xyz[5][2] = isoCache[layerIndex][y][z - 1].cornerPoint[2];
	//			val[5] = data[x + 1][y + 1][z];
	//			cellIsoBool[5] = isoCache[layerIndex][y][z - 1].isoBool;

	//			//calculate corner values that could not be inherited
	//			xyz[3][0] = (float)((x - (res.X / 2)) / ((float)(res.X / 2)))*dim.X;
	//			xyz[3][1] = (float)((y - (res.Y / 2)) / ((float)(res.Y / 2)))*dim.Y;
	//			xyz[3][2] = (float)((z + 1 - (res.Z / 2)) / ((float)(res.Z / 2)))*dim.Z;
	//			val[3] = data[x][y][z + 1];
	//			if (data[x][y][z + 1] < iso_value)		// cubeIndex |= 8;
	//				cellIsoBool[3] = true;
	//			else
	//				cellIsoBool[3] = false;

	//			//save the sixth corner values to isoCache
	//			xyz[6][0] = isoCache[layerIndex][y][z].cornerPoint[0] = (float)((x + 1 - (res.X / 2)) / ((float)(res.X / 2)))*dim.X;
	//			xyz[6][1] = isoCache[layerIndex][y][z].cornerPoint[1] = (float)((y + 1 - (res.Y / 2)) / ((float)(res.Y / 2)))*dim.Y;
	//			xyz[6][2] = isoCache[layerIndex][y][z].cornerPoint[2] = (float)((z + 1 - (res.Z / 2)) / ((float)(res.Z / 2)))*dim.Z;
	//			val[6] = data[x + 1][y + 1][z + 1];
	//			if (data[x + 1][y + 1][z + 1] < iso_value)// cubeIndex |= 64;
	//				cellIsoBool[6] = isoCache[layerIndex][y][z].isoBool = true;
	//			else
	//				cellIsoBool[6] = isoCache[layerIndex][y][z].isoBool = false;

	//			xyz[7][0] = (float)((x - (res.X / 2)) / ((float)(res.X / 2)))*dim.X;
	//			xyz[7][1] = (float)((y + 1 - (res.Y / 2)) / ((float)(res.Y / 2)))*dim.Y;
	//			xyz[7][2] = (float)((z + 1 - (res.Z / 2)) / ((float)(res.Z / 2)))*dim.Z;
	//			val[7] = data[x][y + 1][z + 1];
	//			if (data[x][y + 1][z + 1] < iso_value)	// cubeIndex |= 128;
	//				cellIsoBool[7] = true;
	//			else
	//				cellIsoBool[7] = false;

	//			// get the case index
	//			cubeIndex = cellIsoBool[0] * 1 + cellIsoBool[1] * 2 + cellIsoBool[2] * 4 + cellIsoBool[3] * 8 +
	//				cellIsoBool[4] * 16 + cellIsoBool[5] * 32 + cellIsoBool[6] * 64 + cellIsoBool[7] * 128;

	//			// check if cube is entirely in or out of the surface ----------------------------
	//			if (edgeTable[cubeIndex] != 0) {

	//				// Find the vertices where the surface intersects the cube--------------------

	//				//inherit vertex indices from local variable ---------------------------------
	//				if (edgeTable[cubeIndex] & 8) {
	//					vertList[3] = y0Cache[z];
	//				}
	//				if (edgeTable[cubeIndex] & 256) {
	//					vertList[8] = vertList[11];
	//				}

	//				//inherit vertex indices from isoCache ---------------------------------------
	//				if (edgeTable[cubeIndex] & 1) {
	//					vertList[0] = isoCache[layerIndex][y - 1][z - 1].vertexIndex[1];
	//				}
	//				if (edgeTable[cubeIndex] & 16) {
	//					vertList[4] = isoCache[layerIndex][y][z - 1].vertexIndex[1];
	//				}
	//				if (edgeTable[cubeIndex] & 512) {
	//					vertList[9] = isoCache[layerIndex][y][z - 1].vertexIndex[2];
	//				}
	//				if (edgeTable[cubeIndex] & 2) {
	//					vertList[1] = isoCache[layerIndex][y - 1][z].vertexIndex[0];
	//				}
	//				if (edgeTable[cubeIndex] & 4) {
	//					vertList[2] = isoCache[layerIndex][y - 1][z].vertexIndex[1];
	//				}

	//				//calculate indices that could not be inherited ------------------------------

	//				if (edgeTable[cubeIndex] & 32) {
	//					dVal = (double)(iso_value - val[5]) / (double)(val[6] - val[5]);
	//					//vertexArray[vertexCap][0] = xyz[5][0] + dVal*(xyz[6][0] - xyz[5][0]);
	//					//vertexArray[vertexCap][1] = xyz[5][1] + dVal*(xyz[6][1] - xyz[5][1]);
	//					//vertexArray[vertexCap][2] = xyz[5][2] + dVal*(xyz[6][2] - xyz[5][2]);
	//					vertexArray.Add(FVector(xyz[5][0] + dVal*(xyz[6][0] - xyz[5][0]), xyz[5][1] + dVal*(xyz[6][1] - xyz[5][1]), xyz[5][2] + dVal*(xyz[6][2] - xyz[5][2])));
	//					isoCache[layerIndex][y][z].vertexIndex[0] = vertList[5] = vertexCap;
	//					vertexCap++;
	//				}
	//				if (edgeTable[cubeIndex] & 64) {
	//					dVal = (double)(iso_value - val[6]) / (double)(val[7] - val[6]);
	//					//vertexArray[vertexCap][0] = xyz[6][0] + dVal*(xyz[7][0] - xyz[6][0]);
	//					//vertexArray[vertexCap][1] = xyz[6][1] + dVal*(xyz[7][1] - xyz[6][1]);
	//					//vertexArray[vertexCap][2] = xyz[6][2] + dVal*(xyz[7][2] - xyz[6][2]);
	//					vertexArray.Add(FVector(xyz[6][0] + dVal*(xyz[7][0] - xyz[6][0]), xyz[6][1] + dVal*(xyz[7][1] - xyz[6][1]), xyz[6][2] + dVal*(xyz[7][2] - xyz[6][2])));
	//					isoCache[layerIndex][y][z].vertexIndex[1] = vertList[6] = vertexCap;
	//					vertexCap++;
	//				}
	//				if (edgeTable[cubeIndex] & 128) {
	//					dVal = (double)(iso_value - val[7]) / (double)(val[4] - val[7]);
	//					//vertexArray[vertexCap][0] = xyz[7][0] + dVal*(xyz[4][0] - xyz[7][0]);
	//					//vertexArray[vertexCap][1] = xyz[7][1] + dVal*(xyz[4][1] - xyz[7][1]);
	//					//vertexArray[vertexCap][2] = xyz[7][2] + dVal*(xyz[4][2] - xyz[7][2]);
	//					vertexArray.Add(FVector(xyz[7][0] + dVal*(xyz[4][0] - xyz[7][0]), xyz[7][1] + dVal*(xyz[4][1] - xyz[7][1]), xyz[7][2] + dVal*(xyz[4][2] - xyz[7][2])));
	//					vertList[7] = y0Cache[z] = vertexCap;
	//					vertexCap++;
	//				}
	//				if (edgeTable[cubeIndex] & 1024) {
	//					dVal = (double)(iso_value - val[2]) / (double)(val[6] - val[2]);
	//					//vertexArray[vertexCap][0] = xyz[2][0] + dVal*(xyz[6][0] - xyz[2][0]);
	//					//vertexArray[vertexCap][1] = xyz[2][1] + dVal*(xyz[6][1] - xyz[2][1]);
	//					//vertexArray[vertexCap][2] = xyz[2][2] + dVal*(xyz[6][2] - xyz[2][2]);
	//					vertexArray.Add(FVector(xyz[2][0] + dVal*(xyz[6][0] - xyz[2][0]), xyz[2][1] + dVal*(xyz[6][1] - xyz[2][1]), xyz[2][2] + dVal*(xyz[6][2] - xyz[2][2])));
	//					isoCache[layerIndex][y][z].vertexIndex[2] = vertList[10] = vertexCap;
	//					vertexCap++;
	//				}
	//				if (edgeTable[cubeIndex] & 2048) {
	//					dVal = (double)(iso_value - val[3]) / (double)(val[7] - val[3]);
	//					//vertexArray[vertexCap][0] = xyz[3][0] + dVal*(xyz[7][0] - xyz[3][0]);
	//					//vertexArray[vertexCap][1] = xyz[3][1] + dVal*(xyz[7][1] - xyz[3][1]);
	//					//vertexArray[vertexCap][2] = xyz[3][2] + dVal*(xyz[7][2] - xyz[3][2]);
	//					vertexArray.Add(FVector(xyz[3][0] + dVal*(xyz[7][0] - xyz[3][0]), xyz[3][1] + dVal*(xyz[7][1] - xyz[3][1]), xyz[3][2] + dVal*(xyz[7][2] - xyz[3][2])));
	//					vertList[11] = vertexCap;
	//					vertexCap++;
	//				}

	//				// bind triangle indecies
	//				for (int i = 0; triTable[cubeIndex][i] != -1; i += 3) {
	//					/*triangleArray[triangleCap*3 + 2] = vertList[triTable[cubeIndex][i]];
	//					triangleArray[triangleCap*3 + 1] = vertList[triTable[cubeIndex][i + 1]];
	//					triangleArray[triangleCap*3] = vertList[triTable[cubeIndex][i + 2]];*/
	//					triangleArray.Add(vertList[triTable[cubeIndex][i]]);
	//					triangleArray.Add(vertList[triTable[cubeIndex][i + 1]]);
	//					triangleArray.Add(vertList[triTable[cubeIndex][i + 2]]);

	//					triangleCap++;
	//				}
	//			}
	//		}
	//	}

	//	layerIndex++; // move to next layer
	//				  // create remaining layers ============================================================================
	//	for (x = 1; x < res.X - 1; x++) {
	//		y = 0;
	//		z = 0;
	//		//create first voxel of first row of remaining layers ---------------------------------------------


	//		//inherit corner values from local variable -------------------------------------------------------


	//		//inherit corner values from isoCache -------------------------------------------------------------
	//		xyz[7][0] = isoCache[(layerIndex + 1) % 2][y][z].cornerPoint[0];
	//		xyz[7][1] = isoCache[(layerIndex + 1) % 2][y][z].cornerPoint[1];
	//		xyz[7][2] = isoCache[(layerIndex + 1) % 2][y][z].cornerPoint[2];
	//		val[7] = data[x][y + 1][z + 1];
	//		cellIsoBool[7] = isoCache[(layerIndex + 1) % 2][y][z].isoBool;

	//		//calculate corner values that could not be inherited
	//		xyz[0][0] = (float)((x - (res.X / 2)) / ((float)(res.X / 2)))*dim.X;
	//		xyz[0][1] = (float)((y - (res.Y / 2)) / ((float)(res.Y / 2)))*dim.Y;
	//		xyz[0][2] = (float)((z - (res.Z / 2)) / ((float)(res.Z / 2)))*dim.Z;
	//		val[0] = data[x][y][z];
	//		if (data[x][y][z] < iso_value)			// cubeIndex |= 1;
	//			cellIsoBool[0] = true;
	//		else
	//			cellIsoBool[0] = false;

	//		xyz[1][0] = (float)((x + 1 - (res.X / 2)) / ((float)(res.X / 2)))*dim.X;
	//		xyz[1][1] = (float)((y - (res.Y / 2)) / ((float)(res.Y / 2)))*dim.Y;
	//		xyz[1][2] = (float)((z - (res.Z / 2)) / ((float)(res.Z / 2)))*dim.Z;
	//		val[1] = data[x + 1][y][z];
	//		if (data[x + 1][y][z] < iso_value)		// cubeIndex |= 2;
	//			cellIsoBool[1] = true;
	//		else
	//			cellIsoBool[1] = false;

	//		xyz[2][0] = (float)((x + 1 - (res.X / 2)) / ((float)(res.X / 2)))*dim.X;
	//		xyz[2][1] = (float)((y - (res.Y / 2)) / ((float)(res.Y / 2)))*dim.Y;
	//		xyz[2][2] = (float)((z + 1 - (res.Z / 2)) / ((float)(res.Z / 2)))*dim.Z;
	//		val[2] = data[x + 1][y][z + 1];
	//		if (data[x + 1][y][z + 1] < iso_value)	// cubeIndex |= 4;
	//			cellIsoBool[2] = true;
	//		else
	//			cellIsoBool[2] = false;

	//		xyz[3][0] = (float)((x - (res.X / 2)) / ((float)(res.X / 2)))*dim.X;
	//		xyz[3][1] = (float)((y - (res.Y / 2)) / ((float)(res.Y / 2)))*dim.Y;
	//		xyz[3][2] = (float)((z + 1 - (res.Z / 2)) / ((float)(res.Z / 2)))*dim.Z;
	//		val[3] = data[x][y][z + 1];
	//		if (data[x][y][z + 1] < iso_value)		// cubeIndex |= 8;
	//			cellIsoBool[3] = true;
	//		else
	//			cellIsoBool[3] = false;

	//		xyz[4][0] = (float)((x - (res.X / 2)) / ((float)(res.X / 2)))*dim.X;
	//		xyz[4][1] = (float)((y + 1 - (res.Y / 2)) / ((float)(res.Y / 2)))*dim.Y;
	//		xyz[4][2] = (float)((z - (res.Z / 2)) / ((float)(res.Z / 2)))*dim.Z;
	//		val[4] = data[x][y + 1][z];
	//		if (data[x][y + 1][z] < iso_value)		//cubeIndex |= 16;
	//			cellIsoBool[4] = true;
	//		else
	//			cellIsoBool[4] = false;

	//		xyz[5][0] = (float)((x + 1 - (res.X / 2)) / ((float)(res.X / 2)))*dim.X;
	//		xyz[5][1] = (float)((y + 1 - (res.Y / 2)) / ((float)(res.Y / 2)))*dim.Y;
	//		xyz[5][2] = (float)((z - (res.Z / 2)) / ((float)(res.Z / 2)))*dim.Z;
	//		val[5] = data[x + 1][y + 1][z];
	//		if (data[x + 1][y + 1][z] < iso_value)	// cubeIndex |= 32;
	//			cellIsoBool[5] = true;
	//		else
	//			cellIsoBool[5] = false;

	//		//save the sixth corner values to isoCache
	//		xyz[6][0] = isoCache[layerIndex][y][z].cornerPoint[0] = (float)((x + 1 - (res.X / 2)) / ((float)(res.X / 2)))*dim.X;
	//		xyz[6][1] = isoCache[layerIndex][y][z].cornerPoint[1] = (float)((y + 1 - (res.Y / 2)) / ((float)(res.Y / 2)))*dim.Y;
	//		xyz[6][2] = isoCache[layerIndex][y][z].cornerPoint[2] = (float)((z + 1 - (res.Z / 2)) / ((float)(res.Z / 2)))*dim.Z;
	//		val[6] = data[x + 1][y + 1][z + 1];
	//		if (data[x + 1][y + 1][z + 1] < iso_value)// cubeIndex |= 64;
	//			cellIsoBool[6] = isoCache[layerIndex][y][z].isoBool = true;
	//		else
	//			cellIsoBool[6] = isoCache[layerIndex][y][z].isoBool = false;

	//		// get the case index
	//		cubeIndex = cellIsoBool[0] * 1 + cellIsoBool[1] * 2 + cellIsoBool[2] * 4 + cellIsoBool[3] * 8 +
	//			cellIsoBool[4] * 16 + cellIsoBool[5] * 32 + cellIsoBool[6] * 64 + cellIsoBool[7] * 128;

	//		// check if cube is entirely in or out of the surface -----------------------------------------------
	//		if (edgeTable[cubeIndex] != 0) {

	//			// Find the vertices where the surface intersects the cube---------------------------------------

	//			//inherit vertex indices from local variable ----------------------------------------------------

	//			//inherit vertex indices from isoCache ----------------------------------------------------------
	//			if (edgeTable[cubeIndex] & 128) {
	//				vertList[7] = isoCache[(layerIndex + 1) % 2][y][z].vertexIndex[0];
	//			}

	//			if (edgeTable[cubeIndex] & 2048) {
	//				vertList[11] = isoCache[(layerIndex + 1) % 2][y][z].vertexIndex[2];
	//			}

	//			//calculate indices that could not be inherited ------------------------------
	//			if (edgeTable[cubeIndex] & 1) {
	//				dVal = (double)(iso_value - val[0]) / (double)(val[1] - val[0]);
	//				/*vertexArray[vertexCap][0] = xyz[0][0] + dVal*(xyz[1][0] - xyz[0][0]);
	//				vertexArray[vertexCap][1] = xyz[0][1] + dVal*(xyz[1][1] - xyz[0][1]);
	//				vertexArray[vertexCap][2] = xyz[0][2] + dVal*(xyz[1][2] - xyz[0][2]);*/
	//				vertexArray.Add(FVector(xyz[0][0] + dVal*(xyz[1][0] - xyz[0][0]), xyz[0][1] + dVal*(xyz[1][1] - xyz[0][1]), xyz[0][2] + dVal*(xyz[1][2] - xyz[0][2])));
	//				vertList[0] = vertexCap;
	//				vertexCap++;
	//			}

	//			if (edgeTable[cubeIndex] & 2) {
	//				dVal = (double)(iso_value - val[1]) / (double)(val[2] - val[1]);
	//				/*vertexArray[vertexCap][0] = xyz[1][0] + dVal*(xyz[2][0] - xyz[1][0]);
	//				vertexArray[vertexCap][1] = xyz[1][1] + dVal*(xyz[2][1] - xyz[1][1]);
	//				vertexArray[vertexCap][2] = xyz[1][2] + dVal*(xyz[2][2] - xyz[1][2]);*/
	//				vertexArray.Add(FVector(xyz[1][0] + dVal*(xyz[2][0] - xyz[1][0]), xyz[1][1] + dVal*(xyz[2][1] - xyz[1][1]), xyz[1][2] + dVal*(xyz[2][2] - xyz[1][2])));
	//				vertList[1] = vertexCap;
	//				vertexCap++;
	//			}
	//			if (edgeTable[cubeIndex] & 4) {
	//				dVal = (double)(iso_value - val[2]) / (double)(val[3] - val[2]);
	//				//vertexArray[vertexCap][0] = xyz[2][0] + dVal*(xyz[3][0] - xyz[2][0]);
	//				//vertexArray[vertexCap][1] = xyz[2][1] + dVal*(xyz[3][1] - xyz[2][1]);
	//				//vertexArray[vertexCap][2] = xyz[2][2] + dVal*(xyz[3][2] - xyz[2][2]);
	//				vertexArray.Add(FVector(xyz[2][0] + dVal*(xyz[3][0] - xyz[2][0]), xyz[2][1] + dVal*(xyz[3][1] - xyz[2][1]), xyz[2][2] + dVal*(xyz[3][2] - xyz[2][2])));
	//				vertList[2] = vertexCap;
	//				vertexCap++;
	//			}
	//			if (edgeTable[cubeIndex] & 8) {
	//				dVal = (double)(iso_value - val[3]) / (double)(val[0] - val[3]);
	//				/*	vertexArray[vertexCap][0] = xyz[3][0] + dVal*(xyz[0][0] - xyz[3][0]);
	//				vertexArray[vertexCap][1] = xyz[3][1] + dVal*(xyz[0][1] - xyz[3][1]);
	//				vertexArray[vertexCap][2] = xyz[3][2] + dVal*(xyz[0][2] - xyz[3][2]);*/
	//				vertexArray.Add(FVector(xyz[3][0] + dVal*(xyz[0][0] - xyz[3][0]), xyz[3][1] + dVal*(xyz[0][1] - xyz[3][1]), xyz[3][2] + dVal*(xyz[0][2] - xyz[3][2])));
	//				vertList[3] = vertexCap;
	//				vertexCap++;
	//			}
	//			if (edgeTable[cubeIndex] & 16) {
	//				dVal = (double)(iso_value - val[4]) / (double)(val[5] - val[4]);
	//				//vertexArray[vertexCap][0] = xyz[4][0] + dVal*(xyz[5][0] - xyz[4][0]);
	//				//vertexArray[vertexCap][1] = xyz[4][1] + dVal*(xyz[5][1] - xyz[4][1]);
	//				//vertexArray[vertexCap][2] = xyz[4][2] + dVal*(xyz[5][2] - xyz[4][2]);
	//				vertexArray.Add(FVector(xyz[4][0] + dVal*(xyz[5][0] - xyz[4][0]), xyz[4][1] + dVal*(xyz[5][1] - xyz[4][1]), xyz[4][2] + dVal*(xyz[5][2] - xyz[4][2])));
	//				vertList[4] = z0Cache = vertexCap;
	//				vertexCap++;
	//			}
	//			if (edgeTable[cubeIndex] & 32) {
	//				dVal = (double)(iso_value - val[5]) / (double)(val[6] - val[5]);
	//				//vertexArray[vertexCap][0] = xyz[5][0] + dVal*(xyz[6][0] - xyz[5][0]);
	//				//vertexArray[vertexCap][1] = xyz[5][1] + dVal*(xyz[6][1] - xyz[5][1]);
	//				//vertexArray[vertexCap][2] = xyz[5][2] + dVal*(xyz[6][2] - xyz[5][2]);
	//				vertexArray.Add(FVector(xyz[5][0] + dVal*(xyz[6][0] - xyz[5][0]), xyz[5][1] + dVal*(xyz[6][1] - xyz[5][1]), xyz[5][2] + dVal*(xyz[6][2] - xyz[5][2])));
	//				isoCache[layerIndex][y][z].vertexIndex[0] = vertList[5] = vertexCap;
	//				vertexCap++;
	//			}
	//			if (edgeTable[cubeIndex] & 64) {
	//				dVal = (double)(iso_value - val[6]) / (double)(val[7] - val[6]);
	//				//vertexArray[vertexCap][0] = xyz[6][0] + dVal*(xyz[7][0] - xyz[6][0]);
	//				//vertexArray[vertexCap][1] = xyz[6][1] + dVal*(xyz[7][1] - xyz[6][1]);
	//				//vertexArray[vertexCap][2] = xyz[6][2] + dVal*(xyz[7][2] - xyz[6][2]);
	//				vertexArray.Add(FVector(xyz[6][0] + dVal*(xyz[7][0] - xyz[6][0]), xyz[6][1] + dVal*(xyz[7][1] - xyz[6][1]), xyz[6][2] + dVal*(xyz[7][2] - xyz[6][2])));
	//				isoCache[layerIndex][y][z].vertexIndex[1] = vertList[6] = vertexCap;
	//				vertexCap++;
	//			}

	//			if (edgeTable[cubeIndex] & 256) {
	//				dVal = (double)(iso_value - val[0]) / (double)(val[4] - val[0]);
	//				//vertexArray[vertexCap][0] = xyz[0][0] + dVal*(xyz[4][0] - xyz[0][0]);
	//				//vertexArray[vertexCap][1] = xyz[0][1] + dVal*(xyz[4][1] - xyz[0][1]);
	//				//vertexArray[vertexCap][2] = xyz[0][2] + dVal*(xyz[4][2] - xyz[0][2]);
	//				vertexArray.Add(FVector(xyz[0][0] + dVal*(xyz[4][0] - xyz[0][0]), xyz[0][1] + dVal*(xyz[4][1] - xyz[0][1]), xyz[0][2] + dVal*(xyz[4][2] - xyz[0][2])));
	//				vertList[8] = vertexCap;
	//				vertexCap++;
	//			}
	//			if (edgeTable[cubeIndex] & 512) {
	//				dVal = (double)(iso_value - val[1]) / (double)(val[5] - val[1]);
	//				//vertexArray[vertexCap][0] = xyz[1][0] + dVal*(xyz[5][0] - xyz[1][0]);
	//				//vertexArray[vertexCap][1] = xyz[1][1] + dVal*(xyz[5][1] - xyz[1][1]);
	//				//vertexArray[vertexCap][2] = xyz[1][2] + dVal*(xyz[5][2] - xyz[1][2]);
	//				vertexArray.Add(FVector(xyz[1][0] + dVal*(xyz[5][0] - xyz[1][0]), xyz[1][1] + dVal*(xyz[5][1] - xyz[1][1]), xyz[1][2] + dVal*(xyz[5][2] - xyz[1][2])));
	//				vertList[9] = vertexCap;
	//				vertexCap++;
	//			}
	//			if (edgeTable[cubeIndex] & 1024) {
	//				dVal = (double)(iso_value - val[2]) / (double)(val[6] - val[2]);
	//				//vertexArray[vertexCap][0] = xyz[2][0] + dVal*(xyz[6][0] - xyz[2][0]);
	//				//vertexArray[vertexCap][1] = xyz[2][1] + dVal*(xyz[6][1] - xyz[2][1]);
	//				//vertexArray[vertexCap][2] = xyz[2][2] + dVal*(xyz[6][2] - xyz[2][2]);
	//				vertexArray.Add(FVector(xyz[2][0] + dVal*(xyz[6][0] - xyz[2][0]), xyz[2][1] + dVal*(xyz[6][1] - xyz[2][1]), xyz[2][2] + dVal*(xyz[6][2] - xyz[2][2])));
	//				isoCache[layerIndex][y][z].vertexIndex[2] = vertList[10] = vertexCap;
	//				vertexCap++;
	//			}

	//			// bind triangle indecies
	//			for (int i = 0; triTable[cubeIndex][i] != -1; i += 3) {
	//				/*triangleArray[triangleCap*3 + 2] = vertList[triTable[cubeIndex][i]];
	//				triangleArray[triangleCap*3 + 1] = vertList[triTable[cubeIndex][i + 1]];
	//				triangleArray[triangleCap*3] = vertList[triTable[cubeIndex][i + 2]];*/
	//				triangleArray.Add(vertList[triTable[cubeIndex][i]]);
	//				triangleArray.Add(vertList[triTable[cubeIndex][i + 1]]);
	//				triangleArray.Add(vertList[triTable[cubeIndex][i + 2]]);

	//				triangleCap++;
	//			}
	//		}

	//		//create remaining voxels of first row of remaining layers ===========================
	//		for (z = 1; z < res.Z - 1; z++) {

	//			//inherit corner values from local variable --------------------------------------
	//			xyz[0][0] = xyz[3][0];
	//			xyz[0][1] = xyz[3][1];
	//			xyz[0][2] = xyz[3][2];
	//			val[0] = data[x][y][z];
	//			cellIsoBool[0] = cellIsoBool[3];

	//			xyz[1][0] = xyz[2][0];
	//			xyz[1][1] = xyz[2][1];
	//			xyz[1][2] = xyz[2][2];
	//			val[1] = data[x + 1][y][z];
	//			cellIsoBool[1] = cellIsoBool[2];

	//			//inherit corner values from isoCache --------------------------------------------
	//			xyz[4][0] = isoCache[(layerIndex + 1) % 2][y][z - 1].cornerPoint[0];
	//			xyz[4][1] = isoCache[(layerIndex + 1) % 2][y][z - 1].cornerPoint[1];
	//			xyz[4][2] = isoCache[(layerIndex + 1) % 2][y][z - 1].cornerPoint[2];
	//			val[4] = data[x][y + 1][z];
	//			cellIsoBool[4] = isoCache[(layerIndex + 1) % 2][y][z - 1].isoBool;

	//			xyz[5][0] = isoCache[layerIndex][y][z - 1].cornerPoint[0];
	//			xyz[5][1] = isoCache[layerIndex][y][z - 1].cornerPoint[1];
	//			xyz[5][2] = isoCache[layerIndex][y][z - 1].cornerPoint[2];
	//			val[5] = data[x + 1][y + 1][z];
	//			cellIsoBool[5] = isoCache[layerIndex][y][z - 1].isoBool;

	//			xyz[7][0] = isoCache[(layerIndex + 1) % 2][y][z].cornerPoint[0];
	//			xyz[7][1] = isoCache[(layerIndex + 1) % 2][y][z].cornerPoint[1];
	//			xyz[7][2] = isoCache[(layerIndex + 1) % 2][y][z].cornerPoint[2];
	//			val[7] = data[x][y + 1][z + 1];
	//			cellIsoBool[7] = isoCache[(layerIndex + 1) % 2][y][z].isoBool;

	//			//calculate corner values that could not be inherited ---------------------------
	//			xyz[2][0] = (float)((x + 1 - (res.X / 2)) / ((float)(res.X / 2)))*dim.X;
	//			xyz[2][1] = (float)((y - (res.Y / 2)) / ((float)(res.Y / 2)))*dim.Y;
	//			xyz[2][2] = (float)((z + 1 - (res.Z / 2)) / ((float)(res.Z / 2)))*dim.Z;
	//			val[2] = data[x + 1][y][z + 1];
	//			if (data[x + 1][y][z + 1] < iso_value)	// cubeIndex |= 4;
	//				cellIsoBool[2] = true;
	//			else
	//				cellIsoBool[2] = false;

	//			xyz[3][0] = (float)((x - (res.X / 2)) / ((float)(res.X / 2)))*dim.X;
	//			xyz[3][1] = (float)((y - (res.Y / 2)) / ((float)(res.Y / 2)))*dim.Y;
	//			xyz[3][2] = (float)((z + 1 - (res.Z / 2)) / ((float)(res.Z / 2)))*dim.Z;
	//			val[3] = data[x][y][z + 1];
	//			if (data[x][y][z + 1] < iso_value)		// cubeIndex |= 8;
	//				cellIsoBool[3] = true;
	//			else
	//				cellIsoBool[3] = false;

	//			//save the sixth corner values to isoCache
	//			xyz[6][0] = isoCache[layerIndex][y][z].cornerPoint[0] = (float)((x + 1 - (res.X / 2)) / ((float)(res.X / 2)))*dim.X;
	//			xyz[6][1] = isoCache[layerIndex][y][z].cornerPoint[1] = (float)((y + 1 - (res.Y / 2)) / ((float)(res.Y / 2)))*dim.Y;
	//			xyz[6][2] = isoCache[layerIndex][y][z].cornerPoint[2] = (float)((z + 1 - (res.Z / 2)) / ((float)(res.Z / 2)))*dim.Z;
	//			val[6] = data[x + 1][y + 1][z + 1];
	//			if (data[x + 1][y + 1][z + 1] < iso_value)// cubeIndex |= 64;
	//				cellIsoBool[6] = isoCache[layerIndex][y][z].isoBool = true;
	//			else
	//				cellIsoBool[6] = isoCache[layerIndex][y][z].isoBool = false;

	//			// get the case index
	//			cubeIndex = cellIsoBool[0] * 1 + cellIsoBool[1] * 2 + cellIsoBool[2] * 4 + cellIsoBool[3] * 8 +
	//				cellIsoBool[4] * 16 + cellIsoBool[5] * 32 + cellIsoBool[6] * 64 + cellIsoBool[7] * 128;

	//			// check if cube is entirely in or out of the surface ----------------------------
	//			if (edgeTable[cubeIndex] != 0) {

	//				// Find the vertices where the surface intersects the cube--------------------

	//				//inherit vertex indices from local variable ---------------------------------
	//				if (edgeTable[cubeIndex] & 1) {
	//					vertList[0] = vertList[2];
	//				}


	//				//inherit vertex indices from isoCache ---------------------------------------
	//				if (edgeTable[cubeIndex] & 16) {
	//					vertList[4] = isoCache[layerIndex][y][z - 1].vertexIndex[1];
	//				}
	//				if (edgeTable[cubeIndex] & 128) {
	//					vertList[7] = isoCache[(layerIndex + 1) % 2][y][z].vertexIndex[0];
	//				}
	//				if (edgeTable[cubeIndex] & 256) {
	//					vertList[8] = isoCache[(layerIndex + 1) % 2][y][z - 1].vertexIndex[2];
	//				}
	//				if (edgeTable[cubeIndex] & 512) {
	//					vertList[9] = isoCache[layerIndex][y][z - 1].vertexIndex[2];
	//				}
	//				if (edgeTable[cubeIndex] & 2048) {
	//					vertList[11] = isoCache[(layerIndex + 1) % 2][y][z].vertexIndex[2];
	//				}

	//				//calculate indices that could not be inherited ------------------------------
	//				if (edgeTable[cubeIndex] & 2) {
	//					dVal = (double)(iso_value - val[1]) / (double)(val[2] - val[1]);
	//					/*vertexArray[vertexCap][0] = xyz[1][0] + dVal*(xyz[2][0] - xyz[1][0]);
	//					vertexArray[vertexCap][1] = xyz[1][1] + dVal*(xyz[2][1] - xyz[1][1]);
	//					vertexArray[vertexCap][2] = xyz[1][2] + dVal*(xyz[2][2] - xyz[1][2]);*/
	//					vertexArray.Add(FVector(xyz[1][0] + dVal*(xyz[2][0] - xyz[1][0]), xyz[1][1] + dVal*(xyz[2][1] - xyz[1][1]), xyz[1][2] + dVal*(xyz[2][2] - xyz[1][2])));
	//					vertList[1] = vertexCap;
	//					vertexCap++;
	//				}
	//				if (edgeTable[cubeIndex] & 4) {
	//					dVal = (double)(iso_value - val[2]) / (double)(val[3] - val[2]);
	//					//vertexArray[vertexCap][0] = xyz[2][0] + dVal*(xyz[3][0] - xyz[2][0]);
	//					//vertexArray[vertexCap][1] = xyz[2][1] + dVal*(xyz[3][1] - xyz[2][1]);
	//					//vertexArray[vertexCap][2] = xyz[2][2] + dVal*(xyz[3][2] - xyz[2][2]);
	//					vertexArray.Add(FVector(xyz[2][0] + dVal*(xyz[3][0] - xyz[2][0]), xyz[2][1] + dVal*(xyz[3][1] - xyz[2][1]), xyz[2][2] + dVal*(xyz[3][2] - xyz[2][2])));
	//					vertList[2] = vertexCap;
	//					vertexCap++;
	//				}
	//				if (edgeTable[cubeIndex] & 8) {
	//					dVal = (double)(iso_value - val[3]) / (double)(val[0] - val[3]);
	//					/*			vertexArray[vertexCap][0] = xyz[3][0] + dVal*(xyz[0][0] - xyz[3][0]);
	//					vertexArray[vertexCap][1] = xyz[3][1] + dVal*(xyz[0][1] - xyz[3][1]);
	//					vertexArray[vertexCap][2] = xyz[3][2] + dVal*(xyz[0][2] - xyz[3][2]);*/
	//					vertexArray.Add(FVector(xyz[3][0] + dVal*(xyz[0][0] - xyz[3][0]), xyz[3][1] + dVal*(xyz[0][1] - xyz[3][1]), xyz[3][2] + dVal*(xyz[0][2] - xyz[3][2])));
	//					vertList[3] = vertexCap;
	//					vertexCap++;
	//				}

	//				if (edgeTable[cubeIndex] & 32) {
	//					dVal = (double)(iso_value - val[5]) / (double)(val[6] - val[5]);
	//					//vertexArray[vertexCap][0] = xyz[5][0] + dVal*(xyz[6][0] - xyz[5][0]);
	//					//vertexArray[vertexCap][1] = xyz[5][1] + dVal*(xyz[6][1] - xyz[5][1]);
	//					//vertexArray[vertexCap][2] = xyz[5][2] + dVal*(xyz[6][2] - xyz[5][2]);
	//					vertexArray.Add(FVector(xyz[5][0] + dVal*(xyz[6][0] - xyz[5][0]), xyz[5][1] + dVal*(xyz[6][1] - xyz[5][1]), xyz[5][2] + dVal*(xyz[6][2] - xyz[5][2])));
	//					isoCache[layerIndex][y][z].vertexIndex[0] = vertList[5] = vertexCap;
	//					vertexCap++;
	//				}
	//				if (edgeTable[cubeIndex] & 64) {
	//					dVal = (double)(iso_value - val[6]) / (double)(val[7] - val[6]);
	//					//vertexArray[vertexCap][0] = xyz[6][0] + dVal*(xyz[7][0] - xyz[6][0]);
	//					//vertexArray[vertexCap][1] = xyz[6][1] + dVal*(xyz[7][1] - xyz[6][1]);
	//					//vertexArray[vertexCap][2] = xyz[6][2] + dVal*(xyz[7][2] - xyz[6][2]);
	//					vertexArray.Add(FVector(xyz[6][0] + dVal*(xyz[7][0] - xyz[6][0]), xyz[6][1] + dVal*(xyz[7][1] - xyz[6][1]), xyz[6][2] + dVal*(xyz[7][2] - xyz[6][2])));
	//					isoCache[layerIndex][y][z].vertexIndex[1] = vertList[6] = vertexCap;
	//					vertexCap++;
	//				}
	//				if (edgeTable[cubeIndex] & 1024) {
	//					dVal = (double)(iso_value - val[2]) / (double)(val[6] - val[2]);
	//					//vertexArray[vertexCap][0] = xyz[2][0] + dVal*(xyz[6][0] - xyz[2][0]);
	//					//vertexArray[vertexCap][1] = xyz[2][1] + dVal*(xyz[6][1] - xyz[2][1]);
	//					//vertexArray[vertexCap][2] = xyz[2][2] + dVal*(xyz[6][2] - xyz[2][2]);
	//					vertexArray.Add(FVector(xyz[2][0] + dVal*(xyz[6][0] - xyz[2][0]), xyz[2][1] + dVal*(xyz[6][1] - xyz[2][1]), xyz[2][2] + dVal*(xyz[6][2] - xyz[2][2])));
	//					isoCache[layerIndex][y][z].vertexIndex[2] = vertList[10] = vertexCap;
	//					vertexCap++;
	//				}

	//				// bind triangle indecies
	//				for (int i = 0; triTable[cubeIndex][i] != -1; i += 3) {
	//					/*triangleArray[triangleCap*3 + 2] = vertList[triTable[cubeIndex][i]];
	//					triangleArray[triangleCap*3 + 1] = vertList[triTable[cubeIndex][i + 1]];
	//					triangleArray[triangleCap*3] = vertList[triTable[cubeIndex][i + 2]];*/
	//					triangleArray.Add(vertList[triTable[cubeIndex][i]]);
	//					triangleArray.Add(vertList[triTable[cubeIndex][i + 1]]);
	//					triangleArray.Add(vertList[triTable[cubeIndex][i + 2]]);

	//					triangleCap++;
	//				}
	//			}
	//		}

	//		//create remaining rows of remaining layers ==============================================
	//		for (y = 1; y < res.Y - 1; y++) {

	//			//create first voxel of remaining rows of remaining layers
	//			z = 0;

	//			//create the first voxel of remaining rows of first layer=============================

	//			//inherit corner values from local variable -----------------------------------------

	//			//inherit corner values from isoCache -----------------------------------------------
	//			xyz[2][0] = isoCache[layerIndex][y - 1][z].cornerPoint[0];
	//			xyz[2][1] = isoCache[layerIndex][y - 1][z].cornerPoint[1];
	//			xyz[2][2] = isoCache[layerIndex][y - 1][z].cornerPoint[2];
	//			val[2] = data[x + 1][y][z + 1];
	//			cellIsoBool[2] = isoCache[layerIndex][y - 1][z].isoBool;

	//			xyz[3][0] = isoCache[(layerIndex + 1) % 2][y - 1][z].cornerPoint[0];
	//			xyz[3][1] = isoCache[(layerIndex + 1) % 2][y - 1][z].cornerPoint[1];
	//			xyz[3][2] = isoCache[(layerIndex + 1) % 2][y - 1][z].cornerPoint[2];
	//			val[3] = data[x][y][z + 1];
	//			cellIsoBool[3] = isoCache[(layerIndex + 1) % 2][y - 1][z].isoBool;

	//			xyz[7][0] = isoCache[(layerIndex + 1) % 2][y][z].cornerPoint[0];
	//			xyz[7][1] = isoCache[(layerIndex + 1) % 2][y][z].cornerPoint[1];
	//			xyz[7][2] = isoCache[(layerIndex + 1) % 2][y][z].cornerPoint[2];
	//			val[7] = data[x][y + 1][z + 1];
	//			cellIsoBool[7] = isoCache[(layerIndex + 1) % 2][y][z].isoBool;

	//			//calculate corner values that could not be inherited ---------------------------------
	//			xyz[0][0] = (float)((x - (res.X / 2)) / ((float)(res.X / 2)))*dim.X;
	//			xyz[0][1] = (float)((y - (res.Y / 2)) / ((float)(res.Y / 2)))*dim.Y;
	//			xyz[0][2] = (float)((z - (res.Z / 2)) / ((float)(res.Z / 2)))*dim.Z;
	//			val[0] = data[x][y][z];
	//			if (data[x][y][z] < iso_value)			// cubeIndex |= 1;
	//				cellIsoBool[0] = true;
	//			else
	//				cellIsoBool[0] = false;

	//			xyz[1][0] = (float)((x + 1 - (res.X / 2)) / ((float)(res.X / 2)))*dim.X;
	//			xyz[1][1] = (float)((y - (res.Y / 2)) / ((float)(res.Y / 2)))*dim.Y;
	//			xyz[1][2] = (float)((z - (res.Z / 2)) / ((float)(res.Z / 2)))*dim.Z;
	//			val[1] = data[x + 1][y][z];
	//			if (data[x + 1][y][z] < iso_value)		// cubeIndex |= 2;
	//				cellIsoBool[1] = true;
	//			else
	//				cellIsoBool[1] = false;

	//			xyz[4][0] = (float)((x - (res.X / 2)) / ((float)(res.X / 2)))*dim.X;
	//			xyz[4][1] = (float)((y + 1 - (res.Y / 2)) / ((float)(res.Y / 2)))*dim.Y;
	//			xyz[4][2] = (float)((z - (res.Z / 2)) / ((float)(res.Z / 2)))*dim.Z;
	//			val[4] = data[x][y + 1][z];
	//			if (data[x][y + 1][z] < iso_value)		//cubeIndex |= 16;
	//				cellIsoBool[4] = true;
	//			else
	//				cellIsoBool[4] = false;

	//			xyz[5][0] = (float)((x + 1 - (res.X / 2)) / ((float)(res.X / 2)))*dim.X;
	//			xyz[5][1] = (float)((y + 1 - (res.Y / 2)) / ((float)(res.Y / 2)))*dim.Y;
	//			xyz[5][2] = (float)((z - (res.Z / 2)) / ((float)(res.Z / 2)))*dim.Z;
	//			val[5] = data[x + 1][y + 1][z];
	//			if (data[x + 1][y + 1][z] < iso_value)	// cubeIndex |= 32;
	//				cellIsoBool[5] = true;
	//			else
	//				cellIsoBool[5] = false;

	//			//save the sixth corner values to isoCache
	//			xyz[6][0] = isoCache[layerIndex][y][z].cornerPoint[0] = (float)((x + 1 - (res.X / 2)) / ((float)(res.X / 2)))*dim.X;
	//			xyz[6][1] = isoCache[layerIndex][y][z].cornerPoint[1] = (float)((y + 1 - (res.Y / 2)) / ((float)(res.Y / 2)))*dim.Y;
	//			xyz[6][2] = isoCache[layerIndex][y][z].cornerPoint[2] = (float)((z + 1 - (res.Z / 2)) / ((float)(res.Z / 2)))*dim.Z;
	//			val[6] = data[x + 1][y + 1][z + 1];
	//			if (data[x + 1][y + 1][z + 1] < iso_value)// cubeIndex |= 64;
	//				cellIsoBool[6] = isoCache[layerIndex][y][z].isoBool = true;
	//			else
	//				cellIsoBool[6] = isoCache[layerIndex][y][z].isoBool = false;

	//			// get the case index
	//			cubeIndex = cellIsoBool[0] * 1 + cellIsoBool[1] * 2 + cellIsoBool[2] * 4 + cellIsoBool[3] * 8 +
	//				cellIsoBool[4] * 16 + cellIsoBool[5] * 32 + cellIsoBool[6] * 64 + cellIsoBool[7] * 128;

	//			// check if cube is entirely in or out of the surface ----------------------------
	//			if (edgeTable[cubeIndex] != 0) {

	//				// Find the vertices where the surface intersects the cube--------------------

	//				//inherit vertex indices from local variable ---------------------------------
	//				if (edgeTable[cubeIndex] & 1) {
	//					vertList[0] = z0Cache;
	//				}

	//				//inherit vertex indices from isoCache ---------------------------------------
	//				if (edgeTable[cubeIndex] & 2) {
	//					vertList[1] = isoCache[layerIndex][y - 1][z].vertexIndex[0];
	//				}
	//				if (edgeTable[cubeIndex] & 4) {
	//					vertList[2] = isoCache[layerIndex][y - 1][z].vertexIndex[1];
	//				}
	//				if (edgeTable[cubeIndex] & 8) {
	//					vertList[3] = isoCache[(layerIndex + 1) % 2][y - 1][z].vertexIndex[0];
	//				}
	//				if (edgeTable[cubeIndex] & 128) {
	//					vertList[7] = isoCache[(layerIndex + 1) % 2][y][z].vertexIndex[0];
	//				}
	//				if (edgeTable[cubeIndex] & 2048) {
	//					vertList[11] = isoCache[(layerIndex + 1) % 2][y][z].vertexIndex[2];
	//				}

	//				//calculate indices that could not be inherited ------------------------------


	//				if (edgeTable[cubeIndex] & 16) {
	//					dVal = (double)(iso_value - val[4]) / (double)(val[5] - val[4]);
	//					//vertexArray[vertexCap][0] = xyz[4][0] + dVal*(xyz[5][0] - xyz[4][0]);
	//					//vertexArray[vertexCap][1] = xyz[4][1] + dVal*(xyz[5][1] - xyz[4][1]);
	//					//vertexArray[vertexCap][2] = xyz[4][2] + dVal*(xyz[5][2] - xyz[4][2]);
	//					vertexArray.Add(FVector(xyz[4][0] + dVal*(xyz[5][0] - xyz[4][0]), xyz[4][1] + dVal*(xyz[5][1] - xyz[4][1]), xyz[4][2] + dVal*(xyz[5][2] - xyz[4][2])));
	//					vertList[4] = z0Cache = vertexCap;
	//					vertexCap++;
	//				}
	//				if (edgeTable[cubeIndex] & 32) {
	//					dVal = (double)(iso_value - val[5]) / (double)(val[6] - val[5]);
	//					//vertexArray[vertexCap][0] = xyz[5][0] + dVal*(xyz[6][0] - xyz[5][0]);
	//					//vertexArray[vertexCap][1] = xyz[5][1] + dVal*(xyz[6][1] - xyz[5][1]);
	//					//vertexArray[vertexCap][2] = xyz[5][2] + dVal*(xyz[6][2] - xyz[5][2]);
	//					vertexArray.Add(FVector(xyz[5][0] + dVal*(xyz[6][0] - xyz[5][0]), xyz[5][1] + dVal*(xyz[6][1] - xyz[5][1]), xyz[5][2] + dVal*(xyz[6][2] - xyz[5][2])));
	//					isoCache[layerIndex][y][z].vertexIndex[0] = vertList[5] = vertexCap;
	//					vertexCap++;
	//				}
	//				if (edgeTable[cubeIndex] & 64) {
	//					dVal = (double)(iso_value - val[6]) / (double)(val[7] - val[6]);
	//					//vertexArray[vertexCap][0] = xyz[6][0] + dVal*(xyz[7][0] - xyz[6][0]);
	//					//vertexArray[vertexCap][1] = xyz[6][1] + dVal*(xyz[7][1] - xyz[6][1]);
	//					//vertexArray[vertexCap][2] = xyz[6][2] + dVal*(xyz[7][2] - xyz[6][2]);
	//					vertexArray.Add(FVector(xyz[6][0] + dVal*(xyz[7][0] - xyz[6][0]), xyz[6][1] + dVal*(xyz[7][1] - xyz[6][1]), xyz[6][2] + dVal*(xyz[7][2] - xyz[6][2])));
	//					isoCache[layerIndex][y][z].vertexIndex[1] = vertList[6] = vertexCap;
	//					vertexCap++;
	//				}

	//				if (edgeTable[cubeIndex] & 256) {
	//					dVal = (double)(iso_value - val[0]) / (double)(val[4] - val[0]);
	//					//vertexArray[vertexCap][0] = xyz[0][0] + dVal*(xyz[4][0] - xyz[0][0]);
	//					//vertexArray[vertexCap][1] = xyz[0][1] + dVal*(xyz[4][1] - xyz[0][1]);
	//					//vertexArray[vertexCap][2] = xyz[0][2] + dVal*(xyz[4][2] - xyz[0][2]);
	//					vertexArray.Add(FVector(xyz[0][0] + dVal*(xyz[4][0] - xyz[0][0]), xyz[0][1] + dVal*(xyz[4][1] - xyz[0][1]), xyz[0][2] + dVal*(xyz[4][2] - xyz[0][2])));
	//					vertList[8] = vertexCap;
	//					vertexCap++;
	//				}
	//				if (edgeTable[cubeIndex] & 512) {
	//					dVal = (double)(iso_value - val[1]) / (double)(val[5] - val[1]);
	//					//vertexArray[vertexCap][0] = xyz[1][0] + dVal*(xyz[5][0] - xyz[1][0]);
	//					//vertexArray[vertexCap][1] = xyz[1][1] + dVal*(xyz[5][1] - xyz[1][1]);
	//					//vertexArray[vertexCap][2] = xyz[1][2] + dVal*(xyz[5][2] - xyz[1][2]);
	//					vertexArray.Add(FVector(xyz[1][0] + dVal*(xyz[5][0] - xyz[1][0]), xyz[1][1] + dVal*(xyz[5][1] - xyz[1][1]), xyz[1][2] + dVal*(xyz[5][2] - xyz[1][2])));
	//					vertList[9] = vertexCap;
	//					vertexCap++;
	//				}
	//				if (edgeTable[cubeIndex] & 1024) {
	//					dVal = (double)(iso_value - val[2]) / (double)(val[6] - val[2]);
	//					//vertexArray[vertexCap][0] = xyz[2][0] + dVal*(xyz[6][0] - xyz[2][0]);
	//					//vertexArray[vertexCap][1] = xyz[2][1] + dVal*(xyz[6][1] - xyz[2][1]);
	//					//vertexArray[vertexCap][2] = xyz[2][2] + dVal*(xyz[6][2] - xyz[2][2]);
	//					vertexArray.Add(FVector(xyz[2][0] + dVal*(xyz[6][0] - xyz[2][0]), xyz[2][1] + dVal*(xyz[6][1] - xyz[2][1]), xyz[2][2] + dVal*(xyz[6][2] - xyz[2][2])));
	//					isoCache[layerIndex][y][z].vertexIndex[2] = vertList[10] = vertexCap;
	//					vertexCap++;
	//				}

	//				// bind triangle indecies
	//				for (int i = 0; triTable[cubeIndex][i] != -1; i += 3) {
	//					/*triangleArray[triangleCap*3 + 2] = vertList[triTable[cubeIndex][i]];
	//					triangleArray[triangleCap*3 + 1] = vertList[triTable[cubeIndex][i + 1]];
	//					triangleArray[triangleCap*3] = vertList[triTable[cubeIndex][i + 2]];*/
	//					triangleArray.Add(vertList[triTable[cubeIndex][i]]);
	//					triangleArray.Add(vertList[triTable[cubeIndex][i + 1]]);
	//					triangleArray.Add(vertList[triTable[cubeIndex][i + 2]]);

	//					triangleCap++;
	//				}
	//			}

	//			//create remaining voxels of remaining rows of remaining layers ==================
	//			for (z = 1; z < res.Z - 1; z++) {
	//				//inherit corner values from local variable -----------------------------------

	//				//inherit corner values from isoCache -----------------------------------------
	//				xyz[0][0] = isoCache[(layerIndex + 1) % 2][y - 1][z - 1].cornerPoint[0];
	//				xyz[0][1] = isoCache[(layerIndex + 1) % 2][y - 1][z - 1].cornerPoint[1];
	//				xyz[0][2] = isoCache[(layerIndex + 1) % 2][y - 1][z - 1].cornerPoint[2];
	//				val[0] = data[x][y][z];
	//				cellIsoBool[0] = isoCache[(layerIndex + 1) % 2][y - 1][z - 1].isoBool;

	//				xyz[1][0] = isoCache[layerIndex][y - 1][z - 1].cornerPoint[0];
	//				xyz[1][1] = isoCache[layerIndex][y - 1][z - 1].cornerPoint[1];
	//				xyz[1][2] = isoCache[layerIndex][y - 1][z - 1].cornerPoint[2];
	//				val[1] = data[x + 1][y][z];
	//				cellIsoBool[1] = isoCache[layerIndex][y - 1][z - 1].isoBool;

	//				xyz[2][0] = isoCache[layerIndex][y - 1][z].cornerPoint[0];
	//				xyz[2][1] = isoCache[layerIndex][y - 1][z].cornerPoint[1];
	//				xyz[2][2] = isoCache[layerIndex][y - 1][z].cornerPoint[2];
	//				val[2] = data[x + 1][y][z + 1];
	//				cellIsoBool[2] = isoCache[layerIndex][y - 1][z].isoBool;

	//				xyz[3][0] = isoCache[(layerIndex + 1) % 2][y - 1][z].cornerPoint[0];
	//				xyz[3][1] = isoCache[(layerIndex + 1) % 2][y - 1][z].cornerPoint[1];
	//				xyz[3][2] = isoCache[(layerIndex + 1) % 2][y - 1][z].cornerPoint[2];
	//				val[3] = data[x][y][z + 1];
	//				cellIsoBool[3] = isoCache[(layerIndex + 1) % 2][y - 1][z].isoBool;

	//				xyz[4][0] = isoCache[(layerIndex + 1) % 2][y][z - 1].cornerPoint[0];
	//				xyz[4][1] = isoCache[(layerIndex + 1) % 2][y][z - 1].cornerPoint[1];
	//				xyz[4][2] = isoCache[(layerIndex + 1) % 2][y][z - 1].cornerPoint[2];
	//				val[4] = data[x][y + 1][z];
	//				cellIsoBool[4] = isoCache[(layerIndex + 1) % 2][y][z - 1].isoBool;

	//				xyz[5][0] = isoCache[layerIndex][y][z - 1].cornerPoint[0];
	//				xyz[5][1] = isoCache[layerIndex][y][z - 1].cornerPoint[1];
	//				xyz[5][2] = isoCache[layerIndex][y][z - 1].cornerPoint[2];
	//				val[5] = data[x + 1][y + 1][z];
	//				cellIsoBool[5] = isoCache[layerIndex][y][z - 1].isoBool;

	//				xyz[7][0] = isoCache[(layerIndex + 1) % 2][y][z].cornerPoint[0];
	//				xyz[7][1] = isoCache[(layerIndex + 1) % 2][y][z].cornerPoint[1];
	//				xyz[7][2] = isoCache[(layerIndex + 1) % 2][y][z].cornerPoint[2];
	//				val[7] = data[x][y + 1][z + 1];
	//				cellIsoBool[7] = isoCache[(layerIndex + 1) % 2][y][z].isoBool;

	//				//calculate corner values that could not be inherited -------------------------------------------------
	//				//save the sixth corner values to isoCache
	//				xyz[6][0] = isoCache[layerIndex][y][z].cornerPoint[0] = (float)((x + 1 - (res.X / 2)) / ((float)(res.X / 2)))*dim.X;
	//				xyz[6][1] = isoCache[layerIndex][y][z].cornerPoint[1] = (float)((y + 1 - (res.Y / 2)) / ((float)(res.Y / 2)))*dim.Y;
	//				xyz[6][2] = isoCache[layerIndex][y][z].cornerPoint[2] = (float)((z + 1 - (res.Z / 2)) / ((float)(res.Z / 2)))*dim.Z;
	//				val[6] = data[x + 1][y + 1][z + 1];
	//				if (data[x + 1][y + 1][z + 1] < iso_value)// cubeIndex |= 64;
	//					cellIsoBool[6] = isoCache[layerIndex][y][z].isoBool = true;
	//				else
	//					cellIsoBool[6] = isoCache[layerIndex][y][z].isoBool = false;

	//				// get the case index
	//				cubeIndex = cellIsoBool[0] * 1 + cellIsoBool[1] * 2 + cellIsoBool[2] * 4 + cellIsoBool[3] * 8 +
	//					cellIsoBool[4] * 16 + cellIsoBool[5] * 32 + cellIsoBool[6] * 64 + cellIsoBool[7] * 128;

	//				// check if cube is entirely in or out of the surface ----------------------------
	//				if (edgeTable[cubeIndex] != 0) {

	//					// Find the vertices where the surface intersects the cube--------------------

	//					//inherit vertex indices from local variable ---------------------------------


	//					//inherit vertex indices from isoCache ---------------------------------------
	//					if (edgeTable[cubeIndex] & 1) {
	//						vertList[0] = isoCache[layerIndex][y - 1][z - 1].vertexIndex[1];
	//					}
	//					if (edgeTable[cubeIndex] & 2) {
	//						vertList[1] = isoCache[layerIndex][y - 1][z].vertexIndex[0];
	//					}
	//					if (edgeTable[cubeIndex] & 4) {
	//						vertList[2] = isoCache[layerIndex][y - 1][z].vertexIndex[1];
	//					}
	//					if (edgeTable[cubeIndex] & 8) {
	//						vertList[3] = isoCache[(layerIndex + 1) % 2][y - 1][z].vertexIndex[0];
	//					}
	//					if (edgeTable[cubeIndex] & 16) {
	//						vertList[4] = isoCache[layerIndex][y][z - 1].vertexIndex[1];
	//					}
	//					if (edgeTable[cubeIndex] & 128) {
	//						vertList[7] = isoCache[(layerIndex + 1) % 2][y][z].vertexIndex[0];
	//					}
	//					if (edgeTable[cubeIndex] & 256) {
	//						vertList[8] = isoCache[(layerIndex + 1) % 2][y][z - 1].vertexIndex[2];
	//					}
	//					if (edgeTable[cubeIndex] & 512) {
	//						vertList[9] = isoCache[layerIndex][y][z - 1].vertexIndex[2];
	//					}
	//					if (edgeTable[cubeIndex] & 2048) {
	//						vertList[11] = isoCache[(layerIndex + 1) % 2][y][z].vertexIndex[2];
	//					}

	//					//calculate indices that could not be inherited ------------------------------			


	//					if (edgeTable[cubeIndex] & 32) {
	//						dVal = (double)(iso_value - val[5]) / (double)(val[6] - val[5]);
	//						//vertexArray[vertexCap][0] = xyz[5][0] + dVal*(xyz[6][0] - xyz[5][0]);
	//						//vertexArray[vertexCap][1] = xyz[5][1] + dVal*(xyz[6][1] - xyz[5][1]);
	//						//vertexArray[vertexCap][2] = xyz[5][2] + dVal*(xyz[6][2] - xyz[5][2]);
	//						vertexArray.Add(FVector(xyz[5][0] + dVal*(xyz[6][0] - xyz[5][0]), xyz[5][1] + dVal*(xyz[6][1] - xyz[5][1]), xyz[5][2] + dVal*(xyz[6][2] - xyz[5][2])));
	//						isoCache[layerIndex][y][z].vertexIndex[0] = vertList[5] = vertexCap;
	//						vertexCap++;
	//					}
	//					if (edgeTable[cubeIndex] & 64) {
	//						dVal = (double)(iso_value - val[6]) / (double)(val[7] - val[6]);
	//						/*				vertexArray[vertexCap][0] = xyz[6][0] + dVal*(xyz[7][0] - xyz[6][0]);
	//						vertexArray[vertexCap][1] = xyz[6][1] + dVal*(xyz[7][1] - xyz[6][1]);
	//						vertexArray[vertexCap][2] = xyz[6][2] + dVal*(xyz[7][2] - xyz[6][2]);*/
	//						vertexArray.Add(FVector(xyz[6][0] + dVal*(xyz[7][0] - xyz[6][0]), xyz[6][1] + dVal*(xyz[7][1] - xyz[6][1]), xyz[6][2] + dVal*(xyz[7][2] - xyz[6][2])));
	//						isoCache[layerIndex][y][z].vertexIndex[1] = vertList[6] = vertexCap;
	//						vertexCap++;
	//					}
	//					if (edgeTable[cubeIndex] & 1024) {
	//						dVal = (double)(iso_value - val[2]) / (double)(val[6] - val[2]);
	//						//vertexArray[vertexCap][0] = xyz[2][0] + dVal*(xyz[6][0] - xyz[2][0]);
	//						//vertexArray[vertexCap][1] = xyz[2][1] + dVal*(xyz[6][1] - xyz[2][1]);
	//						//vertexArray[vertexCap][2] = xyz[2][2] + dVal*(xyz[6][2] - xyz[2][2]);
	//						vertexArray.Add(FVector(xyz[2][0] + dVal*(xyz[6][0] - xyz[2][0]), xyz[2][1] + dVal*(xyz[6][1] - xyz[2][1]), xyz[2][2] + dVal*(xyz[6][2] - xyz[2][2])));
	//						isoCache[layerIndex][y][z].vertexIndex[2] = vertList[10] = vertexCap;
	//						vertexCap++;
	//					}

	//					// bind triangle indecies
	//					for (int i = 0; triTable[cubeIndex][i] != -1; i += 3) {
	//						/*triangleArray[triangleCap*3 + 2] = vertList[triTable[cubeIndex][i]];
	//						triangleArray[triangleCap*3 + 1] = vertList[triTable[cubeIndex][i + 1]];
	//						triangleArray[triangleCap*3] = vertList[triTable[cubeIndex][i + 2]];*/
	//						triangleArray.Add(vertList[triTable[cubeIndex][i]]);
	//						triangleArray.Add(vertList[triTable[cubeIndex][i + 1]]);
	//						triangleArray.Add(vertList[triTable[cubeIndex][i + 2]]);

	//						triangleCap++;
	//					}
	//				}
	//			}
	//		}
	//		layerIndex = (layerIndex + 1) % 2;
	//	}


	//	auto UV0 = TArray<FVector2D>();
	//	auto Colors = TArray<FColor>();
	//	for (int i = 0; i < vertexArray.Num(); i++)
	//		Colors.Add(FColor(1.0f, 1.0f, 1.0f));


	//	calcNormals();

	//	_mesh->CreateMeshSection(0, vertexArray, triangleArray, normalArray, UV0, Colors, tangentArray, true);
	//	//debugpoint
	//	UE_LOG(LogTemp, Warning, TEXT("finished..."));
	//	UE_LOG(LogTemp, Warning, TEXT("tris %f"), triangleArray.Num());
	//	UE_LOG(LogTemp, Warning, TEXT("verts %f"), vertexArray.Num());

	//	UE_LOG(LogTemp, Warning, TEXT("edgeTable %f"), edgeTable[130]);
	//	UE_LOG(LogTemp, Warning, TEXT("triTable %f"), triTable[0][0]);
	//}
}