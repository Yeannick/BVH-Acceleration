#pragma once
#include "PrimitiveTypes.h"
#include <iostream>
#include <fstream>
#include <vector>
namespace Utils
{
	bool ParseObj(const std::string& filename, Triangle* _triangles, int& _triangleAmount, bool FlipWinding = false);
	bool ParseObj(const std::string& filename, std::vector<FVector3>& positions, std::vector<FVector3>& normals, std::vector<int>& indices, bool FlipWinding = false);
	
	
}
