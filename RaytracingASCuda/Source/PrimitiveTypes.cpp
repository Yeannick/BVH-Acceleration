#include "PrimitiveTypes.h"
#include "Utils.h"


TriangleMesh::TriangleMesh(const std::string& filename)
{
	/*Triangle* t = new Triangle();
	Utils::ParseObj(filename, t, triangleCount);
	
	triangles = (Triangle*)malloc(sizeof(Triangle) * triangleCount);
	triangles = t;*/

	Utils::ParseObj(filename, positions, normals, indices,true);
	triangleCount = indices.size() / 3;
}

TriangleMesh::TriangleMesh()
{
	
	positions.push_back( {-5,0,0 });
	positions.push_back( {0,0,-5 });
	positions.push_back( {0,0,0  });
	positions.push_back( {-5,0,-5});
	triangleCount = 2;
	indices.push_back(1);
	indices.push_back(0);
	indices.push_back(2);
	indices.push_back(3);
	indices.push_back(0);
	indices.push_back(1);
	normals.clear();
	if (normals.capacity() < indices.size() / 3)
		normals.reserve(indices.size() / 3);

	for (size_t i{ 0 }; i < indices.size(); ++i)
	{
		FVector3& v0 = positions[indices[i]];
		FVector3& v1 = positions[indices[++i]];
		FVector3& v2 = positions[indices[++i]];

		const FVector3 edgeV0V1 = v1 - v0;
		const FVector3 edgeV0V2 = v2 - v0;
		const FVector3 normal = FVector3::Cross(edgeV0V1, edgeV0V2).Normalized();

		normals.emplace_back(normal);
	}



}




