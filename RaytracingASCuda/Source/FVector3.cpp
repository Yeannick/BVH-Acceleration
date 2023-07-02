#include "FVector3.h"

const FVector3 FVector3::UnitX = FVector3{ 1, 0, 0 };
const FVector3 FVector3::UnitY = FVector3{ 0, 1, 0 };
const FVector3 FVector3::UnitZ = FVector3{ 0, 0, 1 };
const FVector3 FVector3::Zero = FVector3{ 0, 0, 0 };

//FVector3::FVector3(const FVector3& from, const FVector3& to) : x(to.x - from.x), y(to.y - from.y), z(to.z - from.z)
//{
//}

FVector3::FVector3(const FVector4& v) : x(v.x), y(v.y), z(v.z) 
{
}



__device__ __host__ FVector3 FVector3::Reflect(const FVector3& v1, const FVector3& v2)
{
	 return v1 - (2.f * FVector3::Dot(v1, v2) * v2);
}
