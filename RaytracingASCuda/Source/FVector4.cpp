#include "FVector4.h"

FVector4::FVector4(const FVector3& v, float _w) :
	x(v.x), y(v.y), z(v.z), w(_w)
{
}
