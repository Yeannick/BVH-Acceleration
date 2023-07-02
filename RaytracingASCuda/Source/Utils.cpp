#include "Utils.h"
bool Utils::ParseObj(const std::string& filename, Triangle* _triangles, int& _triangleAmount, bool FlipWinding)
{
	{
		std::ifstream file(filename);
		if (!file)
			return false;

		std::vector<FVector3> positions;
		std::vector<FVector3> normals;
		std::vector<int> indices;

		std::string sCommand;
		// start a while iteration ending when the end of file is reached (ios::eof)
		while (!file.eof())
		{
			//read the first word of the string, use the >> operator (istream::operator>>) 
			file >> sCommand;
			//use conditional statements to process the different commands	
			if (sCommand == "#")
			{
				// Ignore Comment
			}
			else if (sCommand == "v")
			{
				//Vertex
				float x, y, z;
				file >> x >> y >> z;

				if (FlipWinding) positions.emplace_back(x, y, -z);
				else  positions.emplace_back(x, y, z);
			}
			else if (sCommand == "f")
			{
				float i0, i1, i2;
				file >> i0 >> i1 >> i2;

				indices.push_back((int)i0 - 1);
				if (FlipWinding)
				{
					indices.push_back((int)i2 - 1);
					indices.push_back((int)i1 - 1);
				}
				else
				{
					indices.push_back((int)i1 - 1);
					indices.push_back((int)i2 - 1);
				}
			}
			//read till end of line and ignore all remaining chars
			file.ignore(1000, '\n');

			if (file.eof())
				break;
		}

		//Precompute normals
		std::vector<Triangle> t;
		for (uint64_t index = 0; index < indices.size(); index += 3)
		{
			uint32_t i0 = indices[index];
			uint32_t i1 = indices[index + 1];
			uint32_t i2 = indices[index + 2];

			FVector3 edgeV0V1 = positions[i1] - positions[i0];
			FVector3 edgeV0V2 = positions[i2] - positions[i0];
			FVector3 normal = FVector3::Cross(edgeV0V1, edgeV0V2);

			if (isnan(normal.x))
			{
				int k = 0;
			}

			normal.Normalize();
			if (isnan(normal.x))
			{
				int k = 0;
			}

			normals.push_back(normal);
			Triangle temp{};
			temp.v0 = positions[indices[index]];
			temp.v1 = positions[indices[index + 1]];
			temp.v2 = positions[indices[index + 2]];
			temp.normal = normal;
			t.push_back(temp);


		}
		for (size_t i = 0; i < t.size(); i++)
		{
			_triangles[i] = t[i];
		}
		_triangleAmount = static_cast<int>(indices.size());



		return true;
	}
}

bool Utils::ParseObj(const std::string& filename, std::vector<FVector3>& positions, std::vector<FVector3>& normals, std::vector<int>& indices,bool FlipWinding)
{
	{
		std::ifstream file(filename);
		if (!file)
		{
			std::cout << "NO FILE FOUND \n";
			return false;
		}
			

		/*std::vector<FVector3> positions;
		std::vector<FVector3> normals;
		std::vector<int> indices;*/

		std::string sCommand;
		// start a while iteration ending when the end of file is reached (ios::eof)
		while (!file.eof())
		{
			//read the first word of the string, use the >> operator (istream::operator>>) 
			file >> sCommand;
			//use conditional statements to process the different commands	
			if (sCommand == "#")
			{
				// Ignore Comment
			}
			else if (sCommand == "v")
			{
				//Vertex
				float x, y, z;
				file >> x >> y >> z;

				if (FlipWinding) positions.emplace_back(x, y, -z);
				else  positions.emplace_back(x, y, z);
			}
			else if (sCommand == "f")
			{
				float i0, i1, i2;
				file >> i0 >> i1 >> i2;

				indices.push_back((int)i0 - 1);
				if (FlipWinding)
				{
					indices.push_back((int)i2 - 1);
					indices.push_back((int)i1 - 1);
				}
				else
				{
					indices.push_back((int)i1 - 1);
					indices.push_back((int)i2 - 1);
				}
			}
			//read till end of line and ignore all remaining chars
			file.ignore(1000, '\n');

			if (file.eof())
				break;
		}

		//Precompute normals
		//std::vector<Triangle> t;
		for (uint64_t index = 0; index < indices.size(); index += 3)
		{
			uint32_t i0 = indices[index];
			uint32_t i1 = indices[index + 1];
			uint32_t i2 = indices[index + 2];

			FVector3 edgeV0V1 = positions[i1] - positions[i0];
			FVector3 edgeV0V2 = positions[i2] - positions[i0];
			FVector3 normal = FVector3::Cross(edgeV0V1, edgeV0V2);

			if (isnan(normal.x))
			{
				int k = 0;
			}

			normal.Normalize();
			if (isnan(normal.x))
			{
				int k = 0;
			}

			normals.push_back(normal);
		/*	Triangle temp{};
			temp.v0 = positions[indices[index]];
			temp.v1 = positions[indices[index + 1]];
			temp.v2 = positions[indices[index + 2]];
			temp.normal = normal;
			t.push_back(temp);*/


		}
		/*for (size_t i = 0; i < t.size(); i++)
		{
			_triangles[i] = t[i];
		}
		_triangleAmount = static_cast<int>(indices.size());*/



		return true;
	}
}