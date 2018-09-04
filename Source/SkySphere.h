#pragma once

#include "Decl.h"
#include "Geometry.h"

class SkySphere {
public:
	void loadHDRFile(const std::string &filename);
	Vector3 getRadiance(Vector3 d);
private:
	int width, height;
	std::vector<Vector3> data;
};