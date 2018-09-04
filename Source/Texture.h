#pragma once

class Texture {
public:
	Texture( const std::string &filename );
	int getWidth() const { return width; }
	int getHeight() const { return height; }
	Vector3 getTexel(const Vector2 &uv) const;
private:
	void loadFile( const std::string &filename );

	int bpp;
	int width, height;
	std::vector<Vector3> data;

};