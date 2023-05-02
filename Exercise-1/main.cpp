#include <iostream>
#include <fstream>
#include <array>
#include <vector>

#include "Eigen.h"
#include "VirtualSensor.h"

struct Vertex
{
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	// position stored as 4 floats (4th component is supposed to be 1.0)
	Vector4f position;
	// color stored as 4 unsigned char
	Vector4uc color;
};

struct Triangle {
	unsigned int idx0;
	unsigned int idx1;
	unsigned int idx2;

	Triangle() : idx0{ 0 }, idx1{ 0 }, idx2{ 0 } {}

	Triangle(unsigned int _idx0, unsigned int _idx1, unsigned int _idx2) :
		idx0(_idx0), idx1(_idx1), idx2(_idx2) {}
};

bool WriteMesh(Vertex* vertices, unsigned int width, unsigned int height, const std::string& filename)
{
	float edgeThreshold = 0.01f; // 1cm

	// TODO 2: use the OFF file format to save the vertices grid (http://www.geomview.org/docs/html/OFF.html)
	// - have a look at the "off_sample.off" file to see how to store the vertices and triangles
	// - for debugging we recommend to first only write out the vertices (set the number of faces to zero)
	// - for simplicity write every vertex to file, even if it is not valid (position.x() == MINF) (note that all vertices in the off file have to be valid, thus, if a point is not valid write out a dummy point like (0,0,0))
	// - use a simple triangulation exploiting the grid structure (neighboring vertices build a triangle, two triangles per grid cell)
	// - you can use an arbitrary triangulation of the cells, but make sure that the triangles are consistently oriented
	// - only write triangles with valid vertices and an edge length smaller then edgeThreshold

	// TODO: Get number of vertices
	unsigned int nVertices = width*height;

	// TODO: Determine number of valid faces
	std::vector<Triangle> triangles;
	for (unsigned int i = 0; i < height - 1; i++) {
		for (unsigned int j = 0; j < width - 1; j++) {
			unsigned int i0 = i*width + j;
			unsigned int i1 = (i + 1)*width + j;
			unsigned int i2 = i*width + j + 1;
			unsigned int i3 = (i + 1)*width + j + 1;

			bool valid0 = vertices[i0].position.allFinite();
			bool valid1 = vertices[i1].position.allFinite();
			bool valid2 = vertices[i2].position.allFinite();
			bool valid3 = vertices[i3].position.allFinite();

			if (valid0 && valid1 && valid2) {
				float d0 = (vertices[i0].position - vertices[i1].position).norm();
				float d1 = (vertices[i0].position - vertices[i2].position).norm();
				float d2 = (vertices[i1].position - vertices[i2].position).norm();
				if (edgeThreshold > d0 && edgeThreshold > d1 && edgeThreshold > d2){
					Triangle triangle(i0, i1, i2);
					triangles.push_back(triangle);
				}
			}
			if (valid1 && valid2 && valid3) {
				float d0 = (vertices[i3].position - vertices[i1].position).norm();
				float d1 = (vertices[i3].position - vertices[i2].position).norm();
				float d2 = (vertices[i1].position - vertices[i2].position).norm();
				if (edgeThreshold > d0 && edgeThreshold > d1 && edgeThreshold > d2){
					Triangle triangle(i1, i3, i2);
					triangles.push_back(triangle);
				}
			}
		}
	}
	unsigned nFaces = triangles.size();

	// Write off file
	std::ofstream outFile(filename);
	if (!outFile.is_open()) return false;

	// write header
	outFile << "COFF" << std::endl;

	outFile << "# numVertices numFaces numEdges" << std::endl;
	outFile << nVertices << " " << nFaces << " 0" << std::endl;

	// TODO: save vertices
	for (unsigned int i = 0; i < nVertices; ++i)
	{
		const auto& vertex = vertices[i];
		if (vertex.position.allFinite())
			outFile << vertex.position.x() << " " << vertex.position.y() << " " << vertex.position.z() << " "
			<< int(vertex.color.x()) << " " << int(vertex.color.y()) << " " << int(vertex.color.z()) << " " << int(vertex.color.w()) << std::endl;
		else
			outFile << "0.0 0.0 0.0 0 0 0 0" << std::endl;
	}

	// TODO: save valid faces
	outFile << "# list of faces" << std::endl;
	outFile << "# nVerticesPerFace idx0 idx1 idx2 ..." << std::endl;

	for (unsigned int i = 0; i < triangles.size(); i++) {
		outFile << "3 " << triangles[i].idx0 << " " << triangles[i].idx1 << " " << triangles[i].idx2 << std::endl;
	}
	
	// close file
	outFile.close();

	return true;
}

int main()
{
	// Make sure this path points to the data folder
	std::string filenameIn = "../../Data/rgbd_dataset_freiburg1_xyz/";
	std::string filenameBaseOut = "mesh_";

	// load video
	std::cout << "Initialize virtual sensor..." << std::endl;
	VirtualSensor sensor;
	if (!sensor.Init(filenameIn))
	{
		std::cout << "Failed to initialize the sensor!\nCheck file path!" << std::endl;
		return -1;
	}

	// convert video to meshes
	while (sensor.ProcessNextFrame())
	{
		// get ptr to the current depth frame
		// depth is stored in row major (get dimensions via sensor.GetDepthImageWidth() / GetDepthImageHeight())
		float* depthMap = sensor.GetDepth();
		// get ptr to the current color frame
		// color is stored as RGBX in row major (4 byte values per pixel, get dimensions via sensor.GetColorImageWidth() / GetColorImageHeight())
		BYTE* colorMap = sensor.GetColorRGBX();

		// get depth intrinsics
		Matrix3f depthIntrinsics = sensor.GetDepthIntrinsics();
		Matrix3f depthIntrinsicsInv = depthIntrinsics.inverse();

		float fX = depthIntrinsics(0, 0);
		float fY = depthIntrinsics(1, 1);
		float cX = depthIntrinsics(0, 2);
		float cY = depthIntrinsics(1, 2);

		// compute inverse depth extrinsics
		Matrix4f depthExtrinsicsInv = sensor.GetDepthExtrinsics().inverse();

		Matrix4f trajectory = sensor.GetTrajectory();
		Matrix4f trajectoryInv = sensor.GetTrajectory().inverse();
		
		// TODO 1: back-projection
		// write result to the vertices array below, keep pixel ordering!
		// if the depth value at idx is invalid (MINF) write the following values to the vertices array
		// vertices[idx].position = Vector4f(MINF, MINF, MINF, MINF);
		// vertices[idx].color = Vector4uc(0,0,0,0);
		// otherwise apply back-projection and transform the vertex to world space, use the corresponding color from the colormap
		
		Vertex* vertices = new Vertex[sensor.GetDepthImageWidth() * sensor.GetDepthImageHeight()];
		for (unsigned int v = 0; v < sensor.GetDepthImageHeight(); ++v){
			for (unsigned int u = 0; u < sensor.GetDepthImageWidth(); ++u){
				unsigned int idx = (v * sensor.GetDepthImageWidth()) + u;

				// std::cout << depthMap[idx] << std::endl;
			
				if(depthMap[idx] == MINF){
					vertices[idx].position = Vector4f(MINF, MINF, MINF, MINF);
					vertices[idx].color = Vector4uc(0,0,0,0);
				} 
				else{
					float Z = depthMap[idx];
					float X = (u - cX) * Z / fX;
					float Y = (v - cY) * Z / fY;

					vertices[idx].position = trajectoryInv * depthExtrinsicsInv * Vector4f(X, Y, Z, 1.0f);
					
					Vector3f proj = sensor.GetColorIntrinsics() * (sensor.GetColorExtrinsics() * trajectory * vertices[idx].position).block<3, 1>(0, 0);
					proj /= proj.z();
					unsigned int uCol = (unsigned int)std::floor(proj.x());
					unsigned int vCol = (unsigned int)std::floor(proj.y());
					if (uCol >= sensor.GetColorImageWidth()) uCol = sensor.GetColorImageWidth() - 1;
					if (vCol >= sensor.GetColorImageHeight()) vCol = sensor.GetColorImageHeight() - 1;
					unsigned int idxCol = vCol*sensor.GetColorImageWidth() + uCol;

					vertices[idx].color = Vector4uc(colorMap[4 * idxCol + 0], colorMap[4 * idxCol + 1], colorMap[4 * idxCol + 2], colorMap[4 * idxCol + 3]);
				}
			}
		}

		
		// write mesh file
		std::stringstream ss;
		ss << filenameBaseOut << sensor.GetCurrentFrameCnt() << ".off";
		if (!WriteMesh(vertices, sensor.GetDepthImageWidth(), sensor.GetDepthImageHeight(), ss.str()))
		{
			std::cout << "Failed to write mesh!\nCheck file path!" << std::endl;
			return -1;
		}

		// free mem
		delete[] vertices;
	}

	return 0;
}