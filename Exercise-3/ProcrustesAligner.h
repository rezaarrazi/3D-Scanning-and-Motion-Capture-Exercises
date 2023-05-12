#pragma once
#include "SimpleMesh.h"
#include <iostream>
class ProcrustesAligner {
public:
	Matrix4f estimatePose(const std::vector<Vector3f>& sourcePoints, const std::vector<Vector3f>& targetPoints) {
		ASSERT(sourcePoints.size() == targetPoints.size() && "The number of source and target points should be the same, since every source point is matched with corresponding target point.");

		// We estimate the pose between source and target points using Procrustes algorithm.
		// Our shapes have the same scale, therefore we don't estimate scale. We estimated rotation and translation
		// from source points to target points.

		auto sourceMean = computeMean(sourcePoints);
		auto targetMean = computeMean(targetPoints);
		
		Matrix3f rotation = estimateRotation(sourcePoints, sourceMean, targetPoints, targetMean);
		Vector3f translation = computeTranslation(sourceMean, targetMean, rotation);

		// TODO: Compute the transformation matrix by using the computed rotation and translation.
		// You can access parts of the matrix with .block(start_row, start_col, num_rows, num_cols) = elements
		
		Matrix4f estimatedPose = Matrix4f::Identity();
		estimatedPose.block<3,3>(0,0) = rotation;
		estimatedPose.block<3,1>(0,3) = translation;

		return estimatedPose;
	}

private:
	Vector3f computeMean(const std::vector<Vector3f>& points) {
		// TODO: Compute the mean of input points.
		// Hint: You can use the .size() method to get the length of a vector.

		Vector3f mean = Vector3f::Zero();
		float sum_of_elems_x = 0.0;
		float sum_of_elems_y = 0.0;
		float sum_of_elems_z = 0.0;

		for (unsigned int i = 0; i < points.size(); i++){
			sum_of_elems_x += points[i].x();
			sum_of_elems_y += points[i].y();
			sum_of_elems_z += points[i].z();
		}

		mean.x() = sum_of_elems_x / points.size();
		mean.y() = sum_of_elems_y / points.size();
		mean.z() = sum_of_elems_z / points.size();

		return mean;
	}

	Matrix3f estimateRotation(const std::vector<Vector3f>& sourcePoints, const Vector3f& sourceMean, const std::vector<Vector3f>& targetPoints, const Vector3f& targetMean) {
		// TODO: Estimate the rotation from source to target points, following the Procrustes algorithm.
		// To compute the singular value decomposition you can use JacobiSVD() from Eigen.
		// Hint: You can initialize an Eigen matrix with "MatrixXf m(num_rows,num_cols);" and access/modify parts of it using the .block() method (see above).

		MatrixXf sourceMatrix(sourcePoints.size(), 3);
		MatrixXf targetMatrix(targetPoints.size(), 3);

		// Fill the matrices and subtract the means
		for (size_t i = 0; i < sourcePoints.size(); ++i) {
			sourceMatrix.row(i) = sourcePoints[i] - sourceMean;
			targetMatrix.row(i) = targetPoints[i] - targetMean;
		}

		// Compute the cross-covariance matrix
		MatrixXf crossCovarianceMatrix = targetMatrix.transpose() * sourceMatrix;

		// Compute the singular value decomposition of the cross-covariance matrix
		JacobiSVD<MatrixXf> svd(crossCovarianceMatrix, ComputeThinU | ComputeThinV);

		// Compute the rotation matrix
		Matrix3f rotation = svd.matrixU() * svd.matrixV().transpose();

		// Correct for possible reflection
		if (rotation.determinant() < 0) {
			Matrix3f correction = Matrix3f::Identity();
			correction(2, 2) = -1;
			rotation = svd.matrixU() * correction * svd.matrixV().transpose();
		}

		return rotation;
	}

	Vector3f computeTranslation(const Vector3f& sourceMean, const Vector3f& targetMean, const Matrix3f& rotation) {
		// TODO: Compute the translation vector from source to target points.

		// Compute the rotated source mean
		Vector3f rotatedSourceMean = rotation * sourceMean;

		// Compute the translation vector
		Vector3f translation = targetMean - rotatedSourceMean;

		return translation;
	}
};
