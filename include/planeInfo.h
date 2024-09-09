#pragma once
#include <Eigen/Core>
struct planeInfo
{
	Eigen::Vector3d normal;
	Eigen::Vector3d center;
    planeInfo()
    {

    }
	planeInfo(Eigen::Vector3d normal_, Eigen::Vector3d center_)
	{
		normal = normal_;
		center = center_;
	}
    planeInfo transform(Eigen::Matrix4d & T)
    {
        planeInfo tmpplane;
        Eigen::Vector4d center_H;
        center_H.head(3) = center; center_H(3) = 1;
        tmpplane.center = (T * center_H).head(3);
        tmpplane.normal = T.block<3,3>(0,0) * normal;
        return tmpplane;
    }
  double getZ(Eigen::Vector2d xy)
  {
    // Eigen::Vector2d xy_(xy.x(), xy.y());
    double d = -center.dot(normal);
    return (-d - xy.dot(normal.head(2)))/normal(2);
  }
  double getZ(double x, double y)
  {
    return getZ(Eigen::Vector2d(x, y));
  }
};