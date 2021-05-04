
#include <Eigen/Core>
#include <Eigen/LU>

typedef Eigen::Matrix<float, 4, 4> Matrix4f;
typedef Eigen::Matrix<float, 3, 3> Matrix3f;
typedef Eigen::Matrix<float, 4, 1> Vector4f;

class Transform
{
private: 

    Matrix4f transformMatrix;
    Matrix3f rotationMatrix;


    //float transformMatrix[4][4];
public:
    Transform(float xcord, float ycord, float zcord, float xrot, float yrot, float zrot);
    Vector4f MeasurementTransform(float meassurement);
    ~Transform();
};