#include <iostream>
#include <Eigen/core>
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
    void MeasurementTransform(float meassurement);
    ~Transform();
};

Transform::Transform(float xcord, float ycord, float zcord, float xrot, float yrot, float zrot)
{
    this->transformMatrix(0,3) = xcord;
    this->transformMatrix(1,3) = ycord;
    this->transformMatrix(2,3)= zcord;
    this->transformMatrix(3,0) = 0;
    this->transformMatrix(3,1) = 0;
    this->transformMatrix(3,2) = 0;
    this->transformMatrix(3,3) = 1;

    Matrix3f rotxM;
    rotxM(0,0) = 1;
    rotxM(0,1) = 0;
    rotxM(0,2) = 0;
    rotxM(1,0) = 0;
    rotxM(1,1) = cos(xrot);
    rotxM(1,2) = -sin(xrot);
    rotxM(2,0) = 0;
    rotxM(2,1) = sin(xrot);
    rotxM(2,2) = cos(xrot);

    Matrix3f rotyM;
    rotyM(0,0) = cos(yrot);
    rotyM(0,1) = 0;
    rotyM(0,2)=sin(yrot);
    rotyM(1,0) = 0;
    rotyM(1,1) = 1;
    rotyM(1,2) = 0;
    rotyM(2,0) = -sin(yrot);
    rotyM(2,1) = 0;
    rotyM(2,2) = cos(yrot);

    Matrix3f rotzM;
    rotzM(0,0) = cos(zrot);
    rotzM(0,1) = -sin(zrot);
    rotzM(0,2) = 0;
    rotzM(1,0) = sin(zrot);
    rotzM(1,1) = cos(zrot);
    rotzM(1,2) = 0;
    rotzM(2,0) = 0;
    rotzM(2,1) = 0;
    rotzM(2,2) = 1;

    this->rotationMatrix = rotxM * rotyM * rotzM;



    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            this->transformMatrix(i,j) = this->rotationMatrix(i,j);
        }
        
    }
    

}

void Transform::MeasurementTransform(float meassurement)
{
    Vector4f meassurementVector;
    meassurementVector(0,0) = 0;
    meassurementVector(1,0) = -meassurement;
    meassurementVector(2,0) = 0;
    meassurementVector(3,0) = 1;

    //auto inverseTransform = transformMatrix.inverse();
    Vector4f transformedCordinate = transformMatrix.inverse()*meassurementVector;
    
    std::cout << transformedCordinate << std::endl;
}

Transform::~Transform()
{

}

using namespace std;

int main() {

    float scalar = 1000; // scalar fra mm til meter

    Transform camera(0,0,0,0,0,0);
    Transform sonarRight(33.2/scalar, 33.0/scalar, 42.5/scalar, 0,0, 30.0*M_PI/180.0);
    Transform sonarLeft(-22.0/scalar, 33.0/scalar, 42.5/scalar, 0,0, -30.0*M_PI/180.0);
    Transform sonarMiddle(8.0/scalar, 60.0/scalar, 81.5/scalar, 0,0,0);
    Transform IMU(-17.2/scalar, 6.0/scalar,64.0/scalar, 90.0*M_PI/180.0, 0, 0);

    sonarRight.MeasurementTransform(100.0/scalar);

    return 0;
}

/* koordinater før shift ud af x aksen pga. camera linse bruges som Orego -passer ikke helt længere
imU

25.2   x
2.5 8 y
-64  z 

x rot 90
y rot 0
z rot 0


middle ultralyd
0 x 
-60.5  y 
-81.5 z

x rot 0
y rot 0
z rot 0

højre ultralyd 
-30 x 
-32  y 
-(42.5) z

x rot 0
y rot 0 
z rot 30

venstre
30  x 
-32  y 
-42.5 z 

x rot 0
y rot 0 
z rot -30

camera 
0 x
0 y
0 z	

xrot 0 
y rot 0 
z rot 0 
*/