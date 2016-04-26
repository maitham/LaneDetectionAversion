//
//  createFourPointsForIPM.cpp
//  HistogramSegmentation
//
//  Created by Maitham Dib on 02/03/2016.
//  Copyright © 2016 HelloOpenCV. All rights reserved.
//

#include "createFourPointsForIPM.hpp"
#include <opencv2/opencv.hpp>
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/objdetect.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace std;

Matrix transformGround2ImageUsingHomo(Matrix ImagePoints,cameraInfo cameraInformation)
{
    float fu = cameraInformation.horizontalAperture_fx;
    float fv = cameraInformation.verticalAperture_fy;
    float c1 = cos(cameraInformation.pitchAngle) ;
    float c2 = cos(cameraInformation.yawAngle) ;
    float s1 = sin(cameraInformation.pitchAngle);
    float s2 = sin(cameraInformation.yawAngle);
    float cu = cameraInformation.principalPointX_u;
    float cv = cameraInformation.principalPointY_v;
    float h = cameraInformation.cameraHeight;
    
    Matrix homogeneousMatrix(4,Row(4));
    homogeneousMatrix[0][0] = ((-1/fu)*c2)*h;
    homogeneousMatrix[0][1] = ((1/fv)*s1*s2)*h;
    homogeneousMatrix[0][2] = (((1/fu)*cu*c2) - ((1/fv)*s1*s2) - (c1*s2))*h;
    homogeneousMatrix[0][3] = 0;
    
    homogeneousMatrix[1][0] = ((1/fu) *s2)*h;
    homogeneousMatrix[1][1] = ((1/fv)*s1*c1)*h;
    homogeneousMatrix[1][2] = (((1/fu)*cu*s2) - ((1/fv)*s1*c2) - (c1*c2))*h;
    homogeneousMatrix[1][3] = 0;
    
    homogeneousMatrix[2][0] = 0;
    homogeneousMatrix[2][1] = ((1/fv) *c1)*h;
    homogeneousMatrix[2][2] = (((-1/fv)*cv*c1)+s1)*h;
    homogeneousMatrix[2][3] = 0;
    
    homogeneousMatrix[3][0] = 0;
    homogeneousMatrix[3][1] = ((-1/(h*fv))*c1)*h;
    homogeneousMatrix[3][2] = (((1/(h*fv))*cv*c1) - ((1/h) *s1))*h;
    homogeneousMatrix[3][3] = 0;
    
    
    
    Matrix outputPoints = matrixMultiply(homogeneousMatrix, ImagePoints);
    return outputPoints;
}

Matrix transformImage2groundUsingHomo(Matrix ImagePoints,cameraInfo cameraInformation)
{
    float fu = cameraInformation.horizontalAperture_fx;
    float fv = cameraInformation.verticalAperture_fy;
    float c1 = cos(cameraInformation.pitchAngle) ;
    float c2 = cos(cameraInformation.yawAngle) ;
    float s1 = sin(cameraInformation.pitchAngle);
    float s2 = sin(cameraInformation.yawAngle);
    float cu = cameraInformation.principalPointX_u;
    float cv = cameraInformation.principalPointY_v;
    
    Matrix inverseHomogenousMatrix(4,Row(4));
    inverseHomogenousMatrix[0][0] = (fu*c2) + (cu*c1*s2);
    inverseHomogenousMatrix[0][1] = (cu*c1*c2)-(s2*fu);
    inverseHomogenousMatrix[0][2] = -cu*s1;
    inverseHomogenousMatrix[0][3] = 0;
    
    inverseHomogenousMatrix[1][0] = (s2*((cv*c1)-(fv*s1)));
    inverseHomogenousMatrix[1][1] = (c2*((cv*c1)-(fv*s1)));
    inverseHomogenousMatrix[1][2] = (-fv*c1) -(cv*s1);
    inverseHomogenousMatrix[1][3] = 0;
    
    inverseHomogenousMatrix[2][0] = c1*s2;
    inverseHomogenousMatrix[2][1] = c1*c2;
    inverseHomogenousMatrix[2][2] = -s1;
    inverseHomogenousMatrix[2][3] = 0;
    
    inverseHomogenousMatrix[3][0] = c1*s2;
    inverseHomogenousMatrix[3][1] = c1*c2;
    inverseHomogenousMatrix[3][2] = (-s1);
    inverseHomogenousMatrix[3][3] = 0;
    
    Matrix outputPoints = matrixMultiply(inverseHomogenousMatrix, ImagePoints);
    return outputPoints;
}

trapeziumCoordinates createFourPointsForIPMLeftLane(const float vanishingPointXCoordinate, const float vanishingPointYCoordinate )
{
    //******** Initialise Camera Values
    cameraInfo cameraCalibration;
    cameraCalibration.imageWidth =1280;
    cameraCalibration.imageHeight =720;
    cameraCalibration.horizontalAperture_fx =1.1240482059314654e+03;
    cameraCalibration.verticalAperture_fy =1.1056109173266630e+03;
    cameraCalibration.pitchAngle= atan(tan(cameraCalibration.verticalAperture_fy)*(1-((2*vanishingPointYCoordinate)/cameraCalibration.imageHeight))) ;
    cameraCalibration.yawAngle= atan(tan(cameraCalibration.horizontalAperture_fx)*(((2*vanishingPointXCoordinate)/cameraCalibration.imageWidth)- 1))-2.5*M_PI/180;
    cameraCalibration.rollAngle= 0;
    cameraCalibration.principalPointX_u=6.0962037058929434e+02;
    cameraCalibration.principalPointY_v=3.4095083786207891e+02;
    cameraCalibration.cameraHeight =1.2;
    cameraCalibration.imageHeight = 720;
    cameraCalibration.imageWidth = 1280 ;
    
    //************************************************ Define Matrix
    typedef vector<float> Row; // One row of the matrix
    typedef vector<Row> Matrix; // Matrix: a vector of rows
    
    //***************************************** initialise out camera calibration Matrix
    
    Matrix imagePoint1(4,Row(1));
    Matrix imagePoint2(4,Row(1));
    Matrix imagePoint3(4,Row(1));
    Matrix imagePoint4(4,Row(1));
    
    imagePoint1[0][0] = -2.3 ; //x
    imagePoint1[1][0] = 5.5; //y
    imagePoint1[2][0] = -cameraCalibration.cameraHeight; //z
    imagePoint1[3][0] = 1 ; //
    
    imagePoint2[0][0] = -2.3 ; //x
    imagePoint2[1][0] = 18; //y
    imagePoint2[2][0] = -cameraCalibration.cameraHeight; //z
    imagePoint2[3][0] = 1 ; //
    
    
    imagePoint3[0][0] = -2 ; //x
    imagePoint3[1][0] = 18; //y
    imagePoint3[2][0] = -cameraCalibration.cameraHeight; //z
    imagePoint3[3][0] = 1 ; //
    
    
    imagePoint4[0][0] = -2; //x
    imagePoint4[1][0] = 5.5; //y
    imagePoint4[2][0] = -cameraCalibration.cameraHeight; //z
    imagePoint4[3][0] = 1 ; //
    
    
    trapeziumCoordinates trapCoordinates;
    trapCoordinates.point1 = transformImage2groundUsingHomo(imagePoint1, cameraCalibration);
    trapCoordinates.point2 = transformImage2groundUsingHomo(imagePoint2, cameraCalibration);
    trapCoordinates.point3 = transformImage2groundUsingHomo(imagePoint3, cameraCalibration);
    trapCoordinates.point4 = transformImage2groundUsingHomo(imagePoint4, cameraCalibration);
    
    return trapCoordinates;
}


trapeziumCoordinates createFourPointsForIPMRightLane(const float vanishingPointXCoordinate, const float vanishingPointYCoordinate )
{
    //******** Initialise Camera Values
    cameraInfo cameraCalibration;
    cameraCalibration.imageWidth =1280;
    cameraCalibration.imageHeight =720;
    cameraCalibration.horizontalAperture_fx =1.1240482059314654e+03;
    cameraCalibration.verticalAperture_fy =1.1056109173266630e+03;
    cameraCalibration.pitchAngle= atan(tan(cameraCalibration.verticalAperture_fy)*(1-((2*vanishingPointYCoordinate)/cameraCalibration.imageHeight))) ;
    cameraCalibration.yawAngle= atan(tan(cameraCalibration.horizontalAperture_fx)*(((2*vanishingPointXCoordinate)/cameraCalibration.imageWidth)- 1))-2.5*M_PI/180;

    cameraCalibration.rollAngle= 0;
    cameraCalibration.principalPointX_u=6.0962037058929434e+02;
    cameraCalibration.principalPointY_v=3.4095083786207891e+02;
    cameraCalibration.cameraHeight =1.2;
    cameraCalibration.imageHeight = 720;
    cameraCalibration.imageWidth = 1280 ;
  
    //************************************************ Define Matrix
    typedef vector<float> Row; // One row of the matrix
    typedef vector<Row> Matrix; // Matrix: a vector of rows
    
    //***************************************** initialise out camera calibration Matrix
    
    Matrix imagePoint1(4,Row(1));
    Matrix imagePoint2(4,Row(1));
    Matrix imagePoint3(4,Row(1));
    Matrix imagePoint4(4,Row(1));
    
    imagePoint1[0][0] = 2.3 ; //x
    imagePoint1[1][0] = 5.5; //y
    imagePoint1[2][0] = -cameraCalibration.cameraHeight; //z
    imagePoint1[3][0] = 1 ; //

    imagePoint2[0][0] = 2.3 ; //x
    imagePoint2[1][0] = 18; //y
    imagePoint2[2][0] = -cameraCalibration.cameraHeight; //z
    imagePoint2[3][0] = 1 ; //
    
    
    imagePoint3[0][0] = 1.75; //x
    imagePoint3[1][0] = 18; //y
    imagePoint3[2][0] = -cameraCalibration.cameraHeight; //z
    imagePoint3[3][0] = 1 ; //
    
    
    imagePoint4[0][0] = 1.75 ; //x
    imagePoint4[1][0] = 5.5; //y
    imagePoint4[2][0] = -cameraCalibration.cameraHeight; //z
    imagePoint4[3][0] = 1 ; //
    
    
    trapeziumCoordinates trapCoordinates;
    trapCoordinates.point1 = transformImage2groundUsingHomo(imagePoint1, cameraCalibration);
    trapCoordinates.point2 = transformImage2groundUsingHomo(imagePoint2, cameraCalibration);
    trapCoordinates.point3 = transformImage2groundUsingHomo(imagePoint3, cameraCalibration);
    trapCoordinates.point4 = transformImage2groundUsingHomo(imagePoint4, cameraCalibration);
        
    return trapCoordinates;
}


