//
//  DetectLanes.cpp
//  HistogramSegmentation
//
//  Created by Maitham Dib on 13/03/2016.
//  Copyright © 2016 HelloOpenCV. All rights reserved.
//

#include "DetectLanes.hpp"

using namespace cv;
using namespace std;


cv::Mat detectLanes::mergeLeftAndRightImage(cv::Mat inversePerspectiveViewOfLeftSide2, cv::Mat inversePerspectiveViewOfRightSide2 ){
    //
    Mat sideLanesimg2(Mat(720, 1280, CV_8UC3));
    sideLanesimg2 = cv::Scalar(255);    // or the desired uint8_t value from 0-255
    
    if (inversePerspectiveViewOfLeftSide2.cols>0){
        inversePerspectiveViewOfLeftSide2.copyTo(sideLanesimg2(cv::Rect(0,0, 240,720)));
    }
    
    if(inversePerspectiveViewOfRightSide2.cols>0){
        inversePerspectiveViewOfRightSide2.copyTo(sideLanesimg2(cv::Rect(1040,0, 240,720)));
    }
    return sideLanesimg2;
}
void detectLanes::drawPoints( const std::vector<cv::Point2f>& _points, cv::Mat& _img )
{
    assert(_points.size() == 4);
    
    line(_img, Point(static_cast<int>(_points[0].x), static_cast<int>(_points[0].y)), Point(static_cast<int>(_points[3].x), static_cast<int>(_points[3].y)), CV_RGB( 205,205,0), 2);
    line(_img, Point(static_cast<int>(_points[2].x), static_cast<int>(_points[2].y)), Point(static_cast<int>(_points[3].x), static_cast<int>(_points[3].y)), CV_RGB( 205,205,0), 2);
    line(_img, Point(static_cast<int>(_points[0].x), static_cast<int>(_points[0].y)), Point(static_cast<int>(_points[1].x), static_cast<int>(_points[1].y)), CV_RGB( 205,205,0), 2);
    line(_img, Point(static_cast<int>(_points[2].x), static_cast<int>(_points[2].y)), Point(static_cast<int>(_points[1].x), static_cast<int>(_points[1].y)), CV_RGB( 205,205,0), 2);
    
    for(size_t i=0; i<_points.size(); i++)
    {
        circle(_img, Point(static_cast<int>(_points[i].x), static_cast<int>(_points[i].y)), 2, CV_RGB(238,238,0), -1);
        circle(_img, Point(static_cast<int>(_points[i].x), static_cast<int>(_points[i].y)), 5, CV_RGB(255,255,255), 2);
    }
}


cv::Mat detectLanes::getLeftLaneIpmRobust(cv::Mat &frame, vector<Vec2f> endPoints, cv::Point2f vanishingPoints2, bool &shouldvanishingPointBeReset,bool &shouldIPMBeCorrected){
    trapeziumCoordinates trapLeft = createFourPointsForIPMLeftLane(vanishingPoints2.x, vanishingPoints2.y);
    
    int lowerLeftLanePointx = endPoints[0][0];
    int lowerLeftLanePointy = endPoints[0][1];
    
    int upperLeftLanePointx = endPoints[1][0];
    int upperLeftLanePointy = endPoints[1][1];
    
    int upperLeftIpmPointx = (trapLeft.point3[0][0]/trapLeft.point3[3][0]);
    int upperLeftIpmPointy = (trapLeft.point3[1][0]/trapLeft.point3[3][0]);
    
    int lowerLeftIpmPointx = (trapLeft.point4[0][0]/trapLeft.point4[3][0]);
    int lowerLeftIpmPointy = (trapLeft.point4[1][0]/trapLeft.point4[3][0]);
    
    int upperLeftDiffx = abs(upperLeftLanePointx - upperLeftIpmPointx);
    int lowerLeftDiffx = abs(lowerLeftLanePointx - lowerLeftIpmPointx);
    
    int upperLeftDiffxNonAbs = (upperLeftLanePointx - upperLeftIpmPointx);
    int lowerLeftDiffxNonAbs = (lowerLeftLanePointx - lowerLeftIpmPointx);
    
    int upperLeftDiffy = abs(upperLeftLanePointy - upperLeftIpmPointy);
    int lowerLeftDiffy = abs(lowerLeftLanePointy - lowerLeftIpmPointy);
    
    if(upperLeftDiffx >100|| lowerLeftDiffx>100 || lowerLeftDiffy> 100|| upperLeftDiffy>100){
        cout<<"Reset framecount for Left lane recommended"<<endl;
        shouldvanishingPointBeReset= true;
        shouldIPMBeCorrected=true;
    }else{
        shouldvanishingPointBeReset = false;
        shouldIPMBeCorrected =false;
    }
    
    
    cv::Mat inversePerspectiveViewOfLeftSide2 = createIPMOfLeftSideLaneRobust(frame, trapLeft);
    
    return inversePerspectiveViewOfLeftSide2;
}


cv::Mat detectLanes::getRightLaneIpmRobust(cv::Mat &frame, vector<Vec2f> endPoints, cv::Point2f vanishingPoints2, bool &shouldvanishingPointBeReset){
    trapeziumCoordinates trapRight = createFourPointsForIPMRightLane(vanishingPoints2.x, vanishingPoints2.y);
    
    int lowerRightLanePointx = endPoints[2][0];
    int lowerRightLanePointy = endPoints[2][1];
    
    int upperRightLanePointx = endPoints[3][0];
    int upperRightLanePointy = endPoints[3][1];
    
    int upperRightIpmPointx = (trapRight.point3[0][0]/trapRight.point3[3][0]);
    int upperRightIpmPointy = (trapRight.point3[1][0]/trapRight.point3[3][0]);
    
    int lowerRightIpmPointx = (trapRight.point4[0][0]/trapRight.point4[3][0]);
    int lowerRightIpmPointy = (trapRight.point4[1][0]/trapRight.point4[3][0]);
    
    int upperRightDiffx = abs(upperRightLanePointx - upperRightIpmPointx);
    int lowerRightDiffx = abs(lowerRightLanePointx - lowerRightIpmPointx);
    
    int upperRightDiffy = abs(upperRightLanePointy - upperRightIpmPointy);
    int lowerRightDiffy = abs(lowerRightLanePointy - lowerRightIpmPointy);
    
    
    if(upperRightDiffx>150|| lowerRightDiffx>150 || lowerRightDiffy>150 || upperRightDiffy>150 ){
        cout<<"Reset framecount for right lane recommended"<<endl;
        shouldvanishingPointBeReset = true;
    }else{
        shouldvanishingPointBeReset = false;
    }
    
    cv::Mat inversePerspectiveViewOfRightSide2 = createIPMOfRightSideLaneRobust(frame, trapRight);
    return inversePerspectiveViewOfRightSide2;
}




cv::Mat detectLanes::getSideLanePointsAndDraw(cv::Mat imageToDrawOn,cv::Point2f vanishingPoints){
    
    trapeziumCoordinates trapRight = createFourPointsForIPMRightLane(vanishingPoints.x, vanishingPoints.y);
    vector<Point2f> rightIPMPoints;
    rightIPMPoints.push_back( Point2f(trapRight.point1[0][0]/trapRight.point1[3][0], trapRight.point1[1][0]/trapRight.point1[3][0]) );
    rightIPMPoints.push_back( Point2f(trapRight.point4[0][0]/trapRight.point4[3][0], trapRight.point4[1][0]/trapRight.point4[3][0]) );
    rightIPMPoints.push_back( Point2f(trapRight.point3[0][0]/trapRight.point3[3][0], trapRight.point3[1][0]/trapRight.point3[3][0]) );
    rightIPMPoints.push_back( Point2f(trapRight.point2[0][0]/trapRight.point2[3][0], trapRight.point2[1][0]/trapRight.point2[3][0]) );
    
    drawPoints(rightIPMPoints, imageToDrawOn);
    
    trapeziumCoordinates trapLeft = createFourPointsForIPMLeftLane(vanishingPoints.x, vanishingPoints.y);
    vector<Point2f> leftIPMPoints;
    leftIPMPoints.push_back( Point2f(trapLeft.point1[0][0]/trapLeft.point1[3][0], trapLeft.point1[1][0]/trapLeft.point1[3][0])  );
    leftIPMPoints.push_back( Point2f(trapLeft.point4[0][0]/trapLeft.point4[3][0], trapLeft.point4[1][0]/trapLeft.point4[3][0]) );
    leftIPMPoints.push_back( Point2f(trapLeft.point3[0][0]/trapLeft.point3[3][0], trapLeft.point3[1][0]/trapLeft.point3[3][0]) );
    leftIPMPoints.push_back( Point2f(trapLeft.point2[0][0]/trapLeft.point2[3][0], trapLeft.point2[1][0]/trapLeft.point2[3][0]) );
    
    drawPoints(leftIPMPoints, imageToDrawOn);
    
    return imageToDrawOn;
}




cv::Mat detectLanes::processSobel(cv::Mat frameBGR){
    cv::Mat imgGray;
    cvtColor(frameBGR, imgGray, CV_BGR2GRAY);
    cv::Mat imageSobel;
    cv::Sobel(imgGray, imageSobel, CV_8U, 1, 0, 3);
    cv::Mat imageSobel2;
    cv::Sobel(imageSobel, imageSobel2, CV_8U, 0, 1, 3);
    cv::Mat medianBlur;
    cv::medianBlur(imageSobel2, medianBlur, 3);
    cv::Mat finalOutput;
    cv::threshold(medianBlur, finalOutput, 100, 255, cv::THRESH_BINARY);
    return finalOutput;
}



vector<Vec2f> detectLanes::findLeftLaneUsingSobel(cv::Mat imgSobel){
    vector<Vec2f> lines;
    cv::HoughLines(imgSobel, lines, 1, M_PI/180, 50,0,0);
    // Filter out vector according to the angles of the left and right lane.
    vector<Vec2f> filteredLines(lines.size());
    auto it = copy_if(lines.begin(), lines.end(),
                      filteredLines.begin(),
                      [](const Vec2f &v) {
                          float angle = v[1];
                          return ((angle>55*CV_PI/180 && angle<70*CV_PI/180)) ;
                      });
    filteredLines.resize(std::distance(filteredLines.begin(),it));
    
    
    //Pull out the best left lane according to weighting algorithm
    vector<float> lanePara;
    vector<Vec2f> leftThetaAndRow;
    
    lanePara = decideLanes("left", filteredLines, height, top_margin, left_lane_left_margin, left_lane_right_margin, bottom_margin);
    if (lanePara.size()>0)
    {   //Pull out the intercept
            leftThetaAndRow.push_back(Vec2f(lanePara[6],lanePara[4])); // push back left lane rho and theta
    }else{
        printf("Left lane cannot be found");
    }
    return leftThetaAndRow;
}

vector<Vec2f> detectLanes::findRightLaneUsingSobel(cv::Mat imgSobel){
    vector<Vec2f> lines;
    cv::HoughLines(imgSobel, lines, 1, M_PI/180, 50,0,0);
    // Filter out vector according to the angles of the left and right lane.
    vector<Vec2f> filteredLines(lines.size());
    auto it = copy_if(lines.begin(), lines.end(),
                      filteredLines.begin(),
                      [](const Vec2f &v) {
                          float angle = v[1];
                          return (angle <= 140*CV_PI/180 && angle >=120*CV_PI/180) ;
                      });
    filteredLines.resize(std::distance(filteredLines.begin(),it));
    
    //Pull out the best left lane according to weighting algorithm
    vector<Vec2f> rightThetaAndRow;
    
    //Pull out the best right lane according to weighting algorithm
    vector<float> lanePara2;
    lanePara2 = decideLanes("right", filteredLines, height, top_margin, right_lane_left_margin, right_lane_right_margin, bottom_margin);
    
    if (lanePara2.size()>0){
        // push back right lane rho and theta;
        rightThetaAndRow.push_back(Vec2f(lanePara2[6],lanePara2[4]));
    }
    return rightThetaAndRow ;
}


// Detect Lane Markings using Hough and narrow down so that it returns just left and right lane push back to vector.
vector<Vec2f> detectLanes::HoughTransformReturnLeftRightLane(cv::Mat imgCanny){
    // intiialise lines vector and apply hough transforms
    vector<Vec2f> lines;
    cv::HoughLines( imgCanny, lines, 1, CV_PI/180, houghThreshold);
    // Filter out vector according to the angles of the left and right lane.
    vector<Vec2f> filteredLines(lines.size());
    auto it = copy_if(lines.begin(), lines.end(),
                      filteredLines.begin(),
                      [](const Vec2f &v) {
                          float angle = v[1];
                          return (angle <= 160*CV_PI/180 && angle >=100*CV_PI/180) || (angle>55*CV_PI/180 && angle<70*CV_PI/180) ;
                      });
    filteredLines.resize(std::distance(filteredLines.begin(),it));
    
    //Pull out the best left lane according to weighting algorithm
    vector<float> lanePara;
    vector<Vec2f> leftAndRightLaneThetaAndRow;
    
    lanePara = decideLanes("left", filteredLines, height, top_margin, left_lane_left_margin, left_lane_right_margin, bottom_margin);
    if (lanePara.size()>0){
        // push back left lane rho and theta
        leftAndRightLaneThetaAndRow.push_back(Vec2f(lanePara[6],lanePara[4]));
    }
    
    //Pull out the best right lane according to weighting algorithm
    vector<float> lanePara2;
    lanePara2 = decideLanes("right", filteredLines, height, top_margin, right_lane_left_margin, right_lane_right_margin, bottom_margin);
    if (lanePara2.size()>0)
    { // push back right lane rho and theta;
        leftAndRightLaneThetaAndRow.push_back(Vec2f(lanePara2[6],lanePara2[4]));
    }
    return leftAndRightLaneThetaAndRow;
}

void detectLanes:: drawBoundingBoxes(cv::Mat image){
    cv::circle(image, Point(left_lane_left_margin,top_margin), 20, Scalar(255, 255, 0));
    cv::circle(image, Point(left_lane_left_margin,bottom_margin), 20, Scalar(255, 255, 0));
    cv::circle(image, Point(left_lane_right_margin,top_margin), 20, Scalar(255, 255, 0));
    cv::circle(image, Point(left_lane_right_margin,bottom_margin), 20, Scalar(255, 255, 0));
    cv::circle(image, Point(right_lane_left_margin,top_margin), 20, Scalar(255, 255, 0));
    cv::circle(image, Point(right_lane_left_margin,bottom_margin), 20, Scalar(255, 255, 0));
    cv::circle(image, Point(right_lane_right_margin,top_margin), 20, Scalar(255, 255, 0));
    cv::circle(image, Point(right_lane_right_margin,bottom_margin), 20, Scalar(255, 255, 0));
}



vector<Vec2f> detectLanes::getLaneBeginningAndEndPoints(std::vector<cv::Vec2f> leftLineThetaAndRow,std::vector<cv::Vec2f> rightLineThetaAndRow ){

    float rhoL = leftLineThetaAndRow[0][0];
    float thetaL = leftLineThetaAndRow[0][1];
    
    float rhoR = rightLineThetaAndRow[1][0];
    float thetaR = rightLineThetaAndRow[1][1];
    
    vector<Vec2f> endpoints;
    
    double aL = cos(thetaL), bL = sin(thetaL);
    double aR = cos(thetaR), bR = sin(thetaR);
    
    int LLBI = findIntercept(bottom_margin, rhoL, thetaL);
    int LLTI = (top_margin - (rhoL/bL))/(-aL/bL);
    
    endpoints.push_back(Vec2f(LLBI,bottom_margin));
    endpoints.push_back(Vec2f(LLTI,top_margin));
    
    int RLBI = findIntercept( bottom_margin, rhoR, thetaR);
    int RLTI = (top_margin - (rhoR/bR))/(-aR/bR);
    endpoints.push_back(Vec2f(RLBI,bottom_margin));
    endpoints.push_back(Vec2f(RLTI,top_margin));
    
    return endpoints;
}


// Get Vanishing point from Lines
Point2f detectLanes::getVanishingPoint(vector<Vec2f> leftLaneInitial, vector<Vec2f> rightLaneInitial){
    // Push back lanesSeperate for vanishing point Function
    
    float rhoL = leftLaneInitial[0][0]; // rho1
    float thetaL = leftLaneInitial[0][1]; //theta1
    float rhoR = rightLaneInitial[0][0]; // rho2
    float thetaR = rightLaneInitial[0][1]; // theta2
    
    float aL = cos(thetaL), bL = sin(thetaL);
    float aR = cos(thetaR), bR = sin(thetaR);
    
    // calculate line intersection and compute vanishing point?
    // y = (-cos(theta)/sin(theta)*x + r/sin(theta))
    float x = ((rhoR/bR)-(rhoL/bL))/((aR/bR)-(aL/bL));
    float y = ((-aL/bL)*x) + (rhoL/bL);
    Point2f vanishingPoint(x,y);
    
    return vanishingPoint;
}
// Draw Lines on Image

vector<float> detectLanes::decideLanes(string laneDescriptor,vector<Vec2f> lines, int height,int topMargin,int leftMargin,int rightMargin, int bottomMargin)
{
    int distWeight = 2;
    int angleWeight = 10;
    int laneScore = -100000000;
    vector<float> lane;
    
    if (laneDescriptor.compare("left")==0){
        for(int i=0;i<lines.size();i++){
            float rho = lines[i][0];
            float theta = lines[i][1];
            double a = cos(theta), b = sin(theta);
            float intercept = findIntercept(bottom_margin, rho, theta);
            int topIntercept = (topMargin - (rho/b))/(-a/b);
            
            if (theta>0 && theta<M_PI/2 && intercept<rightMargin && topIntercept<rightMargin && intercept>leftMargin){
                float score = (-1*angleWeight*theta)+ (-1*distWeight * (rightMargin - intercept));
                if (score>laneScore){
                    laneScore=score;
                    Point2f LFBI(intercept, bottomMargin);
                    Point2f LFTI(topIntercept,topMargin);
                    float laneValues[7]= {LFBI.x,LFBI.y, LFTI.x,LFTI.y,theta,static_cast<float>(intercept),rho};
                    lane.insert(lane.end(), laneValues,laneValues+7);
                    
                }
            }
        }
    }
    
    if (laneDescriptor.compare("right")==0){
        for(int i=0;i<lines.size();i++){
            float rho = lines[i][0];
            float theta = lines[i][1];
            double a = cos(theta), b = sin(theta);
            
            int intercept = findIntercept(bottom_margin, rho, theta);
            int topIntercept = (topMargin - (rho/b))/(-a/b);

            if (theta< M_PI && theta>M_PI/2 && intercept<rightMargin  && topIntercept<rightMargin && intercept>leftMargin&& topIntercept>leftMargin){
                float score = (-1*angleWeight*theta)+ (-1*distWeight * (rightMargin - intercept));
                if (score>laneScore){
                    laneScore=score;
                    Point2f RLBI(intercept, bottomMargin);
                    Point2f RFTI(topIntercept,topMargin);
                    float laneValues[7]= {RLBI.x,RLBI.y, RFTI.x,RFTI.y,theta,static_cast<float>(intercept),rho};
                    lane.insert(lane.end(), laneValues,laneValues+7);
                }
            }
        }
    }
    return lane;
}


float detectLanes::findIntercept(int bottomMargin,float rho,float theta)
{
    float bottomIntercept;
    double a = cos(theta), b = sin(theta);
    bottomIntercept = (bottomMargin - (rho/b))/(-a/b);
    return bottomIntercept;
}


void detectLanes::drawLeftAndRightLanesOverlay(cv::Mat image,vector<Vec2f> leftLines, vector<Vec2f> rightLines){
    cv::Mat overlay = image.clone();
    
    float rho1 = leftLines[0][0];
    float theta1 = leftLines[0][1];
    double a1 = cos(theta1), b1 = sin(theta1);
    int botIntercept1 = findIntercept(720, rho1, theta1);
    int topIntercept1 = (top_margin - (rho1/b1))/(-a1/b1);
    Point LTI(topIntercept1,top_margin);
    Point LBI(botIntercept1,720);
    
    float rho2 = rightLines[0][0];
    float theta2 = rightLines[0][1];
    double a2 = cos(theta2), b2 = sin(theta2);
    int botIntercept2 = findIntercept(720, rho2, theta2);
    int topIntercept2 = (top_margin - (rho2/b2))/(-a2/b2);
    Point RTI(topIntercept2,top_margin);
    Point RBI(botIntercept2,720);
    
    
    Point polyPoints[1][4];
    polyPoints[0][0] = Point( LBI.x, LBI.y);
    polyPoints[0][1] = Point( LTI.x, LTI.y);
    polyPoints[0][2] = Point( RTI.x, RTI.y );
    polyPoints[0][3] = Point( RBI.x, RBI.y);
    
    
    const Point* ppt[1] = { polyPoints[0] };
    int npt[] = { 4 };
    
    fillPoly( overlay, ppt, npt, 1, Scalar( 0, 255, 0), 8 );
    cv::addWeighted(overlay, 0.3, image, 1-0.3, 0, image);
}

void detectLanes::drawLeftLinesOnImage(cv::Mat image, vector<Vec2f> lines){
    float rho1 = lines[0][0];
    float theta1 = lines[0][1];
    double a1 = cos(theta1), b1 = sin(theta1);
    int botIntercept1 = findIntercept(720, rho1, theta1);
    int topIntercept1 = (top_margin - (rho1/b1))/(-a1/b1);
    Point2f LTI(topIntercept1,top_margin);
    Point2f LBI(botIntercept1,720);
    line(image, LBI, LTI, CV_RGB(0, 255, 0), 2, 2);
}

void detectLanes::drawRightLinesOnImage(cv::Mat image, vector<Vec2f> lines, int red, int green, int blue){
    float rho2 = lines[0][0];
    float theta2 = lines[0][1];
    double a2 = cos(theta2), b2 = sin(theta2);
    int botIntercept2 = findIntercept(720, rho2, theta2);
    int topIntercept2 = (top_margin - (rho2/b2))/(-a2/b2);
    Point2f RTI(topIntercept2,top_margin);
    Point2f RBI(botIntercept2,720);
    line(image, RTI, RBI, CV_RGB(red, green, blue), 2, 2);
    
}



    
    

