//
//  DetectLanes.hpp
//  HistogramSegmentation
//
//  Created by Maitham Dib on 13/03/2016.
//  Copyright © 2016 HelloOpenCV. All rights reserved.
//

#ifndef DetectLanes_hpp
#define DetectLanes_hpp

#include <stdio.h>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/objdetect.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "createFourPointsForIPM.hpp"
#include "IPM.hpp"


class detectLanes {
    int houghThreshold = 153;
    int procWidth = 1280;
    int procHeight = 720;
    int top_margin = 420;
    int bottom_margin = 600;
    int left_lane_left_margin = 0;
    int left_lane_right_margin = 625;
    int right_lane_left_margin = 625;
    int right_lane_right_margin = 1100;
    int height = 720;
    
    
public:
    std::vector<cv::Vec2f> HoughTransformReturnLeftRightLane(cv::Mat imgCanny);
    cv::Point2f getVanishingPoint(std::vector<cv::Vec2f> leftLaneInitial, std::vector<cv::Vec2f> rightLaneInitial);
    std::vector<cv::Vec2f> getLaneBeginningAndEndPoints(std::vector<cv::Vec2f> leftLineThetaAndRow,std::vector<cv::Vec2f> rightLineThetaAndRow );
    void drawLeftLinesOnImage(cv::Mat image, std::vector<cv::Vec2f> lines);
    void drawRightLinesOnImage(cv::Mat image, std::vector<cv::Vec2f> lines, int red, int green, int blue);
    void drawBoundingBoxes(cv::Mat image);
    std::vector<cv::Vec2f> findLeftLaneUsingSobel(cv::Mat imgCanny);
    std::vector<cv::Vec2f> findRightLaneUsingSobel(cv::Mat imgSobel);
    cv::Mat processSobel(cv::Mat frameBGR);
    void drawLeftAndRightLanesOverlay(cv::Mat image,std::vector<cv::Vec2f> LeftLines, std::vector<cv::Vec2f> rightLines);
    cv::Mat mergeLeftAndRightImage(cv::Mat inversePerspectiveViewOfLeftSide2, cv::Mat inversePerspectiveViewOfRightSide2);
    void drawPoints( const std::vector<cv::Point2f>& _points, cv::Mat& _img );
    cv::Mat getLeftLaneIpmRobust(cv::Mat &frame, std::vector<cv::Vec2f> endPoints, cv::Point2f vanishingPoints2, bool &shouldvanishingPointBeReset,bool &shouldIPMBeCorrected);
    cv::Mat getRightLaneIpmRobust(cv::Mat &frame, std::vector<cv::Vec2f> endPoints, cv::Point2f vanishingPoints2, bool &shouldvanishingPointBeReset);
    cv::Mat getSideLanePointsAndDraw(cv::Mat imageToDrawOn,cv::Point2f vanishingPoints);


private:
    std::vector<float> decideLanes(std::string laneDescriptor,std::vector<cv::Vec2f> lines, int height,int topMargin,int leftMargin,int rightMargin, int bottomMargin);
    float findIntercept(int bottomMargin,float rho,float theta);
};

#endif /* DetectLanes_hpp */

