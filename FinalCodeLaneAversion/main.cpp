// Created by Maitham Dib on 01/03/2016.
// Copyright © 2016 Maitham Dib. All rights reserved.
//

#include <iostream>
#include <string>
#include <stdlib.h>
#include <vector>
#include <math.h>
#include <opencv2/opencv.hpp>
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/objdetect.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include "DetectLanes.hpp"
#include "IPM.hpp"
#include "createFourPointsForIPM.hpp"
#include "KalmanFilterOneLane.hpp"
#include "OpticFlow.hpp"


using namespace std;
using namespace cv;

void checkIfFrameIsRead(Mat frame){
    if (frame.empty()) {
        cout << "Cannot read  frame " << endl;
    }
}

VideoCapture openVideoSafely(String videoPath){
    VideoCapture inputVideo(videoPath);
    if( !inputVideo.isOpened()){
        cout << "Cannot open the video file" << endl;
        return -1;
    }
    return inputVideo;
}


vector<Vec2f> updatedLineFrameRight;
vector<Vec2f> updatedLineFrameLeft;


int main() {
    
    VideoCapture inputVideo = openVideoSafely("/Users/Maitham/Desktop/road.mp4");
    VideoWriter output;
    
    // neccessary codec for video output and specify where to locate video
    int fourcc = CV_FOURCC('S','V','Q','3');
    output.open("/Users/Maitham/Desktop/RoadVideos/Dist_0.2m.avi", fourcc, 16, Size(1280,720));
    
    // Call detectLane Class to pull out functions
    detectLanes laneDetection;
    
    
    // Take in First Frame and initialise Kalman filter
    static Mat frame;
    inputVideo>>frame;
    checkIfFrameIsRead(frame);
    
    
    // Process image -> GetBinary -> CarryOutHoughTransform and get left and Right lane
    cv::Mat finalOutput = laneDetection.processSobel(frame);
    
    //Initialist Left Lane parameters
    vector<Vec2f> leftLaneInitial = laneDetection.findLeftLaneUsingSobel(finalOutput);
    vector<Vec2f> leftLanePrediction;
    KalmanFilterOneLane leftKF(leftLaneInitial);
    leftLanePrediction = leftKF.predictOneLane();
    
    // Initialise Right Lane Parameters
    vector<Vec2f> rightLaneInitial = laneDetection.findRightLaneUsingSobel(finalOutput);
    vector<Vec2f> rightLanePrediction;
    KalmanFilterOneLane rightKF(rightLaneInitial);
    rightLanePrediction = rightKF.predictOneLane();
    
    
    // Detect vanishing points of input video
    cv::Point2f initialVanishingPoint(0,0);
    initialVanishingPoint = laneDetection.getVanishingPoint(leftLaneInitial,rightLaneInitial);
    if (initialVanishingPoint.x==0){
        return -1;
    }
    
    // Initialise required variables
    
    //Frame count variable
    int frameVanishingPointCount=0;
    
    // booleans for optic flow
    bool areIPMImagesWrong = false;
    bool shouldOpticFlowBeApplied = true;
    bool shouldIPMBeCorrected = false;
    //Right Lane
    bool objectFoundCurrentRight  = false;
    bool objectFoundPreviousRight = false;
    bool objectFoundPreviousPreviousRight = false;
    //Left lane
    bool objectFoundCurrentLeft  = false;
    bool objectFoundPreviousLeft = false;
    bool objectFoundPreviousPreviousLeft = false;
    
    int count=0;
    bool canArrowsBeenDrawn=false;

    int countLeft=0;
    bool canArrowsBeenDrawnLeft =false;
    
    
    //INITIATE LANE LOOP
    while(1)
    { // Initialise the variables required for optic flow analysis
        static Mat frame, frame1, frame1_1C, frame2_1C;
        
        /*...............................................GET FIRST FRAME..........................................................*/
        inputVideo>>frame;
        checkIfFrameIsRead(frame);
        if (frame.empty()) {
            cout << "Cannot read  frame " << endl;
            break;
        }
        //increase Count
        frameVanishingPointCount++;
        
        //Get frameColour to output into Dense Optical Flow
        cv::Mat frameColour = frame.clone();
        
        // Get Canny Image -> HoughTransform -> left and right lane -> Vanishing points ->IPM
        cv::Mat imgSobel = laneDetection.processSobel(frame);
        cv::Mat frame_copy=frame.clone();
        
        //Find Left Lane
        vector<Vec2f> leftLaneFrame1 = laneDetection.findLeftLaneUsingSobel(imgSobel);
        //Ensure left lane is found otherwise output prediction
        if(leftLaneFrame1.size()<1){
            leftLaneFrame1 = leftLanePrediction;
            putText(frame_copy, "KF Left Prediction", cvPoint(30,60),FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(0,0,0), 1, CV_AA);
        }else{
            updatedLineFrameLeft = leftKF.updateOneLane(leftLaneFrame1);
            leftLanePrediction = leftKF.predictOneLane();
        }
        
        // Find Right Lane
        vector<Vec2f> rightLaneFrame1 = laneDetection.findRightLaneUsingSobel(imgSobel);
        if(rightLaneFrame1.size()<1){
            rightLaneFrame1 = rightLanePrediction;
            putText(frame_copy, "KF Right Prediction", cvPoint(30,30),FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(0,0,0), 1, CV_AA);
        }else{
            updatedLineFrameRight = rightKF.updateOneLane(rightLaneFrame1);
            rightLanePrediction  = rightKF.predictOneLane();
        }
        
        
        // Get the endPoints for IPM estimation
        vector<Vec2f> endPoints = laneDetection.getLaneBeginningAndEndPoints(leftLaneFrame1,rightLaneFrame1);
        
        // ROBUST INVERSE PERSPECTIVE OF SIDE
        cv::Mat ipmOfLeftLaneRobust = laneDetection.getLeftLaneIpmRobust(frameColour, endPoints, initialVanishingPoint, areIPMImagesWrong,shouldIPMBeCorrected);
        cv::Mat ipmOfRightLaneRobust = laneDetection.getRightLaneIpmRobust(frameColour, endPoints, initialVanishingPoint, areIPMImagesWrong);
        
        /*.....................................CHECK TO SEE IF VP NEEDS RESETTING...................................................*/
        if (frameVanishingPointCount>120 || areIPMImagesWrong){
            cv::Point2f vanishingPoints(0,0);
            vanishingPoints = laneDetection.getVanishingPoint(leftLaneFrame1,rightLaneFrame1);
            // Detect vanishing points of input video
            initialVanishingPoint=vanishingPoints;
            shouldOpticFlowBeApplied =false;
            areIPMImagesWrong=false;
            //reset Frame counter;
            frameVanishingPointCount=0;
        }else{
            shouldOpticFlowBeApplied = true;
            areIPMImagesWrong=false;
            
        }
        
        
        /*........................................GET SECOND FRAME...................................................*/
        inputVideo>>frame;
        checkIfFrameIsRead(frame);
        if (frame.empty()) {
            cout << "Cannot read  frame " << endl;
            break;
        }
        frameVanishingPointCount++;
        
        cv::Mat frameColour2=frame.clone();
        cv::Mat frame2Copy = frame.clone();
        
        //Get sobel of frame and repeat;
        cv::Mat imgSobel2 = laneDetection.processSobel(frame);
        
        //Check for Left Lane
        vector<Vec2f> leftLaneFrame2 = laneDetection.findLeftLaneUsingSobel(imgSobel2);
        if(leftLaneFrame2.size()<1){
            leftLaneFrame2 = leftLanePrediction;
            putText(frame2Copy, "KF Left Prediction", cvPoint(30,60),FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(0,0,0), 1, CV_AA);
        }else{
            updatedLineFrameLeft = leftKF.updateOneLane(leftLaneFrame2);
            leftLanePrediction = leftKF.predictOneLane();
        }
        
        //check for Right Lane
        vector<Vec2f> rightLaneFrame2 = laneDetection.findRightLaneUsingSobel(imgSobel2);
        if(rightLaneFrame2.size()<1){
            rightLaneFrame2 = rightLanePrediction;
            putText(frame2Copy, "KF Right Prediction", cvPoint(30,30),FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(0,0,0), 1, CV_AA);
        }else{
            updatedLineFrameRight = rightKF.updateOneLane(rightLaneFrame2);
            rightLanePrediction = rightKF.predictOneLane();
        }
        
        
        //Get EndPoints for Second Frame
        vector<Vec2f> endPoints2 = laneDetection.getLaneBeginningAndEndPoints(leftLaneFrame2,rightLaneFrame2);
        
        // ROBUST INVERSE PERSPECTIVE OF SIDELANES
        cv::Mat ipmOfLeftLaneRobust2 = laneDetection.getLeftLaneIpmRobust(frameColour2, endPoints2, initialVanishingPoint, areIPMImagesWrong,shouldIPMBeCorrected);
        cv::Mat ipmOfRightLaneRobust2 = laneDetection.getRightLaneIpmRobust(frameColour2, endPoints2, initialVanishingPoint, areIPMImagesWrong);
        
        
        /*.....................................CHECK TO SEE IF VP NEEDS RESETTING...................................................*/
        if (frameVanishingPointCount>120 || areIPMImagesWrong){
            // Push back lanesSeperate for vanishing point Function
            cv::Point2f vanishingPoints(0,0);
            vanishingPoints = laneDetection.getVanishingPoint(leftLaneFrame2, rightLaneFrame2);
            // Detect vanishing points of input video
            //            cout<<"VP1_frame2: "<<vanishingPoints.x<<","<<vanishingPoints.y<<endl;
            initialVanishingPoint= vanishingPoints;
            shouldOpticFlowBeApplied = false;
            areIPMImagesWrong= false;
            //reset Frame counter;
            frameVanishingPointCount=0;
        }else{
            //            cout<<"VP1_frame2: "<<initialVanishingPoint.x<<","<<initialVanishingPoint.y<<endl;
            shouldOpticFlowBeApplied = true;
            areIPMImagesWrong=false;
        }
        
        
        /*........................................PROCESS FRAMES FOR OUTPUT...................................................*/
        // Draw Lines for Left and Right Images and boundary box
        laneDetection.drawRightLinesOnImage(frame2Copy, rightLaneFrame2,0,255,0);
        laneDetection.drawLeftLinesOnImage(frame2Copy, leftLaneFrame2);
        laneDetection.drawLeftAndRightLanesOverlay(frame2Copy,leftLaneFrame2,rightLaneFrame2);
        
        
        
        cv::Mat mergedDenseOpticRobust;
        cv::Mat ipmOfRightLaneRobustDest;
        cv::Mat ipmOfRightLaneRobust2Dest;
        cv::Mat ipmOfLeftLaneRobustDest;
        cv::Mat ipmOfLeftLaneRobust2Dest;
        
        if (shouldOpticFlowBeApplied) {
            /*...................................... ..APPLY OPTIC FLOW............................................................*/
            //  Clone Sidelane frames to draw on
            cv::Mat leftLaneToDrawOnRobust =ipmOfLeftLaneRobust2.clone();
            cv::Mat rightLaneToDrawOnRobust= ipmOfRightLaneRobust2.clone();
            
            
            //  Convert Left to gray for optic Flow and Denoise LEFT LANE
            cvtColor(ipmOfLeftLaneRobust, ipmOfLeftLaneRobust, CV_BGR2GRAY);
            cvtColor(ipmOfLeftLaneRobust2, ipmOfLeftLaneRobust2, CV_BGR2GRAY);
            
            // denoise
            bilateralFilter(ipmOfLeftLaneRobust, ipmOfLeftLaneRobustDest, 8, 16, 4);
            bilateralFilter(ipmOfLeftLaneRobust2, ipmOfLeftLaneRobust2Dest, 8, 16, 4);
    
            cv::Mat leftLaneToDrawOnRobust2 = leftLaneToDrawOnRobust.clone();
            
            //  Get Optic Flow Left
            cv::Mat denseOpticLeftRobust = getDenseOpticFlowRobustLeft(ipmOfLeftLaneRobustDest,ipmOfLeftLaneRobust2Dest,leftLaneToDrawOnRobust,objectFoundCurrentLeft);
            
            //  Convert to Gray and denoise
            cvtColor(ipmOfRightLaneRobust, ipmOfRightLaneRobust, CV_BGR2GRAY);
            cvtColor(ipmOfRightLaneRobust2, ipmOfRightLaneRobust2, CV_BGR2GRAY);
            
            
            bilateralFilter(ipmOfRightLaneRobust, ipmOfRightLaneRobustDest, 8, 16, 4);
            bilateralFilter(ipmOfRightLaneRobust2, ipmOfRightLaneRobust2Dest, 8, 16, 4);
            
            
            cv::Mat rightLaneToDrawOnRobust2 = rightLaneToDrawOnRobust.clone();
            
            // Get optic Flow Right
            cv::Mat denseOpticRightRobust = getDenseOpticFlowRobustRight(ipmOfRightLaneRobustDest,ipmOfRightLaneRobust2Dest,rightLaneToDrawOnRobust,objectFoundCurrentRight);
            
            
            // Merge optic flow into One Image
            mergedDenseOpticRobust= laneDetection.mergeLeftAndRightImage(denseOpticLeftRobust, denseOpticRightRobust);
            
            // Convert both back to BGR for Video Output
            cvtColor(ipmOfLeftLaneRobustDest, ipmOfLeftLaneRobustDest, CV_GRAY2BGR);
            cvtColor(ipmOfRightLaneRobustDest, ipmOfRightLaneRobustDest, CV_GRAY2BGR);
        }else{
            //  Clone Sidelane frames to draw on
            cv::Mat leftLaneToDrawOnRobust =ipmOfLeftLaneRobust2.clone();
            cv::Mat rightLaneToDrawOnRobust= ipmOfRightLaneRobust2.clone();
            
            // Merge optic flow into One Image
            mergedDenseOpticRobust= laneDetection.mergeLeftAndRightImage(leftLaneToDrawOnRobust, rightLaneToDrawOnRobust);
            putText(mergedDenseOpticRobust, "Can't Run Optic Flow", cvPoint(10,30),FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(200,200,250), 1, CV_AA);
        }
        
        /*......................................WRITE IMAGE OUT TO VIDEO............................................................*/
        //Merge the Image with NO Optic Flow Shown
        cv::Mat mergedImageCleanRobust = laneDetection.mergeLeftAndRightImage(ipmOfLeftLaneRobustDest, ipmOfRightLaneRobustDest);
        frame2Copy = laneDetection.getSideLanePointsAndDraw(frame2Copy, initialVanishingPoint);
        
        
        /*......................................Issue Out Alert for required Period..................................................*/
        if (objectFoundCurrentLeft && objectFoundPreviousLeft && objectFoundPreviousPreviousLeft){
            canArrowsBeenDrawnLeft = true;
            countLeft=8;
        }else if (objectFoundCurrentLeft && objectFoundPreviousLeft &&  canArrowsBeenDrawnLeft){
            countLeft = countLeft + 4;
        }
        if(countLeft > 0){
            countLeft--;
            arrowedLine(frame2Copy, Point(0,360), Point(100,360), Scalar(0,0,255),5);
        }else{
            canArrowsBeenDrawnLeft = false;
            countLeft =0;
        }
        objectFoundPreviousLeft = objectFoundCurrentLeft;
        objectFoundPreviousPreviousLeft = objectFoundPreviousLeft;
        
        if (objectFoundCurrentRight && objectFoundPreviousRight && objectFoundPreviousPreviousRight){
            canArrowsBeenDrawn = true;
            count=8;
        }else if (objectFoundCurrentRight && objectFoundPreviousRight && canArrowsBeenDrawn){
            count++;
            count++;
            count++;
            count++;
        }
        if(count > 0){
            count--;
            arrowedLine(frame2Copy, Point(1280,360), Point(1180,360), Scalar(0,0,255),5);
        }else{
            canArrowsBeenDrawn = false;
            count =0;
        }
        objectFoundPreviousRight = objectFoundCurrentRight;
        objectFoundPreviousPreviousRight = objectFoundPreviousRight;

        // Resize image to fit and also copy in colour image to get Only one Output
        cv::resize(frame2Copy, frame2Copy, CvSize(800,720));
        cv::Mat frameCopy3 = frame2Copy.clone();
        
        frame2Copy.copyTo(mergedDenseOpticRobust(cv::Rect(240,0, 800,720)));
        
        //Write to just merged video
        output.write(mergedDenseOpticRobust);
        
        //outputFrame
        imshow("Optical Flow", mergedDenseOpticRobust);
        waitKey();
    }
    output.release();
    
}

