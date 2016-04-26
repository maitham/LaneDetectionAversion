//
//  OpticFlow.cpp
//  LaneCuttingAversion
//
//  Created by Maitham Dib on 18/03/2016.
//  Copyright © 2016 HelloOpenCV. All rights reserved.
//

#include "OpticFlow.hpp"
#include <iostream>
#include <math.h>

using namespace cv;
using namespace std;

cv::Mat getDenseOpticFlowRobustRight(cv::Mat frame1_1C,cv::Mat frame2_1C,cv::Mat imageToDrawOn, bool &objectCuttingIn){
    Mat flow, frame;
    // some faster than mat image container
    UMat  flowUmat, prevgray;
    vector<Vec4f> interestingPoints;
    vector<Vec4f> nonInterestingPoints;
    vector<Vec4f> allPoints;
    vector<float> angles;
    vector<float> absoluteSize;
    
    // For all optical flow you need a sequence of images.. Or at least 2 of them. Previous and current frame
    
    // calculate optical flow
    calcOpticalFlowFarneback(frame1_1C, frame2_1C, flowUmat, 0.5, 2, 50, 3, 5, 1.1, 0);
    // copy Umat container to standard Mat
    flowUmat.copyTo(flow);
    
    // By y += 5, x += 5 you can specify the grid
    for (int y = 0; y < imageToDrawOn.rows; y += 20){
        for (int x = 0; x < imageToDrawOn.cols; x += 20)
        {
            // get the flow from y, x position * 10 for better visibility
            const Point2f flowatxy = flow.at<Point2f>(y, x) * 10;
//            line(imageToDrawOn, Point(x, y), Point(cvRound(x + flowatxy.x), cvRound(y + flowatxy.y)), Scalar(0,0,255));
            allPoints.push_back(Vec4f(x,y,x + flowatxy.x,y + flowatxy.y));
            
            // draw line at flow direction
            int minimumDistance = 15;
            //Minimum Parameters for angle and resultant distance
            double resultantDistance = sqrt((flowatxy.x*flowatxy.x)+(flowatxy.y*flowatxy.y));
            
            
            float angletemp = atanf((abs(flowatxy.y))/(abs(flowatxy.x)));
            //                            cout<< "angletemp= "<<angletemp*180/M_PI<<endl;
            float calculatedAngle;
            if(flowatxy.x<0 && flowatxy.y<0 ){
                calculatedAngle = M_PI-angletemp;
            }else if (flowatxy.x<0 && flowatxy.y>0){
                calculatedAngle =M_PI + angletemp;
            }else if(flowatxy.x>0&&flowatxy.y>0 ){
                calculatedAngle = 2*M_PI - angletemp;
            }else{ 
                calculatedAngle = angletemp;
            }
            //Filter Lines
            if (resultantDistance>minimumDistance){
                if(calculatedAngle > 160*M_PI/180 && calculatedAngle<260*M_PI/180){
                    angles.push_back(calculatedAngle);
                    absoluteSize.push_back(resultantDistance);
                    interestingPoints.push_back(Vec4f(x,y,x + flowatxy.x,y + flowatxy.y));
//                    line(imageToDrawOn, Point(x, y), Point(cvRound(x + flowatxy.x), cvRound(y + flowatxy.y)), Scalar(255,255,0));
                    line(imageToDrawOn, Point(x, y), Point(cvRound(x + flowatxy.x), cvRound(y + flowatxy.y)), Scalar(255,255,0));
                    //                                    cout<<"calculatedAngle= "<<calculatedAngle*180/M_PI<<endl;
                    //                                    imshow("imageDebug", imageToDrawOn);
                    //
                    //                                    waitKey();
                }else{
                    nonInterestingPoints.push_back(Vec4f(x,y,x + flowatxy.x,y + flowatxy.y));
                }
            }
//            if(resultantDistance/10> 6){
//                line(imageToDrawOn, Point(x, y), Point(cvRound(x + flowatxy.x), cvRound(y + flowatxy.y)), Scalar(255,255,255));
//            }
            circle(imageToDrawOn, Point(x, y), 1, Scalar(0, 0, 0), -1);
        }
    }
    
    bool isCarCuttingIn;
    int sizeOfInterestingPoints = interestingPoints.size();
    int sizeOfNonInterestingPoints = nonInterestingPoints.size();
    
    float sumAngles;
    float sumDistances;
    
    //    cout<<"Interesting Points"<<interestingPoints.size()<<endl;
    //    cout<<"Non-Interesting Points"<<nonInterestingPoints.size()<<endl;
    
    if(sizeOfInterestingPoints>sizeOfNonInterestingPoints && sizeOfInterestingPoints>15){
        //average the angles and the magnitude
        for (int k=0; k<angles.size(); k++) {
            sumAngles += angles[k];
            sumDistances += absoluteSize[k];
        }
        float averageAngle =sumAngles/angles.size();
        float averageDistances = sumDistances/absoluteSize.size();
        
        float width  = imageToDrawOn.cols;
        float height = imageToDrawOn.rows;
        
        float averageY;
        float averageX;
        
        if(averageAngle>90*M_PI/180 && averageAngle<180*M_PI/180){
            // x -ve y +ve
            averageAngle = averageAngle-(90*M_PI/180);
            
            averageY = averageDistances * -sin(averageAngle);
            averageX = averageDistances * -(cos(averageAngle));
        }else if(averageAngle>180*M_PI/180 && averageAngle<270*M_PI/180){
            // x -ve y -ve
            averageAngle = averageAngle-(180*M_PI/180);
            
            averageY = averageDistances * (sin(averageAngle));
            averageX = averageDistances * -(cos(averageAngle));
        }else if(averageAngle>270*M_PI/180 && averageAngle<360*M_PI/180){
            // x +ve y -ve
            averageAngle = averageAngle-(270*M_PI/180);
            
            averageY = averageDistances * (sin(averageAngle));
            averageX = averageDistances * (cos(averageAngle));
        }else{
            averageX = averageDistances*cos(averageAngle);
            averageY = averageDistances*sin(averageAngle);
        }
        
        
        //        float y = averageDistances*sin(averageAngle);
        //        float x = averageDistances*cos(averageAngle);
        
        //Draw Large arrows
        arrowedLine(imageToDrawOn, Point(width/2,height/2), Point(averageX+width/2,averageY+height/2), Scalar(255,0,0));
        // turn bool on cutting lane
        isCarCuttingIn = true;
        //        cout<<"Car cutting in"<<endl;
    }else{
        //turn Bool on cutting lane
        //        cout<<"Car NOT cutting in"<<endl;
        isCarCuttingIn = false;
    }
    
    if (isCarCuttingIn) {
        circle(imageToDrawOn, Point(100,50), 10, Scalar(0,0,255),-1, 8, 0);
        objectCuttingIn=true;
    }else{
        circle(imageToDrawOn, Point(100,50), 10, Scalar(255,255,255),-1, 8, 0);
        objectCuttingIn=false;

    }
    
    return imageToDrawOn;
}

cv::Mat getDenseOpticFlowRobustLeft(cv::Mat frame1_1C,cv::Mat frame2_1C,cv::Mat imageToDrawOn, bool &objectCuttingIn){
    Mat flow, frame;
    // some faster than mat image container
    UMat  flowUmat, prevgray;
    vector<Vec4f> interestingPoints;
    vector<Vec4f> nonInterestingPoints;
    vector<Vec4f> allPoints;
    vector<float> angles;
    vector<float> absoluteSize;
    
    // For all optical flow you need a sequence of images.. Or at least 2 of them. Previous and current frame
    
    // calculate optical flow
    calcOpticalFlowFarneback(frame1_1C, frame2_1C, flowUmat, 0.5, 2, 50, 3, 5, 1.1, 0);
    // copy Umat container to standard Mat
    flowUmat.copyTo(flow);
    
    // By y += 5, x += 5 you can specify the grid
    for (int y = 0; y < imageToDrawOn.rows; y += 20){
        for (int x = 0; x < imageToDrawOn.cols; x += 20)
        {
            // get the flow from y, x position * 10 for better visibility
            const Point2f flowatxy = flow.at<Point2f>(y, x) * 10;
//            line(imageToDrawOn, Point(x, y), Point(cvRound(x + flowatxy.x), cvRound(y + flowatxy.y)), Scalar(0,0,255));
            allPoints.push_back(Vec4f(x,y,x + flowatxy.x,y + flowatxy.y));
            
            // draw line at flow direction
            int minimumDistance = 15;
            //Minimum Parameters for angle and resultant distance
            double resultantDistance = sqrt((flowatxy.x*flowatxy.x)+(flowatxy.y*flowatxy.y));
            float angletemp = atanf((abs(flowatxy.y))/(abs(flowatxy.x)));
            //                            cout<< "angletemp= "<<angletemp*180/M_PI<<endl;
            float calculatedAngle;
            if(flowatxy.x<0 && flowatxy.y<0 ){
                calculatedAngle = M_PI-angletemp;
            }else if (flowatxy.x<0 && flowatxy.y>0){
                calculatedAngle =M_PI + angletemp;
            }else if(flowatxy.x>0&&flowatxy.y>0 ){
                calculatedAngle = 2*M_PI - angletemp;
            }else{
                calculatedAngle = angletemp;
            }
            //Filter Lines
            if (resultantDistance>minimumDistance){
                if(calculatedAngle > 160*M_PI/180 && calculatedAngle<260*M_PI/180){
                    angles.push_back(calculatedAngle);
                    absoluteSize.push_back(resultantDistance);
                    interestingPoints.push_back(Vec4f(x,y,x + flowatxy.x,y + flowatxy.y));
                    line(imageToDrawOn, Point(x, y), Point(cvRound(x + flowatxy.x), cvRound(y + flowatxy.y)), Scalar(255,255,0));
//                    line(imageToDrawOn, Point(x, y), Point(cvRound(x + flowatxy.x), cvRound(y + flowatxy.y)), Scalar(255,255,0));
                    //                                    cout<<"calculatedAngle= "<<calculatedAngle*180/M_PI<<endl;
                    //                                    imshow("imageDebug", imageToDrawOn);
                    //
                    //                                    waitKey();
                }else{
                    nonInterestingPoints.push_back(Vec4f(x,y,x + flowatxy.x,y + flowatxy.y));
                }
            }
            circle(imageToDrawOn, Point(x, y), 1, Scalar(0, 0, 0), -1);
        }
    }
    
    bool isCarCuttingIn;
    int sizeOfInterestingPoints = interestingPoints.size();
    int sizeOfNonInterestingPoints = nonInterestingPoints.size();
    
    float sumAngles;
    float sumDistances;
    
    //    cout<<"Interesting Points"<<interestingPoints.size()<<endl;
    //    cout<<"Non-Interesting Points"<<nonInterestingPoints.size()<<endl;
    
    if(interestingPoints.size()>nonInterestingPoints.size() && sizeOfInterestingPoints>15){
        //average the angles and the magnitude
        for (int k=0; k<angles.size(); k++) {
            sumAngles += angles[k];
            sumDistances += absoluteSize[k];
        }
        float averageAngle =sumAngles/angles.size();
        float averageDistances = sumDistances/absoluteSize.size();
        
        float width  = imageToDrawOn.cols;
        float height = imageToDrawOn.rows;
        
        float averageY;
        float averageX;
        
        if(averageAngle>90*M_PI/180 && averageAngle<180*M_PI/180){
            // x -ve y +ve
            averageAngle = averageAngle-(90*M_PI/180);
            
            averageY = averageDistances * -sin(averageAngle);
            averageX = averageDistances * -(cos(averageAngle));
        }else if(averageAngle>180*M_PI/180 && averageAngle<270*M_PI/180){
            // x -ve y -ve
            averageAngle = averageAngle-(180*M_PI/180);
            
            averageY = averageDistances * (sin(averageAngle));
            averageX = averageDistances * -(cos(averageAngle));
        }else if(averageAngle>270*M_PI/180 && averageAngle<360*M_PI/180){
            // x +ve y -ve
            averageAngle = averageAngle-(270*M_PI/180);
            
            averageY = averageDistances * (sin(averageAngle));
            averageX = averageDistances * (cos(averageAngle));
        }else{
            averageX = averageDistances*cos(averageAngle);
            averageY = averageDistances*sin(averageAngle);
        }
        
        
        //        float y = averageDistances*sin(averageAngle);
        //        float x = averageDistances*cos(averageAngle);
        
        //Draw Large arrows
        arrowedLine(imageToDrawOn, Point(width/2,height/2), Point(averageX+width/2,averageY+height/2), Scalar(255,0,0));
        // turn bool on cutting lane
        isCarCuttingIn = true;
        //        cout<<"Car cutting in"<<endl;
    }else{
        //turn Bool on cutting lane
        //        cout<<"Car NOT cutting in"<<endl;
        isCarCuttingIn = false;
    }
    
    if (isCarCuttingIn) {
        circle(imageToDrawOn, Point(100,50), 10, Scalar(0,0,255),-1, 8, 0);
        objectCuttingIn=true;
    }else{
        circle(imageToDrawOn, Point(100,50), 10, Scalar(255,255,255),-1, 8, 0);
        objectCuttingIn=false;
    }
    
    return imageToDrawOn;
}