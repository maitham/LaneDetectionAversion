//
//  OpticFlow.hpp
//  LaneCuttingAversion
//
//  Created by Maitham Dib on 18/03/2016.
//  Copyright © 2016 HelloOpenCV. All rights reserved.
//

#ifndef OpticFlow_hpp
#define OpticFlow_hpp

#include <stdio.h>
#include <string>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

cv::Mat getDenseOpticFlowRobustLeft(cv::Mat frame1_1C,cv::Mat frame2_1C,cv::Mat imageToDrawOn, bool &objectCuttingIn);
cv::Mat getDenseOpticFlowRobustRight(cv::Mat frame1_1C,cv::Mat frame2_1C,cv::Mat imageToDrawOn, bool &objectCuttingIn);

#endif /* OpticFlow_hpp */
