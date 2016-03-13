//
//  ofxbodyDetect.h
//
//  Created by Jonas Fehr on 15/11/15.
// based on https://www.youtube.com/watch?v=th8hUD7Ajg4
// by catsass19
//

#pragma once

#include "ofMain.h"
#include "ofxCv.h"

#include "Body.h"

#define RES_HEIGHT 1280
#define RES_WIDTH 800


class DetectBody
{
public:
    DetectBody();
    
    void setup(int _w, int _h, int _nearThreshold, int _farThreshold);
    void update(cv::Mat image);
    
    // DRAW
   // void drawOverlay(int x = 0, int y =0);
    void drawOverlay(int x, int y, int w, int h);
    //void drawProcess(int x = 0, int y =0, int index = 1);
    void drawProcess(int x, int y, int w, int h, int index = 1);
    void createMask();
    void resetMask();
    void createGenericMask();

    // GET DATA
    vector<Body> getBodies();
    vector<ofVec2f> getBodyPos();
    vector<ofPolyline> getContours();
    
    void setTresholds(float near, float far, float t){
        nearThreshold = near;
        farThreshold = far;
        tilt = t;
    }
    
    cv::Mat gradient;
private:
    
    void getbodyBoundries(cv::Mat _armBlobs);
    ofVec2f findArmBase(cv::Mat tableEdges, Body &_body);
    ofVec2f findPalmCenter(Body _body);
    
    float farThreshold;
    float nearThreshold;
    float tilt;
    
    //CvCapture *capture;
    ofFbo outputImage;

    
    cv::Mat *frame;
    //IplImage* mask = cvCreateImage(cvSize(640,480),IPL_DEPTH_8U,1);
    cv::Mat depthImage;
    cv::Mat inputImage;
    cv::Mat output;
    
    int outputImageIndex = 1;


    cv::Mat activeAreaMask;

    ofVec2f bodyPosition;
    vector<Body> bodies;

    
    int width, height;
    
    vector<vector<cv::Point> > contoursLocal;

};
