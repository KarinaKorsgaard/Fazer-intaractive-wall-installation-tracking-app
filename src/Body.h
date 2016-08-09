//
//  Body.h
//
//  Created by Jonas Fehr on 02/01/16.
//
//
#pragma once

#include "ofMain.h"
#include "ofxCV.h"
#include "ofxOsc.h"



//#include "ofxCvMin.h"

class Body
{
public:
    ofVec2f armBase; // Base of the arm (where the arm intersects the edge of the table)
    int indxBaseStart;
    int indxBaseEnd;
    
    float distanceAtCentroid;
//    int blobMin,blobMax;
    ofPolyline boundaryPoly;
    
    vector<ofPoint> boundary;
    vector<ofVec2f> indices;
    
    //ofVec2f palmCenter; // not used
    ofVec2f centroid;
    ofPixels bodyBlobPix;
    
    cv::Mat bodyBlob = cv::Mat::zeros( cvSize(512, 424), CV_8UC1 );
    cv::Mat bodyBlob2 = cv::Mat::zeros( cvSize(512, 424), CV_8UC1 );

    
    void draw(){
        ofSetColor(255, 0, 0);
        ofNoFill();
        ofSetColor(0, 0, 255);
        boundaryPoly.draw();
        ofDrawEllipse(centroid.x, centroid.y, 50, 50);
    };
};


