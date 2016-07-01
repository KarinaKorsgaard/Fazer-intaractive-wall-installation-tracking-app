#pragma once

#include "ofMain.h"
#include "ofxKinectV2.h"

#include "ofxAutoReloadedShader.h"
#include "DetectBody.h"
#include "ofxXmlSettings.h"
#include "ofxCv.h"
#include "ofxGui.h"

#include "ofxOsc.h"



#define PORT 8000
#define HOST "localhost"

#define K_W 512
#define K_H 424


class ofApp : public ofBaseApp{

	public:
		void setup();
		void update();
		void draw();

		void keyPressed(int key);

    float getAvgDepth(ofRectangle space, ofxKinectV2 *kinect);

    void detectPerson();


    vector<ofPolyline>people2D;
 
    ofImage contourDetectImg;

    ofxKinectV2 kinect;
    DetectBody detectBody;
    
    ofTexture depthTex;
    ofxAutoReloadedShader depthShader;
    ofFbo depthFbo;
    ofFbo testRender;
    ofFbo contourRender;
    ofxCv::ContourFinder contourFinder;
    ofVboMesh mesh; // mesh to rotate

    int imgIndx = 7;
     
    bool bDebug = false;


    
    //close/open kinect counter. Does not work
    int counter = 0;
    
    void getBodyPolys(ofFbo myRender);
    
    // GUI / Controlpanel
    ofxPanel gui;
    
    ofParameterGroup imageSetup;
    ofParameterGroup pointCloudSetup;
    ofParameterGroup testParams;
    ofParameterGroup paramters;
    ofParameter<float>   nearThreshold;
    ofParameter<float>   farThreshold;
    ofParameter<float>   tilt,tiltX,tiltY,tiltZ;

    
    ofParameter<int>   translateX;
    ofParameter<int>   translateY;
    ofParameter<int>   translateZ;
    
    ofParameter<int>   resample;
    
    ofParameter<float>edge;
    ofParameter<float>topDepth;
    ofParameter<float>topEdgeDepth;
    ofParameter<float>botDepth;
    ofParameter<float>edgeDepth;
    
    
    ofParameter<int>appId;
    ofParameter<float>blobMin;
    ofParameter<float>blobMax;
//    ofParameter<int>   bPosX;
//    ofParameter<int>   thresW;
//    ofParameter<int>   bPosY;
//    ofParameter<int>   thresH;
//    ofParameter<int>   floor;
    
    //ofRectangle thePerson;
    
    
    ofxOscSender sender;
    ofxOscSender localSender;

    
    string message;
    

    
   
};
