#pragma once

#include "ofMain.h"
#include "ofxMultiKinectV2.h"
#include "PointCloud.h"
#include "ofxAutoReloadedShader.h"
#include "DetectBody.h"

#include "ofxCv.h"
#include "ofxGui.h"
#include "ofxCsv.h"

class ofApp : public ofBaseApp{

	public:
		void setup();
		void update();
		void draw();

		void keyPressed(int key);
		void keyReleased(int key);
		void mouseMoved(int x, int y );
		void mouseDragged(int x, int y, int button);
		void mousePressed(int x, int y, int button);
		void mouseReleased(int x, int y, int button);
		void mouseEntered(int x, int y);
		void mouseExited(int x, int y);
		void windowResized(int w, int h);
		void dragEvent(ofDragInfo dragInfo);
		void gotMessage(ofMessage msg);
    
    float getAvgDepth(ofRectangle space, ofxMultiKinectV2 *kinect);
    void positions();
    void detectPerson();
    void timeLine();
    
    ofPolyline contourPC;
    ofTrueTypeFont	font;
    ofxMultiKinectV2 kinect;
    PointCloud pointCloud;
    DetectBody detectBody;
    ofImage scanImage;
    
    ofTexture depthTex;
    ofxAutoReloadedShader depthShader;
    ofxAutoReloadedShader scanner;
    ofFbo depthFbo;
    ofFbo scanRender;
    ofFbo renderPC;
    DataPoint** datapoints;
    Actor** actors;
    wng::ofxCsv csv;
    
    int numDatapoints, numActors;
    float falling;
    int top = 0;
    int bot = ofGetWidth();
    int imgIndx = 0;
    
    int setPoints;

   // ofPixels colorPix;
    
    
    
    float scanLine;
    
    bool scanUp = false;
    bool scanDown = false;
    bool freeze = false;
    bool updatePC = true;
    bool setPositions;
    bool isPersonPresent = false;
    int isPPtimer = 0;
    int isPPthres = 100;
    bool ending = false;
    int endTimer = 0;
    int endTimerThres = 100;
    bool endAnimation = false;
    bool resetAll = false;
    bool active = false;
  
    
    bool bDebug = true;
    
    int scanLineHeight = 100;
    
    
    
    
    
   // vector<ofPolyline> contourPoly;
    
    // GUI / Controlpanel
    ofxPanel gui;
    ofParameterGroup imageSetup;
    ofParameterGroup pointCloudSetup;
    ofParameterGroup testParams;
    ofParameterGroup paramters;
    ofParameter<float>   test1; // idle GUI params for testing stuff
    ofParameter<float>   test2;
    ofParameter<float>   nearThreshold;
    ofParameter<float>   farThreshold;
    ofParameter<float>   tilt;

    
    ofParameter<int>   translateX;
    ofParameter<int>   translateY;
    ofParameter<int>   translateZ;
    
    ofParameter<float>edge;
    ofParameter<float>topDepth;
    ofParameter<float>topEdgeDepth;
    ofParameter<float>botDepth;
    ofParameter<float>edgeDepth;
    
    ofParameter<int>   bPosXlow;
    ofParameter<int>   thresX;
    ofParameter<int>   bPosYlow;
    ofParameter<int>   thresY;
    ofParameter<int>   floor;
    
    
   
};
