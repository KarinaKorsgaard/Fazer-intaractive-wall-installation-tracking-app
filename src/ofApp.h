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
    
    void positions();
    ofPolyline contourPC;
    
    
    void loadActors();
    
    float getAvgDepth(ofRectangle space, ofxMultiKinectV2 *kinect);
    
    ofxMultiKinectV2 kinect;

    ofTrueTypeFont	font;
    
    //ofEasyCam ecam;
    
    PointCloud pointCloud;
    ofPixels colorPix;
    
   
    DetectBody detectBody;
    
    float scanLine;
    
    bool scanUp = false;
    bool scanDown = false;
    bool freeze = false;
    bool updatePC = true;
    bool setPositions;
    bool isPersonPresent = false;
    int isPP = 0;
    int isPPthres = 100;
    bool ending = false;
    int endTimer = 0;
    int endTimerThres = 100;
    bool endAnimation = false;
    bool resetAll = false;
    
    bool bDebug = true;
    
    int scanLineHeight = 100;
    
    ofImage scanImage;
    
    ofFbo renderPC;
    
    
    int timer;
    vector<ofPolyline> contourPoly;
    
    // GUI / Controlpanel
    ofxPanel gui;
    ofParameterGroup imageSetup;
    ofParameterGroup pointCloudSetup;
    ofParameterGroup testParams;
    ofParameterGroup paramters;
    ofParameter<float>   test1;
    ofParameter<float>   test2;
    ofParameter<float>   nearThreshold;
    ofParameter<float>   farThreshold;
    
    ofParameter<int>   translateX;
    ofParameter<int>   translateY;
    ofParameter<int>   translateZ;
    ofParameter<int>   scaleX;
    ofParameter<int>   scaleY;
    ofParameter<float>   scaleZ;
    ofParameter<int>   rotate;
     
    ofParameter<float>   tilt;
    ofParameter<int>   nearCut;
    ofParameter<int>   farCut;
    
    ofParameter<float>edge;
    ofParameter<float>topDepth;
    ofParameter<float>topEdgeDepth;
    ofParameter<float>botDepth;
    ofParameter<float>edgeDepth;
    
    int top = 0;
    //int edge = ofGetWidth()*2/3;
    int bot = ofGetWidth();
    
    // de tre verdier hvor gradienten starter og hvor den slutter
    // nok alle tre skal kunne styres med en slider
//    int topDepth = 200;
//    int edgeDepth = 30;
//    int botDepth = 150;
    
    int imgIndx = 0;
    
    ofTexture depthTex;
    ofxAutoReloadedShader depthShader;
    
    ofFbo depthFbo;
    
    DataPoint** datapoints;
    Actor** actors;
    
    wng::ofxCsv csv;
    int numDatapoints, numActors;
    float falling;
    
    int largestBlob;
    int setPoints;
    
    
};
