#pragma once

#include "ofMain.h"
#include "ofxMultiKinectV2.h"
#include "PointCloud.h"
#include "DetectFace.h"
#include "DetectBody.h"
#include "ofxFaceTracker.h"
#include "ofxCv.h"
#include "ofxGui.h"

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
    
    ofxMultiKinectV2 kinect;


    
   // ofEasyCam ecam;
    
    PointCloud pointCloud;
    ofPixels colorPix;
    
    DetectFace detectFace;
    DetectBody detectBody;
    
		
    
    bool bDebug = true;
    
    // GUI / Controlpanel
    ofxPanel gui;
    ofParameterGroup imageSetup;
    
    ofParameter<int>   nearThreshold;
    ofParameter<int>   farThreshold;
    
    int imgIndx = 0;
    
    ofTexture depthTex;
    ofShader depthShader;
    
    ofFbo depthFbo;

};
