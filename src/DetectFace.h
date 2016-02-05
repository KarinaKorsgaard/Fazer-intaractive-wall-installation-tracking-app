//
//  faceDetection.h
//  DAC-VirtualMirror
//
//  Created by Jonas Fehr on 26/01/16.
//
//
#include "ofMain.h"
#include "ofxCv.h"
#include "ofxMultiKinectV2.h"
#include "GpuRegistration.h"
#include "shaders.h"
#include "ofxFaceTracker.h"

using namespace ofxCv;
using namespace cv;

class DetectFace{
public:
    ofxCv::ObjectFinder objectFinder;
    
    ofxMultiKinectV2 *kinect;
    ofTexture colorTex;
    
    ofRectangle getFaceRoi(){return tracker.getHaarRectangle();}
    
    ofxFaceTracker tracker;
    ExpressionClassifier classifier;

    
    
    int imgSize = 100;
    
    void setup(ofxMultiKinectV2 *_kinect){
        kinect = _kinect;
        objectFinder.setup(ofToDataPath("haarcascade_frontalface_alt2.xml"));
        objectFinder.setPreset(ObjectFinder::Fast);
        
        
        tracker.setup();
        tracker.setRescale(.5);
        
        classifier.load("expressions");
        
        
    }
    
    void update() {
        
        colorTex.loadData(kinect->getColorPixelsRef(), GL_RGB);
        
        if(tracker.update(toCv(kinect->getColorPixelsRef()))){
        classifier.classify(tracker);
        }

        
       }
    
    void draw() {

        ofRectangle faceRoi = tracker.getHaarRectangle();
        int d = faceRoi.width*0.2;
        colorTex.drawSubsection(0,0,imgSize,imgSize,faceRoi.x-d, faceRoi.y-d, faceRoi.width+d, faceRoi.height+d);
        //ofDrawBitmapStringHighlight("Age: "+ofToString(tracker.getAge()),imgSize+5, 10);
    }
    void drawInk(){
        ofFbo subSection;
        subSection.allocate(colorTex.getWidth(), colorTex.getHeight(), 3);
        subSection.begin();
        ofBackground(0);
        //colorTex.draw(0,0);
        tracker.draw();
        subSection.end();
        
        ofRectangle faceRoi = tracker.getHaarRectangle();
        int d = faceRoi.width*0.2;
        subSection.getTexture().drawSubsection(0,0,imgSize,imgSize,faceRoi.x-d, faceRoi.y-d, faceRoi.width+d, faceRoi.height+d);
        ofSetColor(255);
        ofNoFill();
        ofDrawRectangle(0,0,imgSize,imgSize);
    }
    
    void drawClassification(){
        int w = 100, h = 12;
        ofPushStyle();
        ofPushMatrix();
        ofTranslate(0, 0);
        int n = classifier.size();
        int primary = classifier.getPrimaryExpression();
        for(int i = 0; i < n; i++){
            ofFill();
            ofSetColor(i == primary ? ofColor::red : ofColor::black);
            ofRect(0, 0, w * classifier.getProbability(i) + .5, h);
            ofSetColor(255);
            ofDrawBitmapString(classifier.getDescription(i), 5, 9);
            ofTranslate(0, h + 5);
        }
        ofPopMatrix();
        ofPopStyle();
    }
    
};