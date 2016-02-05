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
    ofImage cropped;
    ofxCv::ObjectFinder objectFinder;
    
    ofxMultiKinectV2 *kinect;
    ofTexture colorTex;
    ofTexture depthTex;
    ofTexture irTex;
    
    ofShader depthShader;
    ofShader irShader;
    
    GpuRegistration gpuReg;
    
    ofRectangle faceRoi;
    ofRectangle getFaceRoi(){
        ofRectangle faceRoi_half;
        faceRoi_half.x = faceRoi.x/2;
        faceRoi_half.y = faceRoi.y/2;
        faceRoi_half.width = faceRoi.width/2;
        faceRoi_half.height = faceRoi.height/2;
        
        return faceRoi_half;
    }
    
    ofxFaceTracker tracker;

    
    
    int imgSize = 100;
    
    void setup(ofxMultiKinectV2 *_kinect){
        kinect = _kinect;
        objectFinder.setup(ofToDataPath("haarcascade_frontalface_alt2.xml"));
        objectFinder.setPreset(ObjectFinder::Fast);
        cropped.allocate(imgSize, imgSize, OF_IMAGE_COLOR);
        
        depthShader.setupShaderFromSource(GL_FRAGMENT_SHADER, depthFragmentShader);
        depthShader.linkProgram();
        
        irShader.setupShaderFromSource(GL_FRAGMENT_SHADER, irFragmentShader);
        irShader.linkProgram();
        
        gpuReg.setup(kinect->getProtonect(), 2);
        
        tracker.setup();
        tracker.setRescale(.5);
        
        
    }
    
    void update() {
        
        colorTex.loadData(kinect->getColorPixelsRef(), GL_RGB);
        depthTex.loadData(kinect->getDepthPixelsRef(), GL_LUMINANCE);
        irTex.loadData(kinect->getIrPixelsRef(), GL_LUMINANCE);
        
        depthTex.setTextureMinMagFilter(GL_NEAREST, GL_NEAREST);
        gpuReg.update(depthTex, colorTex, true);
        
        
        ofFbo regFbo;
        regFbo.allocate(512*2, 424*2);
        regFbo.begin();
        gpuReg.getRegisteredTexture(true).draw(0, 0, 512*2, 424*2);
        //colorTex.draw(0,0,1920,1080);
        regFbo.end();
        
        
        ofPixels regPix;
        
       regFbo.readToPixels(regPix);
        
        regPix.allocate(512*2, 424*2, OF_PIXELS_RGB);
        
         cout << "imgType: "+ofToString(regPix.getPixelFormat()) +" width: "+ ofToString(regPix.getWidth())+" height: "+ofToString(regPix.getHeight())+"\n";

        
        tracker.update(toCv(kinect->getColorPixelsRef()));

        
     /*  cv::Mat img = toCv(regPix);
        
        objectFinder.update(img);
        if(objectFinder.size() > 0) {
            cout << ofToString(objectFinder.getObject(0))+" \n";
            
            faceRoi = objectFinder.getObject(0);
        }*/
    }
    
    void draw() {

        //  gpuReg.getRegisteredTexture(false).drawSubsection(0,0,imgSize,imgSize,faceRoi.x, faceRoi.y, faceRoi.width, faceRoi.height);
        faceRoi = tracker.getHaarRectangle();
        colorTex.drawSubsection(0,0,imgSize,imgSize,faceRoi.x, faceRoi.y, faceRoi.width, faceRoi.height);
    }
    
    void drawReg() {
        ofPushMatrix();
        ofScale(0.25,0.25);
        ofSetColor(255);
       // gpuReg.getRegisteredTexture(true).draw(0,0);
        ofNoFill();
        tracker.draw();
        ofPopMatrix();
        
    }
    
};