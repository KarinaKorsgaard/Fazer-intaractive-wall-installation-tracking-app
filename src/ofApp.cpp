#include "ofApp.h"
//--------------------------------------------------------------
void ofApp::setup(){
    ofSetVerticalSync(true);
    ofSetFrameRate(60);
    
    
    kinect.open(true, true, 0, 2); // GeForce on MacBookPro Retina
    
    kinect.start();
    
    pointCloud.setup();
    
    detectFace.setup(&kinect);
    
    detectBody.setup(512, 424, nearThreshold, farThreshold);
    
    depthShader.load("depthshader/shader");
    
    // Setup GUI
    imageSetup.setName("imageSetup");
    imageSetup.add(nearThreshold.set("nearThreshold", 100, 0, 255));
    imageSetup.add(farThreshold.set("farThreshold", 255, 0, 255));
    
    gui.setup(imageSetup);
}

//--------------------------------------------------------------
void ofApp::update(){
    kinect.update();
    if (kinect.isFrameNew()) {
        pointCloud.update(&kinect);
        kinect.getColorPixelsRef().allocate(1920, 1080, 3);
        
        kinect.getDepthPixelsRef().allocate(512, 424, 1);
        
        detectFace.update();
        
        depthTex.loadData(kinect.getDepthPixelsRef(), GL_LUMINANCE);
        depthTex.setTextureMinMagFilter(GL_NEAREST, GL_NEAREST);

        
        depthFbo.allocate(512, 424, GL_R16);

        depthFbo.begin();
        depthShader.begin();
        depthShader.setUniform1f("nearThreshold", nearThreshold);
        depthShader.setUniform1f("farThreshold", farThreshold);

        depthTex.draw(0, 0, 512, 424);
        depthShader.end();
        depthFbo.end();
        
        ofShortPixels depthPix;
        depthFbo.readToPixels(depthPix);
        
        ofImage depthImage;
        depthImage.setFromPixels(depthPix);
        
   

        detectBody.update(toCv(depthImage));//kinect.getDepthPixelsRef()));
        detectBody.setTresholds(nearThreshold, farThreshold);
        
    }
    /*
     cam.update();
     if(cam.isFrameNew()) {
     detectFace.update(cam);
     }
     cout << "imageType: "+ofToString(cam.getPixelFormat())+"\n";*/
}

//--------------------------------------------------------------
void ofApp::draw(){
    ofClear(0);
    
    ofPushMatrix();
    
    
    pointCloud.draw();
    ofSetColor(255);
    ofNoFill();
    
    float avgDepth = kinect.getDistanceAt(detectFace.getFaceRoi().x+detectFace.getFaceRoi().width/2, detectFace.getFaceRoi().y+detectFace.getFaceRoi().height/2) ;
    
    ofPoint tl = ofPoint(kinect.getWorldCoordinateAt(detectFace.getFaceRoi().x, detectFace.getFaceRoi().y));
    tl.z = avgDepth;
    
    ofPoint tr = ofPoint(kinect.getWorldCoordinateAt(detectFace.getFaceRoi().x + detectFace.getFaceRoi().width, detectFace.getFaceRoi().y));
    tr.z = avgDepth;
    
    ofPoint bl = ofPoint(kinect.getWorldCoordinateAt(detectFace.getFaceRoi().x, detectFace.getFaceRoi().y + detectFace.getFaceRoi().height));
    bl.z = avgDepth;
    
    ofPoint br = ofPoint(kinect.getWorldCoordinateAt(detectFace.getFaceRoi().x + detectFace.getFaceRoi().width, detectFace.getFaceRoi().y + detectFace.getFaceRoi().height));
    br.z = avgDepth;
    
    // center the points a bit
    //  ofDrawLine(tl, tr);
    //  ofDrawLine(tr, br);
    //  ofDrawLine(br, bl);
    //  ofDrawLine(bl, tl);
    ofPopMatrix();
    
    ofPushMatrix();
    ofTranslate(ofGetWidth()-300, 150);
    detectFace.draw();
    ofTranslate(detectFace.imgSize+5, 0);
    detectFace.drawInk();
    ofTranslate(-detectFace.imgSize-5, detectFace.imgSize+5);
    detectFace.drawClassification();
    
    ofPopMatrix();
    
    if(bDebug){
        detectBody.drawProcess(100,10, 512/2, 424/2, imgIndx);
        detectBody.drawOverlay(100, 10, 512/2, 424/2);
        
        
        gui.draw();
        
        drawHighlightString(
                            string() +
                            "d - debugMode\n" +
                            "b - removeBG PC\n" +
                            "r - reset\n" +
                            "e - add expression\n" +
                            "a - add sample\n" +
                            "s - save expressions\n"
                            "l - load expressions",
                            14, ofGetHeight() - 10 * 12);

        drawHighlightString(string() +
                            "index of the detection Image\n" +
                            "1 depthImage\n" +
                            "2 depthImageCanny\n" +
                            "3 depthImageErodedArms\n" +
                            "4 armDivider\n" +
                            "5 armEdges\n" +
                            "6 activeAreaMask\n" +
                            "7 inputImage\n",
                            14*15, ofGetHeight() - 10 * 12);
        
        
        ofSetWindowTitle("FrameRate: "+ ofToString(ofGetFrameRate()));
        // ofDrawBitmapStringHighlight("Device Count : " + ofToString(ofxMultiKinectV2::getDeviceCount()), 10, 40);
    }
}

float ofApp::getAvgDepth(ofRectangle space, ofxMultiKinectV2 *kinect){
    int div = 0;
    float avgDepth = 5000;
    for(int y = space.y; y < space.y+space.height; y ++) {
        for(int x = space.x; x < space.x+space.width; x ++) {
            avgDepth += kinect->getDistanceAt(x, y);
            div++;
            //            float dist = kinect->getDistanceAt(x, y);
            //            if (dist < avgDepth && dist > 150) avgDepth = dist;
            
        }
    }
    return avgDepth/div;
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
    
    if(key =='d') {
        if(!bDebug){
            bDebug = true;
        } else{
            bDebug = false;
        }
    }
    
    if(key == 'b'){
        pointCloud.setBackgroundSubstract(&kinect);
    }
    
    if(key == 'r') {
        detectFace.tracker.reset();
        detectFace.classifier.reset();
    }
    if(key == 'e') {
        detectFace.classifier.addExpression();
    }
    if(key == 'a') {
        detectFace.classifier.addSample(detectFace.tracker);
    }
    if(key == 's') {
        detectFace.classifier.save("expressions");
    }
    if(key == 'l') {
        detectFace.classifier.load("expressions");
    }
    
    if(key == '1') {
        imgIndx = 1;
    }
    if(key == '2') {
        imgIndx = 2;
    }
    if(key == '3') {
        imgIndx = 3;
    }
    if(key == '4') {
        imgIndx = 4;
    }
    if(key == '5') {
        imgIndx = 5;
    }
    if(key == '6') {
        imgIndx = 6;
    }
    if(key == '7') {
        imgIndx = 7;
    }
    
    
}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){
    
}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){
    
}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){
    
}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){
    
}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){
    
}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y){
    
}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y){
    
}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){
    
}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){
    
}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){
    
}
