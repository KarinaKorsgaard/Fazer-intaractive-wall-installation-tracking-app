

#include "ofApp.h"

#include "DetectBody.h"
#include "ofxCv.h"



using namespace cv;

//static bool shouldRemove(shared_ptr<ofxBox2dBaseShape>shape) {
//    return !ofRectangle(0, -400, ofGetWidth(), ofGetHeight()+400).inside(shape->getPosition());
//}

//--------------------------------------------------------------
void ofApp::setup(){

    ofSetVerticalSync(false);

    depthFbo.allocate(K_W, K_H, GL_R16);
    depthFbo.begin();
    ofClear(0);
    depthFbo.end();
    
    contourRender.allocate(K_W + 100, K_H + 100, GL_R16);
    contourRender.begin();
    ofClear(0);
    contourRender.end();
    
    contourFinder.setSimplify(true);
    //contourFinder.setTargetColor(ofColor::white);
    
    kinect.open(); //
    
    //kinect.start();
    
    
    detectBody.setup(K_W, K_H, nearThreshold, farThreshold);
    
    depthShader.load("depthshader/shader");
    
    // Setup GUI
    ofParameterGroup general;
    general.setName("general");
    general.add(appId.set("appId",1,1,4));
    general.add(blobMin.set("blobMin",200,1,10000));
    general.add(blobMax.set("blobMax",800,1,900000));
    
    general.add(resample.set("resampleSpacing",3,1,30));
    
    imageSetup.setName("imageSetup");
    imageSetup.add(nearThreshold.set("nearThreshold", 10, 0, 8000));
    imageSetup.add(farThreshold.set("farThreshold", 25, 0, 8000));
    imageSetup.add(edge.set("edge", 0, 0, 524));
    imageSetup.add(topDepth.set("topDepth", 0, 0, 8000));
    imageSetup.add(botDepth.set("botDepth", 0, 0, 8000));
    imageSetup.add(edgeDepth.set("edgeDepth", 0, 0, 8000));
    
    
    pointCloudSetup.setName("pointCloudSetup");
    pointCloudSetup.add(tilt.set("tilt", 0, -7, 7));
    pointCloudSetup.add(tiltX.set("pointsize", 0, 0, 30));
    pointCloudSetup.add(tiltY.set("smoothe", 0, 0, 30));
    pointCloudSetup.add(tiltZ.set("closingNum", 1, 0, 10));
    pointCloudSetup.add(translateX.set("translateX", -512, -10000, 10000));
    pointCloudSetup.add(translateY.set("translateY", -512, -30000, 10000));
    pointCloudSetup.add(translateZ.set("translateZ", -2000, -20000, 20000));
    
    paramters.add(general);
    paramters.add(pointCloudSetup);
    paramters.add(imageSetup);
    
    
    gui.setup(paramters);
    gui.loadFromFile("settings.xml");
    
    
    int portNr = PORT;
    
    //169.254.44.160
    ofxXmlSettings xml;
    
    if( xml.loadFile("ip.xml") ){
        cout<< "mySettings.xml loaded!";
    }else{
        cout<< "unable to load ip.xml check data/ folder";
    }
    xml.pushTag("document");
    string ip = xml.getValue("ip", "");
    
    sender.setup("localhost",PORT+appId);
    sender.enableBroadcast();
    localSender.setup("localhost",4321);
    message = "port: "+ ofToString(PORT+appId) + " "+ip;
    
    ofSetFrameRate(30);

    

}



//--------------------------------------------------------------
void ofApp::update(){
    
    
    counter ++;
    kinect.update();
    
    if (kinect.isFrameNew()) {
        counter = 0;
        
        kinect.getRawDepthPixels().allocate(K_W, K_H, 1);
        
        depthTex.loadData(kinect.getRawDepthPixels(), GL_LUMINANCE);
        depthTex.setTextureMinMagFilter(GL_NEAREST, GL_NEAREST);
        
        depthFbo.begin();
        ofClear(0);
        depthShader.begin();
        
        depthShader.setUniform1f("nearThreshold", nearThreshold);
        depthShader.setUniform1f("farThreshold", farThreshold);
        depthShader.setUniform1f("topDepth", topDepth);
        depthShader.setUniform1f("topEdgeDepth", topEdgeDepth);
        depthShader.setUniform1f("botDepth", botDepth);
        depthShader.setUniform1f("edgeDepth", edgeDepth);
        depthShader.setUniform1f("edge", edge);
        depthShader.setUniform2f("u_resolution", K_W, K_H);
        
        depthTex.draw(0, 0, K_W, K_H);
        
        depthShader.end();
        
        depthFbo.end();
        
        ofShortPixels depthPix;
        depthFbo.readToPixels(depthPix);
        ofImage depthImage;
        depthImage.setFromPixels(depthPix);
        
        // Detect if there are objects in the Space
        detectBody.update(ofxCv::toCv(depthImage), &kinect);
        detectBody.setTresholds(nearThreshold, farThreshold, tilt);
        
        mesh.clear();
        vector<Body>bodyPolys = detectBody.getBodies(); // sort by farthest away ->first. ???
        for(int i = 0 ; i<bodyPolys.size();i++){ // get the body polys to pass for the pointcloud
            
            //ofColor col = ofColor(ofRandom(255),ofRandom(255),ofRandom(255));
            int indx = 0;
            
            
            
            for(vector<ofVec2f>::iterator ind=bodyPolys[i].indices.begin(); ind!=bodyPolys[i].indices.end(); ind++){
                indx++;
                if(indx%2 == 0){
                    mesh.addVertex(ofVec3f(ind->x+translateX,ind->y+translateY,-3000));
                }
            }
        }
        
        
        
        contourRender.begin();
        
        ofClear(0);
        ofBackground(0);
        ofSetColor(255);
        glPointSize(tiltX);
        mesh.drawVertices();
        contourRender.end();
        
        ofPixels imagePC;
        contourRender.readToPixels(imagePC);
        contourFinder.setMinArea(blobMin);
        contourFinder.setMaxArea(blobMax);
        
        people2D.clear();
        
        cv::Mat contourPCMat;
        contourPCMat = cv::Mat::zeros( cvSize(RES_WIDTH,RES_HEIGHT), CV_8U );
        
        if(contourRender.isAllocated()){
            
            ofxCv::toCv(imagePC).convertTo(contourPCMat, CV_8UC1);
            
            int closingNum = tiltZ;
            bitwise_not(contourPCMat, contourPCMat);
            erode(contourPCMat, contourPCMat, cv::Mat(), cv::Point(-1,-1), closingNum);
            bitwise_not(contourPCMat, contourPCMat);
            
            if(tiltY>0)ofxCv::blur(contourPCMat, tiltY);
            
            if(contourPCMat.size>0)contourFinder.findContours(contourPCMat);
            
            if(bDebug){
                ofxCv::toOf(contourPCMat, contourDetectImg);
                contourDetectImg.update();
            }
            
            for(int i = 0 ; i<contourFinder.getPolylines().size();i++){
                ofPolyline poly = contourFinder.getPolyline(i);
                poly = poly.getResampledBySpacing(resample);
                
                people2D.push_back(poly);
            }
        }
        
        int numPeople=0;
        // if(method1)numPeople=people.size();
        numPeople=people2D.size();
        
        ofxOscMessage m;
        m.setAddress("/numPeople");
        m.addInt32Arg(numPeople);
        sender.sendMessage(m);
        
        
        for(int i = 0 ; i<numPeople;i++){
            ofxOscMessage msg;
            msg.setAddress("/"+ofToString(i));
            
            
            
            ofPolyline poly;
            poly = people2D[i];
            
            
            for(int u = 0 ; u<poly.size();u++){
                msg.addFloatArg(poly.getVertices()[u].x);
                msg.addFloatArg(poly.getVertices()[u].y);
            }
            sender.sendMessage(msg);
        }
    }
    
    ofDrawBitmapString(message, 10, 900);
    //  getBodyPolys(contourRender);
    if(counter>100){
        ofxOscMessage msg;
        msg.setAddress("/shutdown");
        localSender.sendMessage(msg);
    }
    
 

}

//--------------------------------------------------------------
void ofApp::draw(){
    
    if(bDebug){
        
        
        ofEnableAlphaBlending();
        
        ofPushMatrix();
        ofTranslate(0,0);
        
        if(contourDetectImg.isAllocated())contourDetectImg.draw(0, 0);
       // if(contourRender.isAllocated())contourRender.draw(0,0);
        for(int u= 0; u<people2D.size();u++){
            ofSetColor(ofColor::red);
            people2D[u].draw();
            
            ofSetColor(ofColor::white);
            for(int i = 0; i< people2D[u].getVertices().size();i++){
                ofDrawCircle(people2D[u].getVertices()[i],2);
            }

        }
        
        
        ofPopMatrix();
        
        ofPushMatrix();
        ofTranslate(512, 0);
        detectBody.drawProcess(0,0,K_W,K_H,imgIndx);
        detectBody.drawOverlay(0,0,K_W,K_H);
        ofSetColor(ofColor::green);
        ofDrawLine(0, edge, K_W, edge);
        ofPopMatrix();
        
        gui.draw();
        ofDrawBitmapString(message,10,gui.getHeight()+50);
    }
    
    ofSetWindowTitle("FrameRate: "+ ofToString(ofGetFrameRate()));
    
    
    
    
};


//--------------------------------------------------------------


float ofApp::getAvgDepth(ofRectangle space, ofxKinectV2 *kinect){
    int div = 0;
    float avgDepth = 0;
    for(int y = space.y; y < space.y+space.height; y ++) {
        for(int x = space.x; x < space.x+space.width; x ++) {
            avgDepth += kinect->getWorldCoordinateAt(x, y).z;
            div++;
            
        }
    }
    return avgDepth/div;
    
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
    
    if(key == 'd') {
        bDebug = !bDebug;
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


