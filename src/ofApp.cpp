#include "ofApp.h"

#include "DetectBody.h"
#include "ofxCv.h"

using namespace cv;


//--------------------------------------------------------------
void ofApp::setup(){
    ofSetVerticalSync(true);
    ofSetFrameRate(30);

    ofTrueTypeFont::setGlobalDpi(72);
    scanImage.load("scan.png");
   // font.load("Roboto.ttf", 12, true, true);
    font.load("Museo_Slab.otf", 12, true, true);
    
    //csv----------------------------
    csv.loadFile(ofToDataPath("csv/csv.csv"));
    
    numDatapoints=csv.numRows-1;
    numActors = csv.numCols-1;
    
    datapoints = new DataPoint*[numDatapoints];
    actors = new Actor*[numActors];
    
    
    for(int i=1; i<csv.numCols; i++) {
        actors[i-1] = new Actor;
        string name = csv.getString(0, i);
        actors[i-1]->Name = name;
    
        actors[i-1]->pos.x = ofRandom(ofGetWidth());
        actors[i-1]->pos.y = ofRandom(ofGetHeight());

        //actors[i-1]->size = ofRandom(50,100);
        actors[i-1]->vel = ofVec2f(ofRandom(-5,5),ofRandom(-5,5));
        actors[i-1]->color = ofColor(ofRandom(80-100),ofRandom(80-100),ofRandom(100,255));
        actors[i-1]->font = &font;
        
    }
    
    for(int i=1; i<csv.numRows; i++) {
        datapoints[i-1] = new DataPoint;
        string name = csv.getString(i, 0);
        datapoints[i-1]->Name = name;
        datapoints[i-1]->pos.x = ofRandom(ofGetWidth());
        datapoints[i-1]->pos.y = ofRandom(ofGetHeight());
        
        for(int u=1; u<csv.numCols; u++) {
            int value = csv.getInt(i, u);
            if(value == 1){
                datapoints[i-1]->connections.push_back(actors[u-1]);
                actors[u-1]->size++;
            }
        }
        datapoints[i-1]->font = &font;
    }
    //csv----------------------------end
    
    
    kinect.open(false, true, 0, 2); // GeForce on MacBookPro Retina
    
    kinect.start();
    
    pointCloud.setup();
    
    
    detectBody.setup(512, 424, nearThreshold, farThreshold);
    
    depthShader.load("depthshader/shader");
    scanner.load("depthshader/scan/shader");
    renderPC.allocate(ofGetWidth(),ofGetHeight());
    scanRender.allocate(ofGetWidth(),ofGetHeight());
    
    // Setup GUI
    imageSetup.setName("imageSetup");
    imageSetup.add(nearThreshold.set("nearThreshold", 10, 0, 8000));
    imageSetup.add(farThreshold.set("farThreshold", 25, 0, 8000));
    imageSetup.add(edge.set("edge", 0, 0, 524));
    imageSetup.add(topDepth.set("topDepth", 0, 0, 8000));
    imageSetup.add(botDepth.set("botDepth", 0, 0, 8000));
    imageSetup.add(edgeDepth.set("edgeDepth", 0, 0, 8000));
    
    
  
    pointCloudSetup.setName("pointCloudSetup");
    pointCloudSetup.add(tilt.set("tilt", 0, -7, 7));
    pointCloudSetup.add(translateX.set("translateX", -512, -1000, 1000));
    pointCloudSetup.add(translateY.set("translateY", -512, -3000, 1000));
    pointCloudSetup.add(translateZ.set("translateZ", -1000, -1000, 2000));

    testParams.setName("testParams");
    testParams.add(test1.set("test1", 0, 0,300));
    testParams.add(test2.set("test2", 0, 0, 300));
    
    testParams.add(bPosXlow.set("bPosXlow", 0, 0, ofGetWidth()));
    testParams.add(thresX.set("thresX", 0, 0, ofGetWidth()));
    
    testParams.add(bPosYlow.set("bPosYlow", 0, 0, ofGetHeight()));
    testParams.add(thresY.set("thresY", 0, 0, ofGetHeight()));

    testParams.add(floor.set("floor", 0, -8000, 8000));
    
    
    paramters.add(testParams);
    paramters.add(imageSetup);
    paramters.add(pointCloudSetup);
    
    gui.setup(paramters);
    gui.loadFromFile("settings.xml");

    
}

//--------------------------------------------------------------
void ofApp::update(){

    
    pointCloud.tilt = tilt;
    pointCloud.translateX = translateX;
    pointCloud.translateY = translateY;
    pointCloud.translateZ = translateZ;
    pointCloud.floor = floor;
    
    detectPerson();
    timeLine();
    
    renderPC.begin();
    ofClear(0);
    ofBackgroundGradient(ofColor(50), ofColor(0),OF_GRADIENT_CIRCULAR);
    
    
    ofSetColor(255);
    pointCloud.draw();
    if(scanUp||freeze){
        for(int i = 0; i< numDatapoints;i++ ){
            if(datapoints[i]->isSet && datapoints[i]->pos.y > scanLine){
                datapoints[i]->con = true;
                datapoints[i]->draw();
            }
        }
    }
    for(int i = 0; i<numActors;i++ ){
        actors[i]->draw();
    }

    
    renderPC.end();
    
    // if(scanDown||scanUp){
   
    //}
    
    
    
    
    

    
    if(updatePC){
        kinect.update();
        if (kinect.isFrameNew()) {
            
            
            pointCloud.update(&kinect, detectBody.getbodys());
            
            
            // kinect.getColorPixelsRef().allocate(1920, 1080, 3);
            
            kinect.getDepthPixelsRef().allocate(512, 424, 1);
            
            depthTex.loadData(kinect.getDepthPixelsRef(), GL_LUMINANCE);
            depthTex.setTextureMinMagFilter(GL_NEAREST, GL_NEAREST);
            
            depthFbo.allocate(512, 424, GL_R16);
            
            depthFbo.begin();
            depthShader.begin();
            
            depthShader.setUniform1f("nearThreshold", nearThreshold);
            depthShader.setUniform1f("farThreshold", farThreshold);
            depthShader.setUniform1f("topDepth", topDepth);
            depthShader.setUniform1f("topEdgeDepth", topEdgeDepth);
            depthShader.setUniform1f("botDepth", botDepth);
            depthShader.setUniform1f("edgeDepth", edgeDepth);
            depthShader.setUniform1f("edge", edge);
            depthShader.setUniform2f("u_resolution", 512,424);
            
            depthTex.draw(0, 0, 512, 424);
            depthShader.end();
            depthFbo.end();
            
            ofShortPixels depthPix;
            depthFbo.readToPixels(depthPix);
            
            ofImage depthImage;
            depthImage.setFromPixels(depthPix);
        
            detectBody.update(ofxCv::toCv(depthImage));
            
        }
    }
    



    
    //ofPushMatrix();
    
    

    

    detectBody.test1 = test1;
    detectBody.test2 = test2;
    
    for(int i = 0; i< numActors;i++){
        for(int j = 0; j< numActors;j++){
            int minDist =actors[i]->size + actors[j]->size;
            int x1 = actors[i]->pos.x;
            int x2 = actors[j]->pos.x;
            int y1 = actors[i]->pos.y;
            int y2 = actors[j]->pos.y;
            
            int dist = ofDist(x1,y1,x2,y2);
            
            if(dist<minDist){
                
//                ofVec2f v =actors[j]->vel;
//                actors[j]->vel = actors[i]->vel;
//                actors[i]->vel = v;
                
                ofVec2f collision = actors[i]->pos - actors[j]->pos;
                double distance = collision.length();
                if (distance == 0.0) {              // hack to avoid div by zero
                    collision = ofVec2f(1.0, 0.0);
                    distance = 1.0;
                }
            
                // Get the components of the velocity vectors which are parallel to the collision.
                // The perpendicular component remains the same for both fish
                collision = collision / distance;
                double aci = actors[i]->vel.dot(collision);
                double bci =actors[j]->vel.dot(collision);
                
                // Solve for the new velocities using the 1-dimensional elastic collision equations.
                // Turns out it's really simple when the masses are the same.
                double acf = bci;
                double bcf = aci;
                
                // Replace the collision velocity components with the new ones
                //actors[i]->vel += (acf - aci) * collision;
                actors[j]->vel += (bcf - bci) * collision;
            }
        }
    }

    
}

//--------------------------------------------------------------
void ofApp::draw(){
    
    
    
    
    
    
    
    renderPC.getTexture().bind();

    scanner.begin();
    
    
    
    scanner.setUniform2f("u_resolution", ofGetWidth(),ofGetHeight());
    scanner.setUniform1f("scanline", ofGetFrameNum()%ofGetHeight());
    // scanner.setUniformTexture("tex", renderPC.getTexture(),0);

    ofSetColor(255);
    ofFill();
    //    ofDrawRectangle(0,0,ofGetWidth(),ofGetHeight());

    renderPC.draw(0, 0);
    scanner.end();

    renderPC.getTexture().unbind();

    

    
    
    
    if(bDebug){
        
        ofNoFill();
        detectPerson();
        ofDrawRectangle(bPosXlow, bPosYlow,thresX, thresY);
        
        
        detectBody.drawProcess(10, 300, 512/2, 424/2, imgIndx);
        detectBody.drawOverlay(10, 300, 512/2, 424/2);
        ofSetColor(0,255,0);
        ofDrawLine(10+edge/2, 300, 10+edge/2, 300+424/2);
       
        
        ofDrawBitmapString(ofToString(detectBody.getbodys().size()), 500,10);
        gui.draw();
        
        ofxCv::drawHighlightString(
                                   string() +
                                   "d - debugMode\n" +
                                   "b - removeBG PC\n" +
                                   "r - reset\n" +
                                   "e - add expression\n" +
                                   "a - add sample\n" +
                                   "s - save expressions\n"
                                   "l - load expressions",
                                   14, ofGetHeight() - 10 * 12);
        
        ofxCv::drawHighlightString(string() +
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

    }
    
    
};


//--------------------------------------------------------------


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
    
    if(key == 'p'){
        
        setPositions = true;
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
    actors[setPoints]->pos = ofVec2f(x,y);
    setPoints ++ ;
    setPoints = setPoints%numActors;
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

//--------------------------------------------------------------
void ofApp::positions(){
    
    
    for(int i = 0; i<numDatapoints;i++){
        datapoints[i]->isSet = false;
    }
    
    cv::Mat contourPCMat;
    contourPCMat = cv::Mat::zeros( cvSize(ofGetWidth(),ofGetHeight()), CV_8U );
    ofPixels imagePC;
    renderPC.readToPixels(imagePC);
    ofxCv::toCv(imagePC).convertTo(contourPCMat, CV_8UC1);
    
    int closingNum = 10;
    bitwise_not(contourPCMat, contourPCMat);
    erode(contourPCMat, contourPCMat, cv::Mat(), cv::Point(-1,-1), closingNum);
    bitwise_not(contourPCMat, contourPCMat);
    ofxCv::ContourFinder contourFindPC;
    contourFindPC.setSortBySize(true);
    contourFindPC.findContours(contourPCMat);
    
    if(contourFindPC.getPolylines().size()>0){
        
        contourPC = contourFindPC.getPolyline(0);

        
        int h = contourPC.getBoundingBox().height/2;
        int w = contourPC.getBoundingBox().width/2;
        int a = contourPC.getArea();
        
        int stepX = sqrt((w*w) / numDatapoints);
        int stepY = sqrt((h*h) / numDatapoints);
        //float scale = contourPoly[0].getArea()/100;
        
        int dp = 0;
        int step =1;
        //min step in y so they dont overlap
        if(stepY<10){stepY = 20;}
        for(int y = 0 ; y < ofGetHeight() ; y+=stepY){
            
            step = font.getStringBoundingBox(datapoints[dp]->Name, 0, 0).width+20;
            if(stepX<step){stepX = step;}
            
            for(int x = 0 ; x < ofGetWidth() ; x+=stepX){
                
                ofPoint p = ofPoint(x,y);
                
                if(contourPC.inside(p)){
                    datapoints[dp]->pos.x = x;
                    if(dp%2==0){
                    datapoints[dp]->pos.y = y + stepY/2;
                    }else{
                    datapoints[dp]->pos.y = y;
                    }
                    datapoints[dp]->isSet = true;
                    dp ++;
                    dp = dp%numDatapoints;
                }
            }
        }
    }
}
//--------------------------------------------------------------


void ofApp::detectPerson(){
    
    if(detectBody.getbodys().size()>0){
        ofPolyline p = detectBody.getbodys()[0].boundaryPoly;
        if(p.getBoundingBox().width > p.getBoundingBox().height){
            if(detectBody.getbodys()[0].centroid.x >bPosXlow &&
               detectBody.getbodys()[0].centroid.x <bPosXlow+thresX &&
               detectBody.getbodys()[0].centroid.y >bPosYlow &&
               detectBody.getbodys()[0].centroid.y <bPosYlow+thresY
               ){
                isPersonPresent = true;
            }else{
                isPersonPresent = false;
                
            }
            
            
        }
        if(bDebug){
            ofDrawEllipse(detectBody.getbodys()[0].centroid,20,20);
        }
        
    }
}
//--------------------------------------------------------------
void ofApp::timeLine(){
    //beginning
    if(!isPersonPresent){
        isPPtimer=0;
    }
    
    //if person is present for more than 100 frames -> start scanDown
    if(isPersonPresent && !active){
        isPPtimer++;
        if(isPPtimer>isPPthres){
            scanDown = true;
            active = true;
        }
    }
    
    // if scanline is down, scan go up.
    if(scanDown){
        //        for(int i = 0; i<numActors;i++){
        //            actors[i]->hasConnection = false;
        //        }
        float x = abs (scanLine - ofGetHeight())/100;
        //  scanLine = scanLine*scanLine  ;
        // cout<< scanLine <<endl;
        scanLine +=5;
        if(scanLine > ofGetHeight()){
            scanUp = true;
            scanDown = false;
            positions();
        }
    }
    
    
    // scan up- set datapoints positions, freese PC (updatePC) // when up, keep datapoints (freeze)
    if(scanUp){
        updatePC = false;
        float x = abs (ofGetHeight() - scanLine)/100;
        scanLine -=5;
        if(scanLine < 0){
            scanUp = false;
            freeze = true; // let dataPoints stay even if scanUp = false
            ending = true;
            pointCloud.vel.clear();
            pointCloud.collapse = ofGetHeight()/2;
            
        }
    }
    
    //ending (hold for endTimerThres amount of time
    if(!ending){
        endTimer = 0;
    }
    
    if(ending){
        endTimer++;
        if(endTimer>endTimerThres){
            
            pointCloud.fall();
            for(int i = 0; i<numDatapoints;i++){
                datapoints[i]->fall=true;
            }
            if(datapoints[0]->tl<-1){
                resetAll = true;
            }
            // endAnimation = true;
            // resetAll;
        }
    }
    
    if(resetAll || !isPersonPresent){
        scanLine = 0;
        scanUp = false;
        scanDown = false;
        freeze = false;
        updatePC = true;
        setPositions = false;
        isPersonPresent = false;
        isPPtimer = 0;
        ending = false;
        endTimer = 0;
        endAnimation = false;
        resetAll = false;
        for(int i = 0; i< numDatapoints;i++){
            datapoints[i]->con = false;
            datapoints[i]->tl = 0;
            datapoints[i]->fall = false;
        }
        active = false;
        //datapoints->shuffle();
    }
}
