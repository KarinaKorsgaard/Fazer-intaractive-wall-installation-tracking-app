#include "ofApp.h"

#include "DetectBody.h"
#include "ofxCv.h"

using namespace cv;


//--------------------------------------------------------------
void ofApp::setup(){
    resetAll = true;
    
    ofSetVerticalSync(true);
    ofSetFrameRate(30);
    
    ofTrueTypeFont::setGlobalDpi(72);
    scanImage.load("scan.png");
    // font.load("Roboto.ttf", 12, true, true);
    font.load("Calibre/Calibre-Semibold.otf", 13, true, true);
    mainRender.allocate(800,1280);
    mainRender.begin();
    ofClear(0);
    mainRender.end();
    //csv----------------------------
    csv.loadFile(ofToDataPath("csv/csvALLCAPS.csv"));
    
    numDatapoints=csv.numRows-1;
    numActors = csv.numCols-1;
    
    datapoints = new DataPoint*[numDatapoints];
    actors = new Actor*[numActors];
    
    
    for(int i=1; i<csv.numCols; i++) {
        actors[i-1] = new Actor;
        string name = csv.getString(0, i);
        actors[i-1]->Name = name;
        string str;
        
            str = "logo/"+name+".png";
            if(ofIsStringInString(name, "TWITTER")){
                str = "logo/TWITTER.png";
            }
        actors[i-1]->img.load(str);
        
        string str2;
            str2 = "logo/UC/"+name+".png";
            if(ofIsStringInString(name, "TWITTER")){
                str2 = "logo/UC/TWITTER.png";
            }
        actors[i-1]->img2.load(str2);
        
        actors[i-1]->img.load(str);
        actors[i-1]->pos.x = ofRandom(RES_WIDTH);
        actors[i-1]->pos.y = ofRandom(RES_HEIGHT);
        
        //actors[i-1]->size = ofRandom(50,100);
        actors[i-1]->vel = ofVec2f(0,0);
        actors[i-1]->color = ofColor(ofRandom(80-100),ofRandom(80-100),ofRandom(100,255));
        actors[i-1]->font = &font;
        
    }
    
    for(int i=1; i<csv.numRows; i++) {
        datapoints[i-1] = new DataPoint;
        string name = csv.getString(i, 0);
        datapoints[i-1]->Name = name;
        datapoints[i-1]->pos.x = ofRandom(RES_WIDTH);
        datapoints[i-1]->pos.y = ofRandom(RES_HEIGHT);
        
        for(int u=1; u<csv.numCols; u++) {
            int value = csv.getInt(i, u);
            if(value == 1){
                datapoints[i-1]->connections.push_back(actors[u-1]);
                actors[u-1]->size+=2;
                
            }
        }
        datapoints[i-1]->font = &font;
    }
    //csv----------------------------end
    
    
    kinect.open(); // GeForce on MacBookPro Retina
    
    //kinect.start();
    
    pointCloud.setup();
    
    
    detectBody.setup(512, 424, nearThreshold, farThreshold);
    
    depthShader.load("depthshader/shader");
    scanner.load("depthshader/scan/shader");
    renderPC.allocate(RES_WIDTH,RES_HEIGHT);
    scanRender.allocate(RES_WIDTH,RES_HEIGHT);
    
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
    pointCloudSetup.add(translateZ.set("translateZ", -2000, -2000, 2000));
    
    testParams.setName("testParams");
    
    testParams.add(bPosXlow.set("bPosXlow", 0, 0, RES_WIDTH));
    testParams.add(thresX.set("thresX", 0, 0, RES_WIDTH));
    
    testParams.add(bPosYlow.set("bPosYlow", 0, 0, RES_HEIGHT));
    testParams.add(thresY.set("thresY", 0, 0, RES_HEIGHT));
    
    testParams.add(floor.set("floor", 0, -8000, 8000));
    
    
    paramters.add(testParams);
    paramters.add(imageSetup);
    paramters.add(pointCloudSetup);
    
    gui.setup(paramters);
    gui.loadFromFile("settings.xml");
    syphon.setName("DAC Virtual Mirror");
    
    
}

//--------------------------------------------------------------
void ofApp::update(){
    
//    //does not work!!
//    if(counter>30*5){
//        kinect.close();
//    }
//    if(counter>30*10 && !kinect.isThreadRunning()){
//        //kinect.open(); // GeForce on MacBookPro Retina
//        //sleep(300);
//    }
//    // ---

    pointCloud.tilt = tilt;
    pointCloud.translateX = translateX;
    pointCloud.translateY = translateY;
    pointCloud.translateZ = translateZ;
    pointCloud.floor = floor;
    
    detectPerson();
    timeLine();

    for(int i = 0; i< numActors;i++){
        actors[i]->update();
        actors[i]->rect = &thePerson;
        for(int j = i+1; j< numActors;j++){
            actors[i]->checkCollision(actors[j]);
        }
    }
    
    //update kinect
    counter ++;
    kinect.update();
    if (kinect.isFrameNew()) {
        counter = 0;
        if(updatePC){
            pointCloud.update(&kinect, detectBody.getbodys());
        }
        kinect.getRawDepthPixels().allocate(512, 424, 1);
        
        depthTex.loadData(kinect.getRawDepthPixels(), GL_LUMINANCE);
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


    
    //pointcloud render
    int total = (int)pointCloud.mesh.getVertices().size();
    pointCloud.vbo.setVertexData(&pointCloud.mesh.getVertices()[0], total, GL_STATIC_DRAW);
    renderPC.begin();
    ofClear(0);
    //ofBackgroundGradient(ofColor(50), ofColor(0),OF_GRADIENT_CIRCULAR);
    ofBackground(0);
    ofSetColor(255);
    pointCloud.drawPSpline();
    ofSetColor(255);
    renderPC.end();

    
    
    //main render
    mainRender.begin();
    ofClear(0);
    renderPC.getTexture().bind();
    scanner.begin();
    scanner.setUniform2f("u_resolution", 800.,1280.);
    scanner.setUniform1f("scanline", 1280.-scanLine);
    // scanner.setUniformTexture("tex", renderPC.getTexture(),0);
    
    ofSetColor(255);
    ofFill();
    //    ofDrawRectangle(0,0,RES_WIDTH,RES_HEIGHT);
    renderPC.draw(0, 0);
    scanner.end();
    //renderPC.draw(0, 0);
    renderPC.getTexture().unbind();
    
    if(scanUp||freeze){
        for(int i = 0; i< numDatapoints;i++ ){
            if(datapoints[i]->isSet && datapoints[i]->pos.y > scanLine){
                datapoints[i]->alpha = true;
                datapoints[i]->draw();
            }
        }
    }
    
    for(int i = 0; i<numActors;i++ ){
        if(circleLogo){
            actors[i]->draw();
        }else{
            actors[i]->drawUC();
        }
        
    }
    
    mainRender.end();
    
}

//--------------------------------------------------------------
void ofApp::draw(){
    
    
    mainRender.draw(0,0);
    ofSetColor(255);
    ofFill();
    
    ofFill();
    syphon.publishTexture(&mainRender.getTexture());
    
    
    
    
    if(bDebug){
        
        ofNoFill();
       
        
        ofDrawRectangle(bPosXlow+gui.getWidth(), bPosYlow,thresX, thresY);
        if(detectBody.getbodys().size()>0){
        ofDrawEllipse(detectBody.getbodys()[0].centroid.x + +gui.getWidth(), detectBody.getbodys()[0].centroid.y,20,20);
        }
        
        detectBody.drawProcess(10, 300, 512/2, 424/2, imgIndx);
        detectBody.drawOverlay(10, 300, 512/2, 424/2);
        ofSetColor(0,255,0);
        ofDrawLine(10+edge/2, 300, 10+edge/2, 300+424/2);
        ofSetColor(255);
        
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
                                   14, RES_HEIGHT - 10 * 12);
        
        ofxCv::drawHighlightString(string() +
                                   "index of the detection Image\n" +
                                   "1 depthImage\n" +
                                   "2 depthImageCanny\n" +
                                   "3 depthImageErodedArms\n" +
                                   "4 armDivider\n" +
                                   "5 armEdges\n" +
                                   "6 activeAreaMask\n" +
                                   "7 inputImage\n",
                                   14*15, RES_HEIGHT - 10 * 12);
        
        
        ofSetWindowTitle("FrameRate: "+ ofToString(ofGetFrameRate()));
        ofSetColor(255,0,0);
        font.drawString("frameNum since last update from Kinect:"+ofToString(counter),gui.getWidth()+10,20);
        
    }
    
 
    
};


//--------------------------------------------------------------


float ofApp::getAvgDepth(ofRectangle space, ofxKinectV2 *kinect){
    int div = 0;
    float avgDepth = 5000;
    for(int y = space.y; y < space.y+space.height; y ++) {
        for(int x = space.x; x < space.x+space.width; x ++) {
          //  avgDepth += kinect->getDistanceAt(x, y);
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
    if(key == 's'){
        circleLogo = !circleLogo;
    }
    
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

//--------------------------------------------------------------
void ofApp::positions(){
    
    //set datapoints positions when scanLine is down
    for(int i = 0; i<numDatapoints;i++){
        datapoints[i]->isSet = false;
    }
    
    //get a polyline
    contourPC = getBodyPoly();
    
    //not used atm
    //int h = contourPC.getBoundingBox().height/3;
    //int w = contourPC.getBoundingBox().width/3;
    
    int stepXThis=1;
    int stepX = 1;//sqrt((w*w) / numDatapoints);
    int stepY = 25;//sqrt((h*h) / numDatapoints);
    
    //make data appear in random order
    int dp = ofRandom(1,numDatapoints-1);
    
    //vector to store values that has been set
    vector<int>dpRandom;
    
    //int step =1;
    //if(stepY<25){stepY = 25;}
    
    for(int y = 0 ; y < RES_HEIGHT ; y+=stepY){
        for(int x = 0 ; x < RES_WIDTH ; x+= stepX + stepXThis){
            //set step to 2
            stepXThis = 2;//font.getStringBoundingBox(datapoints[dp]->Name, 0, 0).width;
            stepX = 2;
            ofPoint p = ofPoint(x,y);
            if(contourPC.inside(p)){
                //if point is inside, set stepX
                stepXThis = font.getStringBoundingBox(datapoints[dp]->Name, 0, 0).width;

                datapoints[dp]->pos.x = x;
                datapoints[dp]->pos.y = y;
                datapoints[dp]->isSet = true;
                
                //find new number that has not been set already
                dpRandom.push_back(dp);
                bool breakWhile = false;
                
                if(dpRandom.size() < numDatapoints){
                    while(!breakWhile){
                        dp = ofRandom(numDatapoints);
                        bool found = true;
                        for(int i = 0; i < dpRandom.size();i++){
                            if(dp == dpRandom[i]){
                                found = false;
                            }
                        }
                        if(found ){
                            breakWhile = true;
                        }
                    }
                }
                
                //not sure why, but I need to move the point both the prev and current to make sure they dont overlap.
                stepX = font.getStringBoundingBox(datapoints[dp]->Name, 0, 0).width;
                
            }
        }
    }
}

//--------------------------------------------------------------
void ofApp::detectPerson(){
    
    if(detectBody.getbodys().size()>0){
        ofPolyline p = detectBody.getbodys()[0].boundaryPoly;
        //bounding box and center does not equal getBodyPoly - not translated
        
        //analysing largest blob boundaryPoly
        if(p.getBoundingBox().width > p.getBoundingBox().height){
            if(detectBody.getbodys()[0].centroid.x >bPosXlow &&
               detectBody.getbodys()[0].centroid.x <bPosXlow+thresX &&
               detectBody.getbodys()[0].centroid.y >bPosYlow &&
               detectBody.getbodys()[0].centroid.y <bPosYlow+thresY
               ){
                isPersonPresent = true;
                contourPC = getBodyPoly();
                //make rect of boundingbox to pass to actors
                thePerson = ofRectangle(contourPC.getBoundingBox().x,contourPC.getBoundingBox().y, contourPC.getBoundingBox().width, contourPC.getBoundingBox().height);
            }
        }
        
        else{
            isPersonPresent = false;
            thePerson  = ofRectangle(0,0,0,0);
        }
    }
}
//--------------------------------------------------------------
void ofApp::timeLine(){
    
    //if person is present for more than 100 frames -> start scanDown
    if(isPersonPresent && !active){
        isPPtimer++;
        if(isPPtimer>isPPthres){
            scanDown = true;
            active = true;
        }
    }
    //reset timer is person leaves
    if(!isPersonPresent){
        isPPtimer=0;
    }
    
    // if scanline is down, scan go up.
    if(scanDown){
        scanLine +=15;
        if(scanLine > RES_HEIGHT){
            scanUp = true;
            scanDown = false;
            positions();
        }
    }
    
    // if person leaves while scanning down, reset all.
    if(scanDown && !isPersonPresent){
        resetAll = true;
    }
    
    // scan up- set datapoints positions, freese PC (updatePC) // when up, keep datapoints (freeze)
    if(scanUp){
        updatePC = false;
        scanLine -=5;
        if(scanLine < 0){
            for(int i = 0; i<numDatapoints;i++){
                // length ->start length counter. 
                datapoints[i]->length = true;
            }
            scanUp = false;
            freeze = true; // let dataPoints stay even if scanUp = false
            ending = true;
            pointCloud.vel.clear();
            pointCloud.collapse = RES_HEIGHT/2;
        }
    }
    
    //ending - fall after 10 sec or if person leaves. After fall, reset all
    if(ending){
        endTimer++;
        if(endTimer>endTimerThres || !isPersonPresent){
            startFall = true;
            ending = false;
        }
    }
    
    // start fall after endTimer or person leave. -
    if(startFall){
        lastTimer ++;
        pointCloud.fall();
        
        for(int i = 0; i< numActors;i++){
            actors[i]->hasConnection = false;
        }
        for(int i = 0; i<numDatapoints;i++){
            datapoints[i]->fall=true;
        }
        if(lastTimer >lastTimerThres){
            resetAll = true;
        }
    }
    
    //reset all timers, bools and datapoints
    if(resetAll ){
        lastTimer = 0; 
        endTimer = 0;
        scanLine = 0;
        scanUp = false;
        scanDown = false;
        freeze = false;
        updatePC = true;
        setPositions = false;
        isPPtimer = 0;
        ending = false;
        endTimer = 0;
        endAnimation = false;
        resetAll = false;
        startFall = false;
        for(int i = 0; i< numDatapoints;i++){
            datapoints[i]->length = false;
            datapoints[i]->alpha = false;
            datapoints[i]->tlAlpha = 0;
            datapoints[i]->tlLength = 0;
            datapoints[i]->fall = false;
        }
        active = false;

    }
}

//--------------------------------------------------------------
ofPolyline ofApp::getBodyPoly(){
    //finds the polyline of the shown image
    ofPolyline p;
    
    if(detectBody.getbodys().size()>0){
        ofPolyline p = detectBody.getbodys()[0].boundaryPoly;
        if(p.getBoundingBox().width > p.getBoundingBox().height){
            
            
            cv::Mat contourPCMat;
            contourPCMat = cv::Mat::zeros( cvSize(RES_WIDTH,RES_HEIGHT), CV_8U );
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
                
                p = contourFindPC.getPolyline(0);
            }
        }
    }
    return p;
}
