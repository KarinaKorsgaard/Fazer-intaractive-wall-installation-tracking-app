#include "ofApp.h"

#include "DetectBody.h"
#include "ofxCv.h"


using namespace cv;


//--------------------------------------------------------------
void ofApp::setup(){
    ofSetVerticalSync(true);
    ofSetFrameRate(30);
    ofEnableAntiAliasing();
    
    sender.setup(HOST,PORT);
    
    // LOAD FONTS & IMAGES
    ofTrueTypeFont::setGlobalDpi(72);
    scanImage.load("scan.png");
    // font.load("Roboto.ttf", 12, true, true);
    font.load("Calibre/Calibre-Semibold.otf", 20, true, true);
    
    // INITIALISE FBO
    mainRender.allocate(800,1280);
    mainRender.begin();
    ofClear(0);
    mainRender.end();
    
    // LOAD THE DATA FROM THE CVS-FILE
    //csv----------------------------
    csv.loadFile(ofToDataPath("csv/csvALLCAPS.csv"));
    
    for(int i=1; i<csv.numCols; i++) {
        Actor actor;
        string name = csv.getString(0, i);
        actor.Name = name;
        string str;
        
        str = "logo/"+name+".png";
        if(ofIsStringInString(name, "TWITTER")){
            str = "logo/TWITTER.png";
        }
        ofImage logo;
        logo.load(str);
        actor.img = logo;
        
        str = "logo/UC/"+name+".png";
        if(ofIsStringInString(name, "TWITTER")){
            str = "logo/UC/TWITTER.png";
        }
        logo.load(str);
        actor.img2 = logo;
        
        int xPos[10] = {1, 1, 1, 1, 3, 6, 8, 8, 8, 8}; // 9 subdivisions
        int yPos[10] = {12, 9, 6, 3, 1, 1, 3, 6, 9, 12}; // 18 Subdivisions
        
        actor.pos.x = ofRandom(RES_WIDTH);
        actor.pos.y = ofRandom(RES_HEIGHT);
        
        //actors[i-1]->size = ofRandom(50,100);
        actor.vel = ofVec2f(0,0);
        actor.color = ofColor(ofRandom(80-100),ofRandom(80-100),ofRandom(100,255));
        actor.font = &font;
        actor.orgPos = ofVec2f(800/9*xPos[i], 1280/18*yPos[i]);
        actors.push_back(actor);
        
    }
    

    
    
    for(int i=1; i<csv.numRows; i++) {
        DataPoint dPoint;
        dPoint.actors = &actors;
        string name = csv.getString(i, 0);
        dPoint.Name = name;

        dPoint.pos.x = ofRandom(RES_WIDTH);
        dPoint.pos.y = ofRandom(RES_HEIGHT);
        dPoint.shooter = int(ofRandom(100));
        dPoint.font = &font;
        
        for(int u=1; u<csv.numCols; u++) {
            int value = csv.getInt(i, u);
            if(value == 1){
                dPoint.connectToActorID.push_back(u-1);
                actors[u-1].size+=2;
                
            }
        }
        datapoints.push_back(dPoint);
    }
    //csv----------------------------end
    
    pointCloud.sender = &sender;
    
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
    pointCloudSetup.add(floor.set("floor", 0, -8000, 8000));
    
    testParams.setName("testParams");
    testParams.add(bPosX.set("bPosX", 0, 0, 424));
    testParams.add(thresW.set("thresW", 0, 0, 424));
    testParams.add(bPosY.set("bPosY", 0, 0, 512));
    testParams.add(thresH.set("thresH", 0, 0, 512));
    
    paramters.add(testParams);
    paramters.add(imageSetup);
    paramters.add(pointCloudSetup);
    
    gui.setup(paramters);
    gui.loadFromFile("settings.xml");
    
    syphon.setName("DAC Virtual Mirror");
    
    resetAll = true; // fresh start
    
}

//--------------------------------------------------------------
void ofApp::update(){
    
    pointCloud.tilt = tilt;
    pointCloud.translateX = translateX;
    pointCloud.translateY = translateY;
    pointCloud.translateZ = translateZ;
    pointCloud.floor = floor;
    
    if(!debugAction){
        detectPerson();
    }
    timeLine();
    

    
    // update the actors / big players
    for(int i = 0; i< actors.size();i++){
        actors[i].update();
        actors[i].rect = &thePerson;
        for(int j = i+1; j< actors.size();j++){
            actors[i].checkCollision(&actors[j]);
        }
    }
    
    int xPos[10] = {1, 1, 1, 1, 3, 6, 8, 8, 8, 8}; // 9 subdivisions
    int yPos[10] = {12, 9, 6, 3, 1, 1, 3, 6, 9, 12}; // 18 Subdivisions
    
    if(actorsFixed){
        for(int i = 0; i< actors.size();i++){
            actors[i].pos = ofVec2f(800/9*xPos[i], 1280/18*yPos[i]);
        }
    }
    
    
    //update kinect
    counter ++;
    kinect.update();
    
    if (kinect.isFrameNew()) {
        counter = 0;
        if(updatePC){
            pointCloud.update(&kinect, detectBody.getBodies());
        }
        kinect.getRawDepthPixels().allocate(512, 424, 1);
        
        depthTex.loadData(kinect.getRawDepthPixels(), GL_LUMINANCE);
        depthTex.setTextureMinMagFilter(GL_NEAREST, GL_NEAREST);
        
        // Remove background and floor
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
        
        // Detect if there are objects in the Space
        detectBody.update(ofxCv::toCv(depthImage));
    }
    
    
    
    //pointcloud render
    int total = (int)pointCloud.mesh.getVertices().size();
    pointCloud.vbo.setVertexData(&pointCloud.mesh.getVertices()[0], total, GL_STATIC_DRAW);
    renderPC.begin();
    ofClear(0);
    ofBackground(0);
    ofSetColor(255);
    pointCloud.drawPSpline();
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
    renderPC.draw(0, 0);
    scanner.end();
    renderPC.getTexture().unbind();
    
    if(scanUp||freeze){
        for(int i = 0; i< datapoints.size();i++ ){
            if(datapoints[i].isSet   && datapoints[i].pos.y > scanLine){
                if(scanLine>0){
                    datapoints[i].bAlpha = true;
                    
                    ofxOscMessage m;
                    m.setAddress("/datapoints");
                    m.addIntArg(1);
                    sender.sendMessage(m);
                
                } // needed to make them disapear in fall
                datapoints[i].draw();
                
                
                
                
            }
        }
    }
    for(int i = 0; i<actors.size();i++ ){
        if(circleLogo){
            actors[i].draw();
        }else{
            actors[i].drawUC();
        }
        
    }
    
    if(flash && doFlash){
        if(flashTimer < flashTimerThres/3){
            ofSetColor(flashTimer*(255/flashTimerThres));
        }
        if(flashTimer < flashTimerThres/3){
            ofSetColor((flashTimerThres-flashTimer)*(255/flashTimerThres));
        }
        ofDrawRectangle(0, 0, RES_WIDTH, RES_HEIGHT);
    }
    
    mainRender.end();
    
}

//--------------------------------------------------------------
void ofApp::draw(){
    
    
    mainRender.draw(0,0);
    ofSetColor(255);
    ofFill();
    syphon.publishTexture(&mainRender.getTexture());
    
    
    
    
    if(bDebug){
    
        float scale = 0.75;
        
        // Draw the incoming depth image for the body detection.
        int xPrev = 250;
        int yPrev = 10;
        ofPushMatrix();
        ofTranslate(xPrev+424*scale, yPrev);
        ofScale(scale,scale,scale);
        ofRotateZ(90);
        
        ofNoFill();
        // draw PreviewWindow;
        detectBody.drawProcess(0, 0, 512, 424, imgIndx);
        detectBody.drawOverlay(0, 0, 512, 424);
        ofSetColor(0,255,0);
        ofDrawLine(edge, 0, edge, 424); // edge line
        ofSetColor(255);
        ofDrawRectangle(0, 0, 512, 424); // frame
        
        // draw detect Space
        ofSetColor(255, 0, 255);
        ofDrawRectangle(bPosX-thresW/2, bPosY-thresH/2, thresW, thresH);
        if(detectBody.getBodies().size()>0){
            if(isPersonPresent)ofSetColor(255, 0, 0);
            ofDrawEllipse(detectBody.getBodies()[0].centroid.x, detectBody.getBodies()[0].centroid.y,20,20);
        }
        ofPopMatrix();
        
        // show image of the erode/dilated PC
        scale = 0.25;
        ofPushMatrix();
        ofTranslate(10, 400);
        ofScale(scale,scale,scale);
        ofFill();
        ofSetColor(255);
        contourDetectImg.draw(0,0);
        if(contourPC.size()>0){
            ofNoFill();
            ofSetColor(0, 0, 255);
            contourPC.draw();
            ofSetColor(255,0,255);
            ofDrawRectangle(contourPC.getBoundingBox());
        }
        // draw frame
        ofNoFill();
        ofSetColor(255);
        ofDrawRectangle(0, 0, contourDetectImg.getWidth(), contourDetectImg.getHeight());
        ofPopMatrix();
        
        
        
        ofSetColor(255);
        
        ofDrawBitmapString(ofToString(detectBody.getBodies().size()), 500,10);
        gui.draw();
        ofDrawBitmapString("frameNum since last update from Kinect:"+ofToString(counter),14, RES_HEIGHT - 10 * 14);
        
        ofDrawBitmapString(
                           string() +
                           "d - debugMode\n" +
                           "s - switch Icons",
                           14, RES_HEIGHT - 10 * 12);
        
        ofDrawBitmapString(string() +
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
    if(key == 's'){
        circleLogo = !circleLogo;
    }
    if(key == 'a'){
        actorsFixed = !actorsFixed;
    }
    if(key == 'p' && debugAction){
        isPersonPresent = !isPersonPresent;
    }
    if(key == 'f'){
        doFlash = !doFlash;
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
void ofApp::positions(){
    
    random_shuffle(datapoints.begin(), datapoints.end()); // shuffle the points
    
    //set datapoints positions when scanLine is down
    for(int i = 0; i<datapoints.size();i++){
        datapoints[i].isSet = false;
    }
    
    //get a polyline
    contourPC = getBodyPoly();
    
    ofRectangle cBndBox = contourPC.getBoundingBox();
   // ofRectangle cBndBox = ofRectangle(0, 0, 500, 500);
    if(debugAction) cBndBox = ofRectangle(0, 0, 500, 500);
    int wordH= font.getStringBoundingBox(datapoints[0].Name,0,0).height+22;
    int stepY = cBndBox.getHeight()/23 + ofRandom(-5,5);
    if(stepY <  wordH)stepY = wordH + ofRandom(-5,5);
    
    vector<ofPoint> unifDistPoints;
    int d = 0;
    int stepX = 1;//cBndBox.getWidth()/3;
    
    for (int iY=0; iY < cBndBox.getHeight(); iY+=stepY ){
        
        
        
        int stepX = cBndBox.getWidth()/3;
        if(d%2 == 1){stepX = cBndBox.getWidth()/4;}
        
        
        for(int iX=0; iX< cBndBox.getWidth(); iX+=stepX ){
            ofPoint p;
            p.x = iX+cBndBox.x;
            p.y = iY+cBndBox.y;
             if(contourPC.inside(p)||debugAction){
                datapoints[d].pos = p;
                datapoints[d].isSet = true;
                int wordWidth = font.getStringBoundingBox(datapoints[d].Name,0,0).width+14;
                if(stepX<wordWidth)stepX = wordWidth;
                d++;
                d= d%datapoints.size();
            
                unifDistPoints.push_back(p);
            
            
            }
        }
    }
    
    
    
   // random_shuffle(unifDistPoints.begin(), unifDistPoints.end()); // shuffle the points
    
//    for( int i = 0; i < unifDistPoints.size() && i < datapoints.size(); i++){
//        datapoints[i].pos = unifDistPoints[i];
//        
//    }
}

//--------------------------------------------------------------
void ofApp::detectPerson(){
    if(detectBody.getBodies().size()>0){
        isPersonPresent = false;
        
        ofPolyline p = detectBody.getBodies()[0].boundaryPoly;
        // check if the detected blob is taler than wide
        if(p.getBoundingBox().width > p.getBoundingBox().height){
            // check ig the center of the blob is in the cone we are capturing
            if(detectBody.getBodies()[0].centroid.x > bPosX-thresW/2 &&
               detectBody.getBodies()[0].centroid.x < bPosX+thresW/2 &&
               detectBody.getBodies()[0].centroid.y > bPosY-thresH/2 &&
               detectBody.getBodies()[0].centroid.y < bPosY+thresH/2
               ){
                isPersonPresent = true;
                contourPC = getBodyPoly();
                
                //make rect of boundingbox to pass to actors
                thePerson = ofRectangle(contourPC.getBoundingBox().x,contourPC.getBoundingBox().y, contourPC.getBoundingBox().width, contourPC.getBoundingBox().height);
            }
        } else{
            thePerson  = ofRectangle(0,0,0,0);
        }
    }
}
//--------------------------------------------------------------
void ofApp::timeLine(){
    
    //reset timer if person leaves
    if(!isPersonPresent){
        isPPtimer=0;
    }
    
    //if person is present for more than 100 frames -> start scanDown or flash
    if(isPersonPresent && !active){
        isPPtimer++;
        if(isPPtimer>isPPthres){
            ofxOscMessage m;
            if(!doFlash){
                scanDown = true;
                m.setAddress("/scanDown");
            }
            if(doFlash){
                flash = true;
                m.setAddress("/flash");
                scanLine = RES_HEIGHT;
            }
            
            active = true;
            m.addIntArg(1);
            sender.sendMessage(m);
        }
    }
    
    //
    if(scanDown || scanUp){
        ofxOscMessage m;
        m.setAddress("/scanner");
        float f = scanLine / RES_HEIGHT;
        m.addFloatArg(f);
        sender.sendMessage(m);
    }
    
    // if scanline is down, scan go up.
    if(scanDown){
        scanLine +=15;
        if(scanLine > RES_HEIGHT){ // start scanUp
            scanUp = true;
            scanDown = false;
            positions();
            
            ofxOscMessage m;
            m.setAddress("/scanUp");
            m.addIntArg(1);
            sender.sendMessage(m);
        }
    }
    
    if(flash){ // start flash
        flashTimer ++;
        if(flashTimer > flashTimerThres){ // start scanUp
            scanUp =true;
            flash = false;
            positions();
            
            ofxOscMessage m;
            m.setAddress("/scanUp");
            m.addIntArg(1);
            sender.sendMessage(m);
        }
    }
    
    // scan up- set datapoints positions, freese PC (updatePC) // when up, keep datapoints (freeze)
    if(scanUp){
        updatePC = false;
        scanLine -=5;
        
        if(scanLine < 0){
            for(int i = 0; i<datapoints.size();i++){
                // length ->start length counter.
                datapoints[i].bLength = true;
                
            }
            scanUp = false;
            freeze = true; // let dataPoints stay even if scanUp = false
            ending = true;
            pointCloud.vel.clear();
            pointCloud.collapse = RES_HEIGHT/2;
            
            ofxOscMessage m;
            m.setAddress("/network");
            m.addIntArg(1);
            sender.sendMessage(m);
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
        
        if(datapoints[0].tlAlpha>0){
            ofxOscMessage m;
            m.setAddress("/rain");
            m.addFloatArg(datapoints[0].tlAlpha);
            sender.sendMessage(m);
        }
        
        for(int i = 0; i< actors.size();i++){
            actors[i].hasConnection = false;
        }
        for(int i = 0; i<datapoints.size();i++){
            datapoints[i].fall=true;
        }
        if(lastTimer > lastTimerThres){
            resetAll = true;
        }
    }
    
    // if person leaves while scanning down, reset all.
    if(scanDown && !isPersonPresent){
        resetAll = true;
    
    }
    
    // if person leaves while flash down, reset all.
    if(flash && !isPersonPresent){
        resetAll = true;
    }
    
    
    //reset all timers, bools and datapoints
    if(resetAll ){
        ofxOscMessage m;
        m.setAddress("/stopAll");
        m.addIntArg(1);
        sender.sendMessage(m);
        
        flash=false;
        flashTimer=0;
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
        for(int i = 0; i< datapoints.size();i++){
            datapoints[i].bLength = false;
            datapoints[i].bAlpha = false;
            datapoints[i].tlAlpha = 0;
            datapoints[i].tlLength = 0;
            datapoints[i].fall = false;
        }
        active = false;
    }
}

//--------------------------------------------------------------
ofPolyline ofApp::getBodyPoly(){
    //finds the polyline of the shown image
    ofPolyline p;
    
    if(detectBody.getBodies().size()>0){
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
        
        if(bDebug){
            ofxCv::toOf(contourPCMat, contourDetectImg);
            contourDetectImg.update();
        }
        
        if(contourFindPC.getPolylines().size()>0){
            
            p = contourFindPC.getPolyline(0);
        }
    }
    return p;
}
