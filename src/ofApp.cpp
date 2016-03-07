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
    font.load("font.ttf", 12, true, true);
    
    renderPC.allocate(ofGetWidth(),ofGetHeight());
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
        if(i%2==0){
            actors[i-1]->att.x = ofRandom(ofGetWidth()-100,ofGetWidth());
        }
        else if(i%2==1){
            actors[i-1]->att.x = ofRandom(0,100);
        }
        actors[i-1]->att.y = ofRandom(ofGetHeight()/2,0);
        //actors[i-1]->att = actors[i-1]->pos;
        actors[i-1]->color = ofColor(ofRandom(80-100),ofRandom(80-100),ofRandom(100,255));
        actors[i-1]->font = &font;
    }
    
    for(int i=1; i<csv.numRows; i++) {
        datapoints[i-1] = new DataPoint;
        string name = csv.getString(i, 0);
        datapoints[i-1]->Name = name;
        datapoints[i-1]->pos.x = ofRandom(ofGetWidth());
        datapoints[i-1]->pos.y = ofRandom(ofGetHeight());
        
        for(int u=1; u<csv.numCols-1; u++) {
            int value = csv.getInt(i, u);
            if(value == 1){
                datapoints[i-1]->connections.push_back(actors[u-1]);
            }
        }
        int at = ofRandom(datapoints[i-1]->connections.size());
        datapoints[i-1]->att = actors[at]->pos;
        datapoints[i-1]->font = &font;
    }
    //csv----------------------------end
    
    
    kinect.open(false, true, 0, 2); // GeForce on MacBookPro Retina
    
    kinect.start();
    
    pointCloud.setup();
    
    
    detectBody.setup(512, 424, nearThreshold, farThreshold);
    
    depthShader.load("depthshader/shader");
    
    // Setup GUI
    imageSetup.setName("imageSetup");
    imageSetup.add(nearThreshold.set("nearThreshold", 10, 0, 8000));
    imageSetup.add(farThreshold.set("farThreshold", 25, 0, 8000));
    imageSetup.add(edge.set("edge", 0, 0, 524));
    imageSetup.add(topDepth.set("topDepth", 0, 0, 8000));
    imageSetup.add(botDepth.set("botDepth", 0, 0, 8000));
    imageSetup.add(edgeDepth.set("edgeDepth", 0, 0, 8000));
    
    
  
    pointCloudSetup.setName("pointCloudSetup");
    pointCloudSetup.add(tilt.set("tilt", 0, -180, 180));
    pointCloudSetup.add(translateX.set("translateX", -512, -1000, 1000));
    pointCloudSetup.add(translateY.set("translateY", -512, -1000, 1000));
    pointCloudSetup.add(translateZ.set("translateZ", -1000, -1000, 2000));

    testParams.setName("testParams");
    testParams.add(test1.set("test1", 0, 0,300));
    testParams.add(test2.set("test2", 0, 0, 300));
    
    paramters.add(testParams);
    paramters.add(imageSetup);
    paramters.add(pointCloudSetup);
    
    gui.setup(paramters);
    gui.loadFromFile("settings.xml");

    
}

//--------------------------------------------------------------
void ofApp::update(){
    pointCloud.nearCut = nearCut;
    pointCloud.farCut = farCut;
    pointCloud.tilt = tilt;

    
    //beginning
    if(!isPersonPresent){
        isPP=0;
    }
    //if person is present for more than 100 frames -> start scanDown
    if(isPersonPresent){
        isPP++;
        if(isPP>isPPthres){
            scanDown = true;
            
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
        if(scanLine > ofGetHeight()-scanLineHeight){
            scanUp = true;
            scanDown = false;
            //setPositions = true;
            positions();
        }
    }

    
    // scan up- set datapoints positions, freese PC (updatePC) // when up, keep datapoints (freeze)
    if(scanUp){
        updatePC = false;
        float x = abs (ofGetHeight() - scanLine)/100;
        scanLine -=5;
        if(scanLine < 0-scanLineHeight){
            scanUp = false;
            freeze = true; // let dataPoints stay even if scanUp = false
            ending = true;
        }
    }
    
    //ending (hold for endTimerThres amount of time
    if(!ending){
        endTimer = 0;
    }
    
    if(ending){
        endTimer++;
        if(endTimer>endTimerThres){
            // endAnimation = true;
             resetAll;
        }
    }
    
    if(resetAll){
        scanLine = 0;
        scanUp = false;
        scanDown = false;
        freeze = false;
        updatePC = true;
        setPositions = false;
        isPersonPresent = false;
        isPP = 0;
        ending = false;
        endTimer = 0;
        endAnimation = false;
        resetAll = false;
        for(int i = 0; i< numDatapoints;i++){
            datapoints[i]->con = false;
            datapoints[i]->tl = 0;
        }
    }
    
    
    //cout<<contourPoly.size()<<endl;
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
            
            
            
            detectBody.update(ofxCv::toCv(depthImage));//kinect.getDepthPixelsRef()));
            //detectBody.setTresholds(nearThreshold, farThreshold, tilt);
            
        }
    }
    
    
    //    if(freeze){
    //        falling +=0.1;
    //
    //        vector<ofVec3f>vert = pointCloud.mesh.getVertices();
    //        pointCloud.mesh.clear();
    //        for(int i = 0; i< vert.size();i++){
    //            ofVec3f pt;
    //            if(vert[i].y<424){
    //                pt = ofVec3f(vert[i].x,vert[i].y-falling,vert[i].z);
    //            }
    //            else{
    //                pt = ofVec3f(vert[i].x,vert[i].y,vert[i].z);
    //            }
    //            pointCloud.mesh.addVertex(pt);
    //
    //        }
    //    }
    
    /*
     cam.update();
     if(cam.isFrameNew()) {
     detectFace.update(cam);
     }
     cout << "imageType: "+ofToString(cam.getPixelFormat())+"\n";*/
    
    //scanLine = scaleZ;
    // pointCloud.scanLine = scanLine;//-ofGetHeight()+(ofGetHeight()-(scanLine))+324;//(ofGetHeight()-((scanLine)))-424;
    detectBody.test1 = test1;
    detectBody.test2 = test2;
    
}

//--------------------------------------------------------------
void ofApp::draw(){
    
    
    renderPC.begin();
    ofClear(0);
    

    ofPushMatrix();
    
   
    ofScale(-1, -1, 1);
   // ofTranslate(-387 + translateX, -602 + translateY, -1000 +translateZ);
    ofTranslate(translateX,translateY,translateZ);
    ofRotateX(tilt);
    //ofRotateY(180);
    //ofPushMatrix();
   // ofTranslate(ofGetWidth()/2, ofGetHeight()/2,translateZ/2);
   // ofRotateY(ofGetFrameNum());

    ofSetColor(255);
    pointCloud.draw(0,0,0);
  //  pointCloud.draw(0,0,0);

    //ofPopMatrix();
    ofPopMatrix();
    renderPC.end();
    
    ofSetColor(255);
    ofBackgroundGradient(ofColor(50), ofColor(0),OF_GRADIENT_CIRCULAR);
    
    
    
    renderPC.draw(0,0);
    
    
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
    
    if(scanDown||scanUp){
       // ofSetLineWidth(1);
        float m = ofGetFrameNum()*50 % ofGetWidth();
        scanImage.draw(m-ofGetWidth(),scanLine,ofGetWidth()+(ofGetHeight()-m),20);

    }
    
    ofSetColor(255);
    contourPC.draw();
//    
//    
//    cv::Mat contourPCMat;
//    contourPCMat = cv::Mat::zeros( cvSize(ofGetWidth(),ofGetHeight()), CV_8U );
//    ofPixels imagePC;
//    renderPC.readToPixels(imagePC);
//    //imagePC.convertTo(contourPCMat, CV_8UC1);
//    ofxCv::toCv(imagePC).convertTo(contourPCMat, CV_8UC1);
//    
//    int closingNum = 10;
//    bitwise_not(contourPCMat, contourPCMat);
//    erode(contourPCMat, contourPCMat, cv::Mat(), cv::Point(-1,-1), closingNum);
//    // dilate(contourPCMat, contourPCMat, cv::Mat(), cv::Point(-1,-1), closingNum);
//    bitwise_not(contourPCMat, contourPCMat);
//    ofxCv::ContourFinder contourFindPC;
//    contourFindPC.setSortBySize(true);
//    contourFindPC.findContours(contourPCMat);
//    
//    if(contourFindPC.getPolylines().size()>0){
//        
//        contourPC = contourFindPC.getPolyline(0);
//        
//        
//        ofBeginShape();
//        ofFill();
//        ofSetColor(200, 0, 0);
//        ofVertex(0, 0);
//        ofVertex(ofGetWidth(), 0);
//        ofVertex(ofGetWidth(), ofGetHeight());
//        ofVertex(0, ofGetHeight());
//        ofVertex(0, 1);
//        for( int i = 0; i < contourPC.getVertices().size(); i++) {
//            
//            ofVertex(contourPC.getVertices().at(i).x, contourPC.getVertices().at(i).y);
//            
//        }
//        
//        ofEndShape();
//    }
    
    
    if(bDebug){
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
        // ofDrawBitmapStringHighlight("Device Count : " + ofToString(ofxMultiKinectV2::getDeviceCount()), 10, 40);
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
        
        //setPositions = true;
        scanDown  = true;
        //        for(int i  = 0 ; i<numDatapoints;i++ ){
        //            datapoints[i]->tl = 0;
        //        }
        
    }
    
    if(key == 'r'){
        scanLine = 0;
        scanUp = false;
        scanDown = false;
        freeze = false;
        updatePC = true;
        setPositions = false;
        isPersonPresent = false;
        isPP = 0;
        isPPthres = 100;
        ending = false;
        endTimer = 0;
        endTimerThres = 100;
        endAnimation = false;
        resetAll = false;
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
    //imagePC.convertTo(contourPCMat, CV_8UC1);
    ofxCv::toCv(imagePC).convertTo(contourPCMat, CV_8UC1);
    
    int closingNum = 10;
    bitwise_not(contourPCMat, contourPCMat);
    erode(contourPCMat, contourPCMat, cv::Mat(), cv::Point(-1,-1), closingNum);
    // dilate(contourPCMat, contourPCMat, cv::Mat(), cv::Point(-1,-1), closingNum);
    bitwise_not(contourPCMat, contourPCMat);
    ofxCv::ContourFinder contourFindPC;
    contourFindPC.setSortBySize(true);
    contourFindPC.findContours(contourPCMat);
    
    if(contourFindPC.getPolylines().size()>0){
        
        contourPC = contourFindPC.getPolyline(0);

        
        int h = contourPC.getBoundingBox().height*.90;
        int w = contourPC.getBoundingBox().width*.80;
        int a = contourPC.getArea();
       // int aPixelsPrWord = (w*h)/numDatapoints;
        
        int stepX = sqrt((w*w) / numDatapoints);
        int stepY = sqrt((h*h) / numDatapoints);
        //float scale = contourPoly[0].getArea()/100;
        
        int dp = 0;
        int step =1;
        if(stepY<20){stepY = 20;}
        for(int y = 0 ; y < ofGetHeight() ; y+=stepY){
            
            step = font.getStringBoundingBox(datapoints[dp]->Name, 0, 0).width+20;
            if(stepX<step){stepX = step;}
            
            for(int x = 0 ; x < ofGetWidth() ; x+=stepX){
                
                ofPoint p = ofPoint(x,y);
                
                if(contourPC.inside(p)){
                    //  float dist = kinect.getDistanceAt(x, y);
                    //  ofVec3f pt= kinect.getWorldCoordinateAt(y, x, dist);
                    datapoints[dp]->pos.x = x;
                    if(dp%2==0){
                    datapoints[dp]->pos.y = y + stepY/2;
                    }else{
                    datapoints[dp]->pos.y = y;
                    }
                    //    datapoints[dp]->posZ = pt.z;
                    datapoints[dp]->isSet = true;
                    dp ++;
                    dp = dp%numDatapoints;
                    
                }
            }
        }
        
    }


}
