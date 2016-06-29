//
//  DetectBody.cpp
//
//  Created by Jonas Fehr on 23/11/15.
//
//

#include "DetectBody.h"


using namespace cv;


DetectBody::DetectBody(){
    
};

void DetectBody::setup( int _w, int _h, int _nearThreshold, int _farThreshold)
{
    farThreshold = _farThreshold;
    nearThreshold = _nearThreshold;
    
    width = _w;
    height = _h;
    
    outputImage.allocate(width, height, GL_RGBA);//A32F_ARB);
    outputImage.begin();
    ofClear(255,255,255, 255);
    ofNoFill();
    ofSetLineWidth(2);
    ofEnableSmoothing();
    outputImage.end();
    
    output = cv::Mat::zeros( cvSize(width,height), CV_8U );
    
    
    bodyPosition = ofVec2f(-1, -1);
    
    // draw the Mask
    activeAreaMask = cv::Mat::zeros( cvSize(width,height), CV_8U );
    
    
    ofImage imgToLoad;
    if( imgToLoad.load("activeAreaMask.png") ){
        ofxCv::toCv(imgToLoad).convertTo(activeAreaMask, CV_8U);
    }else{
        createGenericMask();
    }
}

void DetectBody::update(cv::Mat image, ofxKinectV2 *kinect)
{
    image.convertTo(depthImage, CV_8UC1);
    
    depthImage.copyTo(inputImage);
    
    outputImage.begin();
    ofClear(255,255,255, 0);
    outputImage.end();
    
    bool justActiveArea = false; // norm true
    if(justActiveArea) {
        bitwise_and(depthImage, activeAreaMask, depthImage);
    }
    
    
    //erode first, then dilate to reduce noise.
    int closingNum = 1;
    bitwise_not(depthImage, depthImage);
    erode(depthImage, depthImage, cv::Mat(), cv::Point(-1,-1), closingNum);
    dilate(depthImage, depthImage, cv::Mat(), cv::Point(-1,-1), closingNum);
    bitwise_not(depthImage, depthImage);
    
    
    bool armSeparation = true;
    if(armSeparation)
    {
        // After the technique presented in KinectArms
        // Get edges around arms and substract
        // ev. normalize to get bigger differences between uppe rand lower...
        
        // create Canny
        //IplImage* depthImageCanny = cvCreateImage(cvSize(width,height),IPL_DEPTH_8U,1);
        Mat depthImageCanny = Mat::zeros( cvSize(width,height), CV_8UC1 );
        
        
       // Canny(depthImage, depthImageCanny, 27,79, 3); // for K:V2 130, 100, 3);
        
       // dilate(depthImageCanny, depthImageCanny, cv::Mat(), cv::Point(-1,-1), 2);
       // erode(depthImageCanny, depthImageCanny, cv::Mat(), cv::Point(-1,-1), 1);
        
        threshold(depthImage, depthImage, 50, 255, THRESH_BINARY );
        
        Mat depthImageErodedArms = Mat::zeros( cvSize(width,height), CV_8UC1 );
        int closingNum = 1;
        erode(depthImage, depthImageErodedArms, cv::Mat(), cv::Point(-1,-1), closingNum);
        
        // crossing
        Mat armDivider = Mat::zeros( cvSize(width,height), CV_8UC1 );
        bitwise_and(depthImageCanny, depthImageErodedArms, armDivider);
        
        //diliate the devider
        closingNum = 1;
        erode(armDivider, armDivider, cv::Mat(), cv::Point(-1,-1), closingNum);
        
        Mat armEdges = Mat::zeros( cvSize(width,height), CV_8UC1 );
        bitwise_or(depthImageCanny, armDivider, armEdges); // edges depthImage
        bitwise_not(armEdges, armEdges); // invert for && depthImageing
        bitwise_and(depthImage,armEdges,depthImage);
        //bitwise_xor(depthImage,armDivider,depthImage); // would do the job also
        
        
        
        // select which output is shown
        switch ( outputImageIndex ){
            case 1: output = depthImage;
                break;
            case 2: output = depthImageCanny;
                break;
            case 3: output = depthImageErodedArms;
                break;
            case 4: output = armDivider;
                break;
            case 5: output = armEdges;
                break;
            case 6: output = activeAreaMask;
                break;
            case 7: output = inputImage;
                break;
            default: output = depthImage;
                break;
        }
        
    }
    
    
    
    // 2nd approach
    
    
    getbodyBoundries(depthImage);
    
    //
    //    Mat dilatedTableEdges = Mat::zeros( cvSize(width,height), CV_8UC1 );
    //    erode(activeAreaMask, dilatedTableEdges, cv::Mat(), cv::Point(-1,-1), 5); // last 2 closing num, may change
    //
    
    if(bodies.size()>0){
        for(vector<Body>::iterator b=bodies.begin(); b!=bodies.end(); b++)
        {
            
            for(int y=0; y<b->bodyBlob.rows; y++)
            {
                for(int x=0; x<b->bodyBlob.cols; x++)
                {
                    if(b->bodyBlob.at<bool>(y, x))
                    {
                        ofVec3f pt = kinect->getWorldCoordinateAt(x,y);
                        
                        float y = pt.y;
                        
                        pt.y = y*cos(tilt)-pt.z*sin(tilt);
                        
                        b->indices.push_back(ofVec2f(pt.x,pt.y));
                        //indices.data[indices.size] = Point2Di(x, y);
                        //indices.size++;
                    }
                }
            }
        }
    }
    
    
    
}

// DRAW

void DetectBody::drawOverlay(int x, int y, int w, int h)
{
    outputImage.begin();
    for (int i = 0; i < bodies.size(); i++)
    {
        bodies[i].draw();
    }
    outputImage.end();
    
    ofSetColor(255, 255);
    outputImage.draw(x, y, w, h);
}

void DetectBody::drawProcess(int x, int y, int w, int h, int index)
{
    ofImage imgOut;
    imgOut.allocate(512, 424, OF_IMAGE_GRAYSCALE);
    ofxCv::toOf(output, imgOut);
    imgOut.update();
    
    imgOut.draw(x, y, w, h);
    outputImageIndex = index;
    /*  1 depthImage
     2 depthImageCanny
     3 depthImageErodedArms
     4 armDivider
     5 armEdges
     6 activeAreaMask
     7 inputImage
     default depthImage
     */
}

// GET DATA


vector<Body> DetectBody::getBodies(){
    ofSort(bodies,sortMe);
    return bodies;
}

vector<ofPolyline> DetectBody::getContours(){
    vector<ofPolyline> bodiesPolys;
    for(int i = 0; i<bodies.size();i++){
        
        
        
        bodiesPolys.push_back(bodies[i].boundaryPoly);
    }
    return bodiesPolys;
}









// funktions adapted from KinectArm

struct CompareContourArea
{
    CompareContourArea(const std::vector<double>& areaVec)
    : mAreaVec(areaVec) {}
    
    // Sort contour indices into decreasing order, based on a vector of
    // contour areas.  Later, we will use these indices to order the
    // contours (which are stored in a separate vector).
    bool operator()(size_t a, size_t b) const
    {
        return mAreaVec[a] > mAreaVec[b];
    }
    
    const std::vector<double>& mAreaVec;
};

void DetectBody::getbodyBoundries(cv::Mat _armBlobs){
    // cv::Mat armBlobsMat(DEPTH_RES_Y, DEPTH_RES_X, CV_8UC1, (void*)armBlobs.data);
    
    // adapted form ofxCv
    
    vector<vector<cv::Point> > allContours;
    bool simplify = false;
    int simplifyMode = simplify ? CV_CHAIN_APPROX_SIMPLE : CV_CHAIN_APPROX_NONE;
    cv::findContours(_armBlobs, allContours, CV_RETR_EXTERNAL, simplifyMode);
    
    int imgMinArea = 1000;
    
    // filter the contours
    vector<size_t> allIndices;
    vector<double> allAreas;
    double imgArea = _armBlobs.rows * _armBlobs.cols;
    for(size_t i = 0; i < allContours.size(); i++) {
        
        double curArea = contourArea(Mat(allContours[i]));
        
        allAreas.push_back(curArea);
        //        if(ofxCv::toOf(allContours[i]).getBoundingBox().width>ofxCv::toOf(allContours[i]).getBoundingBox().height){
        //            allIndices.push_back(i);
        //        }
        // allAreas[i]=contourArea(Mat(allContours[i]));
        if(curArea >= imgMinArea) {
            allIndices.push_back(i);
        }
        // }
    }
    
    // sort by size
    if (allIndices.size() > 1) {
        std::sort(allIndices.begin(), allIndices.end(), CompareContourArea(allAreas));
    }
    
    
    //    for(size_t i = 0; i < allIndices.size(); i++) {
    //        contoursLocal.push_back(allContours[allIndices[i]]);
    //        if( ofxCv::toOf(contoursLocal[i]).getBoundingBox().width>ofxCv::toOf(contoursLocal[i]).getBoundingBox().height){
    //            Body b = *new Body;
    //            b.boundaryPoly =ofxCv::toOf(contoursLocal[i]);
    //            bodies.push_back(b);
    //        }
    //    }
    //
    // generate polylines and bounding boxes from the contours
    contoursLocal.clear();
    bodies.clear();
    bodies.resize(allIndices.size());
    // boundingRects.clear();
    for(size_t i = 0; i < allIndices.size(); i++) {
        contoursLocal.push_back(allContours[allIndices[i]]);
        bodies[i].boundaryPoly = ofxCv::toOf(contoursLocal[i]);
        
        
        //  boundingRects.push_back(boundingRect(contours[i]));
        
        vector<cv::Point> tmp;
        for(size_t i2 = 0; i2 < contoursLocal[i].size(); i2++) {
            bodies[i].boundary.push_back(ofxCv::toOf(contoursLocal[i][i2]));
            tmp.push_back(contoursLocal[i][i2]);
        }
        
        std::vector<std::vector<cv::Point> > contourVec;
        contourVec.push_back(contoursLocal[i]);
        
        Mat drawedContour = cv::Mat::zeros( cvSize(width,height), CV_8U );
        cv::drawContours(drawedContour,contourVec,0,Scalar(255,255,255),CV_FILLED, 0);
        
        
        bodies[i].bodyBlob = drawedContour;
        
        
        
        
        ofVec2f center;
        cv::Moments m = moments(contoursLocal[i]);
        center = ofxCv::toOf(cv::Point2f(m.m10 / m.m00, m.m01 / m.m00));
        bodies[i].centroid = center;
        
    }
    
    
    
    
    
    /*
     
     // from kinectArms
     
     const int smallBoundarySize = 40;
     vector<CvSeq*> contours;
     for(CvSeq* contour = firstContour; contour!= 0; contour = contour->h_next)
     {
     // Remove any small boundaries
     if(contour->total < smallBoundarySize)
     continue;
     
     contours.push_back(contour);
     }
     
     
     
     // Get the contours of the blobs
     static CvSeq* firstContour = 0;
     
     Mat armBlobsDilated = _armBlobs;
     
     dilate(armBlobsDilated, armBlobsDilated, cv::Mat(), cv::Point(-1,-1), 1);
     
     CvMat armsBlobCvMat = armBlobsDilated;//_armBlobs;
     static CvMemStorage* storage = cvCreateMemStorage(0);
     
     
     int numContours = cvFindContours(&armsBlobCvMat, storage, &firstContour, sizeof(CvContour), CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
     
     
     
     // Put contours in a nice list and get rid of all small contours
     const int smallBoundarySize = 40;
     vector<CvSeq*> contours;
     for(CvSeq* contour = firstContour; contour!= 0; contour = contour->h_next)
     {
     // Remove any small boundaries
     if(contour->total < smallBoundarySize)
     continue;
     
     contours.push_back(contour);
     }
     
     
     // Clear all previous bodies
     bodies.clear();
     
     
     // Get the body boundaries (take as many as we are allowed)
     bodies.resize(contours.size());
     for(int i=0; i<bodies.size(); i++)
     {
     CvSeq& contour = *contours[i];
     bodies[i].boundrySeq = &contour;
     //  ((bodyPimpl*)bodies[i].bodyPimpl)->SetBoundary(contour);
     
     CvSeqReader reader;
     cvStartReadSeq(&contour, &reader, 0);
     
     for(int j=0; j<contour.total; j++)
     {
     CvPoint point;
     memcpy(&point, reader.ptr, contour.elem_size);
     
     bodies[i].boundary.push_back(ofVec2f(point.x, point.y));
     
     CV_NEXT_SEQ_ELEM(contour.elem_size, reader);
     }
     
     
     
     //   cvDrawContours(&blobImage, const_cast<CvSeq*>(((bodyPimpl*)bodyPimpl)->GetBoundary()), color, color, 0, thickness);
     
     IplImage armBlobImage = bodies[i].armBlob;
     
     cvDrawContours( &armBlobImage, &contour, CV_RGB(255, 255, 255), CV_RGB(255, 255, 255), 0, -1);
     //bodies[i].armBlob = Mat(armBlobImage, true);
     
     //   cv::drawContours(bodies[i].armBlob, contours, i, CV_RGB(255, 255, 255));
     
     
     }
     */
}

ofVec2f DetectBody::findArmBase(cv::Mat tableEdges, Body &_body)
{
    
    ofVec2f armBase;
    
    
    
    /* Mat image = Mat::zeros( cvSize(width,height),  CV_8U );
     tableEdges.convertTo(image, CV_8U, 5);
     IplImage* img = new IplImage(image);
     cvCopy(img, output.getCvImage());
     output.flagImageChanged();*/
    
    
    threshold(tableEdges, tableEdges, 1, 1, THRESH_TRUNC);
    
    
    
    
    
    // Find a boundary point that isn't on the table edge
    int startIndex = -1;
    /*for(int iy=0; iy<tableEdges.rows; iy++){
     for(int ix=0; ix<tableEdges.cols; ix++){
     cout << tableEdges.at<bool>(iy, ix);
     }
     cout << "\n";
     }*/
    
    bool flag = false;
    
    for(int j=0; j<_body.boundary.size(); j++)
    {
        
        // cout << "R (default) = " << endl <<        tableEdges          << endl << endl;
        // cout << ofToString(tableEdges.at<bool>((int)_body.boundary[j].y, (int)_body.boundary[j].x))<<"\n";
        if(tableEdges.at<bool>((int)_body.boundary[j].y,(int)_body.boundary[j].x)){
            startIndex = j;
            break;
        }
    }
    
    if(startIndex == -1)
    {
        armBase = ofVec2f(-1, -1);
        return armBase;
    }
    
    
    // Make variables to hold properties of the arm base
    int maxCount = 0;
    int maxStartIndex = 0;
    
    // Go around boundary looking for the largest segment
    
    
    
    // Find base start
    int baseStartIndex = 0;
    int baseEndIndex = 0;
    bool flagCross = false;
    
    int count = 0;
    for(int j=0; j<_body.boundary.size(); j++)
    {
        ofVec2f point = _body.boundary[(j + startIndex) % _body.boundary.size()];
        ofVec2f fwrdPoint = _body.boundary[(j + startIndex + 1) % _body.boundary.size()];
        
        bool startingBase =  tableEdges.at<bool>(point.y, point.x) && !tableEdges.at<bool>(fwrdPoint.y, fwrdPoint.x);
        bool endingBase =    !tableEdges.at<bool>(point.y, point.x) && tableEdges.at<bool>(fwrdPoint.y, fwrdPoint.x);
        
        if(!tableEdges.at<bool>(point.y, point.x)){
            count++;
            
        }
        
        
        
        
        if(startingBase)
        {
            // We are starting a base
            baseStartIndex = (j + startIndex + 1) % _body.boundary.size();
            count = 0;
        }
        else if(endingBase)
        {
            if(count > maxCount)
            {
                maxCount = count;
                maxStartIndex = baseStartIndex;
            }
        }
        
    }
    
    
    // Check if we found any base
    if(maxCount == 0)
    {
        armBase = ofVec2f(-1, -1);
        return armBase;
    }
    
    
    
    
    // Calculate the center of the base
    int armBaseIndex = ((maxStartIndex + (maxStartIndex + maxCount)) / 2) % _body.boundary.size();
    
    armBase = _body.boundary[armBaseIndex];
    
    _body.indxBaseStart = maxStartIndex;
    _body.indxBaseEnd = maxStartIndex+maxCount;
    
    outputImage.begin();
    ofSetColor(255, 0, 255);
    ofDrawCircle(armBase, 10);
    outputImage.end();
    
    
    return armBase;
}

//ofVec2f DetectBody::findPalmCenter(Body _body)
//{
//    ofVec2f palmCenter;
//
//    Mat distImage = Mat::zeros( cvSize(width,height),  CV_32FC1 );
//
//    //  threshold(_body.armBlob, _body.armBlob, 1, 255, THRESH_TRUNC);
//    cv::distanceTransform(_body.bodyBlob, distImage, CV_DIST_C, 3);  //!!Could use CV_DIST_L2 for euclidian distance
//
//
//    /*Mat image = Mat::zeros( cvSize(width,height),  CV_8UC1 );
//     distImage.convertTo(image, CV_8UC1, 5);
//     IplImage* img = new IplImage(image);
//     cvCopy(img, output.getCvImage());
//     output.flagImageChanged();
//     */
//    // Only check points that are farther from the arm base than the geometric center
//    float armCenterMagnitude = _body.centroid.squareDistance(_body.armBase);
//
//    // Get arm blob indices
//    indices.clear();
//    for(int y=0; y<_body.bodyBlob.rows; y++)
//    {
//        for(int x=0; x<_body.bodyBlob.cols; x++)
//        {
//            if(_body.bodyBlob.at<bool>(y, x))
//            {
//                _body.indices.push_back(ofVec2f(x, y));
//                //indices.data[indices.size] = Point2Di(x, y);
//                //indices.size++;
//            }
//        }
//    }
//
//
//
//    // Find the max value of the distance transform that is further from arm base than geometric center is
//    ofVec2f maxValuePoint;
//    float maxValue = 0;
//
//    for(int i=0; i<indices.size(); i++)
//    {
//        ofVec2f point = indices[i];
//        float pointMagnitude = point.squareDistance(_body.armBase);
//
//        float value = distImage.at<float>(point.y, point.x) * pointMagnitude;
//        //  cout << ofToString(pointMagnitude > armCenterMagnitude)+"\n";
//        if(pointMagnitude > armCenterMagnitude && value > maxValue)  // pointMagnitude > armCenterMagnitude always true;
//        {
//            maxValuePoint = point;
//            maxValue = value;
//
//        }
//
//    }
//
//
//
//    float palmThreshold = 0;
//    if(maxValue > palmThreshold){
//        palmCenter = maxValuePoint;
//    }else{
//        palmCenter = ofVec2f(-1, -1);
//    }
//    return palmCenter;
//}

void DetectBody::createMask(){
    inputImage.copyTo(activeAreaMask);
    threshold(activeAreaMask, activeAreaMask, 150, 255, THRESH_BINARY);
    
    ofImage imgToSave;
    ofxCv::toOf(activeAreaMask, imgToSave);
    imgToSave.save("activeAreaMask.png");
}
void DetectBody::resetMask(){
    activeAreaMask = cv::Mat::zeros( cvSize(width,height), CV_8U );
    cv::Point p1 = cv::Point(0,0);//ofxCv::toCv(ofVec2f((width-height)/2, 0));
    cv::Point p2 = cv::Point(width, height);//ofxCv::toCv(ofVec2f((width-height)/2+height, height));
    rectangle(activeAreaMask, p1, p2, CV_RGB(255,255,255),-1,CV_AA,0);//
}
void DetectBody::createGenericMask(){
    //width = 512;
    //height = 424;
    activeAreaMask = cv::Mat::zeros( cvSize(width,height), CV_8U );
    // circle(areaMask,ofxCv::toCv(ofVec2f(width/2, height/2)),height/4,CV_RGB(255,255,255),-1,CV_AA,0);
    int red = 40;
    cv::Point p1 = ofxCv::toCv(ofVec2f((width-height)/2 + red, red));
    cv::Point p2 = ofxCv::toCv(ofVec2f((width-height)/2+height-red, height-red));
    rectangle(activeAreaMask, p1, p2, CV_RGB(255,255,255),-1,CV_AA,0);
    
    ofImage imgToSave;
    ofxCv::toOf(activeAreaMask, imgToSave);
    imgToSave.save("activeAreaMask.png");
}

