//
//  pointCloud.hpp
//  VirtualMirror
//
//  Created by Jonas Fehr on 25/01/16.
//
//

#include "ofMain.h"
#include "ofxMultiKinectV2.h"
#include "DetectBody.h"

class PointCloud{
    
public:
    
    int pointSize = 2;
    int step = 2;
    float scanLine;
    int floor;
    int collapse = 0;
    int translateX,translateY,translateZ;
    float tilt;
    
    
    vector<ofVec3f> vel;
    vector<bool> bol;
    
    ofFbo renderer;
    ofVbo vbo;
    ofShader shaderSplineReplace;
    ofTexture texSpline;
    
    
    void setStep(int _step){ step = _step;}
    void setPointSize(int _pointSize){ pointSize = _pointSize;}
    
    ofMesh mesh;
    //ofMesh frozenMesh;
    vector<ofVec3f>frozenVetices;
    
    void setup(){
        
        
        texSpline.allocate(128,128, GL_RGBA);
        
        renderer.allocate(ofGetWidth(), ofGetHeight(), GL_RGBA);//A32F_ARB);
        renderer.begin();
        ofClear(255,255,255, 0);
        // load the texure
        //ofDisableArbTex();
        ofLoadImage(texSpline, "dot.png");
        ofEnableSmoothing();
        renderer.end();
        
        vector<ofVec3f> vel;
        // load the shader
        
        // shaderSplineReplace.setupShaderFromSource(GL_VERTEX_SHADER, splineReplaceShader);
        // shaderSplineReplace.linkProgram();
        shaderSplineReplace.load("shaders/shader");
        shaderSplineReplace.setUniform2f("screenSize", (float)ofGetWidth(), (float)ofGetHeight());
    }
    
    void update(ofxMultiKinectV2 *kinect, vector<Body> bodies){
        //use larger ofPoly? - to get a les sharp outline...
        mesh.clear();
        
        if(bodies.size()>0){
            for(vector<ofVec2f>::iterator ind=bodies[0].indices.begin(); ind!=bodies[0].indices.end(); ind++){
                
                float dist = kinect->getDistanceAt(ind->x, ind->y);
                
                ofVec3f pt = kinect->getWorldCoordinateAt(ind->y,ind->x,dist);
                
                pt.x += translateX;
                pt.y += translateY;
                pt.z += translateZ;
                
                float y = pt.y;
                float z = pt.z;
                
                pt.y = y*cos(tilt)-z*sin(tilt);
                pt.z = y*sin(tilt)+z*cos(tilt);
                
                // pt.set(x,y,dist);
                
                // ofColor c;
                // float h = ofMap(dist, 0, 8000, 255, 100, true);
                // c.setHsb(255, h ,255);
                
                
                // c = ofFloatColor(ofNoise((dist+ofGetFrameNum()/10)*0.001, (dist+ofGetFrameNum())*0.001));
                
                // mesh.addColor(c);
                
                mesh.addVertex(pt);
                
            }
        }
        
    }//end update
    
    void draw(){
        glPointSize(pointSize);
        mesh.drawVertices();
    }
    
    ofFloatPixels backgroundPix;
    float getBackgroundAt(int x, int y) {
        if (!backgroundPix.isAllocated()) {
            return 0.0f;
        }
        return backgroundPix[x + y * backgroundPix.getWidth()] * 0.1; // mm to cm
    }
    
    void setBackgroundSubstract(ofxMultiKinectV2 *kinect){
        backgroundPix = kinect->getDepthPixelsRef();
        // to avoid noise, reduce depth for some cm.
        for(int i = 0; i<backgroundPix.size(); i++){
            if(backgroundPix[i]>100){
                backgroundPix[i]-=100;
            }
        }
    }
    
    void fall(){
        collapse+= ofRandom(20);
        vector<ofVec3f> p = mesh.getVertices();
        float acc = 0.8;
        
        vel.resize(p.size());

        for(int i = 0 ; i<p.size();i++){
            if(p[i].y<collapse){
                vel[i].y+=acc;
                p[i].y+=vel[i].y;
                if(vel[i].y>1){
                    p[i].x+=ofRandom(-.5,.5);
                    p[i].z+=ofRandom(-.5,.5);
                }
                
                
                if(p[i].y > floor){
                    vel[i].y *= -ofRandom(.3,.6);
                    p[i].y = floor;
                }
            }
        }
        mesh.clear();
        mesh.addVertices(p);
    }
};