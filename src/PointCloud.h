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
    
    int nearCut = 250;
    int farCut = 316;
    
    int tilt = 134;
    
    //vector<float>newDist;
    //vector<float>prevDist;
    
    ofFbo renderer;
    ofVbo vbo;
    ofShader shaderSplineReplace;
    ofTexture texSpline;
    
    
    
    
    
    
    void setStep(int _step){ step = _step;}
    void setPointSize(int _pointSize){ pointSize = _pointSize;}
    void setActiveSpace(int _nearCut, int _farCut){ nearCut = _nearCut; farCut = _farCut;}
    
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
        
        
        // load the shader
        
        // shaderSplineReplace.setupShaderFromSource(GL_VERTEX_SHADER, splineReplaceShader);
        // shaderSplineReplace.linkProgram();
        shaderSplineReplace.load("shaders/shader");
        shaderSplineReplace.setUniform2f("screenSize", (float)ofGetWidth(), (float)ofGetHeight());
    }
    
    void update(ofxMultiKinectV2 *kinect, vector<Body> bodies){
        //use larger ofPoly? - to get a les sharp outline...
        mesh.clear();
        
        //        int h = kinect->getDepthPixelsRef().getHeight();
        //        int w = kinect->getDepthPixelsRef().getWidth();
        //
        //
        //        for(int x = 0; x < w; x += step) {
        //            float nearCutTilt = ofMap(x,0,w,nearCut-tilt,nearCut);
        //            float farCutTilt = ofMap(x,0,w,farCut-tilt,farCut);
        //
        //            for(int y = 0; y < h; y += step) {
        //
        //                float dist = kinect->getDistanceAt(x, y);
        //                //newDist.push_back(dist);
        //
        //
        //                float bgDist = farCut;
        //                if(backgroundPix.isAllocated()){
        //                    bgDist = getBackgroundAt(x,y);
        //                }
        //
        //                if(dist > nearCutTilt && dist < farCutTilt && dist<bgDist
        //                   ) {
        //
        //                    ofVec3f pt= kinect->getWorldCoordinateAt(y, x, dist);
        //                    // pt.set(x,y,dist);
        //
        //                    ofColor c;
        //                    float h = ofMap(dist, 50, 200, 0, 255, true);
        //                    c.setHsb(h, 255, 255);
        //                    //c = ofFloatColor(ofNoise((x+ofGetFrameNum()/10)*0.001, (y+ofGetFrameNum())*0.01));
        //
        //                    // mesh.addColor(c);
        //
        //                    mesh.addVertex(pt);
        //
        //                }
        //
        //            }
        //        }
        //
        //        for(vector<Body>::iterator b=bodies.begin(); b!=bodies.end(); b++){
        //             for(vector<ofVec2f>::iterator ind=b->indices.begin(); ind!=b->indices.end(); ind++){
        if(bodies.size()>0){
            for(vector<ofVec2f>::iterator ind=bodies[0].indices.begin(); ind!=bodies[0].indices.end(); ind++){
                
                float dist = kinect->getDistanceAt(ind->x, ind->y);
                
                ofVec3f pt= kinect->getWorldCoordinateAt(ind->y, ind->x, dist);
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
    
    
    
    void draw(int x, int y, int z){
        // ofFbo preRender;
        // preRender.allocate(ofGetWidth(), ofGetHeight(), GL_RGB);
        
        // preRender.begin();
        // ofBackground(0);
        
        
        glPointSize(pointSize);
        ofPushMatrix();
        ofTranslate(x,y,z);
        mesh.drawVertices();
        ofPopMatrix();
        // preRender.end();
        
        
        //post.begin();
        //  preRender.draw(0,0);
        //post.end();
        
        // upload the data to the vbo
        
        /*    // vector to store values for shader communication
         int total = mesh.getNumVertices();
         for(int i = 0; i < total; i++){
         p_pos[i] = mesh.getVertex(i);
         //   p_pos[i].x = mesh.getVertex(i).x+524;
         //   p_pos[i].y = -mesh.getVertex(i).y-424;
         //   p_pos[i].z = -mesh.getVertex(i).z-700;
         }
         
         vbo.updateVertexData(&p_pos[0], total);
         
         
         renderer.begin();
         ofPushMatrix();
         ofScale(1, -1, -1);
         ofTranslate(524, -424, -700);
         
         ofEnableAlphaBlending();
         ofSetColor(100);
         
         ofEnableBlendMode(OF_BLENDMODE_ADD);
         ofEnablePointSprites();
         shaderSplineReplace.begin();
         shaderSplineReplace.setUniform2f("screenSize", (float)ofGetWidth(), (float)ofGetHeight());
         texSpline.bind();
         vbo.draw(GL_POINTS, 0, (int)mesh.getNumVertices());
         texSpline.unbind();
         shaderSplineReplace.end();
         ofDisablePointSprites();
         ofDisableBlendMode();
         
         glDepthMask(GL_TRUE);
         ofPopMatrix();
         renderer.end();
         
         ofSetColor(255, 255);
         
         renderer.draw(0, 0);
         */
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
    
};