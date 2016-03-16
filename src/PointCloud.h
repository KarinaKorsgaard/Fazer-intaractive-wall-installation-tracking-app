//
//  pointCloud.hpp
//  VirtualMirror
//
//  Created by Jonas Fehr on 25/01/16.
//
//

#include "ofMain.h"
#include "ofxKinectV2.h"
#include "DetectBody.h"
#include "ofxAutoReloadedShader.h"
#include "ofxOsc.h"

#define RES_HEIGHT 1280
#define RES_WIDTH 800

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
    
    ofVbo vbo;
    ofxAutoReloadedShader shaderSplineReplace;
    ofTexture texSpline;
    
    
    void setStep(int _step){ step = _step;}
    void setPointSize(int _pointSize){ pointSize = _pointSize;}
    
    ofMesh mesh;

    //ofMesh frozenMesh;
    vector<ofVec3f>frozenVetices;
    
    void setup(){
        
        
        texSpline.allocate(128,128, GL_RGBA);

        ofDisableArbTex();
        ofLoadImage(texSpline, "dot.png");
        ofEnableArbTex();
        
        vector<ofVec3f> vel;

        shaderSplineReplace.load("shaders/shader");
        shaderSplineReplace.setUniform2f("screenSize", (float)RES_WIDTH, (float)RES_HEIGHT);
    }
    
    void update(ofxKinectV2 *kinect, vector<Body> bodies){
        
        mesh.clear();
        int indx = 0;
        
        if(bodies.size()>0){
            for(vector<ofVec2f>::iterator ind=bodies[0].indices.begin(); ind!=bodies[0].indices.end(); ind++){
                indx++;
                
                if(indx%2 == 0){
                    
                    ofVec3f pt = kinect->getWorldCoordinateAt(ind->x,ind->y);
                    
                    float swap = pt.x;
                    pt.x = pt.y;
                    pt.y = swap;
                    
                    pt.x += translateX;
                    pt.y += translateY;
                    pt.z += translateZ;
                    
                    float y = pt.y;
                    float z = pt.z;
                    
                    pt.y = y*cos(tilt)-z*sin(tilt);
                    pt.z = y*sin(tilt)+z*cos(tilt);
                    
                    pt.x = RES_WIDTH-pt.x;
                    pt.y = RES_HEIGHT-pt.y;
                    
                    mesh.addVertex(pt);
                }
                
            }
        }
        
    }//end update
    
    void draw(){
        glPointSize(pointSize);
        mesh.drawVertices();
    }
    
    void drawPSpline(){
        glDepthMask(GL_FALSE);
        
        ofSetColor(255,120);
        
        ofEnableAlphaBlending();
        // this makes everything look glowy :)
        ofEnableBlendMode(OF_BLENDMODE_ADD);
        // ofEnableBlendMode(OF_BLENDMODE_SCREEN);
        
        ofEnablePointSprites();
        shaderSplineReplace.begin();
        // bind the texture so that when all the points
        // are drawn they are replace with our dot image
        texSpline.bind();
        vbo.draw(GL_POINTS, 0, (int)mesh.getVertices().size());
        texSpline.unbind();
        shaderSplineReplace.end();
        ofDisablePointSprites();
        ofDisableBlendMode();
        //ofDisableAlphaBlending();
        
        glDepthMask(GL_TRUE);
    }
    
    void fall(){
        collapse += ofRandom(140);
        vector<ofVec3f> p = mesh.getVertices();
        float acc = 2.8;
        
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
                    vel[i].y *= -ofRandom(.5,.7);
                    p[i].y = floor;
                    
                }
            }
        }
        mesh.clear();
        mesh.addVertices(p);
    }
};