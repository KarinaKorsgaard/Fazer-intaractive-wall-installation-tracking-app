//
//  Body.h
//
//  Created by Jonas Fehr on 02/01/16.
//
//
#pragma once

#include "ofMain.h"
#include "ofxCV.h"
#include "ofxOsc.h"

#define RES_HEIGHT 1280
#define RES_WIDTH 800

//#include "ofxCvMin.h"

class Body
{
public:
    ofVec2f armBase; // Base of the arm (where the arm intersects the edge of the table)
    int indxBaseStart;
    int indxBaseEnd;
    
    ofPolyline boundaryPoly;
    
    vector<ofPoint> boundary;
    vector<ofVec2f> indices;
    
    //ofVec2f palmCenter; // not used
    ofVec2f centroid;
    ofPixels bodyBlobPix;
    
    cv::Mat bodyBlob = cv::Mat::zeros( cvSize(512, 424), CV_8UC1 );
    cv::Mat bodyBlob2 = cv::Mat::zeros( cvSize(512, 424), CV_8UC1 );
    
    
    void draw(){
        ofSetColor(255, 0, 0);
        ofNoFill();
        ofSetColor(0, 0, 255);
        boundaryPoly.draw();
        ofDrawEllipse(centroid.x, centroid.y, 50, 50);
    };
    
};



class Actor
{
public:
    
    string Name;
    // float posX,posY;
    ofVec2f pos;
    ofVec2f orgPos;
    ofVec2f vel;
    float size = 50;
    ofImage img;
    ofImage img2;
    ofColor color;
    ofTrueTypeFont *font;
    
    ofxOscMessage m;

    int counter = 0;
    bool hasConnection = false;
    
    int alpha = 0;
    ofRectangle *rect = new ofRectangle;
    
    float friction = -0.98;
    float gravity = -0.02;
    float spring = 0.03;
    
  
    void checkCollision(Actor *ofActor){
        float x = pos.x;
        float y = pos.y;
        float dx = ofActor->pos.x-x;
        float dy = ofActor->pos.y-y;
        
        float minDist = size/2+ofActor->size/2;
        float distance = sqrt(dx*dx + dy*dy);
        
        if(distance < minDist){
            float angle = atan2(dy, dx);
            float targetX = x + cos(angle) * minDist;
            float targetY = y + sin(angle) * minDist;
            float ax = (targetX - ofActor->pos.x) * spring;
            float ay = (targetY - ofActor->pos.y) * spring;
            vel.x -= ax;
            vel.y -= ay;
            ofActor->vel.x += ax;
            ofActor->vel.y += ay;
            
        }
        
    }
    
    void update(){
        
        float boundaryXH = RES_WIDTH - size/2;
        float boundaryYH = RES_HEIGHT - size/2;
        float boundaryXL = size/2;
        float boundaryYL = size/2;
        
        
        // bounce of body bounding box set in void detectPerson();
        // top:
        if(pos.x>rect->x && pos.x<rect->x+rect->width && pos.y> rect->y){
            pos.y = rect->y;
            vel*=friction;
        }
        // side 1
        if(pos.y>rect->y && pos.y<rect->y+rect->height && pos.x> rect->x){
            pos.y = rect->y;
            vel*=friction;
        }
        // side 2
        if(pos.y>rect->y && pos.y<rect->y+rect->height && pos.x < rect->x + rect->width){
            pos.y = rect->y;
            vel*=friction;
        }
        
        //bounce of walls
        if(pos.x>boundaryXH){
            pos.x= boundaryXH;
            vel*=friction;
        }
        if(pos.x<boundaryXL){
            pos.x= boundaryXL;
            vel*=friction;
        }
        if(pos.y>boundaryYH){
            pos.y =boundaryYH;
            vel*=-1; // no friction on ceiling
        }
        if(pos.y<boundaryYL){
            pos.y=boundaryYL;
            vel*=friction;
        }
        
        // remove from top corners / avoid getting stuck
        if(pos.x > RES_WIDTH-size/2 && pos.y < size/2){
            vel.set(-1,1);
        }
        if(pos.x < size/2 && pos.y < size/2){
            vel.set(1,1);
        }
//
        // min vel = 0.5
        if(vel.length() <0.5){
            vel *= 1.2;
        }
//
        // add gravity (gravity = 0 atm)
        if(pos.y>RES_HEIGHT/2){
            vel.y += gravity;
        }
        
        pos+=vel;
        
        // counter for alpha value- hasConnection controlled by dataPoints
        if(hasConnection && alpha < 230){
            alpha +=5;

            
            
        }else if(!hasConnection && alpha >0){
            alpha -=5;
        }
    }
    void drawUC(){
            float w = size;
            float h = size*(img2.getHeight()/img2.getWidth());
            ofSetColor(255, alpha);
            img2.draw(pos.x-w/2,pos.y-h/2,w,h);
    }


    void draw(){
            float w = size;
            float h = size*(img.getHeight()/img.getWidth());
            //int ratio = w/h;
            ofSetColor(255, alpha);
            img.draw(pos.x-w/2,pos.y-h/2,w,h);
    };
};



class DataPoint
{
public:
    
    string Name;
    ofVec2f pos;
    
    vector<Actor> *actors;
    vector<int>connectToActorID;
    bool isSet;
    float mover = 0;
    
    int shooter;
    bool drawLines = true;
    bool fall = false;
    
    bool bAlpha = false;
    bool bLength = false;
    float tlAlpha;
    float tlLength;
    bool sendOsc = true; 
    ofxOscMessage m;
    float lPos =0;
    float lVel =0.05;
    
    ofVec2f att;
    ofTrueTypeFont *font;
    
    void draw(){
        
        //if(tlLength>0.95)
        shooter++;
        shooter = shooter%100;
        
        //alpha counter
        if(bAlpha && tlAlpha<1){
            tlAlpha=1;
        } else if (!bAlpha && tlAlpha > 0){
            tlAlpha -= lPos;
            lPos += lVel;
        }
        
        //send beginning to appear and disappered. 
//        if(bAlpha){
//            m.clear();
//            m.setAddress("/dataPoint_"+Name);
//            m.addFloatArg(tlAlpha);
//        }

        //cout << "alpha: "<< alpha << " tlAlpha: " << tlAlpha << endl;
        
        //start line length counter
        if(bLength && tlLength<1.){
            tlLength+=0.03;
        } else if (!bLength && tlLength > 0){
            tlLength-=0.05;
        }
        
        if(tlAlpha>0){
            // draw lines if con. con = true if scanLine is all up.
            if(bLength){
                for(int i = 0; i<connectToActorID.size();i++){
                    // start drawing the connection to the actors
                    ofVec2f posAct = actors->at(connectToActorID[i]).pos;
                    actors->at(connectToActorID[i]).hasConnection = true;
                    ofVec2f c = pos.getInterpolated(posAct, tlLength);
                    ofVec2f d = pos.getInterpolated(posAct, (float)shooter/100);
                    
                    ofSetColor(255);
                    if(tlLength >= 1) ofDrawEllipse(d.x,d.y,2,2);
                    // ofDrawEllipse(e.x,e.y,2,2);
                    
                    ofSetColor(255,tlAlpha*100);
                    ofSetLineWidth(0.5);
                    ofDrawLine(pos,c);
                }
            }

            
            // draw dataPoints_
            ofFill();
            ofSetColor(0,tlAlpha*200);
            ofRectangle rect = font->getStringBoundingBox(Name, 0,0);
            ofRectangle myRect;
            int frame = 6;
            int adjustR = 30;
            myRect.x = pos.x-frame*2 - adjustR;//-rect.width/2;
            myRect.y = pos.y-frame-rect.height/2;
            myRect.width = rect.width+frame*2;
            myRect.height = rect.height+frame*2;
            
            ofDrawRectRounded(myRect, 5);
            
            ofSetColor(255,tlAlpha*255);
            font->drawString(Name,pos.x-frame-adjustR,pos.y+frame);
            // draw dataPoints_end
            
            if(fall){
                bLength = false;
                bAlpha = false;
                
                if(tlLength > 0){
                    for(int i = 0; i<connectToActorID.size();i++){
                        // start drawing the connection to the actors
                        ofVec2f posAct = actors->at(connectToActorID[i]).pos;
                        ofVec2f c = posAct.getInterpolated(pos, tlLength);
                        
                        
                        ofSetColor(255,tlAlpha*100);
                        ofSetLineWidth(0.5);
                        ofDrawLine(posAct,c);
                    }
                }
            }
        }
    };
    
};
