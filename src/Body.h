//
//  Body.h
//
//  Created by Jonas Fehr on 02/01/16.
//
//
#pragma once

#include "ofMain.h"
#include "ofxCV.h"

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
    ofVec2f vel;
    float size = 50;
    ofImage img;
    ofImage img2;
    ofColor color;
    ofTrueTypeFont *font;

    int counter = 0;
    bool hasConnection = false;
    
    int alpha =0;
    ofRectangle *rect = new ofRectangle;
    
    float friction = -0.98;
    float gravity = 0;//-0.02;
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
        if(pos.x>RES_WIDTH-size/2){
            pos.x= RES_WIDTH-size/2;
            vel*=friction;
        }
        if(pos.x<size/2){
            pos.x= size/2;
            vel*=friction;
        }
        if(pos.y>RES_HEIGHT-size/2){
            pos.y= RES_HEIGHT-size/2;
            vel*=-1; // no friction on ceiling
        }
        if(pos.y<size/2){
            pos.y= size/2;
            vel*=friction;
        }
        
        // remove from top corners / avoid getting stuck
        if(pos.x > RES_WIDTH-size/2 && pos.y < size/2){
            vel.set(-1,1);
        }
        if(pos.x < size/2 && pos.y < size/2){
            vel.set(1,1);
        }
        
        // min vel = 0.5
        if(vel.length() <0.5){
            vel *= 1.2;
        }
        
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
            ofSetColor(255,alpha);
            img2.draw(pos.x-w/2,pos.y-h/2,w,h);
    }


    void draw(){
            float w = size;
            float h = size*(img.getHeight()/img.getWidth());
            //int ratio = w/h;
            ofSetColor(255,alpha);
            img.draw(pos.x-w/2,pos.y-h/2,w,h);
    };
};



class DataPoint
{
public:
    
    string Name;
    ofVec2f pos;
    
    // float posX,posY,posZ;
    vector<Actor*>connections;
    bool isSet;
    float tl;
    float mover = 0;
    bool con = false;
    bool addPower;
    int shooter;
    bool drawLines = true;
    bool fall = false;
    bool alpha = false;
    
    ofVec2f att;
    ofTrueTypeFont *font;
    
    
    void draw(){
        
        if(fall){
            tl-=0.01;
            con = false;
        }
        
        shooter++;
        shooter = shooter%100;
        

        if(alpha && tl<0.95){
            tl+=0.01;
            addPower = true;
        }

        if(tl>=0){
            
            ofFill();
            
            //ofDrawBitmapString(Name, posX, posY);
            
            ofSetColor(0,tl*200);
            ofRectangle rect = font->getStringBoundingBox(Name, 0,0);
            ofRectangle myRect;
            myRect.x = pos.x-6-rect.width+40;
            myRect.y = pos.y-6-rect.height;
            myRect.width = rect.width+12;
            myRect.height = rect.height+12;
            
            ofDrawRectRounded(myRect, 5);
            
            ofSetColor(255,tl*255);
            font->drawString(Name,pos.x-rect.width+40,pos.y);
            
            for(int i = 0; i<connections.size();i++){
                
                //float dist = ofDist(posX,posY,connections[i].posX,connections[i].posY);
                
                ofVec2f a; a.set(pos.x-(rect.width+40)/2, pos.y);
                ofVec2f b = connections[i]->pos;
                ofVec2f c = (b - a)*tl+a;
                ofVec2f d = (b - a)*shooter/100+a;
                //  ofVec2f e = (a - b)*shooter/1000+b;
                
                ofSetColor(20,255,20);
                ofDrawEllipse(d.x,d.y,1,1);
                // ofDrawEllipse(e.x,e.y,2,2);
                if(con){
                    connections[i]->hasConnection = true;
                
                }
                ofSetColor(255,tl*100);
                ofSetLineWidth(0.5);
                ofDrawLine(pos,c);
            }
            
        }
    };
    
};
