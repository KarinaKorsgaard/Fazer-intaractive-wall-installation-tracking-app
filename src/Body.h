//
//  Body.h
//
//  Created by Jonas Fehr on 02/01/16.
//
//
#pragma once

#include "ofMain.h"
#include "ofxCV.h"

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
    ofColor color;
    ofTrueTypeFont *font;
    bool pulse;
    int pulsingTimer;
    int pulsingTimerThres = 180;
    float orgSize = 50;
    bool amPulsing = false;
    int counter = 0;
    bool hasConnection = false;
    bool red;
    
    
    float friction = -0.98;
    float gravity = -0.02;
    float spring = 0.03;
  
    void checkCollision(Actor *ofActor){

        ofVec2f bVect;
        
//        int x = pos.x;
//        int y = pos.y;
//        int ox =ofActor->pos.x;
//        int oy =ofActor->pos.y;
//        
//        bVect.set(ofActor->pos.x - x, ofActor->pos.y - y);
//        
//        float bVectMag = sqrt(bVect.x * bVect.x + bVect.y * bVect.y);
//        
//        if (bVectMag < size/2  + ofActor->size/2){
//            red = true;
//            float theta  = atan2(bVect.y, bVect.x);
//            float sine = sin(theta);
//            float cosine = cos(theta);
//            
//            Actor bTemp0;
//            Actor bTemp1;
//            bTemp1.pos.x  = cosine * bVect.x + sine * bVect.y;
//            bTemp1.pos.y  = cosine * bVect.y - sine * bVect.x;
//            
//            // rotate Temporary velocities
//            ofVec2f vTemp0;
//            ofVec2f vTemp1;
//            vTemp0.x  = cosine * vel.x + sine * vel.y;
//            vTemp0.y  = cosine * vel.y - sine * vel.x;
//            vTemp1.x  = cosine * ofActor->vel.x + sine * ofActor->vel.y;
//            vTemp1.y  = cosine * ofActor->vel.y - sine * ofActor->vel.x;
//            
//            ofVec2f vFinal0;
//            ofVec2f vFinal1;
//            // final rotated velocity for b[0]
//            vFinal0.x = ((size/2 - ofActor->size/2) * vTemp0.x + 2 * ofActor->size/2 * vTemp1.x)
//            / (size/2+ ofActor->size/2);
//            vFinal0.y = vTemp0.y;
//            // final rotated velocity for b[0]
//            vFinal1.x = ((ofActor->size/2 - size/2) * vTemp1.x + 2 * size/2 * vTemp0.x)
//            / (size/2 + ofActor->size/2);
//            vFinal1.y = vTemp1.y;
//            
//            // hack to avoid clumping
//            bTemp0.pos.x += vFinal0.x;
//            bTemp1.pos.x += vFinal1.x;
//            bTemp0.pos.y += vFinal0.y;
//            bTemp1.pos.y += vFinal1.y;
//            
//            // rotate balls
//            Actor bFinal0;
//            Actor bFinal1;
//            bFinal0.pos.x = cosine * bTemp0.pos.x - sine * bTemp0.pos.y;
//            bFinal0.pos.y = cosine * bTemp0.pos.y + sine * bTemp0.pos.x;
//            bFinal1.pos.x = cosine * bTemp1.pos.x - sine * bTemp1.pos.y;
//            bFinal1.pos.y = cosine * bTemp1.pos.y + sine * bTemp1.pos.x;
//            
//            // update balls to screen position
//            ofActor->pos.x = x + bFinal1.pos.x;
//            ofActor->pos.y = y + bFinal1.pos.y;
//            pos.x = x + bFinal0.pos.x;
//            pos.y = y + bFinal0.pos.y;
//            
//            // update velocities
//            vel.x = cosine * vFinal0.x - sine * vFinal0.y;
//            vel.y = cosine * vFinal0.y + sine * vFinal0.x;
//            ofActor->vel.x = cosine * vFinal1.x - sine * vFinal1.y;
//            ofActor->vel.y = cosine * vFinal1.y + sine * vFinal1.x;
        
//            pos+=vel;
//            
//            ofActor->pos+=ofActor->vel;
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
 

        
        else{red = false;}
        
        
        
    }
    
    void update(){
        if(pos.x>ofGetWidth()-size/2){
            pos.x= ofGetWidth()-size/2;
            vel*=friction;

        }
        if(pos.x<size/2){
            pos.x= size/2;
            vel*=friction;

        }
        if(pos.y>ofGetHeight()-size/2){
            pos.y= ofGetHeight()-size/2;
            vel*=friction;

        }
        if(pos.y<size/2){
            pos.y= size/2;
            vel*=friction;
        }
        
        vel.y += gravity;
        pos+=vel;

        
    }

    void draw(){
//        pulsingTimer += orgSize / 50 ;
//        pulsingTimer=pulsingTimer%180;
//        size = orgSize + sin(ofDegToRad(pulsingTimer))*10;
//       
//        if(pulse ){
//        
//        pulsingTimer += orgSize / 5 ;
//      
//        size = orgSize + sin(ofDegToRad(pulsingTimer))*15;
//        
//       
//            if(pulsingTimer>pulsingTimerThres){
//                pulse = false;
//                size = orgSize;
//                amPulsing = false;
//                pulsingTimer = 0;
//            }
//        }
        
       // size = 50+50*sin(ofGetFrameNum());
        
        
    
//        if(tl>1){
//            tl = 0;
//            Ppos = pos;
//            
//            att.x = pos.x + ofRandom(-10,10);
//            att.y = pos.y + ofRandom(-10,10);
//            
//            if(att.x>ofGetWidth()){
//                att.x =ofGetWidth()-ofRandom(50);
//            }
//            else if(att.x<0){
//                att.x =ofRandom(50);
//            }
//            if(att.y>ofGetHeight()){
//                att.y =ofGetHeight()-ofRandom(50);
//            }
//            else if(att.y<0){
//                att.y =ofRandom(50);
//            }
//        }
//        tl+=0.01;
//        
//        if(tll>1){tlDown = true;}
//        if(tll<0){tlDown = false;}
//        if(tlDown){tll-=0.01;}
//        if(!tlDown){tll+=0.01;}
//        
//        
//        
//        ofSetColor(255,0,0);
//        pos = (att - Ppos)*tl+Ppos;
    
        ofFill();
        if(!red){
        ofSetColor(255,50);
        }
        else if(red){
            ofSetColor(255,0,0,50);
        }
        ofDrawCircle(pos, size/2);
        float w = size;
        float h = size*(img.getHeight()/img.getWidth());
        //int ratio = w/h;
        ofSetColor(255,160);
        img.draw(pos.x-w/2,pos.y-h/2,w,h);
       // ofSetColor(255);
       // font->drawString(Name,pos.x-size/2,pos.y+size/2);
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
    ofVec2f att;
    ofTrueTypeFont *font;
    bool addPower;
    int shooter;
    bool drawLines = true;
    bool fall = false;
    
    
    void draw(){
        
        if(fall){
            tl-=0.01;
            con = false;
        }
        
        shooter++;
        shooter = shooter%100;
        
//        if(shooter==0){
//            for(int i = 0; i<connections.size();i++){
//                connections[i]->pulse = true;
//            }
//        }
        
        if(con && tl<0.95){
            tl+=0.01;
            addPower = true;
        }

        if(tl>=0){
            for(int i = 0; i<connections.size();i++){
                
                //float dist = ofDist(posX,posY,connections[i].posX,connections[i].posY);
                
                ofVec2f a = pos;
                ofVec2f b = connections[i]->pos;
                ofVec2f c = (b - a)*tl+a;
                ofVec2f d = (b - a)*shooter/100+a;
                //  ofVec2f e = (a - b)*shooter/1000+b;
                
                ofSetColor(20,255,20);
                ofDrawEllipse(d.x,d.y,1,1);
                // ofDrawEllipse(e.x,e.y,2,2);
                
                //connections[i]->hasConnection = true;
                ofSetColor(255,tl*100);
                ofSetLineWidth(0.5);
                ofDrawLine(pos,c);
            }
            ofFill();
            
            //ofDrawBitmapString(Name, posX, posY);
            
            ofSetColor(0,tl*200);
            ofRectangle rect = font->getStringBoundingBox(Name, 0,0);
            ofRectangle myRect;
            myRect.x = pos.x-5-rect.width/2;
            myRect.y = pos.y-5-rect.height;
            myRect.width = rect.width+10;
            myRect.height = rect.height+10;
            
            ofDrawRectRounded(myRect, 5);
            
            ofSetColor(255,tl*255);
            font->drawString(Name,pos.x-rect.width/2,pos.y-2);
            
        }
    };
    
};
