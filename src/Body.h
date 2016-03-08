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
    int size = 0;
    ofImage img;
    ofColor color;
    ofTrueTypeFont *font;
    
    bool hasConnection = false;

    void draw(){
        
       // size = 50+50*sin(ofGetFrameNum());
        pos+=vel;
        
        if(pos.x>ofGetWidth()){
            pos.x= ofGetWidth();
            vel*=-1;
        }
        if(pos.x<0){
            pos.x= 0;
            vel*=-1;
        }
        if(pos.y>ofGetHeight()){
            pos.y= ofGetHeight();
            vel*=-1;
        }
        if(pos.y<0){
            pos.y= 0;
            vel*=-1;
        }

        
    
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
        ofSetColor(255,100);
        ofDrawCircle(pos, size);
        ofSetColor(255);
        font->drawString(Name,pos.x-size/2,pos.y+size/2);
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
        
        if(con && tl<1){
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
                
                ofDrawEllipse(d.x,d.y,1,1);
                // ofDrawEllipse(e.x,e.y,2,2);
                
                connections[i]->hasConnection = true;
                ofSetColor(255,tl*100);
                ofSetLineWidth(0.5);
                ofDrawLine(pos,c);
            }
            ofFill();
            
            //ofDrawBitmapString(Name, posX, posY);
            
            ofSetColor(0,tl*600);
            ofRectangle rect = font->getStringBoundingBox(Name, 0,0);
            ofRectangle myRect;
            myRect.x = pos.x-5-rect.width/2;
            myRect.y = pos.y-5-rect.height;
            myRect.width = rect.width+10;
            myRect.height = rect.height+10;
            
            ofDrawRectRounded(myRect, 5);
            
            ofSetColor(255,tl*600);
            font->drawString(Name,pos.x-rect.width/2,pos.y);
            
        }
    };
    
};
