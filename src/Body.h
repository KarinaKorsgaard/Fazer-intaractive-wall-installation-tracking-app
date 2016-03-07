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
    
    ofVec2f palmCenter;
    ofVec2f centroid;
    ofPixels bodyBlobPix;
    
    cv::Mat bodyBlob = cv::Mat::zeros( cvSize(512, 424), CV_8UC1 );
    cv::Mat bodyBlob2 = cv::Mat::zeros( cvSize(512, 424), CV_8UC1 );
    
    
    void draw(){
        ofSetColor(255, 0, 0);
        ofNoFill();
        //ofDrawCircle(palmCenter, 20);
        // ofDrawCircle(centroid, 10);
        //ofDrawLine(palmCenter, armBase);
        
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
    ofVec2f Ppos;
    ofVec2f att;
    float tl = 0;
    int size = 0;
    ofImage img;
    ofColor color;
    ofTrueTypeFont *font;
    bool hasConnection = false;
    bool tlDown;
    float tll;
    void draw(){
        
        if(tl>1){
            
            
            tl = 0;
            Ppos = pos;
            
            att.x = pos.x + ofRandom(-10,10);
            att.y = pos.y + ofRandom(-10,10);
            
            if(att.x>ofGetWidth()){
                att.x =ofGetWidth()-ofRandom(50);
            }
            else if(att.x<0){
                att.x =ofRandom(50);
            }
            if(att.y>ofGetHeight()){
                att.y =ofGetHeight()-ofRandom(50);
            }
            else if(att.y<0){
                att.y =ofRandom(50);
            }
            
        }
        tl+=0.01;
        
        if(tll>1){tlDown = true;}
        if(tll<0){tlDown = false;}
        
        if(tlDown){tll-=0.01;}
        
        if(!tlDown){tll+=0.01;}
        
        
        
        ofSetColor(255,0,0);
      //  ofDrawBitmapString(ofToString(tl), pos.x, pos.y);
        pos = (att - Ppos)*abs(tl)+Ppos;
        
       // if(hasConnection){
            ofFill();
            ofSetColor(255,100);
            ofDrawEllipse(pos,50+tll*10,50+tll*10);

            ofSetColor(255);
            font->drawString(Name,pos.x-30,pos.y+10);
            //ofDrawBitmapString(Name, posX, posY);
            //img.draw(posX,posY);
        //}
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
    
    void update(){
        
    }
    
    void draw(){
        shooter++;
        shooter = shooter%100;
        
        if(con && tl<1){
            tl+=0.01;
            addPower = true;
        }
        if(tl == 1 && addPower){
            for(int i = 0; i<connections.size();i++){
                connections[i]->size++;
            }
            addPower = false;
        }
        
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
        
        
    };
    
};
