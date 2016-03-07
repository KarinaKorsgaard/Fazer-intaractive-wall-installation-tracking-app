//
//  shaders.h
//  DAC-VirtualMirror
//
//  Created by Jonas Fehr on 26/01/16.
//
//
#define STRINGIFY(x) #x

static string depthFragmentShader =
STRINGIFY(
          uniform sampler2DRect tex;
          uniform float nearTreshold;
          uniform float farThreshold;
          
          uniform float botDepth;
          uniform float topDepth;
          uniform float edge;
          uniform float edgeDepth;
          void main()
          {
              vec4 col = texture2DRect(tex, gl_TexCoord[0].xy);
              float value = col.g;
              
//              float low1 = botDepth;
//              float edge1 = edgeDepth;
//              float high1 = topDepth;
              
              int width = ofGetWidth();
              int height = ofGetHeight();
              
              int top = 0;
              int bot = ofGetWidth();
              
              float d = 0.0;
              
              cv::Mat gradient = cv::Mat::zeros( cvSize(width,height), CV_8U );
              
              
              for(int x=0; x<gradient.cols; x++)
              {
                  if(x>=top && x<=edge){
                      float i = (float)(x-top)/(edge-top);
                      float val = i*(edgeDepth-topDepth)+topDepth;
                      
                      for(int y=0; y<gradient.rows; y++)
                      {
                          gradient.at<char>(y,x) = (int)val;
                      }
                      //fma(t, edgeDepth, fma(-t, topDepth, topDepth));
                  }else if(x>edge && x<=bot){
                      float i = (float)(x-edge)/(width-edge);
                      float val = i*(botDepth-edgeDepth)+edgeDepth;
                      for(int y=0; y<gradient.rows; y++)
                      {
                          gradient.at<char>(y,x) = (int)val;
                          
                          vec4 gradColor = vec4(val/255.):
                          if(gl_TexCoord.xy < gradColor.xy){
                              
                              d = 0.0;
                          }
                      }
                  }
              }
//            //  vec4 gradColor = vec4(val/255.):
//             // detectBody.gradient = gradient;
//              
//              for(int x=0; x<depthImage.cols; x++){
//                  for(int y=0; y<depthImage.rows; y++){
//                      
//                      if(gl_TexCoord.xy < gradient.at<int>(y,x)){
//                          //depthImage.at<int>(y,x) = 0.;
//                          
//                          d = 0.0;
//                      }
//                  }
//              }
//
//              
              float low2 = 1.0;
              float high2 = 0.0;
            //  float d = clamp(low2 + (value - low1) * (high2 - low2) / (high1 - low1), 0.0, 1.0);
              if (d == 1.0) {
                  d = 0.0;
              }
              gl_FragColor = vec4(vec3(d), 1.0);
          }
          );

static string irFragmentShader =
STRINGIFY(
          uniform sampler2DRect tex;
          void main()
          {
              vec4 col = texture2DRect(tex, gl_TexCoord[0].xy);
              float value = col.r / 65535.0;
              gl_FragColor = vec4(vec3(value), 1.0);
          }
          );



