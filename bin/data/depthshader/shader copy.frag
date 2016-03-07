uniform sampler2DRect tex;
uniform float nearTreshold;
uniform float farThreshold;

uniform float edge;
uniform float botDepth;
uniform float botEdgeDepth;
uniform float topDepth;
uniform float topEdgeDepth;

uniform vec2 u_resolution;

void main(void) {
    
    vec4 col = texture2DRect(tex, gl_TexCoord[0].xy);
    float value = col.r;
    float low1 = nearTreshold;
    float high1 = farThreshold;
    float low2 = 1.0;
    float high2 = 0.0;

//    vec4 col = texture2DRect(tex, gl_TexCoord[0].xy);
//    float value = col.r;
//    float low1 = nearTreshold;
//    float high1 = farThreshold;
//    
    vec2 uv = gl_FragCoord.xy / u_resolution.xy;
    vec3 rgb;
//    
//    
//    // let's try a simple chrome effect
   // vec3 blue = vec3(topDepth);
    vec3 blue = vec3(botDepth);
    vec3 brown = vec3(topDepth);
    vec3 white = vec3(botEdgeDepth);
    vec3 black = vec3(topEdgeDepth);
//
    float horizon = edge;
//
    if (uv.x < horizon)
    {
        // Y will range 0 to horizon, so adjust up to get 0..1
        float yRange = uv.x * (1.0 / horizon);
        rgb = mix(brown, black, yRange);
    }
    else
    {
        // Y will range horizon to 1, so adjust range and subtract 1 to get 0..1
        float yRange = (uv.x * (1.0/horizon)) - 1.0;
        rgb = mix(white, blue, yRange);
    }
//
//    //fragColor = vec4(rgb, 1.0);
//    
    float d = clamp(low2 + (value - low1) * (high2 - low2) / (high1 - low1), 0.0, 1.0);
    
    if(d<rgb.r ){
        d = 0.0;
    }
////
//    
//    
//    float low2 = 1.0;
//    float high2 = 0.0;
//    float d = clamp(low2 + (value - low1) * (high2 - low2) / (high1 - low1), 0.0, 1.0);
//    if (d == 1.0) {
//        d = 0.0;
//    }
//    gl_FragColor = vec4(vec3(d),1.0));
    
        
        
    if (d == 1.0) {
        d = 0.0;
    }
  //667  gl_FragColor = vec4(rgb,1.);//vec4(vec3(d), 1.0);
    gl_FragColor = vec4(vec3(d), 1.0);

}

