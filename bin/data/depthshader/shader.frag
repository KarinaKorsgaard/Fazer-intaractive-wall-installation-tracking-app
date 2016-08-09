uniform sampler2DRect tex;
uniform float nearThreshold;
uniform float farThreshold;

uniform float edge;
uniform float edge2;
uniform float botDepth;
uniform float edgeDepth;
uniform float topDepth;
uniform float edgeDepth2;
//uniform float topEdgeDepth;

uniform vec2 u_resolution;

void main(void) {
    
    vec4 col = texture2DRect(tex, gl_TexCoord[0].xy);
    float value = col.r;
    
    float low1 = 0.;
    float high1 = 8000.; // Max Distance
    float low2 = 1.0;
    float high2 = 0.0;
    
    vec2 uv = gl_FragCoord.xy / u_resolution.xy;
    
    float top = 0.;
    float width = u_resolution.y;
    float bot = u_resolution.x;
    
    float x = uv.y*width;
    float d = 1.;

    
   if(x <= edge){
        float i = (x-top)/(edge-top);
        float val = i*(edgeDepth-topDepth)+topDepth;
       
        if(value > val) {
            d = 0.;
        } else {
            d = clamp(low2 + (value - low1) * (high2 - low2) / (high1 - low1), 0.0, 1.0);
        }
    }
    
    if(x>edge && x<edge2){
        

        if(value > edgeDepth) {
            d = 0.;
        } else {
            d = clamp(low2 + (value - low1) * (high2 - low2) / (high1 - low1), 0.0, 1.0);
        }
    
    }
    
    if(x > edge2){
        float i = (x-width)/(edge2-width);
        float val = i*(edgeDepth-botDepth)+botDepth;
        if(value > val) {
            d = 0.;
        }
        else {
            d = clamp(low2 + (value - low1) * (high2 - low2) / (high1 - low1), 0.0, 1.0);
        }
    }
    
    if (d == 1.0) {
        d = 0.0;
    }
    gl_FragColor = vec4(vec3(d), 1.0);
    
}

