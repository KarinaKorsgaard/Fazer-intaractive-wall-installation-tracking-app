uniform vec2 u_resolution;
uniform float scanline;
uniform sampler2DRect tex0;

varying vec2 texCoordVarying;

float hash(float x)
{
    return fract(sin(x*.0127863)*17143.321); //decent hash for noise generation
}

void main()
{
    vec2 uv = gl_FragCoord.xy / u_resolution.xy;
    vec4 tex = texture2DRect(tex0, texCoordVarying);
    //vec4 tex = texture2DRect(tex0, gl_TexCoord[0].xy);
    vec3 text = tex.xyz;
    
    
    vec3 defaultColor = vec3(.9,0.9,1.0);
    
    vec3 finalColor = defaultColor;
    float value = (u_resolution.y-scanline)/u_resolution.y;
    vec2 aniuv = uv - value;
    finalColor *= abs (0.05 / sin((aniuv.y)) * .12 ) + 0.0;

    if(finalColor.r>0.9){
        finalColor -= text * defaultColor;
        //finalColor*=hash( u_resolution.x);
//        if(finalColor.r<0.5){
//            finalColor = vec3(1.);
//        }
    }else{
        finalColor += text * defaultColor;
    }
    
    
    gl_FragColor =  vec4(finalColor, 1.);
    //gl_FragColor = vec4(1.);
}