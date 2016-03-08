uniform vec2 u_resolution;
uniform float scanline;
uniform sampler2DRect tex;

void main()
{
    vec2 uv = gl_FragCoord.xy / u_resolution.xy;
    vec4 col = texture2DRect(tex, gl_TexCoord[0].xy);
    
   // vec3 tex = texture2D(tex0, uv).xyz;
    
    
    vec4 defaultColor = vec4(0.6,0.8,1.0,1.);
    
    vec4 finalColor = defaultColor;
    float value = scanline/u_resolution.y;
    vec2 aniuv = uv - value;
    finalColor *= abs (0.9 / sin((aniuv.y)) * 0.01 ) + 0.2;
    if(col.b<0.9){
        finalColor += col * defaultColor;
    }else if(col.b>0.9){
        finalColor -= col * defaultColor;
    }
    
    if(finalColor.r>.6){
        finalColor = vec4(1.);
    }
    
    gl_FragColor = finalColor;// vec4(finalColor, 1.0);
    //gl_FragColor = vec4(1.);
}