uniform vec2 u_resolution;
uniform float scanline;
uniform sampler2DRect tex0;

varying vec2 texCoordVarying;

void main()
{
    vec2 uv = gl_FragCoord.xy / u_resolution.xy;
    vec4 tex = texture2DRect(tex0, texCoordVarying);
    //vec4 tex = texture2DRect(tex0, gl_TexCoord[0].xy);
    vec3 text = tex.xyz;
    
    
    vec3 defaultColor = vec3(1.);
    
    vec3 finalColor = defaultColor;
    float value = scanline/u_resolution.y;
    vec2 aniuv = uv - value;
    finalColor *= abs (0.9 / sin((aniuv.y)) * 0.01 ) + 0.0;
   
        finalColor += text * defaultColor;

    
    gl_FragColor =  vec4(finalColor, 1.);
    //gl_FragColor = vec4(1.);
}