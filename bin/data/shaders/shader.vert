void main() {

    //gl_Position   = gl_ModelViewProjectionMatrix * gl_Vertex;
    gl_PointSize  = 5.0;
    gl_FrontColor = gl_Color;
    
    
    float spriteSize = 10.5;
    vec2 screenSize = vec2(800., 1280.);
    
    vec4 eyePos = gl_ModelViewMatrix * gl_Vertex;
    vec4 projVoxel = gl_ProjectionMatrix * vec4(spriteSize,spriteSize,eyePos.z,eyePos.w);
    vec2 projSize = screenSize * projVoxel.xy / projVoxel.w;
    gl_PointSize = 0.552 * (projSize.x+projSize.y);
    //gl_PointSize = 1.9 * (projSize.x+projSize.y);
    
    gl_Position = gl_ProjectionMatrix * eyePos;
    gl_FrontColor = gl_Color;

}