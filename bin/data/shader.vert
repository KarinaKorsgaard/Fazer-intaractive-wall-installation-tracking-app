uniform vec2 screenSize;


void main()
{
    float spriteSize = gl_Normal.x;
    
    vec4 eyePos = gl_ModelViewMatrix * gl_Vertex;
    vec4 projVoxel = gl_ProjectionMatrix * vec4(spriteSize,spriteSize,eyePos.z,eyePos.w);
    vec2 projSize = screenSize * projVoxel.xy / projVoxel.w;
    gl_PointSize = 0.152 * (projSize.x+projSize.y);
    //gl_PointSize = 1.9 * (projSize.x+projSize.y);

    gl_Position = gl_ProjectionMatrix * eyePos;
    gl_FrontColor = gl_Color;
}
