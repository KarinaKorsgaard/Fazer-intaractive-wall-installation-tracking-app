void main() {

    gl_Position   = gl_ModelViewProjectionMatrix * gl_Vertex;
    gl_PointSize  = 10.0;
    gl_FrontColor = gl_Color;

}