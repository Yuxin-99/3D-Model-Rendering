uniform int rcState;
varying vec3 newPosition;
uniform int u_time0;

void main() {
    //Paint it red
    gl_FragColor = vec4(1, 0, 0, 1);

    // Change the colour based on keyboards
    if (1 == rcState)
    {
        gl_FragColor = vec4(0.93, 0.51, 0.93, 1);
    }
    if (2 == rcState)
    {
        gl_FragColor = vec4(0, 0.67, 1, 1);
    }
    if (3 == rcState)
    {
        gl_FragColor = vec4(0.05, 0.11, 0.22, 1);
    }
    if (4 == rcState)
    {
        vec3 color1 = vec3(0.0, 0.0, 1.0);
        vec3 color2 = vec3(1.0, 1.0, 1.0);
        float factor = sin(newPosition.x*5.0+newPosition.y*5.0+newPosition.z*5.0)*0.5+0.5;
        vec3 color=mix(color1,color2,factor)*1.0;
        gl_FragColor = vec4(color,1.0);
        return;
    }
    if (5 == rcState)
    {
        gl_FragColor = vec4(1.0,1.0,1.0,0.0);
    }
}