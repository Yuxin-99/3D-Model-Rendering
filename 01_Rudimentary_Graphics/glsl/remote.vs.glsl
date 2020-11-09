// The uniform variable is set up in the javascript code and the same for all vertices
uniform vec3 remotePosition;
varying vec3 newPosition;
uniform int rcState;
uniform int u_time0;

float rand(float n){
    return fract(sin(n) * 190608.1112);
}

void main() {
    newPosition = remotePosition + position;

    // Multiply each vertex by the model-view matrix and the projection matrix to get final vertex position
    gl_Position = projectionMatrix * modelViewMatrix * vec4(newPosition, 1.0);

    if (4 == rcState)
    {
        newPosition = vec3(newPosition.x + sin(float(u_time0)), newPosition.y - 1.0, newPosition.z);
        gl_Position = projectionMatrix * modelViewMatrix * vec4(newPosition, 1.0);
    }
    if (5 == rcState)
    {
        newPosition = vec3(newPosition.x, -newPosition.y, newPosition.z);
        gl_Position = projectionMatrix * modelViewMatrix * vec4(newPosition, 1.0);
    }
    if (6 == rcState)
    {
        vec4 worldPosition = modelMatrix * vec4(newPosition,1.0);
        vec3 origin = vec3(0,3.5,0);
        float distanceToOrigin = distance(origin.xyz, worldPosition.xyz);
        newPosition = newPosition.xyz + rand(distanceToOrigin) * normal * 0.1;
        gl_Position = projectionMatrix * modelViewMatrix * vec4(newPosition, 1.0);
    }
}