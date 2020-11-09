// Create shared variable for the vertex and fragment shaders
varying vec3 interpolatedNormal;
varying float distanceToRemote;
uniform vec3 remotePosition;
varying vec4 finalPos;
varying float distanceToOrigin;

uniform int rcState;
uniform int u_time;

float rand(float n){
    return fract(sin(n) * 190608.1112);
}

void main() {
    // Set shared variable to vertex normal
    interpolatedNormal = normal;

    vec3 newPos = vec3(0, 0, -0.35) + position;

    mat4 scaleMatrix = mat4(5.6,0,0,0,0,5.6,0,0,0,0,5.6,0,0,0,0,1);
    // Multiply each vertex by the model-view matrix and the projection matrix to get final vertex position
    gl_Position = projectionMatrix * modelViewMatrix * scaleMatrix * vec4(newPos, 1.0);

    finalPos = (scaleMatrix * vec4(newPos, 1.0));
    vec4 worldPosition = modelMatrix * finalPos;
    distanceToRemote = distance(remotePosition.xyz, worldPosition.xyz);
    distanceToOrigin = worldPosition.y - 0.0;

    if (6 == rcState)
    {
        vec3 newPos0 = finalPos.xyz + rand(distanceToRemote) * normal;
        gl_Position = projectionMatrix * modelViewMatrix * vec4(newPos0, 1.0);
    }

}
