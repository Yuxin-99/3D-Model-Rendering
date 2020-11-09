// Create shared variable. The value is given as the interpolation between normals computed in the vertex shader
varying vec3 interpolatedNormal;
varying float distanceToRemote;
varying vec4 finalPos;
varying float distanceToOrigin;

uniform int rcState;
uniform int u_time;

vec4 colorR = vec4(0.05,0.79,0.94,1.0);
vec4 colorS = vec4(1.0,0.38,0.20,1.0);
vec4 colorE = vec4(1.0,0.20,0.0,1.0);

void main() {
  // Set final rendered color according to the surface normal
  if (distanceToRemote < 2.1)
  {
    gl_FragColor = vec4(1,1,0,1);
  }
  else
  {
    gl_FragColor = vec4(normalize(interpolatedNormal), 1.0);
  }

  if (4 == rcState)
  {
    vec3 color1 = vec3(0.0, 0.0, 1.0);
    vec3 color2 = vec3(1.0, 1.0, 1.0);
    float factor = sin(finalPos.x*5.0+finalPos.y*5.0+finalPos.z*5.0)*0.5+0.5;
    vec3 color=mix(color1,color2,factor)*1.0;
    gl_FragColor = vec4(color,1.0);
    return;
  }

  if (5 == rcState)
  {
    if (u_time < 6) {
      if (distanceToOrigin < 3.0) {
        gl_FragColor = colorR;
      } else if (distanceToOrigin < 6.0) {
        gl_FragColor = colorS;
      } else if (distanceToOrigin < 9.0) {
        gl_FragColor = colorE;
      }
    } else if ((6 <= u_time) && (u_time < 12)) {
      if (distanceToOrigin < 3.0) {
        gl_FragColor = colorE;
      } else if (distanceToOrigin < 6.0) {
        gl_FragColor = colorR;
      } else if (distanceToOrigin < 9.0) {
        gl_FragColor = colorS;
      }
    } else if (u_time >= 12) {
      if (distanceToOrigin < 3.0) {
        gl_FragColor = colorS;
      } else if (distanceToOrigin < 6.0) {
        gl_FragColor = colorE;
      } else if (distanceToOrigin < 9.0) {
        gl_FragColor = colorR;
      }
    }
  }

  if (0 == rcState)
  {
    if (distanceToRemote < 2.1)
    {
      gl_FragColor = vec4(1,1,0,1);
    }
    else
    {
      gl_FragColor = vec4(normalize(interpolatedNormal), 1.0);
    }
  }
}
