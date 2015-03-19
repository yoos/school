varying vec4 vColor;
varying vec3 vMCposition, vECposition;
varying vec3 vNormal, vLight, vEye;

float uLightX = 0.0;
float uLightY = 0.0;
float uLightZ = -4.0;

vec3 LIGHTPOS = vec3(uLightX, uLightY, uLightZ);

void main()
{
	gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;
	vColor = gl_Color;
	vMCposition = gl_Vertex.xyz;

	// Set lighting
	vECposition = vec3(gl_ModelViewMatrix * gl_Vertex);   // Just using xyz
	vNormal = normalize(gl_NormalMatrix * gl_Normal);     // Surface normal
	vLight  = LIGHTPOS - vECposition;                     // Vertex to light
	vEye    = vec3(0,0,0) - vECposition;                  // Vertex to eye
}
