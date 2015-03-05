#version 330 compatibility

out vec3 vMCposition, vECposition;
out vec3 vNormal, vLight, vEye;

uniform float uLightX, uLightY, uLightZ;

vec3 LIGHTPOS = vec3(uLightX, uLightY, uLightZ);

void main()
{
	gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;
	vMCposition = gl_Vertex.xyz;

	// Set lighting
	vECposition = vec3(gl_ModelViewMatrix * gl_Vertex);   // Just using xyz
	vNormal = normalize(gl_NormalMatrix * gl_Normal);   // Surface normal
	vLight  = LIGHTPOS - vECposition;                    // Vertex to light
	vEye    = vec3(0,0,0) - vECposition;                 // Vertex to eye
}
