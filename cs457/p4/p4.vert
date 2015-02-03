#version 330 compatibility

out vec4  vColor;
out float vLightIntensity;
out vec2  vST;
out vec3  vMCposition;

const vec3 LIGHTPOS = vec3( 0., 0., 100. );

void
main( )
{
	vec3 tnorm = normalize( vec3( gl_NormalMatrix * gl_Normal ) );
	vec3 ECposition = vec3( gl_ModelViewMatrix * gl_Vertex );
	vLightIntensity  = abs( dot( normalize(LIGHTPOS - ECposition), tnorm )  );

	vColor = gl_Color;
	vST = gl_MultiTexCoord0.st;
	vMCposition = gl_Vertex.xyz;
	gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;
}

// vim: ft=c
