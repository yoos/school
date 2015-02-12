#version 330 compatibility

out vec3  vNormal, vLight, vEye;

uniform float uK;    // Pleat amplitude
uniform float uY0;   // Top of folds
uniform float uP;    // Sine wave period
uniform float uNoiseAmp;    // Noise amplitude
uniform float uNoiseFreq;   // Noise frequency
uniform float uLightX, uLightY, uLightZ;

uniform sampler3D Noise3;

vec3 RotateNormal( float angx, float angy, vec3 n )
{
	float cx = cos( angx );
	float sx = sin( angx );
	float cy = cos( angy );
	float sy = sin( angy );

	// rotate about x:
	float yp =  n.y*cx - n.z*sx;    // y'
	n.z      =  n.y*sx + n.z*cx;    // z'
	n.y      =  yp;
	// n.x      =  n.x;

	// rotate about y:
	float xp =  n.x*cy + n.z*sy;    // x'
	n.z      = -n.x*sy + n.z*cy;    // z'
	n.x      =  xp;
	// n.y      =  n.y;

	return normalize( n );
}

void main()
{
	// Do the sine thing
	float x = gl_Vertex.x;
	float y = gl_Vertex.y;
	float z = uK * (uY0-y) * sin(2.*3.14159*x/uP);
	float w = gl_Vertex.w;
	gl_Position = gl_ModelViewProjectionMatrix * vec4(x,y,z,w);

	// Add noise
	vec4 nvx = uNoiseAmp * texture3D(Noise3, uNoiseFreq*vec3(x,y,z) );
	vec4 nvy = uNoiseAmp * texture3D(Noise3, uNoiseFreq*vec3(x,y,z+0.5));

	// Calculate tangents
	float dzdx = uK * (uY0-y) * (2.*3.14159/uP) * cos(2.*3.14159*x/uP);
	float dzdy = -uK * sin(2.*3.14159*x/uP);
	vec3 Tx = vec3(1., 0., dzdx);   // Tangent x vector
	vec3 Ty = vec3(0., 1., dzdy);   // Tangent y vector

	// Set lighting
	vec3 ECposition = vec3(gl_ModelViewMatrix * vec4(x,y,z,w));   // Just using xyz
	vNormal = normalize(gl_NormalMatrix * cross(Tx, Ty));
	vNormal = RotateNormal(nvx, nvy, vNormal);
	vNormal = normalize(gl_NormalMatrix * vNormal);
	vLight  = normalize(vec3(uLightX, uLightY, uLightZ) - ECposition);
	vEye    = normalize(vec3(0., 0., 0.) - ECposition);
}

// vim: syntax=c
