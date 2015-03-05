#version 330 compatibility

in vec3 vMCposition, vECposition;
in vec3 vNormal, vLight, vEye;

uniform float uKa, uKd, uKs, uShininess;
uniform vec4 uColor;
uniform float uAlpha;       // Color alpha channel
uniform vec4 uSpecularColor;
uniform int uFogStep;
uniform float uFogNoise;

uniform float Timer;
uniform sampler3D Noise3;

void main()
{
	// If alpha is zero, discard pixel.
	if (uColor.a == 0) {
		discard;
	}

	// Normalize some ctors
	vec3 normLight = normalize(vLight);
	vec3 normEye   = normalize(vEye);

	// Set color
	vec4 ambient = uKa * uColor;
	float cd = max(dot(vNormal,normLight), 0.);   // Diffuse color
	vec4 diffuse = uKd * cd * uColor;

	float cs = 0.;   // Specular color
	if (dot(vNormal,normLight) > 0.) {
		vec3 ref = normalize(2. * vNormal * dot(vNormal,normLight) - normLight);
		cs = pow(max(dot(normEye,ref), 0.), uShininess);
	}
	vec4 specular = uKs * cs * uSpecularColor;

	vec4 objColor = vec4((ambient.rgb + diffuse.rgb + specular.rgb), uAlpha);

	// Generate fog
	float fogDensity = 0.;
	vec3 sampleLoc = vMCposition;
	float dist = length(vEye);
	while (dist > 0 && fogDensity < 1) {
		fogDensity += texture3D(Noise3, uFogNoise * vECposition + Timer)/(uFogStep);
		sampleLoc += normEye/uFogStep;
		dist -= 1./uFogStep;
	}

	// Step off more obviously
	fogDensity *= atan(length(vEye)*4-10)/3.14159 + 0.5;

	gl_FragColor = mix(objColor, vec4(1,1,1,1), fogDensity);
}
