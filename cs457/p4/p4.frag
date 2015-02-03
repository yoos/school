#version 330 compatibility

in vec4  vColor;
in float vLightIntensity;
in vec2  vST;
in vec3  vMCposition;

uniform float uAd;   // Ellipse diameter 1
uniform float uBd;   // Ellipse diameter 2
uniform float uNoiseAmp;   // Noise amplitude
uniform float uNoiseFreq;   // Noise frequency
uniform float uAlpha;   // Color alpha channel
uniform float uTol;   // Width of blend
uniform sampler3D Noise3;

uniform float Ka, Kd, Ks;
uniform float Shininess;
uniform vec4 SpecularColor;

void
main()
{
	float s = vST.s;
	float t = vST.t;
	float sp = 2*s;
	float tp = t;

	// Add noise to coordinates
	vec4 nv = texture3D(Noise3, uNoiseFreq * vMCposition);
	float n = nv.r + nv.g + nv.b + nv.a;   // (1, 3)
	n = (n-2);   // (-1, 1)
	float delta = uNoiseAmp * n;

	// Calculate ellipse geometry
	int numins = int( sp / uAd );
	int numint = int( tp / uBd );
	float so = numins*uAd + 0.5*uAd;   // Ellipse center s
	float to = numint*uBd + 0.5*uBd;   // Ellipse center t

	// Calculate (noisy) distance from center
	float dist_center = sqrt(pow((sp-so)/(uAd/2), 2) + pow((tp-to)/(uBd/2), 2));   // Distance from ellipse center
	dist_center += delta;   // Add noise

	// Set color
	float mix_ratio = smoothstep(1-uTol, 1+uTol, dist_center);   // [0, 1]
	vec4 Color = mix(vColor, vec4(0., 0.8, 1.0, uAlpha), mix_ratio);

	// Set lighting
	vec3 Normal, Light, Eye;
	Normal = normalize(vec3(-100, 25, 100));
	Light = normalize(vec3(-100, 50, 100));
	Eye = normalize(vec3(-100, 100, 100));

	vec4 ambient = Ka * Color;
	float cd = max(dot(Normal, Light), 0.);
	vec4 diffuse = Kd * cd * Color;

	float cs = 0.;
	if (dot(Normal,Light) > 0.) {
		vec3 ref = normalize(2. * Normal * dot(Normal,Light) - Light);
		cs = pow(max(dot(Eye,ref), 0.), Shininess);
	}
	vec4 specular = Ks * cs * SpecularColor;

	gl_FragColor = Color;//vec4(ambient + diffuse + specular);

	// If color alpha is zero, discard pixel.
	if (gl_FragColor.a == 0) {
		discard;
	}

	// Apply lighting model
	gl_FragColor.rgb *= vLightIntensity;
}

// vim: ft=c
