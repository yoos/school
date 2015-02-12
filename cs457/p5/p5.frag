#version 330 compatibility

in vec3  vNormal, vLight, vEye;

uniform float uKa, uKd, uKs, uShininess;
uniform vec4 uColor;
uniform float uAlpha;       // Color alpha channel
uniform vec4 uSpecularColor;

void main()
{
	// If alpha is zero, discard pixel.
	if (uColor.a == 0) {
		discard;
	}

	// Set color
	vec4 ambient = uKa * uColor;
	float cd = max(dot(vNormal,vLight), 0.);   // Diffuse color
	vec4 diffuse = uKd * cd * uColor;

	float cs = 0.;   // Specular color
	if (dot(vNormal,vLight) > 0.) {
		vec3 ref = normalize(2. * vNormal * dot(vNormal,vLight) - vLight);
		cs = pow(max(dot(vEye,ref), 0.), uShininess);
	}
	vec4 specular = uKs * cs * uSpecularColor;

	gl_FragColor = vec4((ambient.rgb + diffuse.rgb + specular.rgb), uAlpha);
}

// vim: syntax=c
