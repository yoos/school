#version 330 compatibility

uniform float uScenter, uTcenter;
uniform float uDs, uDt;
uniform float uMagFactor, uRotAngle, uSharpFactor;
uniform bool uCircle;
uniform sampler2D uImageUnit;

const float PI = 3.14159265;
const float PI_OVER_4 = PI/4.;
const float SIN_PI_OVER_4 = 0.707;

in vec2 vST;

bool isInside(vec2 vST)
{
	if (uCircle) {
		if (sqrt(pow(((uScenter - vST.s)/(uDs/2.)), 2) +
				 pow(((uTcenter - vST.t)/(uDt/2.)), 2)) < 1.) {
			return true;
		}
	}
	else {
		if ((abs(uScenter - vST.s) < uDs/2.) &&
			(abs(uTcenter - vST.t) < uDt/2.)) {
			return true;
		}
	}

	return false;
}

void main()
{
	vec2 vSTp = vST;
	vec4 color = texture2D(uImageUnit, vST);
	vec2 lc = vec2(uScenter, uTcenter);   // Lens center
	vec2 res = textureSize(uImageUnit, 0);   // Image resolution used in sharpening step

	if (isInside(vST)) {
		// Magnify
		vSTp = lc + (vSTp-lc) / uMagFactor;

		// Rotate
		vec2 lST = vSTp-lc;
		vSTp.s = lc.s + lST.s*cos(-uRotAngle) - lST.t*sin(-uRotAngle);
		vSTp.t = lc.t + lST.s*sin(-uRotAngle) + lST.t*cos(-uRotAngle);

		// Update color
		color = texture2D(uImageUnit, vSTp);

		// Sharpen
		vec2 stp0 = vec2(1./res.s, 0. );
		vec2 st0p = vec2(0. , 1./res.t);
		vec2 stpp = vec2(1./res.s,  1./res.t);
		vec2 stpm = vec2(1./res.s, -1./res.t);
		vec4 i00   = texture2D( uImageUnit, vSTp );
		vec4 im1m1 = texture2D( uImageUnit, vSTp-stpp );
		vec4 ip1p1 = texture2D( uImageUnit, vSTp+stpp );
		vec4 im1p1 = texture2D( uImageUnit, vSTp-stpm );
		vec4 ip1m1 = texture2D( uImageUnit, vSTp+stpm );
		vec4 im10  = texture2D( uImageUnit, vSTp-stp0 );
		vec4 ip10  = texture2D( uImageUnit, vSTp+stp0 );
		vec4 i0m1  = texture2D( uImageUnit, vSTp-st0p );
		vec4 i0p1  = texture2D( uImageUnit, vSTp+st0p );
		vec4 target = vec4(0.,0.,0.,1.);
		target += 1.*(im1m1+ip1m1+ip1p1+im1p1);
		target += 2.*(im10+ip10+i0m1+i0p1);
		target += 4.*(i00);
		target /= 16.;
		color = vec4(mix(target, color, uSharpFactor).rgb, 1.);
	}

	gl_FragColor = color;
}

// vim: syntax=c
