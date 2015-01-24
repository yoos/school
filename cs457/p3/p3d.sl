displacement
p3d(
		float A = 0.05,   // Ellipse horizontal diameter
		B = 0.10,         // Ellipse vertical diameter
		Ramp = 0.20,      // fraction of diameter used in the ramp
		Height = 0.2,     // displacement height
		Noise_factor = 5;
   )
{
	// Scale current location
	float up = 2. * u;   // Make things visually proportional.
	float vp = v;

	// Distort location
	float nf = 1.;   // Noise "frequency"
	float i;
	for (i=0.; i<Noise_factor; i+=1.) {
	   up += (float noise(P*nf) - 0.5) / nf;
	   vp += (float noise(P*nf) - 0.5) / nf;
	   nf *= 2.;
	}

	// Calculate ellipse geometry
	float numinu = floor( up / A );
	float numinv = floor( vp / B );
	float uo = numinu*A + A/2;   // Ellipse center u
	float vo = numinv*B + B/2;   // Ellipse center v

	// Calculate distance away from ellipse edge and ramp accordingly
	float dist = 1 - sqrt(pow((up-uo)/(A/2),2) + pow((vp-vo)/(B/2),2));   // Distance from edge
	float t = smoothstep( 0., Ramp, dist );   // Ramps smoothly from 0 to Ramp distance.

	// Set height
	float TheHeight = 0.;   // Base height

	// Displace
	if (dist > 0.)
		TheHeight += t*Height*noise(P);   // apply the blending

	if( TheHeight != 0. )
	{
#ifdef DISPLACEMENT_MAPPING
		P = P + normalize(N) * TheHeight;
		N = calculatenormal(P);
#else
		// Bump mapping - useful in reducing number of required polygons in
		// GLSL. Due to microfaceting, not as useful in Renderman.
		N = calculatenormal( P + normalize(N) * TheHeight );
#endif
	}
}
