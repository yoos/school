surface
p3s(
		float A = 0.05,    // Ellipse horizontal diameter
		B = 0.10,          // Ellipse vertical diameter
		Ks = 0.4,          // specular coefficient
		Kd = 0.8,          // diffuse  coefficient
		Ka = 0.2,          // ambient  coefficient
		Roughness = 0.1,   // specular roughness
		Noise_factor = 5;
		color SpecularColor = color( 1, 1, 1 )   // specular color
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

	// Define color based on distance from edge, u, and v
	float dist = 1 - sqrt(pow((up-uo)/(A/2),2) + pow((vp-vo)/(B/2),2));   // Distance from edge
	//float c = 2*u+v+.5*smoothstep( 0., 1., dist );   // Ramps smoothly from 0 to Ramp distance.

	// Set color
	color TheColor = color (0.8, 0.2, 0.0);   // Lava
	color LandColor = color "hsv" (0.1, 0.5, 0.2);   // Igneous landmasses

	// Vary color
	if (dist > 0.)
		TheColor = LandColor;

	// Some vectors for color
	varying vector Nf = faceforward( normalize( N ), I );
	vector V = normalize( -I );

	Oi = Os;   // use whatever opacity the rib file gave us
	Ci =        TheColor * Ka * ambient();
	Ci = Ci  +  TheColor * Kd * diffuse(Nf);
	Ci = Ci  +  SpecularColor * Ks * specular( Nf, V, Roughness );
}

