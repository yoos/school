surface
p3s(
		float A = 0.05,    // Ellipse horizontal diameter
		B = 0.10,          // Ellipse vertical diameter
		Ks = 0.6,          // specular coefficient
		Kd = 0.8,          // diffuse  coefficient
		Ka = 0.2,          // ambient  coefficient
		Roughness = 0.1;   // specular roughness
		color SpecularColor = color( 1, 1, 1 )   // specular color
   )
{
	// Calculate ellipse geometry
	float up = 2. * u;   // Make things visually proportional.
	float vp = v;
	float numinu = floor( up / A );
	float numinv = floor( vp / B );
	float uo = numinu*A + A/2;   // Ellipse center u
	float vo = numinv*B + B/2;   // Ellipse center v

	// Define color based on distance from edge, u, and v
	float dist = 1 - sqrt(pow((up-uo)/(A/2),2) + pow((vp-vo)/(B/2),2));   // Distance from edge
	float c = 2*u+v+.5*smoothstep( 0., 1., dist );   // Ramps smoothly from 0 to Ramp distance.
	color RAINBOW = color "hsv" (c, 1.0, 1.0);

	// Some vectors for color
	varying vector Nf = faceforward( normalize( N ), I );
	vector V = normalize( -I );

	Oi = Os;   // use whatever opacity the rib file gave us

	color TheColor = Cs;
	if (pow((up-uo)/(A/2),2) + pow((vp-vo)/(B/2),2) < 1)
		TheColor = RAINBOW;

	Ci =        TheColor * Ka * ambient();
	Ci = Ci  +  TheColor * Kd * diffuse(Nf);
	Ci = Ci  +  SpecularColor * Ks * specular( Nf, V, Roughness );
}

