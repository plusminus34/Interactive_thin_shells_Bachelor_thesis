#pragma once

static const P3D RED         = P3D(1., 0., 0.);
static const P3D GREEN       = P3D(0., 1., 0.);
static const P3D BLUE        = P3D(0., 0., 1.);
static const P3D MAGENTA     = P3D(1., 0., 1.);
static const P3D CYAN        = P3D(0., 1., 1.);
static const P3D YELLOW      = P3D(1., 1., 0.);
static const P3D ORANGE      = P3D(1., .5, 0.);
static const P3D WHITE       = P3D(1., 1., 1.);
static const P3D BLACK       = P3D(0., 0., 0.);
static const P3D GRAY        = P3D(.5, .5, .5);
// http://www.colourlovers.com/palette/1718713/Monokai
static const P3D CLAY        = P3D(39./255., 40./255., 34./255.);
static const P3D ORCHID      = P3D(249./255., 38./255., 114./255.);
static const P3D RATIONALITY = P3D(102./255., 217./255., 239./255.);
static const P3D HENN1NK     = P3D(166./255., 226./255., 46./255.);
static const P3D PUMPKIN     = P3D(253./255., 151./255., 31./255.);
static const P3D LAVISH      = P3D(174./255., 129./255., 255./255.);
static const P3D GOLDCLOVER  = P3D(230./255., 219./255., 116./255.);

static const P3D LIGHT_CLAY = (CLAY + WHITE)*.5;
static const P3D LIGHT_ORCHID = (ORCHID + WHITE)*.5;
static const P3D LIGHT_RATIONALITY = (RATIONALITY + WHITE)*.5;
static const P3D LIGHT_HENN1NK = (HENN1NK + WHITE)*.5;
static const P3D LIGHT_PUMPKIN = (PUMPKIN + WHITE)*.5;
static const P3D LIGHT_LAVISH = (LAVISH + WHITE)*.5;

static const P3D DARK_CLAY = (CLAY + BLACK)*.5;
static const P3D DARK_ORCHID = (ORCHID + BLACK)*.5;
static const P3D DARK_RATIONALITY = (RATIONALITY + BLACK)*.5;
static const P3D DARK_HENN1NK = (HENN1NK + BLACK)*.5;
static const P3D DARK_PUMPKIN = (PUMPKIN + BLACK)*.5;
static const P3D DARK_LAVISH = (LAVISH + BLACK)*.5;

static const double KELLY_COLORS[20][3] = {
	{255./255., 179./255.,   0./255.},
	{128./255.,  62./255., 117./255.},
	{255./255., 104./255.,   0./255.},
	{166./255., 189./255., 215./255.},
	{193./255.,   0./255.,  32./255.},
	{206./255., 162./255.,  98./255.},
	{129./255., 112./255., 102./255.},
	{  0./255., 125./255.,  52./255.},
	{246./255., 118./255., 142./255.},
	{  0./255.,  83./255., 138./255.},
	{255./255., 122./255.,  92./255.},
	{ 83./255.,  55./255., 122./255.},
	{255./255., 142./255.,   0./255.},
	{179./255.,  40./255.,  81./255.},
	{244./255., 200./255.,   0./255.},
	{127./255.,  24./255.,  13./255.},
	{147./255., 170./255.,   0./255.},
	{ 89./255.,  51./255.,  21./255.},
	{241./255.,  58./255.,  19./255.},
	{ 35./255.,  44./255.,  22./255.}};

const auto kelly_color = [](int i) -> P3D {
	// NOTE: Unintialized warning.
	if (i == -1) { return MAGENTA; };
	// --
	double r = KELLY_COLORS[i % 20][0];
	double g = KELLY_COLORS[i % 20][1];
	double b = KELLY_COLORS[i % 20][2];
	return P3D(r, g, b);
};
