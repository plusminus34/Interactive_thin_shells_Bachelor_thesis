#include <GUILib/ColorMaps.h>

Eigen::Vector3f ColorMaps::getColorAt(const ColorMaps::ColorMap &colorMap, float t, float startAt, float endAt)
{
	// clamp t to [0, 1]
	t = std::max(0.f, t);
	t = std::min(1.f, t);

	// scale by start and end
	t = startAt + t*(endAt - startAt);

	int im = std::floor(t*(float)(colorMap.size()));
	int ip = std::min(im+1, (int)colorMap.size()-1);

	if(im == ip)
	{
		return colorMap[im];
	}
	else
	{
		float tm = (float)im/((float)colorMap.size()-1);
		float tp = (float)ip/((float)colorMap.size()-1);

		return (t-tm)/(tp-tm)*colorMap[im] + (tp-t)/(tp-tm)*colorMap[ip];
	}
}

