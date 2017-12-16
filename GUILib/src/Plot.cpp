#include <GUILib/Plot.h>

#include <nanogui/theme.h>
#include <nanogui/opengl.h>
#include <nanogui/serializer/core.h>

using namespace nanogui;

void PlotData::updateMinMax()
{
	mMinVal(0) = mXValues.minCoeff();
	mMaxVal(0) = mXValues.maxCoeff();
	mMinVal(1) = mYValues.minCoeff();
	mMaxVal(1) = mYValues.maxCoeff();
}

Plot::Plot(Widget *parent, const std::string &caption)
	: Widget(parent), mCaption(caption) {
	mBackgroundColor = Color(20, 128);
	mForegroundColor = Color(255, 255, 255, 128);
	mShowLegend = false;
	mShowTicks = false;
	mNumTicks = Vector2i(10, 10);
	mTickHeight = 10;
	mTextColor = Color(240, 192);
}

void Plot::setPlotData(const std::string &name, const PlotData &data)
{
	mDataColl[name] = data;
}

void Plot::draw(NVGcontext *ctx) {
	Widget::draw(ctx);

	nvgBeginPath(ctx);
	nvgRect(ctx, mPos.x(), mPos.y(), mSize.x(), mSize.y());
	nvgFillColor(ctx, mBackgroundColor);
	nvgFill(ctx);

	// plot data
	for(const auto &d : mDataColl) {
		const PlotData &data = d.second;
		nvgBeginPath(ctx);

		if(data.mXValues.size() > 0)
		{
			float xvalue = scaledXValue(data.mXValues[0]);
			float yvalue = scaledYValue(data.mYValues[0]);
			float vx = mPos.x() + (xvalue) * mSize.x();
			float vy = mPos.y() + (1-yvalue) * mSize.y();
			nvgMoveTo(ctx, vx, vy);

			for (size_t i = 1; i < (size_t) data.mXValues.size(); i++) {
				float xvalue = scaledXValue(data.mXValues[i]);
				float yvalue = scaledYValue(data.mYValues[i]);
				float vx = mPos.x() + (xvalue) * mSize.x();
				float vy = mPos.y() + (1-yvalue) * mSize.y();
				nvgLineTo(ctx, vx, vy);
			}

			nvgStrokeColor(ctx, data.mColor);
			nvgStrokeWidth(ctx, data.mWidth);
			nvgStroke(ctx);
		}
	}


	if(mShowTicks)
	{
		// set up stroke
		nvgStrokeColor(ctx, mForegroundColor);
		nvgStrokeWidth(ctx, 1.f);

		// set up font
		nvgFontSize(ctx, 14.0f);

		// ticks on x axis
		{
			nvgTextAlign(ctx, NVG_ALIGN_CENTER | NVG_ALIGN_BOTTOM);

			// compute tick step
			float tickStep = computeTickStep(0);
			float mag = computeMagnitude(tickStep);
			float start = std::floor(mDataMin(0)/std::pow(10, mag))*std::pow(10, mag);;
			start = std::floor(start/tickStep)*tickStep;

			float vy = mPos.y() + mSize.y();
			for (float x = start; x <= mDataMax(0); x+=tickStep) {
				float vx = mPos.x() + scaledXValue(x) * mSize.x();
				nvgBeginPath(ctx);
				nvgMoveTo(ctx, vx, vy);
				nvgLineTo(ctx, vx, vy-mTickHeight);
				nvgStroke(ctx);

				char c[100];
				std::sprintf(c, "%g", x);
				nvgFillColor(ctx, mForegroundColor);
				nvgText(ctx, vx, vy-mTickHeight, c, NULL);
			}
		}

		// ticks on y axis
		{
			nvgTextAlign(ctx, NVG_ALIGN_LEFT | NVG_ALIGN_MIDDLE);

			// compute tick step
			float tickStep = computeTickStep(1);
			float mag = computeMagnitude(tickStep);
			float start = std::floor(mDataMin(1)/std::pow(10, mag))*std::pow(10, mag);;
			start = std::floor(start/tickStep)*tickStep;

			float vx = mPos.x();
			for (float y = start; y <= mDataMax(1); y+=tickStep) {
				float vy = mPos.y() + (1-scaledYValue(y)) * mSize.y();
				nvgBeginPath(ctx);
				nvgMoveTo(ctx, vx, vy);
				nvgLineTo(ctx, vx+mTickHeight, vy);
				nvgStroke(ctx);

				char c[100];
				std::sprintf(c, "%g", y);
				nvgFillColor(ctx, mForegroundColor);
				nvgText(ctx, vx+mTickHeight+4, vy, c, NULL);
			}
		}

	}

	nvgFontFace(ctx, "sans");

	if(mShowLegend)
	{
		nvgFontSize(ctx, 14.0f);
		nvgTextAlign(ctx, NVG_ALIGN_LEFT | NVG_ALIGN_TOP);

		int index = 0;
		for(const auto &d : mDataColl) {
			const std::string &name = d.first;
			const PlotData &data = d.second;

			float offset = mSize.y() - 14.f*(float)(mDataColl.size()-index);

			nvgFillColor(ctx, data.mColor);
			nvgText(ctx, mPos.x() + 3, mPos.y() + offset, name.c_str(), NULL);
			index++;
		}

	}


	if (!mCaption.empty()) {
		nvgFontSize(ctx, 14.0f);
		nvgTextAlign(ctx, NVG_ALIGN_LEFT | NVG_ALIGN_TOP);
		nvgFillColor(ctx, mTextColor);
		nvgText(ctx, mPos.x() + 3, mPos.y() + 1, mCaption.c_str(), NULL);
	}

	if (!mHeader.empty()) {
		nvgFontSize(ctx, 18.0f);
		nvgTextAlign(ctx, NVG_ALIGN_RIGHT | NVG_ALIGN_TOP);
		nvgFillColor(ctx, mTextColor);
		nvgText(ctx, mPos.x() + mSize.x() - 3, mPos.y() + 1, mHeader.c_str(), NULL);
	}

	if (!mFooter.empty()) {
		nvgFontSize(ctx, 15.0f);
		nvgTextAlign(ctx, NVG_ALIGN_RIGHT | NVG_ALIGN_BOTTOM);
		nvgFillColor(ctx, mTextColor);
		nvgText(ctx, mPos.x() + mSize.x() - 3, mPos.y() + mSize.y() - 1, mFooter.c_str(), NULL);
	}

	nvgBeginPath(ctx);
	nvgRect(ctx, mPos.x(), mPos.y(), mSize.x(), mSize.y());
	nvgStrokeColor(ctx, Color(100, 255));
	nvgStroke(ctx);
}

void Plot::save(Serializer &s) const {
	Widget::save(s);
	s.set("caption", mCaption);
	s.set("header", mHeader);
	s.set("footer", mFooter);
	s.set("backgroundColor", mBackgroundColor);
	s.set("foregroundColor", mForegroundColor);
	s.set("textColor", mTextColor);
//	s.set("values", mXValues);
}

bool Plot::load(Serializer &s) {
	if (!Widget::load(s)) return false;
	if (!s.get("caption", mCaption)) return false;
	if (!s.get("header", mHeader)) return false;
	if (!s.get("footer", mFooter)) return false;
	if (!s.get("backgroundColor", mBackgroundColor)) return false;
	if (!s.get("foregroundColor", mForegroundColor)) return false;
	if (!s.get("textColor", mTextColor)) return false;
//	if (!s.get("values", mXValues)) return false;
	return true;
}

void Plot::updateMinMax()
{
	mDataMin = Vector2f(HUGE_VAL, HUGE_VAL);
	mDataMax = Vector2f(-HUGE_VAL, -HUGE_VAL);

	for(const auto &d : mDataColl) {
		const PlotData &data = d.second;
		for (int i = 0; i < 2; ++i) {
			mDataMin[i] = std::min(mDataMin[i], data.mMinVal[i]);
			mDataMax[i] = std::max(mDataMax[i], data.mMaxVal[i]);
		}
	}

	float range = (mDataMax[1] - mDataMin[1]);
	mDataMin[1] -= 0.1*range;
	mDataMax[1] += 0.1*range;

//	for (int i = 0; i < 2; ++i) {
//		float range = (mDataMax[i] - mDataMin[i]);
//		mDataMin[i] -= 0.1*range;
//		mDataMax[i] += 0.1*range;
//	}
}

float Plot::computeTickStep(int dim) const
{
	float range = mDataMax(dim)-mDataMin(dim);
	float tickStep = range/(float)mNumTicks(dim);
	float mag = std::floor(std::log10(tickStep));
	return std::round(tickStep/std::pow(10, mag))*std::pow(10, mag);
}

float Plot::computeMagnitude(float x)
{
	return std::floor(std::log10(x));
}
