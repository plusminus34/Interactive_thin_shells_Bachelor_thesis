/*
	Plot.h -- Plot widget, based off nanogui/graph.h
*/

#pragma once

#include <nanogui/widget.h>
#include <Eigen/Eigen>

struct PlotData
{
public:
	PlotData() {}

	PlotData(const Eigen::VectorXf &xValues, const Eigen::VectorXf &yValues)
		: mXValues(xValues), mYValues(yValues){
		updateMinMax();
	}

	PlotData(const Eigen::VectorXf &xValues, const Eigen::VectorXf &yValues, const nanogui::Color &color, float width = 1)
		: mXValues(xValues), mYValues(yValues), mColor(color), mWidth(width){
		updateMinMax();
	}

	void updateMinMax();

	Eigen::VectorXf mXValues;
	Eigen::VectorXf mYValues;
	Eigen::Vector2f mMinVal;
	Eigen::Vector2f mMaxVal;

	nanogui::Color mColor = nanogui::Color(1.f, 1.f, 1.f, 1.f);
	float mWidth = 1;
};

class Plot : public nanogui::Widget {
public:
	Plot(nanogui::Widget *parent, const std::string &caption = "Untitled");

	const std::string &caption() const { return mCaption; }
	void setCaption(const std::string &caption) { mCaption = caption; }

	const std::string &header() const { return mHeader; }
	void setHeader(const std::string &header) { mHeader = header; }

	const std::string &footer() const { return mFooter; }
	void setFooter(const std::string &footer) { mFooter = footer; }

	const nanogui::Color &backgroundColor() const { return mBackgroundColor; }
	void setBackgroundColor(const nanogui::Color &backgroundColor) { mBackgroundColor = backgroundColor; }

	const nanogui::Color &foregroundColor() const { return mForegroundColor; }
	void setForegroundColor(const nanogui::Color &foregroundColor) { mForegroundColor = foregroundColor; }

	const nanogui::Color &textColor() const { return mTextColor; }
	void setTextColor(const nanogui::Color &textColor) { mTextColor = textColor; }

	bool showLegend() const { return mShowLegend; }
	void setShowLegend(bool showLegend) { mShowLegend = showLegend; }

	const PlotData &plotData(const std::string &name) const { return mDataColl.at(name); }
	PlotData &plotData(const std::string &name) { return mDataColl[name]; }
	void setPlotData(const std::string &name, const PlotData &data);

	const std::map<std::string, PlotData> &dataColl() { return mDataColl; }

	// TODO: come up with better names
	void updateMinMax();
	const Eigen::Vector2f &dataMin() const { return mDataMin; }
	void setDataMin(const Eigen::Vector2f &dataMin) { mDataMin = dataMin; }
	const Eigen::Vector2f &dataMax() const { return mDataMax; }
	void setDataMax(const Eigen::Vector2f &dataMax) { mDataMax = dataMax; }

	bool showTicks() const { return mShowTicks; }
	void setShowTicks(bool showTicks) { mShowTicks = showTicks; }

	const Eigen::Vector2i &numTicks() const { return mNumTicks; }
	void setNumTicks(const Eigen::Vector2i &numTicks) { mNumTicks = numTicks; }

	// TODO: should we rather override `preferredSize` or use `setSize` from `Widget`?
	//	virtual Eigen::Vector2i preferredSize(NVGcontext *ctx) const override;
	virtual void draw(NVGcontext *ctx) override;

	virtual void save(nanogui::Serializer &s) const override;
	virtual bool load(nanogui::Serializer &s) override;

private:
	float scaledXValue(float value) { return (value-mDataMin(0))/(mDataMax(0)-mDataMin(0)); }
	float scaledYValue(float value) { return (value-mDataMin(1))/(mDataMax(1)-mDataMin(1)); }

	float computeTickStep(int dim) const;
	static float computeMagnitude(float x);

protected:
	std::string mCaption, mHeader, mFooter;
	nanogui::Color mBackgroundColor, mForegroundColor, mTextColor;
	bool mShowLegend, mShowTicks;
	Eigen::Vector2i mNumTicks;
	int mTickHeight;

	std::map<std::string, PlotData> mDataColl;
	Eigen::Vector2f mDataMin, mDataMax;
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
