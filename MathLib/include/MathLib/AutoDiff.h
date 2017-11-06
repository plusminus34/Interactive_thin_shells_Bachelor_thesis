#ifndef AUTO_DIFF_H
#define AUTO_DIFF_H

#include <ostream>
#include <cmath>
#include <cassert>

template <class ValueT, class DerivT>
class AutoDiffT
{
public:
	AutoDiffT() {
	}

	template<class T>
	AutoDiffT(const T &c)
		: m_x((ValueT)c), m_d((DerivT)0) {
	}

	AutoDiffT(const ValueT &x, const DerivT &d)
		: m_x(x), m_d(d) {
	}

	bool operator==(const AutoDiffT<ValueT, DerivT> &s) const {
		return s.value() == this->value();
	}

	bool operator!=(const AutoDiffT<ValueT, DerivT> &s) const {
		return s.value() != this->value();
	}

	AutoDiffT<ValueT, DerivT> operator+(const AutoDiffT<ValueT, DerivT> &s) const {
		return AutoDiffT<ValueT, DerivT>(m_x + s.m_x, m_d + s.m_d);
	}

	AutoDiffT<ValueT, DerivT> operator-(const AutoDiffT<ValueT, DerivT> &s) const {
		return AutoDiffT<ValueT, DerivT>(m_x - s.m_x, m_d - s.m_d);
	}

	AutoDiffT<ValueT, DerivT> operator-() const {
		return AutoDiffT<ValueT, DerivT>(-m_x, -m_d);
	}

	AutoDiffT<ValueT, DerivT> &operator+=(const AutoDiffT<ValueT, DerivT> &s) {
		m_x = m_x + s.m_x;
		m_d = m_d + s.m_d;

		return *this;
	}

	AutoDiffT<ValueT, DerivT> &operator-=(const AutoDiffT<ValueT, DerivT> &s) {
		m_x = m_x - s.m_x;
		m_d = m_d - s.m_d;

		return *this;
	}

	AutoDiffT<ValueT, DerivT> operator*(const AutoDiffT<ValueT, DerivT> &s) const {
		// D(x*y) = x*dy + dx*y
		return AutoDiffT<ValueT, DerivT>(m_x * s.m_x,
			s.m_d * m_x + m_d * s.m_x);
	}

	AutoDiffT<ValueT, DerivT> operator/(const AutoDiffT<ValueT, DerivT> &s) const {
		// D(x/y) = (dx*y - x*dy) / y*y
		ValueT denom = s.m_x * s.m_x;
		return AutoDiffT<ValueT, DerivT>(m_x / s.m_x,
			m_d*(s.m_x/denom) - s.m_d*(m_x/denom));
	}

	AutoDiffT<ValueT, DerivT> &operator*=(const AutoDiffT<ValueT, DerivT> &s) {
		*this = (*this) * s;
		return *this;
	}

	AutoDiffT<ValueT, DerivT> &operator/=(const AutoDiffT<ValueT, DerivT> &s) {
		*this = (*this) / s;
		return *this;
	}

	bool operator>(const AutoDiffT<ValueT, DerivT> &d) const {
		return m_x > d.m_x;
	}

	bool operator<(const AutoDiffT<ValueT, DerivT> &d) const {
		return m_x < d.m_x;
	}

	bool operator>=(const AutoDiffT<ValueT, DerivT> &d) const {
		return m_x >= d.m_x;
	}

	bool operator<=(const AutoDiffT<ValueT, DerivT> &d) const {
		return m_x <= d.m_x;
	}

	const ValueT &value() const { return m_x; }

	ValueT &value() { return m_x; }

	const DerivT &deriv() const { return m_d; }

	DerivT &deriv() { return m_d; }

private:
	ValueT m_x;			//!< The variable value
	DerivT m_d;			//!< Value of derivative(s)
};


template<class ValueT, class DerivT>
AutoDiffT<ValueT, DerivT> operator+(const ValueT &a, const AutoDiffT<ValueT, DerivT> &b)
{
	return AutoDiffT<ValueT, DerivT>(a + b.value(), b.deriv());
}

template<class ValueT, class DerivT>
AutoDiffT<ValueT, DerivT> operator-(const ValueT &a, const AutoDiffT<ValueT, DerivT> &b)
{
	return AutoDiffT<ValueT, DerivT>(a - b.value(), -b.deriv());
}

template<class ValueT, class DerivT>
AutoDiffT<ValueT, DerivT> operator*(const ValueT &a, const AutoDiffT<ValueT, DerivT> &b)
{
	return AutoDiffT<ValueT, DerivT>(a * b.value(), b.deriv() * a);
}

template<class ValueT, class DerivT>
AutoDiffT<ValueT, DerivT> operator/(const ValueT &nom, const AutoDiffT<ValueT, DerivT> &denom)
{
	// D(a/x) = -a/x^2 * dx
	return AutoDiffT<ValueT, DerivT>(nom / denom.value(),
		denom.deriv() * (-(nom / (denom.value() * denom.value()))));
}


template<class ValueT, class DerivT>
AutoDiffT<ValueT, DerivT> sin(const AutoDiffT<ValueT, DerivT> &s)
{
	// D(sin(x)) = cos(x) * dx
	return AutoDiffT<ValueT, DerivT>(sin(s.value()),
		s.deriv() * cos(s.value()));
}

template<class ValueT, class DerivT>
AutoDiffT<ValueT, DerivT> cos(const AutoDiffT<ValueT, DerivT> &s)
{
	// D(cos(x)) = -sin(x) * dx
	return AutoDiffT<ValueT, DerivT>(cos(s.value()),
		s.deriv() * (-sin(s.value())));
}

template<class ValueT, class DerivT>
AutoDiffT<ValueT, DerivT> tan(const AutoDiffT<ValueT, DerivT> &s)
{
	ValueT tanVal = tan(s.value());

	// D(tan(x)) = (1 + tan(x)^2) * dx
	return AutoDiffT<ValueT, DerivT>(tanVal,
		s.deriv() * ((ValueT)1 + tanVal*tanVal));
}

template<class ValueT, class DerivT>
AutoDiffT<ValueT, DerivT> acos(const AutoDiffT<ValueT, DerivT> &s)
{
	// D(acos(x)) = -1/sqrt(1-x*x) * dx
	return AutoDiffT<ValueT, DerivT>(acos(s.value()),
		s.deriv() * (ValueT(-1)/sqrt(ValueT(1)-s.value()*s.value())));
}

template<class ValueT, class DerivT>
AutoDiffT<ValueT, DerivT> atan2(const AutoDiffT<ValueT, DerivT> &y, const AutoDiffT<ValueT, DerivT> &x)
{
	// D(atan2(y, x)) = (x*dy - y*dx)/len
	// len = x*x + y*y;
	ValueT len = x.value()*x.value() + y.value()*y.value();

	return AutoDiffT<ValueT, DerivT>(atan2(y.value(), x.value()),
		(x.value()*y.deriv() - y.value()*x.deriv())/len);
}

template<class ValueT, class DerivT>
AutoDiffT<ValueT, DerivT> sqrt(const AutoDiffT<ValueT, DerivT> &s)
{
	if(s.value() == ValueT(0))
		return 0;

	// D(sqrt(x)) = 1.0/2.0 * 1.0/sqrt(x) * dx
	return AutoDiffT<ValueT, DerivT>(sqrt(s.value()),
		ValueT(1)/ValueT(2) * ValueT(1)/sqrt(s.value()) * s.deriv());
}

template<class ValueT, class DerivT>
AutoDiffT<ValueT, DerivT> log(const AutoDiffT<ValueT, DerivT> &s)
{
	// D(log(x)) = 1.0 / x * dx
	return AutoDiffT<ValueT, DerivT>(log(s.value()),
		ValueT(1) / s.value() * s.deriv());
}

template<class ValueT, class DerivT>
AutoDiffT<ValueT, DerivT> log10(const AutoDiffT<ValueT, DerivT> &s)
{
	// TODO: implement!!
	throw std::runtime_error("log10 autodiff not implemented yet");
	// D(log(x)) = 1.0 / x * dx
	return AutoDiffT<ValueT, DerivT>(log(s.value()),
		ValueT(1) / s.value() * s.deriv());
}

template<class ValueT, class DerivT>
AutoDiffT<ValueT, DerivT> pow(const AutoDiffT<ValueT, DerivT> &s, const double &exponent)
{
	// D(x^b) = b*x^{b-1} * dx
	return AutoDiffT<ValueT, DerivT>(pow(s.value(), exponent),
		pow(s.value(), exponent-1) * s.deriv() * exponent);
}

template<class ValueT, class DerivT>
AutoDiffT<ValueT, DerivT> pow(const AutoDiffT<ValueT, DerivT> &s, const AutoDiffT<ValueT, DerivT> &exponent)
{
	// D(a^b) = a^b * (dbdx*ln(a) + b*dadx/a)
	return AutoDiffT<ValueT, DerivT>(pow(s.value(), exponent.value()),
		pow(s.value(), exponent.value()) * (exponent.deriv()*log(s.value()) + exponent.value()*s.deriv()/s.value()));
}

template<class ValueT, class DerivT>
AutoDiffT<ValueT, DerivT> fabs(const AutoDiffT<ValueT, DerivT> &s)
{
	if(s.value() >= 0)
		return s;
	else
		return -s;
}

template<class ValueT, class DerivT>
AutoDiffT<ValueT, DerivT> floor(const AutoDiffT<ValueT, DerivT> &s)
{
	// TODO: implement!!
	throw std::runtime_error("floor autodiff not implemented yet");
	return AutoDiffT<ValueT, DerivT>(floor(s.value()), 0); // this is wrong ...
}

template<class ValueT, class DerivT>
AutoDiffT<ValueT, DerivT> ceil(const AutoDiffT<ValueT, DerivT> &s)
{
	// TODO: implement!!
	throw std::runtime_error("ceil autodiff not implemented yet");
	// D(log(x)) = 1.0 / x * dx
	return AutoDiffT<ValueT, DerivT>(0, 0);
}

template<class ValueT, class DerivT>
std::ostream& operator<<(std::ostream& stream, const AutoDiffT<ValueT, DerivT> &s) {
	stream << s.value() << "(" << s.deriv() << ")";
	return stream;
}

#endif
