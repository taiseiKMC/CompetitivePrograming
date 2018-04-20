using mod_type = long long;
template<mod_type MODULO=1000000007>
class Mod
{
	public:
		using value_type = mod_type;

	private:
		value_type value;
		constexpr value_type Normalize(value_type x) const
		{
			return x<0?(x%MODULO+MODULO):(x%MODULO);
		}

	public:
		constexpr Mod<MODULO>():value(0){}
		constexpr Mod<MODULO>(const value_type &val):value(Normalize(val)) {}
		
		explicit operator value_type () const
		{
			return value;
		}

		constexpr const Mod<MODULO> operator -() const
		{
			return Mod<MODULO>(MODULO - value);
		}
		constexpr const Mod<MODULO> operator +(const Mod<MODULO> &rhs) const
		{
			return Mod<MODULO>(value + rhs.value);
		}
		constexpr const Mod<MODULO> operator -(const Mod<MODULO> &rhs) const
		{
			return Mod<MODULO>(value + (-rhs).value);
		}
		constexpr const Mod<MODULO> operator *(const Mod<MODULO> &rhs) const
		{
			return Mod<MODULO>(value * rhs.value);
		}
		Mod<MODULO> &operator +=(const Mod<MODULO> &rhs)
		{
			return *this = *this + rhs;
		}
		Mod<MODULO> &operator -=(const Mod<MODULO> &rhs)
		{
			return *this = *this - rhs;
		}
		Mod<MODULO> &operator *=(const Mod<MODULO> &rhs)
		{
			return *this = *this * rhs;
		}


		Mod<MODULO> pow(value_type p) const;

		Mod<MODULO> inv() const
		{
			return pow(MODULO-2);
		}

		const Mod<MODULO> operator /(const Mod<MODULO> &rhs) const
		{
			return *this * rhs.inv();
		}
		Mod<MODULO> &operator /=(const Mod<MODULO> &rhs)
		{
			return *this = *this / rhs;
		}

		constexpr const bool operator <(const Mod<MODULO> &rhs) const
		{
			return value < rhs.value;
		}
		constexpr const bool operator ==(const Mod<MODULO> &rhs) const
		{
			return value == rhs.value;
		}
};

template<mod_type MODULO>
Mod<MODULO> Mod<MODULO>::pow(value_type p) const
{
	Mod<MODULO> tmp=1, mult=*this;
	while(p)
	{
		if((p&1)>0) tmp*=mult;
		p>>=1;
		mult*=mult;
	}
	return tmp;
}

namespace std
{
	template<mod_type MODULO>
	ostream& operator<<(ostream& os, const Mod<MODULO> mod)
	{
		os<<(typename Mod<MODULO>::value_type)mod;
		return os;
	}
};