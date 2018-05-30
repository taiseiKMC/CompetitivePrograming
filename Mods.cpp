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


//MODSが空の場合のみマッチ
template<mod_type... MODS>
struct Mods
{
	using value_type = Mod<>::value_type;
	constexpr Mods<MODS...>(){}
	constexpr Mods<MODS...>(value_type m){}

	constexpr const Mods<MODS...> operator -() const { return *this; }
	constexpr const Mods<MODS...> operator +(const Mods<MODS...> &rhs) const 
	{ return *this; }
	constexpr const Mods<MODS...> operator -(const Mods<MODS...> &rhs) const
	{ return *this; }
	constexpr const Mods<MODS...> operator *(const Mods<MODS...> &rhs) const
	{ return *this; }
	Mods<MODS...> &operator +=(const Mods<MODS...> &rhs)
	{ return *this; }
	Mods<MODS...> &operator -=(const Mods<MODS...> &rhs)
	{ return *this; }
	Mods<MODS...> &operator *=(const Mods<MODS...> &rhs)
	{ return *this; }
	Mods<MODS...> pow(value_type p) const
	{ return *this; }
	Mods<MODS...> inv() const
	{ return *this; }
	const Mods<MODS...> operator /(const Mods<MODS...> &rhs) const
	{ return *this; }
	Mods<MODS...> &operator /=(const Mods<MODS...> &rhs)
	{ return *this; }
	constexpr const bool operator ==(const Mods<MODS...> &rhs) const
	{ return true; }
};

template<mod_type MOD, mod_type... Tail>
struct Mods<MOD, Tail...>
{
	using value_type = Mod<>::value_type;
	Mod<MOD> m;
	Mods<Tail...> ms;
	constexpr Mods<MOD, Tail...>():m(0),ms(0){}
	constexpr Mods<MOD, Tail...>(value_type m_):m(m_),ms(m_){}
	constexpr Mods<MOD, Tail...>(Mod<MOD> m_, Mods<Tail...> ms_):m(m_),ms(ms_){}

	constexpr const Mods<MOD, Tail...> operator -() const
	{
		return Mods<MOD, Tail...>(-m, -ms);
	}
	constexpr const Mods<MOD, Tail...> operator +(const Mods<MOD, Tail...> &rhs) const
	{
		return Mods<MOD, Tail...>(m + rhs.m, ms + rhs.ms);
	}
	constexpr const Mods<MOD, Tail...> operator -(const Mods<MOD, Tail...> &rhs) const
	{
		return Mods<MOD, Tail...>(m - rhs.m, ms - rhs.ms);
	}
	constexpr const Mods<MOD, Tail...> operator *(const Mods<MOD, Tail...> &rhs) const
	{
		return Mods<MOD, Tail...>(m * rhs.m, ms * rhs.ms);
	}
	Mods<MOD, Tail...> &operator +=(const Mods<MOD, Tail...> &rhs)
	{
		return *this = *this + rhs;
	}
	Mods<MOD, Tail...> &operator -=(const Mods<MOD, Tail...> &rhs)
	{
		return *this = *this - rhs;
	}
	Mods<MOD, Tail...> &operator *=(const Mods<MOD, Tail...> &rhs)
	{
		return *this = *this * rhs;
	}

	Mods<MOD, Tail...> pow(value_type p) const
	{
		return Mods<MOD, Tail...>(m.pow(p), ms.pow(p));
	}
	Mods<MOD, Tail...> inv() const
	{
		return Mods<MOD, Tail...>(m.inv(), ms.inv());
	}

	const Mods<MOD, Tail...> operator /(const Mods<MOD, Tail...> &rhs) const
	{
		return *this * rhs.inv();
	}
	Mods<MOD, Tail...> &operator /=(const Mods<MOD, Tail...> &rhs)
	{
		return *this = *this / rhs;
	}
	constexpr const bool operator ==(const Mods<MOD, Tail...> &rhs) const
	{
		return m == rhs.m && ms == rhs.ms;
	}
};