//c++14

class Mod
{
	public:
		using value_type = long long;

	private:
		static const value_type MODULO = 1e9+7;
		value_type value;

		constexpr value_type Normalize(value_type x) const
		{
			return x<0?(x%MODULO+MODULO):(x%MODULO);
		}

	public:
		constexpr Mod():value(0){}
		constexpr Mod(const value_type &val):value(Normalize(val)) {}
		
		constexpr explicit operator value_type () const
		{
			return value;
		}

		constexpr const Mod operator -() const
		{
			return Mod(MODULO - value);
		}
		constexpr const Mod operator +(const Mod &rhs) const
		{
			return Mod(value + rhs.value);
		}
		constexpr const Mod operator -(const Mod &rhs) const
		{
			return Mod(value + (-rhs).value);
		}
		constexpr const Mod operator *(const Mod &rhs) const
		{
			return Mod(value * rhs.value);
		}
		constexpr Mod &operator +=(const Mod &rhs)
		{
			value = Normalize(value + rhs.value);
			return *this;
		}
		constexpr Mod &operator -=(const Mod &rhs)
		{
			value = Normalize(value - rhs.value);
			return *this;
		}
		constexpr Mod &operator *=(const Mod &rhs)
		{
			value = Normalize(value * rhs.value);
			return *this;
		}


		constexpr Mod pow(value_type p) const;

		constexpr Mod inv() const
		{
			return pow(MODULO-2);
		}

		constexpr const Mod operator /(const Mod &rhs) const
		{
			return *this * rhs.inv();
		}
		constexpr Mod &operator /=(const Mod &rhs)
		{
			return *this = *this / rhs;
		}  
		constexpr bool operator ==(const Mod &rhs)
		{
			return value == rhs.value;
		}
};

constexpr Mod Mod::pow(value_type p) const
{
	Mod tmp=1, mult=*this;
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
	ostream& operator<<(ostream& os, const Mod mod)
	{
		os<<(typename Mod::value_type)mod;
		return os;
	}
};



template<size_t N>
struct Factorial
{
	private:
		Mod ary[N];
	public:
		constexpr explicit Factorial():ary()
		{
			ary[0]=1;
			for(size_t i=1;i<N;i++)
				ary[i]=ary[i-1]*i;
		}

		constexpr size_t size() const {   return N;  }

		constexpr Mod operator[] (const int id) const
		{
			return ary[id];
		}
};

template<size_t N>
class FactorialInv
{
	private:
		Mod ary[N];
	public:
		constexpr explicit FactorialInv(const Factorial<N> &fact):ary()
		{
			for(size_t i=0;i<N;i++)
				ary[i]=fact[i].inv();
		}

		//FactorialInv& operator=(FactorialInv&&)=default;

		constexpr Mod operator[] (const int id) const
		{
			return ary[id];
		}
};

template<size_t N, size_t M = N>
class Combination
{
	private:
		const Factorial<N> fact;
		const FactorialInv<M> fact_inv;
	public:
		constexpr Combination(const Factorial<N> fact_, const FactorialInv<M> fact_inv_)
		:fact(fact_),fact_inv(fact_inv_)
		{}

		constexpr Mod operator()(const int n, const int m) const
		{
			return (fact)[n] * (fact_inv)[m] * (fact_inv)[n-m];
		}
};