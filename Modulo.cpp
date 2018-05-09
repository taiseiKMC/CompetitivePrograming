
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
		
		explicit operator value_type () const
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
		Mod &operator +=(const Mod &rhs)
		{
			return *this = *this + rhs;
		}
		Mod &operator -=(const Mod &rhs)
		{
			return *this = *this - rhs;
		}
		Mod &operator *=(const Mod &rhs)
		{
			return *this = *this * rhs;
		}


		Mod pow(value_type p) const;

		Mod inv() const
		{
			return pow(MODULO-2);
		}

		const Mod operator /(const Mod &rhs) const
		{
			return *this * rhs.inv();
		}
		Mod &operator /=(const Mod &rhs)
		{
			return *this = *this / rhs;
		}  
		constexpr bool operator ==(const Mod &rhs)
		{
			return value == rhs.value;
		}
};

Mod Mod::pow(value_type p) const
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

class Factorial
{
	private:
		vector<Mod> ary;
	public:
		explicit Factorial(const size_t size):ary(vector<Mod>(size))
		{
			ary[0]=1;
			for(size_t i=1;i<size;i++)
				ary[i]=ary[i-1]*i;
		}

		size_t size() const {   return ary.size();  }

		Mod operator[] (const int id) const
		{
			return ary[id];
		}
};
class FactorialInv
{
	private:
		vector<Mod> ary;
	public:
		explicit FactorialInv(const Factorial &fact):ary(vector<Mod>(fact.size()))
		{
			for(size_t i=0;i<ary.size();i++)
				ary[i]=fact[i].inv();
		}

		//FactorialInv& operator=(FactorialInv&&)=default;

		Mod operator[] (const int id) const
		{
			return ary[id];
		}
};

class Combination
{
	private:
		const Factorial *fact;
		const FactorialInv *fact_inv;
	public:
		Combination(const Factorial &fact_, const FactorialInv &fact_inv_):fact(&fact_),fact_inv(&fact_inv_)
		{}

		Mod operator()(const int n, const int m) const
		{
			return (*fact)[n] * (*fact_inv)[m] * (*fact_inv)[n-m];
		}
};
