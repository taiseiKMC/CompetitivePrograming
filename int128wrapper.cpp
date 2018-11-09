
class Bint
{
	public:
		using value_type = __int128;
	private:
		value_type value;
	public:
		constexpr Bint():value(0){}
		constexpr Bint(const value_type &val):value(val) {}
		constexpr Bint(const long long &val):value(val) {}
		constexpr Bint(const int &val):value(val) {}
		Bint(const string &s)
		{
			value = 0;
			for (size_t i = 0; i < s.length(); i++)
				if ('0' <= s[i] && s[i] <= '9')
					value = value * 10 + s[i] - '0';
		}
		
		template<typename T>
		explicit operator T () const
		{
			return (T)value;
		}

		constexpr const Bint operator -() const
		{
			return Bint(-value);
		}
		constexpr const Bint operator +(const Bint &rhs) const
		{
			return Bint(value + rhs.value);
		}
		constexpr const Bint operator -(const Bint &rhs) const
		{
			return Bint(value + (-rhs).value);
		}
		constexpr const Bint operator *(const Bint &rhs) const
		{
			return Bint(value * rhs.value);
		}
		constexpr const Bint operator /(const Bint &rhs) const
		{
			return Bint(value / rhs.value);
		}
		constexpr const Bint operator %(const Bint &rhs) const
		{
			return Bint(value % rhs.value);
		}
		Bint &operator +=(const Bint &rhs)
		{
			return *this = *this + rhs;
		}
		Bint &operator -=(const Bint &rhs)
		{
			return *this = *this - rhs;
		}
		Bint &operator *=(const Bint &rhs)
		{
			return *this = *this * rhs;
		}
		Bint &operator /=(const Bint &rhs)
		{
			return *this = *this / rhs;
		}
		Bint &operator %=(const Bint &rhs)
		{
			return *this = *this % rhs;
		}
		
		Bint &operator ++()
		{
			return *this += 1;
		}
		Bint operator ++(int)
		{
			auto tmp = *this;
			*this += 1;
			return tmp;
		}
		Bint &operator --()
		{
			return *this -= 1;
		}
		Bint operator --(int)
		{
			auto tmp = *this;
			*this -= 1;
			return tmp;
		}

		Bint pow(long long p) const
		{
			Bint tmp=1, mult=*this;
			while(p>0)
			{
				if((p&1)>0) tmp*=mult;
				p>>=1;
				mult*=mult;
			}
			return tmp;
		}

		constexpr bool operator ==(const Bint &rhs) const
		{
			return value == rhs.value;
		}
		constexpr bool operator !=(const Bint &rhs) const
		{
			return value != rhs.value;
		}
		constexpr bool operator >(const Bint &rhs) const
		{
			return value > rhs.value;
		}
		constexpr bool operator <(const Bint &rhs) const
		{
			return value < rhs.value;
		}
		constexpr bool operator >=(const Bint &rhs) const
		{
			return value >= rhs.value;
		}
		constexpr bool operator <=(const Bint &rhs) const
		{
			return value <= rhs.value;
		}

		string to_s() const
		{
			if(value==0)
			{
				return "0";
			}
			bool minus=value<0;
			char str[128];
			int ind=0;
			Bint tmp(*this);
			while(tmp!=0)
			{
				str[ind++] = abs((int)(tmp%10)) + '0';
				tmp/=10;
			}
			if(minus)
				str[ind++]='-';
			str[ind]='\0';
			for(int i=0;i<ind/2;i++)
				swap(str[i], str[ind-1-i]);
			return string(str);
		}
};

ostream& operator<<(ostream &os, Bint &value)
{
	os<<value.to_s();
	return os;
}