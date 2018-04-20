//c++14

template<int N> struct IsPrime {
	bool is_prime[N];
	constexpr IsPrime() : is_prime() {
		is_prime[0]=true;
		is_prime[1]=true;
		for(int i=2;i<N;i++)
		{
			if(is_prime[i])  continue;
			for(int j=i+i;j<N;j+=i)
				is_prime[j]=true;
		}
	}
	constexpr bool operator[] (const int p) const
	{
		return !is_prime[p];
	}
};



//g++14

template<unsigned int N> struct IsPrime {
	static const int size=(N>>5) + 1;
	unsigned int is_prime[size];
	constexpr IsPrime() : is_prime() {
		is_prime[0]=0x3;
		for(unsigned int i=2;i<N;i++)
		{
			if((is_prime[i>>5] & (1u<<(i&0x1f))) > 0)  continue;
			for(int j=i+i;j<N;j+=i)
				is_prime[j>>5]|=(1u<<(j&0x1f));
		}
	}
	constexpr bool operator[] (const int p) const
	{
		return (is_prime[p>>5] & (1u<<(p&0x1f))) == 0;
	}
};