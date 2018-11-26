
namespace FFT
{
	const bool is_pow2(const int p)
	{
		int t=1;
		while(t<p)
			t<<=1;
		return t==p;
	}

	const long double PI=atan2l(0, -1);
	using Complex=complex<long double>;
	class FT
	{
		private:
			//FFTの順方向、逆方向をまとめた関数
			template<class T> 
			static const vector<Complex> FFT_with_flag(const vector<T>& ary, const bool inverse=false);
			
			//1要素のvector<Complex>を作成する
			//目的はTとcomplex<T>のマッチング
			template<class T> 
			static const vector<Complex> create_complex(const complex<T> x)
			{
				return vector<Complex>(1, Complex(x));
			} 
			template<class T> 
			static const vector<Complex> create_complex(const T x)
			{
				return vector<Complex>(1, Complex(x, 0));
			}
		public:
			/*
				FFTの引数は2の冪で、配列の長さの半分まで正しく変換できることが保証される
				余った要素は0で埋める
			*/
			template<class T> 
			static const vector<Complex> FFT(const vector<T>& ary)
			{
				return FFT_with_flag(ary,false);
			}
			template<class T> 
			static const vector<Complex> IFFT(const vector<T>& ary)
			{
				auto iary=FFT_with_flag(ary, true);
				for(int i=0;i<(int)iary.size();i++)
					iary[i]/=iary.size();
				return iary;
			}
		
	};

	template<class T> 
	const vector<Complex> FT::FFT_with_flag(const vector<T>& ary, const bool inverse)
	{
		int n=ary.size();
		if(n==1)
			return create_complex(ary[0]);

		//2の冪でなければいけない
		assert(is_pow2(n));

		//偶数成分のフーリエ変換
		vector<T> ev(n/2);
		for(int i=0;i<n/2;i++)
			ev[i]=ary[i*2];
		auto even=FFT_with_flag(ev, inverse);

		//奇数成分のフーリエ変換
		vector<T> od(n/2);
		for(int i=0;i<n/2;i++)
			od[i]=ary[i*2+1];
		auto odd = FFT_with_flag(od, inverse);

		//以下合性
		if(inverse)
			for(int i=0;i<n/2;i++)
				odd[i]*=Complex(cosl(2*PI*i/n), sinl(2*PI*i/n));
		else
			for(int i=0;i<n/2;i++)
				odd[i]*=Complex(cosl(2*PI*i/n), -sinl(2*PI*i/n));
		
		vector<Complex> ret(n);
		for(int i = 0; i < n/2; i++)
			ret[i] = even[i] + odd[i];
		for(int i = 0; i < n/2; i++)
			ret[i+n/2] = even[i] - odd[i];
		return ret;
	}

	const int least_pow2(const int p)
	{
		int t=1;
		while(t<p)
			t<<=1;
		return t;
	}

	//畳み込み計算
	template<class T>
	vector<Complex> convolution(const vector<T> a, const vector<T> b) {
		vector<Complex> p(least_pow2(a.size())*2), q(least_pow2(b.size())*2);
		copy(a.begin(), a.end(), p.begin());
		copy(b.begin(), b.end(), q.begin());

		auto pf = FT::FFT(p), qf = FT::FFT(q);
		for (size_t i = 0;i < pf.size();i++) pf[i] *= qf[i];
		return FT::IFFT(pf);
	}
}

//most significant bit
int msb(long long p)
{
	int t=0;
	while(p>0)
	{
		p>>=1;
		t++;
	}
	return t;
}

/*
-fsanitizeを外さないと動かない
*/
class Bint
{
	private:
		using data_type = long long;
		bool minus=false;
		vector<data_type> vals;
		static const int DIGIT = 28;
		static const data_type MOD = 1<<DIGIT;
		static vector<Bint> decimal_base_pows;

		size_t get_size() const
		{
			return vals.size();
		}
		data_type &operator[] (const size_t p)
		{
			while(p>=vals.size()) vals.push_back(0);
			return vals[p];
		}
		data_type get_val(const size_t p) const
		{
			if(p>=vals.size()) return 0;
			return vals[p];
		}
		bool is_zero() const
		{
			return get_size()==0;
		}
		void remove_zeroary()
		{
			for(size_t i=vals.size()-1;i>=0;i--)
				if(vals[i]==0)
					vals.pop_back();
				else
					break;
			if(is_zero()) minus=false;
		}
	public:
		static const Bint Zero, One;
		Bint() {}
		Bint(long long val);
		Bint(int val);
		Bint(const string &s);

		explicit operator int () const
		{
			int ret=get_val(0);
			if(get_size()>=1)
				ret+=get_val(1)<<DIGIT;
			return ret;
		}
		explicit operator long long () const
		{
			long long ret=0;
			for(size_t i=0;i<get_size();i++)
				ret+=get_val(i)<<(DIGIT*i);
			return ret;
		}

		static void swap(Bint& a, Bint& b)
		{
			using std::swap;
			swap(a.vals, b.vals);
			swap(a.minus, b.minus);
		}


		bool operator ==(const Bint &rhs) const;
		bool operator !=(const Bint &rhs) const
		{
			return !(*this == rhs);
		}
		bool operator >(const Bint &rhs) const;
		bool operator >=(const Bint &rhs) const
		{
			return *this > rhs || *this == rhs;
		}
		bool operator <(const Bint &rhs) const
		{
			return !(*this >= rhs);
		}
		bool operator <=(const Bint &rhs) const
		{
			return !(*this > rhs);
		}

		Bint abs() const
		{
			Bint tmp(*this);
			tmp.minus = false;
			return tmp;
		}
		Bint operator -() const
		{
			Bint tmp(*this);
			if(!tmp.is_zero())
				tmp.minus^=true;
			return tmp;
		}

		Bint add(const Bint &rhs) const;
		Bint subtract(const Bint &rhs) const;
		Bint operator +(const Bint &rhs) const
		{
			if(minus^rhs.minus)
				return subtract(-rhs);
			else
				return add(rhs);
		}
		Bint operator -(const Bint &rhs) const
		{
			if(minus^rhs.minus)
				return add(-rhs);
			else
				return subtract(rhs);
		}
		Bint &operator +=(const Bint &rhs)
		{
			return *this = *this + rhs;
		}
		Bint &operator -=(const Bint &rhs)
		{
			return *this = *this - rhs;
		}
		Bint &operator ++()
		{
			return *this += One;
		}
		Bint operator ++(int)
		{
			auto tmp = *this;
			*this += One;
			return tmp;
		}
		Bint &operator --()
		{
			return *this -= One;
		}
		Bint operator --(int)
		{
			auto tmp = *this;
			*this -= One;
			return tmp;
		}

		
		Bint operator <<(size_t p) const;
		Bint &operator <<=(const size_t p)
		{
			return *this = (*this<<p);
		}
		Bint operator >>(size_t p) const;
		Bint &operator >>=(const size_t p)
		{
			return *this = (*this>>p);
		}


		Bint operator *(const Bint &rhs) const;
		Bint &operator *=(const Bint &rhs)
		{
			return *this = *this * rhs;
		}
		

		Bint classical_div(const Bint &rhs) const;
		Bint newton_div(const Bint &rhs) const;

		Bint operator /(const Bint &rhs) const
		{
			return classical_div(rhs);
			//return newton_div(rhs);
		}
		Bint &operator /=(const Bint &rhs)
		{
			return *this = *this / rhs;
		}
		Bint operator %(const Bint &rhs) const
		{
			Bint tmp(*this);
			tmp-=tmp/rhs*rhs;
			return tmp;
		}
		Bint &operator %=(const Bint &rhs)
		{
			return *this = *this % rhs;
		}
		
		Bint pow(long long p) const
		{
			Bint tmp=One, mult=*this;
			while(p>0)
			{
				if((p&1)>0) tmp*=mult;
				p>>=1;
				mult*=mult;
			}
			return tmp;
		}


		string to_hex() const;

		string karatsuba_radix_conv() const;
		string naive_to_decimal() const;
		string to_s() const
		{
			//return naive_to_decimal();
			return karatsuba_radix_conv();
		}
};
const Bint Bint::Zero=0;
const Bint Bint::One=1;


Bint::Bint(long long val)
{
	using std::abs;
	if(val<0)
		minus = true;
	while(val!=0)
	{
		vals.push_back(abs(val%MOD));
		val/=MOD;
	}
}
Bint::Bint(int val)
{
	using std::abs;
	if(val<0)
		minus = true;
	while(val!=0)
	{
		vals.push_back(abs(val%MOD));
		val/=MOD;
	}
}
Bint::Bint(const string &s)
{
	for (size_t i = 0; i < s.length(); i++)
		if ('0' <= s[i] && s[i] <= '9')
			*this = *this * 10 + s[i] - '0';
	if(s[0]=='-') minus=true;
}

bool Bint::operator ==(const Bint &rhs) const
{
	if(minus^rhs.minus) return false;
	if(get_size() != rhs.get_size()) return false;
	for(size_t i=0;i<get_size();i++)
		if(vals[i] != rhs.get_val(i)) return false;
	return true;
}
bool Bint::operator >(const Bint &rhs) const
{
	if(!minus && rhs.minus) return true;
	if(minus && !rhs.minus) return false;
	if(minus)
	{
		if(get_size() < rhs.get_size()) return true;
		else if(get_size() > rhs.get_size()) return false;
		for(int i=get_size()-1;i>=0;i--)
			if(vals[i] < rhs.get_val(i)) return true;
			else if(vals[i] > rhs.get_val(i)) return false;
	}
	else
	{
		if(get_size() > rhs.get_size()) return true;
		else if(get_size() < rhs.get_size()) return false;
		for(int i=get_size()-1;i>=0;i--)
			if(vals[i] > rhs.get_val(i)) return true;
			else if(vals[i] < rhs.get_val(i)) return false;
	}
	//等しいとき
	return false;
}


Bint Bint::add(const Bint &rhs) const
{
	Bint tmp(*this);
	size_t i=0;
	data_type carry=0;
	while(carry != 0 || i<max(tmp.get_size(), rhs.get_size()))
	{
		tmp[i] += rhs.get_val(i) + carry;
		if(tmp[i] >= MOD)
		{
			tmp[i] -= MOD;
			carry = 1;
		}
		else carry = 0;
		i++;
	}
	return tmp;
}
//thisとrhsの符号は同じor0
Bint Bint::subtract(const Bint &rhs) const
{
	Bint lt(*this), rt(rhs);
	if(is_zero())
	{
		rt.minus^=true;
		return rt;
	}
	if(lt.abs()<rt.abs())
	{
		swap(lt, rt);
		lt.minus^=true;
	}
	size_t i=0;
	data_type carry=0;
	while(carry != 0 || i<max(lt.get_size(), rt.get_size()))
	{
		lt[i] -= rt.get_val(i) + carry;
		if(lt[i] < 0)
		{
			lt[i] += MOD;
			carry = 1;
		}
		else carry = 0;
		i++;
	}
	lt.remove_zeroary();
	return lt;
}


Bint Bint::operator <<(size_t p) const
{
	Bint tmp=Zero;
	size_t slide=p/DIGIT;
	p%=DIGIT;
	for(size_t i=0;i<slide;i++)
		tmp[i]=0;
	for(size_t i=0;i<vals.size();i++)
		tmp[i+slide] = vals[i];
	data_type carry=0, ncarry;
	for(size_t i=0;i<tmp.get_size();i++)
	{
		ncarry=tmp[i]>>(DIGIT-p);
		tmp[i]=carry + (tmp[i]<<p)%MOD;
		carry=ncarry;
	}
	if(carry!=0)
		tmp[tmp.get_size()] = carry;
	return tmp;
}
Bint Bint::operator >>(size_t p) const
{
	Bint tmp(*this);
	size_t slide=p/DIGIT;
	p%=DIGIT;
	if(vals.size()<slide) return Zero;
	for(size_t i=0;i<vals.size()-slide;i++)
		tmp[i] = get_val(i+slide);
	for(size_t i=vals.size()-slide;i<tmp.get_size();i++)
		tmp[i]=0;
	data_type carry=0, ncarry;
	for(int i=tmp.get_size()-1;i>=0;i--)
	{
		ncarry = (tmp[i]<<(DIGIT-p))%MOD;
		tmp[i]=carry + (tmp[i]>>p);
		carry=ncarry;
	}
	tmp.remove_zeroary();
	
	return tmp;
}


/*
FFTによる乗算
*/
Bint Bint::operator *(const Bint &rhs) const
{
	using namespace FFT;
	auto l=vals, r=rhs.vals;
	size_t len=least_pow2(max(l.size(),r.size()))<<1;
	while(l.size()<len) l.push_back(0);
	while(r.size()<len) r.push_back(0);

	auto comp=convolution(l, r);

	Bint tmp=0;
	for(size_t i=0;i<(comp.size()>>1); i++)
	{
		//誤差〜
		auto c=(data_type)(comp[i].real() + 0.5);
		tmp[i]+=c;
		data_type carry = tmp[i]>>DIGIT;
		if(carry>0)
			tmp[i+1]+=carry;
		tmp[i]%=MOD;
	}
	tmp.remove_zeroary();
	tmp.minus=minus^rhs.minus;
	return tmp;
}


/*
古典的アルゴリズム
*/
Bint Bint::classical_div(const Bint &rhs) const
{
	assert(rhs != Zero);
	if(is_zero()) return Zero;
	long long d1=(get_size()-1)*DIGIT+msb(vals[get_size()-1]);
	long long d2=(rhs.get_size()-1)*DIGIT+msb(rhs.get_val(rhs.get_size()-1));
	auto b=d1-d2;
	if(b<0) return Zero;
	Bint m=rhs<<b;
	if(*this < m)
	{
		b--;
		m>>=1;
	}
	//rhs<<b <= *this < rhs<<(b+1)

	Bint c=Zero,x(*this);
	for(int i=0;i<=b;i++)
	{
		c<<=1;
		auto a=x-m;
		if(a>=0)
		{
			c++;
			x=a;
		}
		m>>=1;
	}
	c.minus=minus^rhs.minus;
	//print("div:", to_hex(), "/", rhs.to_hex(), c.to_hex(), b);
	return c;
}

/*
ニュートン法による除算(ボツ)
*/
Bint Bint::newton_div(const Bint &rhs) const
{
	assert(rhs != Zero);
	if(is_zero()) return Zero;
	long long d1=(get_size()-1)*DIGIT+msb(vals[get_size()-1]);
	long long d2=(rhs.get_size()-1)*DIGIT+msb(rhs.get_val(rhs.get_size()-1));
	long long n =d1+d2;

	Bint m=Zero, x=One, c(2);
	x<<=d1;
	c<<=n;
	while(m!=x)
	{
		m=x;
		x*=c-rhs*x;
		x>>=n;
	}
	x *= *this;
	x>>=n;
	if(*this >= (x+One)*rhs)
		x++;
	return x;
}


string Bint::to_hex() const
{
	if(is_zero()) return "0x0";

	using std::swap;
	string ret="";
	char str[DIGIT/4+2];
	for(size_t i=0;i<get_size();i++)
	{
		sprintf(str, "%07llx", vals[i]);
		for(int j=0;j<DIGIT/4/2;j++)
			swap(str[j], str[DIGIT/4-1-j]);
		ret+=string(str);
	}
	while(ret[ret.size()-1]=='0')
		ret.pop_back();
	ret+="x0";
	if(minus) ret+="-";
	for(size_t i=0;i<ret.size()/2;i++)
		swap(ret[i], ret[ret.size()-1-i]);
	return ret;
}


vector<Bint> Bint::decimal_base_pows(1, Bint(100000000));
/*
http://poset.jp/ompa/node3.html
*/
string Bint::karatsuba_radix_conv() const
{
	if(is_zero()) return "0";
	int ind=0;
	while(*this > decimal_base_pows[ind])
	{
		ind++;
		if((int)decimal_base_pows.size()<=ind)
			decimal_base_pows.push_back(decimal_base_pows.back().pow(2));
	}
	
	using std::swap;
	string ret="";
	function<void(Bint, int)> f=[&](const Bint &b, const int p)
	{
		if(p==-1)
		{
			char str[10];
			sprintf(str, "%08lld", b.get_val(0));
			for(int j=0;j<4;j++)
				swap(str[j], str[7-j]);
			ret+=string(str);
			return;
		}
		Bint tmp=b/decimal_base_pows[p];
		f(b-tmp*decimal_base_pows[p], p-1);
		f(tmp, p-1);
	};
	f(*this, ind-1);

	while(ret[ret.size()-1]=='0')
		ret.pop_back();
	if(minus) ret+="-";
	for(size_t i=0;i<ret.size()/2;i++)
		swap(ret[i], ret[ret.size()-1-i]);
	return ret;
}

string Bint::naive_to_decimal() const
{
	if(is_zero()) return "0";
	using std::swap;
	const data_type mod=100000000;
	Bint tenpow=mod;
	Bint tmp(*this);
	string ret="";
	char str[10];
	while(tmp.abs()>0)
	{
		Bint div=tmp/tenpow;
		sprintf(str, "%08lld", (tmp-div*tenpow).vals[0]);
		for(int j=0;j<4;j++)
			swap(str[j], str[7-j]);
		ret+=string(str);
		tmp=div;
	}
	while(ret[ret.size()-1]=='0')
		ret.pop_back();
	if(minus) ret+="-";
	for(size_t i=0;i<ret.size()/2;i++)
		swap(ret[i], ret[ret.size()-1-i]);
	return ret;
}

ostream& operator<<(ostream &os, const Bint &value)
{
	os<<value.to_s();
	return os;
}


/*
テストコード
*/
int main()
{
	Bint a(10),ten(10);
	for(int i=0;i<500;i++)
	{
		a*=ten;
		cout<<a<<" "<<a.to_hex()<<endl;
	}
	auto arctan=[](int x,int dig)
	{
		Bint a(10);
		a=a.pow(dig);
		bool f=1;
		Bint div=1;
		Bint xp=x;
		Bint ret=a/xp;
		Bint tmp(2);
		while(tmp.abs()>1)
		{
			xp*=x*x;
			div+=2;
			tmp=a/(div*xp);
			if(f) tmp=-tmp;
			f=!f;
			ret+=tmp;
		}
		return ret;
	};
    Bint tmp=Bint(4) * (Bint(4) * arctan(5,1000) - arctan(239,1000));
	cout<<tmp<<endl;
}