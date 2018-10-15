#include <bits/stdc++.h>
#include <iomanip>
#define DEBUG 1

using namespace std;

typedef long long LL;
typedef long double LD;
typedef pair<int, int> PII;
typedef pair<LL, LL> PLL;
typedef pair<LD, LD> PLDLD;
typedef vector<int> VI;
typedef vector<LL> VLL;
typedef vector<char> VB;

#define FOR(i,a,b) for(int i=(a);i<(int)(b);++i)
#define REP(i,n) FOR(i,0,n)
#define CLR(a) memset((a), 0 ,sizeof(a))
#define ALL(a) a.begin(),a.end()
#define UNQ(a) a.erase(std::unique(ALL(a)),a.end());
#define endl "\n"

const LD EPS=1e-10;
const long long INFLL=(LL)(1e9)*(LL)(1e9);
const int INF=1e9+7;

template<class T>
void chmin(T& a, const T b)
{
	if(a>b)
		a=b;
}
template<class T>
void chmax(T& a, const T b)
{
	if(a<b)
		a=b;
}

const LL powLL(const LL p, const LL q)
{
	LL t=1;
	for(int i=0;i<q;i++)
		t*=p;
	return t;
}

template <typename T>
struct has_iter
{
	private:
		template <typename U>
		static constexpr true_type check(typename U::iterator*);
		template <typename U>
		static constexpr false_type check(...);

	public:
		static constexpr bool value = decltype(check<T>(nullptr))::value;
};


template<typename T, typename U = typename T::iterator>
void print(const T& container)
{
		auto&& first=begin(container), last=end(container);
		auto&& back=prev(last);
		for(auto e=first; e!=last; e=next(e))
			cout<<*e<<" \n"[e==back];
}


extern void* enabler;
template<typename Head, typename enable_if<!has_iter<Head>::value>::type*& = enabler>
void print(const Head& head)
{
	cout<<head<<endl;
}

template<> void print<string>(const string& container)
{
	cout<<container<<endl;
}

template<typename Head, typename... Tail>
void print(const Head& head, const Tail&... tail)
{
	cout<<head<<" ";
	print(tail...);
}

template<typename... Args>
void printd(const Args&... args)
{
	#ifdef DEBUG
		print(args...);
	#endif
}

template<typename Head>
void input(Head& head)
{
	cin>>head;
}

template<typename Head, typename... Tail>
void input(Head& head, Tail&... tail)
{
	cin>>head;
	input(tail...);
}

void io_speedup()
{
	cin.tie(0);
	cout.tie(0);
	ios::sync_with_stdio(false);
}

template<typename T>
istream& operator >> (istream& is, vector<T>& vec)
{
	for(T& x: vec) is >> x;
	return is;
}

template<int N, typename... Ts, typename enable_if<N == sizeof...(Ts)-1>::type*& = enabler>
void tuple_out(ostream &os,const tuple<Ts...> &t)
{
	os<<get<N>(t);
}
template<int N, typename... Ts, typename enable_if<N < sizeof...(Ts)-1>::type*& = enabler>
void tuple_out(ostream &os,const tuple<Ts...> &t)
{
	os<<get<N>(t)<<", ";
	tuple_out<N+1, Ts...>(os, t);
}

template<typename... Ts>
ostream& operator << (ostream& os, const tuple<Ts...>& t)
{
	os<<'(';
	tuple_out<0, Ts...>(os, t);
	os<<')';
	return os;
}

template<typename T>
vector<T> read(int n)
{
	vector<T> t(n);
	cin>>t;
	return t;
}

template<typename T>
T read()
{
	T t;
	cin>>t;
	return t;
}

template<typename Head, typename... Tail>
struct vector_demensions
{
	using type=vector<typename vector_demensions<Tail...>::type>;
};

template<typename Head>
struct vector_demensions<Head> { using type=Head; };

template<typename T>
vector<T> make_vectors(int size, T val)
{
	return vector<T>(size, val);
}

template<typename T=int, typename... Args>
auto make_vectors(int size, Args... tail)
	-> typename vector_demensions<Args..., T>::type
{
	auto val=make_vectors<T>(forward<Args>(tail)...);
	return vector<decltype(val)>(size, val);
}

int main()
{
}