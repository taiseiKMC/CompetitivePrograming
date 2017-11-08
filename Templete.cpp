#include <bits/stdc++.h>
#include <iomanip>
 
using namespace std;
 
typedef long long LL;
typedef long double LD;
typedef pair<int, int> PII;
typedef pair<LL, LL> PLL;
typedef pair<LD, LD> PLDLD;
typedef vector<int> VI;
typedef vector<char> VB;
 
#define FOR(i,a,b) for(int i=(a);i<(int)(b);++i)
#define REP(i,n)  FOR(i,0,n)
#define CLR(a) memset((a), 0 ,sizeof(a))
#define ALL(a) a.begin(),a.end()
 
const LD eps=1e-5;
const long long INFL=(LL)(1e9)*(LL)(1e9);
const int INF=1e9;
 
template<class T>
void chmin(T& a, const T& b)
{
	if(a>b)
		a=b;
}
template<class T>
void chmax(T& a, const T& b)
{
	if(a<b)
		a=b;
}
 
const LL pow(const LL p, const LL q)
{
	LL t=1;
	REP(i,q)
		t*=p;
	return t;
}

//print for container
template<typename Iterator>
void print(const Iterator& first, const Iterator& last)
{
	auto&& back=prev(last);
	for(auto e=first; e!=last; e=next(e))
		cout<<*e<<" \n"[e==back];
}

template<typename Head>
void print(const Head& head)
{
	cout<<head<<endl;
}

template<typename Head, typename... Tail>
void print(const Head& head, const Tail&... tail)
{
	cout<<head<<" ";
	print(tail...);
}

void io_speedup()
{
	cin.tie(0);
	ios::sync_with_stdio(false);
}

template<typename T>
T read()
{
	T t;
	cin>>t;
	return t;
}


int main()
{
}
