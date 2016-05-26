#include <bits/stdc++.h>

using namespace std;

typedef long long LL;
typedef pair<int, int> PII;
typedef long double LD;
typedef pair<LD, LD> PLDLD;

#define FOR(i,a,b) for(int i=(a);i<(b);++i)
#define REP(i,n)  FOR(i,0,n)
#define CLR(a) memset((a), 0 ,sizeof(a))
LL MOD = 1e9+7;
LL INF = 1e9;


using namespace std;
template<typename T>
struct BIT
{
    const int SIZE;
    const function<T(T,T)> op;
    const function<T(T)> inv;
    const T e;
    vector<T> node;
    
    BIT(const vector<T> &ary, const function<T(T,T)> f, const T e_, const function<T(T)> inv_);
    
    const int least_square(const int k) const
    {
        int tmp=k;
        for(int i=1; 64>i; i<<=1)
            tmp |= (tmp >> i);
        return tmp+1;
    }
    
    const void add(const int k, const T x);
    const T fold(const int k) const;
    const T fold(const int a,const int b) const
    {
        return op(fold(b), inv(a==0?e:fold(a-1)));
    }
    
    /*const void print() const
    {
        REP(i, SIZE)
        cout<<node[i]<<" \n"[i==tree_size-1];
    }*/
};

template<typename T>
BIT<T>::BIT(const vector<T> &ary, const function<T(T,T)> f, const T e_, const function<T(T)> inv_):SIZE(least_square(ary.size()+1)),op(f),e(e_),inv(inv_)
{
    node=vector<T>(SIZE, e);
    
    for(int i=1;i<=ary.size();i++)
        add(i, ary[i-1]);
}


template<typename T>
const void BIT<T>::add(const int k, const T x) //k...0 origin
{
    for(int j=k+1;j<=SIZE;j+=(j&-j))
        node[j]=op(node[j], x);
}
    
template<typename T>
const T BIT<T>::fold(const int k) const
{
    T tmp=e;
    for(int j=k+1;j>0;j=j&(j-1))
        tmp=op(tmp, node[j]);
    return tmp;
}


#define PLUS [](int p,int q){return p+q;},0,[](int x){return -x;}
