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
    const T e;
    vector<T> node;
    
    BIT(const vector<T> &ary, const function<T(T,T)> f, const T e_);
    
    const int child_l(const int k) const
    {
        return k*2+1;
    }
    const int child_r(const int k) const
    {
        return k*2+2;
    }
    const int parent(const int k) const
    {
        return (k-1)/2;
    }
    const int least_square(const int k) const
    {
        int tmp=k;
        for(int i=1; 64>i; i<<=1)
            tmp |= (tmp >> i);
        return tmp+1;
    }
    
    const T init(const int k);
    const void update(const int k, const T x);
    const T fold(const int a, const int b, const int k, const int l, const int r) const;
    const T fold(const int a,const int b) const
    {
        return fold(a, b, 0, 0, leaf_number);
    }
    
    /*const void print() const
    {
        REP(i, tree_size)
        cout<<node[i]<<" \n"[i==tree_size-1];
    }*/
};

template<typename T>
BIT<T>::BIT(const vector<T> &ary, const function<T(T,T)> f, const T e_):SIZE(least_square(ary.size())),op(f),e(e_)
{
    
    node=vector<T>(SIZE);
    
    for(int i=0;i<SIZE;i++)
        if(i<SIZE)
            node[i+leaf_number-1]=ary[i];
        else
            node[i+leaf_number-1]=e;
                
    init(0);
}
    
    
template<typename T>
const T BIT<T>::init(const int k)
{
    if(k>=leaf_number-1) return node[k];
    return node[k]=op(init(child_l(k)),init(child_r(k)));
}
    
template<typename T>
const void BIT<T>::update(const int k, const T x)
{
    int tmp = k+leaf_number-1;
    node[tmp]=x;
    while(tmp > 0)
    {
        tmp=parent(tmp);
        node[tmp]=op(node[child_l(tmp)], node[child_r(tmp)]);
    }
}
    
template<typename T>
const T BIT<T>::fold(const int a, const int b, const int k, const int l, const int r) const
{
    if(r <= a || b <= l)
        return e;  //[a,b)と[l,r)が交わらない
    if(a <= l && r <= b) 
        return node[k];    //[a,b)が[l,r)を含む
    
    return op(sum(a,b,child_l(k),l,(l+r)/2), sum(a,b,child_r(k),(l+r)/2, r));
}


#define PLUS [](int p,int q){return p+q;},0
#define MULT [](int p,int q){return p*q;},1
#define MAX [](int p,int q){return max(p,q);},INT_MIN
#define MIN [](int p,int q){return min(p,q);},INT_MAX

int main()
{
    
}