//n種類の遅延変数に対応したい
//いや無理だしいらんでしょ
using namespace std;
template<typename T>
struct Lazy
{
    vector<T> lazy;
    T e;
    const function<T(T,T)> op;
    const function<T(T,T,int)> range_op;
    Lazy(const int size, const function<T(T,T)> f, const function<T(T,T,int)> g, const T e_)
    :lazy(vector<T>(size, e_)),op(f),range_op(g),e(e_)
    {}
    const void add_lazy(const T x, const int k)
    {
        lazy[k]=op(lazy[k], x);
    }
    const T evalute(const T x, const T y, const int size) const
    {
        return range_op(x,y,size);
    }
    const void reset(const int k)
    {
        lazy[k]=e;
    }
};

template<typename T>
struct LazySegmentTree
{
    const int SIZE;
    int tree_size, leaf_number;
    const function<T(T,T)> op;
    vector<Lazy<T> > lazies;
    const T e;
    vector<T> node;
    
    LazySegmentTree(const vector<T> &ary, const function<T(T,T)> f, const T e_);
    
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
    const void add_lazy(const function<T(T,T)> f, const function<T(T,T,int)> g, const T e_)
    {
        lazies.push_back(Lazy<T>(tree_size, f, g, e_));
    }
    
    const void update(const int a, const int b, const T x, const int index, const int k, const int l, const int r);
    const void update(const int a, const int b, const T x, const int index=0)
    {
        return update(a,b,x,index,0,0,leaf_number);
    }
    const T fold(const int a, const int b, const int k, const int l, const int r);
    const T fold(const int a,const int b)
    {
        return fold(a, b, 0, 0, leaf_number);
    }
    
    const void evalute_lazy(const int k, const int size);
    
    /*const void print() const
    {
        REP(i, tree_size)
        cout<<node[i]<<" \n"[i==tree_size-1];
    }*/
};

template<typename T>
LazySegmentTree<T>::LazySegmentTree(const vector<T> &ary, const function<T(T,T)> f, const T e_)
    :SIZE(ary.size()),op(f),e(e_)
{
    tree_size=SIZE;
    leaf_number=least_square(tree_size);
    tree_size=leaf_number*2;
    
    node=vector<T>(tree_size-1);
    
    for(int i=0;i<leaf_number;i++)
        if(i<SIZE)
            node[i+leaf_number-1]=ary[i];
        else
            node[i+leaf_number-1]=e;
                
    init(0);
}
    
    
template<typename T>
const T LazySegmentTree<T>::init(const int k)
{
    if(k>=leaf_number-1) return node[k];
    return node[k]=op(init(child_l(k)),init(child_r(k)));
}
    
template<typename T>
const void LazySegmentTree<T>::update(const int a, const int b, const T x, const int index, const int k, const int l, const int r)
{
    if(r <= a || b <= l)
        return;  //[a,b)と[l,r)が交わらない
    if(a <= l && r <= b) 
        lazies[index].add_lazy(x,k);    //[a,b)が[l,r)を含む
    else
    {
        node[k]=lazies[index].evalute(node[k], x, min(b,r)-max(a,l));
        update(a,b,x,index,child_l(k),l,(l+r)/2);
        update(a,b,x,index,child_r(k),(l+r)/2, r);
    }
}
    
template<typename T>
const T LazySegmentTree<T>::fold(const int a, const int b, const int k, const int l, const int r)
{
    if(r <= a || b <= l)
        return e;  //[a,b)と[l,r)が交わらない
        
    evalute_lazy(k,r-l);
    if(a <= l && r <= b) 
        return node[k];    //[a,b)が[l,r)を含む
    
    return op(fold(a,b,child_l(k),l,(l+r)/2), fold(a,b,child_r(k),(l+r)/2, r));
}

template<typename T>
const void LazySegmentTree<T>::evalute_lazy(const int k, const int size)
{
    for(int i=0;i<lazies.size();i++)
    {
        if(size>1)
        {
            lazies[i].add_lazy(lazies[i].lazy[k], child_l(k));
            lazies[i].add_lazy(lazies[i].lazy[k], child_r(k));
        }
        //cout<<k<<" "<<lazies[i].lazy[k]<<" "<<node[k]<<" ";
        node[k]=lazies[i].evalute(node[k], lazies[i].lazy[k], size);
        lazies[i].reset(k);
        //cout<<node[k]<<endl;
    }
}

#define PLUS [](int p,int q){return p+q;},0
#define MULT [](int p,int q){return p*q;},1
#define MAX [](int p,int q){return max(p,q);},INT_MIN
#define MIN [](int p,int q){return min(p,q);},INT_MAX
#define RANGEADD [](int p,int q){return p+q;},[](int p,int q,int r){return p+q*r;},0
