//より素朴な遅延セグ木
//区間処理は自分で埋める

template<typename T>
struct LazySegmentTree
{
    const int SIZE;
    int tree_size, leaf_number;
    const function<T(T,T)> op;
    vector<T> lazy;
    const T e, e2;
    vector<T> node;
    
    LazySegmentTree(const vector<T> &ary, const function<T(T,T)> f, const T e_, const T e2_);
    
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
        if(k==0) return 0;
        int tmp=k-1;
        for(int i=1; 64>i; i<<=1)
            tmp |= (tmp >> i);
        return tmp+1;
    }
    
	const T init(const int k);
	const void add_lazy(const T x, const int k);
    
    const void update(const int a, const int b, const T x, const int k, const int l, const int r);
    const void update(const int a, const int b, const T x)
    {
        return update(a,b,x,0,0,leaf_number);
    }
    const T fold(const int a, const int b, const int k, const int l, const int r);
    const T fold(const int a,const int b)
    {
        return fold(a, b, 0, 0, leaf_number);
    }
    
    const void evalute_lazy(const int k, const int size);
    
    const void print() const
    {
        REP(i, tree_size)
        cout<<node[i]<<" \n"[i==tree_size-1];
    }
};

template<typename T>
LazySegmentTree<T>::LazySegmentTree(const vector<T> &ary, const function<T(T,T)> f, const T e_, const T e2_)
    :SIZE(ary.size()),op(f),e(e_),e2(e2_)
{
    tree_size=SIZE;
    leaf_number=least_square(tree_size);
    tree_size=leaf_number*2;
    
    node=vector<T>(tree_size-1);
    lazy=vector<T>(tree_size-1, e2);
    
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
const void LazySegmentTree<T>::update(const int a, const int b, const T x, const int k, const int l, const int r)
{
    evalute_lazy(k, r-l);
    if(r <= a || b <= l)
        return;  //[a,b)と[l,r)が交わらない
    if(a <= l && r <= b)
    {
        add_lazy(x,k);    //[a,b)が[l,r)を含む
        evalute_lazy(k, r-l);
    }
    else
    {
        update(a,b,x,child_l(k),l,(l+r)/2);
        update(a,b,x,child_r(k),(l+r)/2, r);
        node[k]=op(node[child_l(k)],node[child_r(k)]);
    }
}
    
template<typename T>
const T LazySegmentTree<T>::fold(const int a, const int b, const int k, const int l, const int r)
{
    if(r <= a || b <= l)
        return e;  //[a,b)と[l,r)が交わらない
        
    evalute_lazy(k, r-l);
    if(a <= l && r <= b) 
        return node[k];    //[a,b)が[l,r)を含む
    
    return op(fold(a,b,child_l(k),l,(l+r)/2), fold(a,b,child_r(k),(l+r)/2, r));
}

template<typename T>
const void LazySegmentTree<T>::add_lazy(const T x, const int k)
{
    //[fill in here]
    lazy[k]^=x;
}

template<typename T>
const void LazySegmentTree<T>::evalute_lazy(const int k, const int size)
{
    if(size>1)
    {
        add_lazy(lazy[k], child_l(k));
        add_lazy(lazy[k], child_r(k));
    }
    //cout<<k<<" "<<lazies[i].lazy[k]<<" "<<node[k]<<" ";

    //[fill in here]
    node[k]=lazy[k]?size-node[k]:node[k];
    lazy[k]=e2;
    
    //cout<<node[k]<<endl;
}

#define RADD [](int p,int q){return p+q;},0,0