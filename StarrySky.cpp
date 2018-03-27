template<typename T>
struct StarrySkyTree
{
    const int SIZE;
    int tree_size, leaf_number;
    const function<T(T,T)> op;
    const T e;
    vector<T> node, segadd;
    
    StarrySkyTree(const vector<T> &ary, const function<T(T,T)> f, const T e_);
    
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
    const void add(const int a, const int b, const T x, const int k, const int l, const int r);
    const void add(const int a, const int b, const T x)
    {
        return add(a, b, x, 0, 0, leaf_number);
    }

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
StarrySkyTree<T>::StarrySkyTree(const vector<T> &ary, const function<T(T,T)> f, const T e_):SIZE(ary.size()),op(f),e(e_)
{
    tree_size=SIZE;
    leaf_number=least_square(tree_size);
    tree_size=leaf_number*2;
    node=vector<T>(tree_size);
    segadd=vector<T>(tree_size, 0);
    tree_size--;
    
    for(int i=0;i<leaf_number;i++)
        if(i<SIZE)
            node[i+leaf_number-1]=ary[i];
        else
            node[i+leaf_number-1]=e;
                
    init(0);
}
    
    
template<typename T>
const T StarrySkyTree<T>::init(const int k)
{
    if(k>=leaf_number-1) return node[k];
    return node[k]=op(init(child_l(k)),init(child_r(k)));
}
    
template<typename T>
const void StarrySkyTree<T>::add(const int a, const int b, const T x, const int k, const int l, const int r)
{
    if(r <= a || b <= l)
        return;  //[a,b)と[l,r)が交わらない
    if(a <= l && r <= b) 
    {
        segadd[k] += x;
        return;    //[a,b)が[l,r)を含む
    }

    add(a, b, x, child_l(k), l, (l+r)/2);
    add(a, b, x, child_r(k), (l+r)/2, r);
    
    node[k] = min(node[child_l(k)] + segadd[child_l(k)], node[child_r(k)] + segadd[child_r(k)]);
}
    
template<typename T>
const T StarrySkyTree<T>::fold(const int a, const int b, const int k, const int l, const int r) const
{
    if(r <= a || b <= l)
        return e;  //[a,b)と[l,r)が交わらない
    if(a <= l && r <= b) 
        return node[k] + segadd[k];    //[a,b)が[l,r)を含む
    
    return op(fold(a,b,child_l(k),l,(l+r)/2), fold(a,b,child_r(k),(l+r)/2, r)) + segadd[k];
}

#define MAX [](int p,int q){return max(p,q);},INT_MIN
#define MIN [](int p,int q){return min(p,q);},INT_MAX
