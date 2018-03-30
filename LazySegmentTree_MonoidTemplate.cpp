/*
OperatorMonoid:作用素モノイド
act:T x U (x {size})->T
*/

template<class Monoid, class OperatorMonoid>
struct LazySegmentTree
{
    using M = Monoid;
    using OM = OperatorMonoid;
    static_assert(is_same<typename M::type, typename OM::type>::value, "");
    using T = typename M::type;
    using U = typename OM::act_type;

    const int SIZE;
    int tree_size, leaf_number;
    vector<T> node;
    vector<U> lazy_node;
    
    
    LazySegmentTree(const vector<T> &ary);
    
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
    
    const void update(const int a, const int b, const U x, const int k, const int l, const int r);
    const void update(const int a, const int b, const U x)
    {
        return update(a, b, x, 0, 0, leaf_number);
    }
    const T fold(const int a, const int b, const int k, const int l, const int r);
    const T fold(const int a, const int b)
    {
        return fold(a, b, 0, 0, leaf_number);
    }

	const void add_lazy(const int k, const U &x)
	{ 
		lazy_node[k] = OM::op(lazy_node[k], x);
	};
	const void evalute_lazy(const int k, const int size)
	{
		if(size>1)
		{
			add_lazy(child_l(k), lazy_node[k]);
			add_lazy(child_r(k), lazy_node[k]);
		}
		//cout<<k<<" "<<lazies[i].lazy[k]<<" "<<node[k]<<" ";

		node[k] = OM::act(node[k], lazy_node[k], size);
		lazy_node[k] = OM::id();

		//cout<<node[k]<<endl;
	};

	const void print() const
	{
		for(int i=0;i<tree_size;i++)
			cout<<node[i]<<" \n"[i==tree_size-1];
	}
};


template<class M, class OM>
LazySegmentTree<M, OM>::LazySegmentTree(const vector<T> &ary)
:SIZE(ary.size())
{
    tree_size=SIZE;
    leaf_number=least_square(tree_size);
    tree_size=leaf_number*2;
    
    node=vector<T>(tree_size-1);
    lazy_node=vector<U>(tree_size-1, OM::id());
    
    for(int i=0;i<leaf_number;i++)
        if(i<SIZE)
            node[i+leaf_number-1]=ary[i];
        else
            node[i+leaf_number-1]=M::id();
                
    init(0);
}

template<class M, class OM>
const typename M::type LazySegmentTree<M, OM>::init(const int k)
{
	if(k>=leaf_number-1) return node[k];
	return node[k] = M::op(init(child_l(k)),init(child_r(k)));
}

template<class M, class OM>
const void LazySegmentTree<M, OM>::update(const int a, const int b, const U x, const int k, const int l, const int r)
{
	if(r <= a || b <= l)	//[a,b)と[l,r)が交わらない
	{
		evalute_lazy(k, r-l);
		return;  
	}
	if(a <= l && r <= b)	//[a,b)が[l,r)を含む
	{
		add_lazy(k, x);	
		evalute_lazy(k, r-l);
	}
	else
	{
		evalute_lazy(k, r-l);
		update(a,b,x,child_l(k),l,(l+r)/2);
		update(a,b,x,child_r(k),(l+r)/2, r);
        node[k] = M::op(node[child_l(k)], node[child_r(k)]);
	}
}

template<class M, class OM>
const typename M::type LazySegmentTree<M, OM>::fold(const int a, const int b, const int k, const int l, const int r)
{
	if(r <= a || b <= l)
		return M::id();  //[a,b)と[l,r)が交わらない

	evalute_lazy(k, r-l);
	if(a <= l && r <= b) 
		return node[k];    //[a,b)が[l,r)を含む

	return M::op(fold(a,b,child_l(k),l,(l+r)/2), fold(a,b,child_r(k),(l+r)/2, r));
}


template <typename T>
struct MinMonoid 
{
	using type = T;
	static constexpr T id() { return numeric_limits<T>::max(); }
	static T op(const T &a, const T &b) { return min(a, b); }
};

template <typename T>
struct MaxMonoid 
{
	using type = T;
	static constexpr T id() { return numeric_limits<T>::min(); }
	static T op(const T &a, const T &b) { return max(a, b); }
};

template <typename T>
struct PlusMonoid 
{
	using type = T;
	static constexpr T id() { return 0; }
	static T op(const T &a, const T &b) { return a + b; }
};

template <typename T>
struct MultMonoid 
{
	using type = T;
	static constexpr T id() { return 1; }
	static T op(const T &a, const T &b) { return a * b; }
};

template <typename T, typename U>
struct RangeAddOperatorMonoid
{
	using type = T;
	using act_type = U;
	static constexpr U id() { return 0; }
	static U op(const U &a, const U &b) { return a + b; }
	static T act(const T &a, const U &b, const int size) { return a + b * size; }
};

template <typename T, typename U>
struct RangeSubstOperatorMonoid
{
	using type = T;
	using act_type = U;
	static constexpr U id() { return -1; }
	static U op(const U &a, const U &b) { return b; }
	static T act(const T &a, const U &b, const int size) { return b == -1 ? a : b; }
};

template <typename T>
using RARS = LazySegmentTree<PlusMonoid<T>, RangeAddOperatorMonoid<T, T>>;

template <typename T>
using RARM = LazySegmentTree<MaxMonoid<T>, RangeAddOperatorMonoid<T, T>>;