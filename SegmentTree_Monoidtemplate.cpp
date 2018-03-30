/*
usage:
SegmentTree<T> tree(array, PLUS) で(array, +, 1)のセグツリーができる
update: 更新
fold:   畳み込み
*/

template<class Monoid>
struct SegmentTree
{
	using M = Monoid;
	using T = typename M::type;

	const int SIZE;
	int tree_size, leaf_number;
	vector<T> node;
	
	SegmentTree(const vector<T> &ary);
	
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
	const void update(const int k, const T x);
	const T fold(const int a, const int b, const int k, const int l, const int r) const;
	const T fold(const int a,const int b) const
	{
		return fold(a, b, 0, 0, leaf_number);
	}
	
	const void print() const
	{
		for(int i=0;i<tree_size;i++)
			cout<<node[i]<<" \n"[i==tree_size-1];
	}
};

template<class M>
SegmentTree<M>::SegmentTree(const vector<T> &ary):SIZE(ary.size())
{
	tree_size=SIZE;
	leaf_number=least_square(tree_size);
	tree_size=leaf_number*2;
	node=vector<T>(tree_size);
	tree_size--;
	
	for(int i=0;i<leaf_number;i++)
		if(i<SIZE)
			node[i+leaf_number-1]=ary[i];
		else
			node[i+leaf_number-1]=M::id();
				
	init(0);
}
	
	
template<class M>
const typename SegmentTree<M>::T SegmentTree<M>::init(const int k)
{
	if(k>=leaf_number-1) return node[k];
	return node[k]=M::op(init(child_l(k)),init(child_r(k)));
}
	
template<class M>
const void SegmentTree<M>::update(const int k, const T x)
{
	int tmp = k+leaf_number-1;
	node[tmp]=x;
	while(tmp > 0)
	{
		tmp=parent(tmp);
		node[tmp]=M::op(node[child_l(tmp)], node[child_r(tmp)]);
	}
}
	
template<class M>
const typename SegmentTree<M>::T SegmentTree<M>::fold(const int a, const int b, const int k, const int l, const int r) const
{
	if(r <= a || b <= l)
		return M::id();  //[a,b)と[l,r)が交わらない
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
struct AddMonoid 
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

template <typename T>
using RMQ = SegmentTree<MinMonoid<T>>;

template <typename T>
using RSM = SegmentTree<AddMonoid<T>>;