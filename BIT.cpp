template<typename T>
class BIT
{
	private:
		const int tree_size;
		const function<T(T,T)> op;
		const T e;
		const function<T(T)> inv;

		vector<T> node;
		const int least_square(const int k) const
		{
			int tmp=k;
			for(int i=1; 64>i; i<<=1)
				tmp |= (tmp >> i);
			return tmp+1;
		}

    public:
		const int SIZE;
		BIT(const vector<T> &ary, const function<T(T,T)> f, const T e_, const function<T(T)> inv_);
    
		const void add(const int k, const T x);
		const T fold(const int k) const;
		const T fold(const int a,const int b) const
		{
			return op(fold(b), inv(a==0?e:fold(a-1)));
		}
		
		const void print() const
		{
			REP(i, tree_size)
			    cout<<node[i]<<" \n"[i==tree_size-1];
		}
};

template<typename T>
BIT<T>::BIT(const vector<T> &ary, const function<T(T,T)> f, const T e_, const function<T(T)> inv_)
	:tree_size(least_square(ary.size()+1)),op(f),e(e_),inv(inv_),SIZE(ary.size())
{
    node=vector<T>(tree_size, e);
    
    for(int i=1;i<=SIZE;i++)
        add(i, ary[i-1]);
}


template<typename T>
const void BIT<T>::add(const int k, const T x) //k...0 origin
{
    for(int j=k+1;j<=tree_size;j+=(j&-j))
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

template<typename T>
class RARS
{
	public:
		BIT<T> bit0, bit1;
		RARS(const vector<T> &ary, const function<T(T,T)> f, const T e_, const function<T(T)> inv_)
		:bit0(ary, f, e_, inv_), bit1(vector<T>(ary.size()), f, e_, inv_)
		{}

		const void add(const int l, const int r, const int x)
		{
			bit0.add(l, -x*l);
			bit1.add(l,x);
			bit0.add(r,x*r);
			bit1.add(r,-x);
		}

		const T sum(const int k) const
		{
			return bit0.fold(k)+bit1.fold(k)*k;
		}

		const T get(const int k) const
		{
			return sum(k+1)-sum(k);
		}
		
		const void print() const
		{
			REP(i, bit0.SIZE)
				cout<<get(i)<<" \n"[i==bit0.SIZE-1];
		}
};