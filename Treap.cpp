/*
	verified: http://judge.u-aizu.ac.jp/onlinejudge/description.jsp?id=1508
*/

enum Side:int
{
	left=0,
	right=1
};

mt19937 rnd(0);
template<typename T, typename U=T>
class TreapNode : public enable_shared_from_this<TreapNode<T, U>>
{
	public:
		T val;
		U sum=INF;
		int p;
		size_t size=1;
		shared_ptr<TreapNode<T>> child[2];

		TreapNode(T val_, int priority=-1):val(val_),p(priority)
		{
			if(p==-1)
				p=rnd()>>2;

			sum=val;
		}

		void update()
		{
			int leftSize = child[Side::left]?child[Side::left]->size:0;
			int rightSize = child[Side::right]?child[Side::right]->size:0;
			size = leftSize + rightSize + 1;

			//other operations
			sum=min(child[Side::left]?child[Side::left]->sum:INF, child[Side::right]?child[Side::right]->sum:INF);
			chmin(sum, val);
		}
		shared_ptr<TreapNode<T>> nth(size_t n)
		{
			size_t c=child[Side::left]?child[Side::left]->size:0;
			if(c==n)
				return this->shared_from_this();
			Side s=static_cast<Side>(n>c);
			return child[s]->nth(n-(s?(c+1):0));
		}

};

template<typename T, typename U=T, class Compare = less<T>>
class Treap
{
	using Node = TreapNode<T, U>;
	private:
		Compare compare;
		shared_ptr<Node> construct_tree(size_t l, size_t r, vector<T> &ary)
		{
			if(l>=r) return shared_ptr<Node>();
			auto ptr=make_shared<Node>(Node(ary[(l+r)/2]));
			ptr->child[Side::left]=construct_tree(l, (l+r)/2, ary);
			ptr->child[Side::right]=construct_tree((l+r)/2+1, r, ary);
			ptr->update();
			return ptr;
		}
		shared_ptr<Node> findNode(const T x) const
		{
			shared_ptr<Node> node=root, pre;
			while(node)
			{
				pre=node;
				if(x == node->val)
					return node;
				else if(compare(x, node->val))
					node = node->child[Side::left];
				else
					node = node->child[Side::right];
			}
			return pre;
		}
		shared_ptr<Node> rotate(shared_ptr<Node> ptr, const Side s)
		{
			shared_ptr<Node> top = ptr->child[1-s];
			ptr->child[1-s]=top->child[s];
			top->child[s] = ptr;
			ptr->update();
			top->update();
			return top;
		}

		shared_ptr<Node> __add(shared_ptr<Node> ptr, const int k, const T val, const int p)
		{
			if(!ptr) return make_shared<Node>(Node(val, p));
			int c=ptr->child[Side::left]?ptr->child[Side::left]->size:0;
			Side s=static_cast<Side>(k>c);
			ptr->child[s]=__add(ptr->child[s], k-(s?(c+1):0), val, p);
			ptr->update();
			if(ptr->p < ptr->child[s]->p) ptr=rotate(ptr, static_cast<Side>(1-s));
			return ptr;
		}
		shared_ptr<Node> __remove_top(shared_ptr<Node> ptr)
		{
			Side s;
			if(!ptr->child[Side::left] && !ptr->child[Side::right])
				return shared_ptr<Node>();
			else if(!ptr->child[Side::left])
				s=Side::left;
			else if(!ptr->child[Side::right])
				s=Side::right;
			else if(ptr->child[Side::left]->p < ptr->child[Side::right]->p)
				s=Side::right;
			else
				s=Side::left;

			auto par=rotate(ptr,s);
			par->child[s]=__remove_top(ptr);
			par->update();
			return par;
		}
		shared_ptr<Node> __remove(shared_ptr<Node> ptr, const int k)
		{
			int c=ptr->child[Side::left]?ptr->child[Side::left]->size:0;
			if(k==c)
			{
				return __remove_top(ptr);
			}
			else
			{
				Side s=static_cast<Side>(k>c);
				ptr->child[s]=__remove(ptr->child[s], k-(s?(c+1):0));
				ptr->update();
				return ptr;
			}
		}
		void __update(const shared_ptr<Node> ptr, const int k, const int x)
		{
			int c=ptr->child[Side::left]?ptr->child[Side::left]->size:0;
			if(k==c)
			{
				ptr->val=x;
				ptr->update();
				return;
			}
			Side s=static_cast<Side>(k>c);
			__update(ptr->child[s], k-(s?(c+1):0), x);
			ptr->update();
		}
	public:
		shared_ptr<Node> root;
		Treap(){}
		Treap(vector<T> ary):root(construct_tree(0, ary.size(), ary)){}
		void add(const int k, const T x, const int p=-1);
		{
			if(!root)
			{
				root=make_shared<Node>(Node(x, p));
				return;
			}
			root=__add(root, k, x, p);
		}
		void remove(const int k)
		{
			root=__remove(root, k);
		}
		void update(const int k, const T x)
		{
			__update(root, k, x);
		}


		template<T compare>
		inline T __bound(const T x) const
		{
			shared_ptr<Node> node=root, pre;
			while(node)
			{
				if(compare(x, node->val))
				{
					pre = node;
					node = node->child[Side::left];
				}
				else
					node = node->child[Side::right];
			}
			return pre->val;
		}
		T upper_bound(const T x) const
		{
			struct CompareEq
			{
				T operator() (const T x, const T y)
				{
					return x==y || compare(x,y);
				}
			};
			return __bound<CompareEq>(x);
		}
		T lower_bound(const T x) const
		{
			return __bound<Compare>(x);
		}


		bool contain(const T x) const
		{
			auto node = findNode(x);
			return node->val==x;
		}
		size_t size() const
		{
			return root?root->size:0;
		}
		T nth(const size_t n) const
		{
			return root->nth(n)->val;
		}
		U query() const
		{
			return root->sum;
		}

		pair<Treap<T, U, Compare>, Treap<T, U, Compare>> split(const size_t n)
		{
			add(n, 0, 1<<30);
			Treap<T, U, Compare> l, r;
			l.root=root->child[Side::left];
			r.root=root->child[Side::right];
			return make_pair(l, r);
		}

		static Treap<T, U, Compare> merge(Treap<T, U, Compare> l, Treap<T, U, Compare> r)
		{
			Treap<T, U, Compare> treap;
			treap.root=make_shared<Node>(Node(0, 1<<30));
			treap.root->child[Side::left]=l.root;
			treap.root->child[Side::right]=r.root;
			treap.root->update();
			treap.remove(l.size());
			return treap;
		}


		void debug_print() const
		{
			vector<T> ary;
			function<void(shared_ptr<Node>)> f=[&](shared_ptr<Node> ptr)
			{
				if(ptr->child[Side::left])
					f(ptr->child[Side::left]);
				ary.push_back(ptr->val);
				if(ptr->child[Side::right])
					f(ptr->child[Side::right]);
			};
			f(root);
			printd(ary);
		}
};
