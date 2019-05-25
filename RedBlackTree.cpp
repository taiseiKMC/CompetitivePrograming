enum Side:int
{
	left=0,
	right=1
};
Side oppositeSide(const Side s)
{
	return (Side)(1-s);
}

enum Color:int
{
	Black=0,
	Red,
};


template<typename T, typename U=T>
class RBNode : public enable_shared_from_this<RBNode<T, U>>
{
	public:
		T val;
		Color col;
		size_t size=1;
		shared_ptr<RBNode<T>> child[2];
		weak_ptr<RBNode<T>> parent;

		RBNode(T val_, Color c):val(val_),col(c)
		{
		}

		shared_ptr<RBNode<T>> getBros()
		{
			auto par = parent.lock();
			if(!par) return shared_ptr<RBNode<T>>();
			auto side = getParSide();
			return par->child[1-side];
		}
		Side getParSide()
		{
			auto par = parent.lock();
			auto side = par->child[Side::left] == this->shared_from_this() ? Side::left : Side::right;
			return side;
		}

		shared_ptr<RBNode<T>> nth(size_t n)
		{
			size_t c=child[Side::left]?child[Side::left]->size:0;
			if(c==n)
				return this->shared_from_this();
			Side s=static_cast<Side>(n>c);
			return child[s]->nth(n-(s?(c+1):0));
		}

};

template<typename T, typename U=T, class Compare = less<T>>
class RBTree
{
	using Node = RBNode<T, U>;
	private:
		Compare compare;

		shared_ptr<Node> rotate(shared_ptr<Node> ptr, const Side s)
		{
			shared_ptr<Node> top = ptr->child[1-s];
			ptr->child[1-s]=top->child[s];
			top->child[s] = ptr;
			if(ptr->child[1-s])
			{
				ptr->child[1-s]->parent = ptr;
			}
			if(root==ptr)
			{
				root = top;
			}
			else
			{
				ptr->parent.lock()->child[ptr->getParSide()]=top;
				top->parent=ptr->parent;
			}
			ptr->parent = top;
			
			//ptr->update();
			//top->update();
			return top;
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
	public:
		shared_ptr<Node> root;
		RBTree(){}

		void add(const T x)
		{
			if(!root)
			{
				root = make_shared<Node>(Node(x, Color::Black));
				return;
			}
			auto par = findNode(x);
			Side side;
			if(par->val == x) return;
			if(compare(x, par->val))
				side = Side::left;
			else
				side=Side::right;
			auto node = make_shared<Node>(Node(x, Color::Red));
			par->child[side] = node;
			node->parent = par;
			while(true)
			{
				if(node == root)
				{
					node->col = Color::Black;
					return;
				}
				par = node->parent.lock();
				if(par->col==Color::Black)
					return;
				auto uncle = par->getBros();
				auto grandpar = par->parent.lock();

				//par->col==Color::Red, bros==null, grandpar->col==Color::Black
				if(uncle && uncle->col == Color::Red)
				{
					par->col = Color::Black;
					grandpar->col = Color::Red;
					uncle->col = Color::Black;
					node = grandpar;
					continue;
				}
				
				Side nodeside = node->getParSide();
				Side parside = par->getParSide();
				if(nodeside != parside)
				{
					par = rotate(par, oppositeSide(nodeside));
				}
				par->col = Color::Black;
				grandpar->col = Color::Red;
				rotate(grandpar, oppositeSide(parside));
				break;
			}
		}
		shared_ptr<Node> getMaxNode(shared_ptr<Node> ptr)
		{
			while(ptr->child[Side::right])
				ptr=ptr->child[Side::right];
			return ptr;
		}
		void remove(const int x)
		{
			auto node = findNode(x);
			if(node->val != x) return; //not found
			if(node->child[Side::right] && node->child[Side::left])
			{
				auto rep = getMaxNode(node->child[Side::left]);
				T val = rep->val;
				remove(rep->val);
				node->val=val;
			}
			else
			{
				auto chi=shared_ptr<Node>();
				if(node->child[Side::left])
					chi=node->child[Side::left];
				else
					chi=node->child[Side::right];

				if(node->col==Color::Red || chi && chi->col==Color::Red)
				{
					if(node == root)
					{
						//chi exist
						root=chi;
						chi->parent=weak_ptr<Node>();
						chi->col = Color::Black;
					}
					else
					{
						auto s = node->getParSide();
						if(!chi)
						{
							node->parent.lock()->child[s]=shared_ptr<Node>();
						}
						else
						{
							node->parent.lock()->child[s]=chi;
							chi->parent=node->parent;
							chi->col=Color::Black;
						}
					}
					return;
				}
				
				auto nodeptr = node;
				while(true)
				{
					//node->col=black
					if(root==node) break;
					auto bros = node->getBros();
					auto par = node->parent.lock();
					Side nodeside = node->getParSide();
					if(bros->col==Color::Red)
					{
						par->col=Color::Red;
						bros->col=Color::Black;

						rotate(par, nodeside);
						bros = par->child[oppositeSide(nodeside)];
					}
					//bros=black
					if((!bros->child[Side::left] || bros->child[Side::left]->col == Color::Black) &&
						(!bros->child[Side::right] || bros->child[Side::right]->col == Color::Black))
					{
						if(par->col == Color::Black)
						{
							bros->col = Color::Red;
							node = par;
							continue;
						}
						else
						{
							bros->col = Color::Red;
							par->col = Color::Black;
							break;
						}
					}
					
					bool broschildred[2];
					broschildred[Side::left] = bros->child[Side::left] && bros->child[Side::left]->col == Color::Red;
					broschildred[Side::right] = bros->child[Side::right] && bros->child[Side::right]->col == Color::Red;
					if(broschildred[nodeside] && !broschildred[oppositeSide(nodeside)])
					{
						par->child[oppositeSide(nodeside)] = rotate(bros, oppositeSide(nodeside));
						bros = par->child[oppositeSide(nodeside)];
					}
					//bros = black, bros->child[1-nodeside] = Color::Red
					bros->col = par->col;
					bros->child[1-nodeside]->col=Color::Black;
					par->col = Color::Black;
					par = rotate(par, nodeside);
					break;
				}


				if(nodeptr == root)
				{
					if(chi)
					{
						root=chi;
						chi->parent=weak_ptr<Node>();
					}
					else
						root=shared_ptr<Node>();
				}
				else
				{
					auto s = nodeptr->getParSide();
					if(chi)
					{
						nodeptr->parent.lock()->child[s]=chi;
						chi->parent=nodeptr->parent;
					}
					else
					{
						nodeptr->parent.lock()->child[s]=shared_ptr<Node>();
					}
				}
			}

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
			if(root)
				f(root);
			printd(ary);
		}
		void debug_print2() const
		{
			vector<T> ary;
			function<void(shared_ptr<Node>)> f=[&](shared_ptr<Node> ptr)
			{
				cout<<"(";
				cout<<"BR"[ptr->col==Color::Red]<<" "<<ptr->val<<", ";
				if(ptr->child[Side::left])
					f(ptr->child[Side::left]);
				cout<<", ";
				if(ptr->child[Side::right])
					f(ptr->child[Side::right]);
				cout<<")";
			};
			if(root)
				f(root);
			cout<<endl;
		}
};

mt19937 mt(0);
int main()
{
	RBTree<int> tree;
	int n = read<int>();
	VI ary(n);
	REP(i,n)
		ary[i]=mt()>>1;
	vector<PII> ary2(n);
	REP(i,n)
		ary2[i]=PII(ary[i], i);
	sort(ALL(ary2));
	REP(i,n)
		ary[ary2[i].second]=i;
	REP(i,n)
	{
		tree.add(ary[i]);
	}
	shuffle(ALL(ary), mt);
	tree.debug_print2();
	REP(i,n)
	{
		tree.remove(ary[i]);
		tree.debug_print2();
	}
	tree.debug_print();
}