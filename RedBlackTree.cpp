enum Side : int
{
	Left = 0,
	Right = 1
};
constexpr Side operator!(const Side& s)
{
	return static_cast<Side>(1 - s);
}

enum Color : int
{
	Black = 0,
	Red,
};


template<typename T>
class RBNode : public enable_shared_from_this<RBNode<T>>
{
	public:
		T value;
		Color color;
		size_t size = 1;
		shared_ptr<RBNode<T>> child[2];
		weak_ptr<RBNode<T>> parent;

		RBNode(T val, Color c):value(val),color(c)
		{}

		shared_ptr<RBNode<T>> getBrother()
		{
			auto par = parent.lock();
			if(!par) return shared_ptr<RBNode<T>>();
			auto side = getParentSide();
			return par->child[!side];
		}
		Side getParentSide()
		{
			auto par = parent.lock();
			auto side = par->child[Side::Left] == this->shared_from_this() ? Side::Left : Side::Right;
			return side;
		}
		
		static shared_ptr<RBNode<T>> rotate(shared_ptr<RBNode<T>> ptr, const Side s)
		{
			auto top = ptr->child[!s];
			ptr->child[!s] = top->child[s];
			top->child[s] = ptr;
			if(ptr->child[!s])
			{
				ptr->child[!s]->parent = ptr;
			}
			if(auto par = ptr->parent.lock())
			{
				par->child[ptr->getParentSide()] = top;
				top->parent = ptr->parent;
			}
			ptr->parent = top;
			return top;
		}

		shared_ptr<RBNode<T>> nth(const size_t n)
		{
			size_t c = child[Side::Left] ? child[Side::Left]->size : 0;
			if(c == n)
				return this->shared_from_this();
			auto s = static_cast<Side>(n > c);
			return child[s]->nth(n - (s ? (c + 1) : 0));
		}

};

template<typename T, class Compare = less<T>>
class RBTree
{
	using Node = RBNode<T>;
	private:
		struct ND
		{
			shared_ptr<Node> node;
			size_t depth;
		};
		Compare compare;

		shared_ptr<Node> rotate(shared_ptr<Node> ptr, const Side s)
		{
			auto top = Node::rotate(ptr, s);
			if(root == ptr)
			{
				root = top;
				root->parent = weak_ptr<Node>();
			}
			return top;
		}

		shared_ptr<Node> findNode(const T& x) const
		{
			shared_ptr<Node> node=root, pre;
			while(node)
			{
				pre=node;
				if(x == node->value)
					return node;
				else if(compare(x, node->value))
					node = node->child[Side::Left];
				else
					node = node->child[Side::Right];
			}
			return pre;
		}

		static shared_ptr<Node> joinLeft(const ND& l, const T& k, const ND& r)
		{
			if(!r.node || (r.node->color == Color::Black && l.depth == r.depth))
			{
				auto node = make_shared<Node>(Node(k, Color::Red));
				node->child[Side::Left] = l.node;
				node->child[Side::Right] = r.node;
				if(l.node) l.node->parent = node;
				if(r.node) r.node->parent = node;
				return node;
			}
			//depthLeft < depthRight
			auto right = ND{r.node->child[Side::Left], r.depth - (r.node->color == Color::Black ? 1 : 0)};
			auto left = joinLeft(l, k, right);
			r.node->child[Side::Left] = left;
			left->parent = r.node;
			auto ll = left->child[Side::Left];
			if(r.node->color == Color::Black
				&& ll && ll->color == Color::Red
				&& left->color == Color::Red)
			{
				ll->color = Color::Black;
				return Node::rotate(r.node, Side::Right);
			}
			return r.node;
		}
		static shared_ptr<Node> joinRight(const ND& l, const T& k, const ND& r)
		{
			if(!l.node || (l.node->color == Color::Black && l.depth == r.depth))
			{
				auto node = make_shared<Node>(Node(k, Color::Red));
				node->child[Side::Left] = l.node;
				node->child[Side::Right] = r.node;
				if(l.node) l.node->parent = node;
				if(r.node) r.node->parent = node;
				return node;
			}
			//depthLeft > depthRight
			auto left = ND{l.node->child[Side::Right], l.depth - (l.node->color == Color::Black ? 1 : 0)};
			auto right = joinRight(left, k, r);
			l.node->child[Side::Right] = right;
			right->parent = l.node;
			auto rr = right->child[Side::Right];
			if(l.node->color == Color::Black
				&& rr && rr->color == Color::Red
				&& right->color == Color::Red)
			{
				rr->color = Color::Black;
				return Node::rotate(l.node, Side::Left);
			}
			return l.node;
		}
		static ND join(const ND& l, const T& k, const ND& r)
		{
			if(l.depth == r.depth)
			{
				auto node = make_shared<Node>(Node(k, Color::Black));
				node->child[Side::Left] = l.node;
				node->child[Side::Right] = r.node;
				if(l.node) l.node->parent = node;
				if(r.node) r.node->parent = node;
				return ND{node, l.depth + 1};
			}
			else
			{
				shared_ptr<Node> node;
				if(l.depth < r.depth)
					node = joinLeft(l, k, r);
				else if(l.depth > r.depth)
					node = joinRight(l, k, r);

				size_t depth = max(l.depth, r.depth);
				if(node->color == Color::Red)
				{
					depth++;
					node->color = Color::Black;
				}
				return ND{node, depth};
			}
		}

		static tuple<ND, bool, ND> split(ND nd, const T& key)
		{
			auto node = nd.node;
			auto d = nd.depth - (node->color == Color::Black ? 1 : 0);
			if(node->value == key)
			{
				auto l = ND{node->child[Side::Left], d};
				auto r = ND{node->child[Side::Right], d};
				if(l.node->color == Color::Red)
				{
					l.node->color = Color::Black;
					l.depth++;
				}
				if(r.node->color == Color::Red)
				{
					r.node->color = Color::Black;
					r.depth++;
				}
				return make_tuple(l, true, r);
			}
			else if(node->value < key)
			{
				if(node->child[Side::Right])
				{
					ND rl, right;
					bool b;
					tie(rl, b, right) = split(ND{node->child[Side::Right], d}, key);
					auto left = join(ND{node->child[Side::Left], d}, node->value, rl);
					return make_tuple(left, b, right);
				}
				else
				{
					if(node->color == Color::Red)
					{
						node->color = Color::Black;
						nd.depth++;
					}
					return make_tuple(nd, false, ND{shared_ptr<Node>(), static_cast<size_t>(0)});
				}
			}
			else
			{
				if(node->child[Side::Left])
				{
					ND left, lr;
					bool b;
					tie(left, b, lr) = split(ND{node->child[Side::Left], d}, key);
					auto right = join(lr, node->value, ND{node->child[Side::Right], d});
					return make_tuple(left, b, right);
				}
				else
				{
					if(node->color == Color::Red)
					{
						node->color = Color::Black;
						nd.depth++;
					}
					return make_tuple(ND{shared_ptr<Node>(), static_cast<size_t>(0)}, false, nd);
				}
			}
		}
		static ND _merge(const ND& t0, const ND& t1)
		{
			if(!t1.node) return t0;
			if(!t0.node) return t1;

			auto d = t1.depth - (t1.node->color == Color::Black ? 1 : 0);
			ND left0, right0;
			bool b;
			tie(left0, b, right0) = split(t0, t1.node->value);

			/*
			printd("split:", t1.node->value, (int)left0.depth, (int)right0.depth);
			RBTree<T> tree;
			tree.root = left0.node;
			tree.debug_print2();
			tree.root = right0.node;
			tree.debug_print2();
			*/

			auto left = _merge(left0, ND{t1.node->child[Side::Left], d});
			auto right = _merge(right0, ND{t1.node->child[Side::Right], d});

			/*
			printd("dmerge:", (int)left.depth, (int)right.depth);
			tree.root = left.node;
			tree.debug_print2();
			tree.root = right.node;
			tree.debug_print2();
			*/

			return join(left, t1.node->value, right);
		}

	public:
		shared_ptr<Node> root;
		RBTree(){}

		void add(const T& x)
		{
			if(!root)
			{
				root = make_shared<Node>(Node(x, Color::Black));
				return;
			}
			auto par = findNode(x);
			Side side;
			if(par->value == x) return;
			if(compare(x, par->value))
				side = Side::Left;
			else
				side = Side::Right;
			auto node = make_shared<Node>(Node(x, Color::Red));
			par->child[side] = node;
			node->parent = par;
			while(true)
			{
				if(node == root)
				{
					node->color = Color::Black;
					return;
				}
				par = node->parent.lock();
				if(par->color == Color::Black)
					return;
				auto uncle = par->getBrother();
				auto grandpar = par->parent.lock();

				//par->color==Color::Red, bros==null, grandpar->color==Color::Black
				if(uncle && uncle->color == Color::Red)
				{
					par->color = Color::Black;
					grandpar->color = Color::Red;
					uncle->color = Color::Black;
					node = grandpar;
					continue;
				}
				
				Side nodeside = node->getParentSide();
				Side parside = par->getParentSide();
				if(nodeside != parside)
				{
					par = rotate(par, !nodeside);
				}
				par->color = Color::Black;
				grandpar->color = Color::Red;
				rotate(grandpar, !parside);
				break;
			}
		}
		shared_ptr<Node> getMaxNode(shared_ptr<Node> ptr) const
		{
			while(ptr->child[Side::Right])
				ptr = ptr->child[Side::Right];
			return ptr;
		}
		void remove(const T& x)
		{
			auto node = findNode(x);
			if(node->value != x) return; //not found
			if(node->child[Side::Right] && node->child[Side::Left])
			{
				auto rep = getMaxNode(node->child[Side::Left]);
				T value = rep->value;
				remove(rep->value);
				node->value=value;
			}
			else
			{
				shared_ptr<Node> chi;
				if(node->child[Side::Left])
					chi = node->child[Side::Left];
				else
					chi = node->child[Side::Right];

				if(node->color == Color::Red || chi && chi->color == Color::Red)
				{
					if(node == root)
					{
						//chi exists
						root = chi;
						chi->parent = weak_ptr<Node>();
						chi->color = Color::Black;
					}
					else
					{
						auto s = node->getParentSide();
						if(!chi)
						{
							node->parent.lock()->child[s] = shared_ptr<Node>();
						}
						else
						{
							node->parent.lock()->child[s] = chi;
							chi->parent = node->parent;
							chi->color = Color::Black;
						}
					}
					return;
				}
				
				auto nodeptr = node;
				while(true)
				{
					//node->color=black
					if(root==node) break;
					auto bros = node->getBrother();
					auto par = node->parent.lock();
					Side nodeside = node->getParentSide();
					if(bros->color == Color::Red)
					{
						par->color = Color::Red;
						bros->color = Color::Black;

						rotate(par, nodeside);
						bros = par->child[!nodeside];
					}
					//bros=black
					if((!bros->child[Side::Left] || bros->child[Side::Left]->color == Color::Black) &&
						(!bros->child[Side::Right] || bros->child[Side::Right]->color == Color::Black))
					{
						if(par->color == Color::Black)
						{
							bros->color = Color::Red;
							node = par;
							continue;
						}
						else
						{
							bros->color = Color::Red;
							par->color = Color::Black;
							break;
						}
					}
					
					bool broschildred[2];
					broschildred[Side::Left] = bros->child[Side::Left] && bros->child[Side::Left]->color == Color::Red;
					broschildred[Side::Right] = bros->child[Side::Right] && bros->child[Side::Right]->color == Color::Red;
					if(broschildred[nodeside] && !broschildred[!nodeside])
					{
						par->child[!nodeside] = rotate(bros, !nodeside);
						bros = par->child[!nodeside];
					}
					//bros = black, bros->child[1-nodeside] = Color::Red
					bros->color = par->color;
					bros->child[1-nodeside]->color = Color::Black;
					par->color = Color::Black;
					par = rotate(par, nodeside);
					break;
				}


				if(nodeptr == root)
				{
					if(chi)
					{
						root = chi;
						chi->parent = weak_ptr<Node>();
					}
					else
						root = shared_ptr<Node>();
				}
				else
				{
					auto s = nodeptr->getParentSide();
					if(chi)
					{
						nodeptr->parent.lock()->child[s]=chi;
						chi->parent = nodeptr->parent;
					}
					else
					{
						nodeptr->parent.lock()->child[s] = shared_ptr<Node>();
					}
				}
			}

		}

		static RBTree<T> merge(RBTree<T>&& t0, RBTree<T>&& t1)
		{
			if(t0.size() == 0) return t1;
			if(t1.size() == 0) return t0;

			auto nd = _merge(ND{t0.root, t0.depth()}, ND{t1.root, t1.depth()});
			RBTree<T> ret;
			ret.root = nd.node;
			return ret;
		}
		RBTree<T> & merge(RBTree<T>&& t)
		{
			if(size() == 0)
			{
				root = t.root;
			}
			else if(t.size() != 0)
			{
				auto nd = _merge(ND{root, depth()}, ND{t.root, t.depth()});
				root = nd.node;
			}
			return *this;
		}

		template<T compare>
		inline T __bound(const T& x) const
		{
			shared_ptr<Node> node=root, pre;
			while(node)
			{
				if(compare(x, node->value))
				{
					pre = node;
					node = node->child[Side::Left];
				}
				else
					node = node->child[Side::Right];
			}
			return pre->value;
		}
		T upper_bound(const T& x) const
		{
			struct CompareEq
			{
				T operator() (const T& x, const T& y)
				{
					return x==y || compare(x,y);
				}
			};
			return __bound<CompareEq>(x);
		}
		T lower_bound(const T& x) const
		{
			return __bound<Compare>(x);
		}


		bool contain(const T& x) const
		{
			auto node = findNode(x);
			return node->value==x;
		}
		size_t size() const
		{
			return root ? root->size : 0;
		}

		//the number of black nodes in a way from root to a leaf
		size_t depth() const
		{
			size_t i = 0;
			auto node = root;
			while(node)
			{
				if(node->color == Color::Black) i++;
				node = node->child[Side::Left];
			}
			return i;
		}
		T nth(const size_t n) const
		{
			return root->nth(n)->value;
		}

		void debug_print() const
		{
			vector<T> ary;
			function<void(shared_ptr<Node>)> f = [&](shared_ptr<Node> ptr)
			{
				if(ptr->child[Side::Left])
					f(ptr->child[Side::Left]);
				ary.push_back(ptr->value);
				if(ptr->child[Side::Right])
					f(ptr->child[Side::Right]);
			};
			if(root)
				f(root);
			printd(ary);
		}
		void debug_print2() const
		{
			vector<T> ary;
			function<void(shared_ptr<Node>)> f = [&](shared_ptr<Node> ptr)
			{
				cout<<"(";
				cout<<"BR"[ptr->color==Color::Red]<<" "<<ptr->value<<", ";
				if(ptr->child[Side::Left])
					f(ptr->child[Side::Left]);
				cout<<", ";
				if(ptr->child[Side::Right])
					f(ptr->child[Side::Right]);
				cout<<")";
			};
			if(root)
				f(root);
			cout<<endl;
		}
};

int main()
{
	mt19937 mt(0);

	RBTree<int> t0, t1;
	int n = read<int>();
	int m = read<int>();
	VI ary(n+m);
	iota(ALL(ary), 0);
	shuffle(ALL(ary), mt);
	REP(i,n)
		t0.add(ary[i]);
	REP(i,m)
		t1.add(ary[i+n]);
	t0.add(0);
	t0.debug_print2();
	t1.debug_print2();

	auto tree = t0.merge(move(t1));
	tree.debug_print2();
	tree.debug_print();
}
