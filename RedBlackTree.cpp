template<typename T, class Compare>
class RBTree;

template<typename T>
class RBNode;

template<typename T, class Compare>
class RBTreeIterator //Bidirectional Iterator like
{
	private:
		using Tree = RBTree<T, Compare>;
		struct Ptr { shared_ptr<RBNode<T>> node; bool is_end; };
		Tree const & tree;
		Ptr ptr;
		RBTreeIterator(Tree const & tree_, shared_ptr<RBNode<T>> node):
		tree(tree_), ptr(Ptr{node, false}) {}
		RBTreeIterator(Tree const & tree_): //end of the iterator
		tree(tree_), ptr(Ptr{shared_ptr<RBNode<T>>(), true}) {}

	public:
		using difference_type   = ptrdiff_t;
		using value_type        = T;
		using pointer           = T*;
		using reference         = T&;
		using iterator_category = bidirectional_iterator_tag;
		RBTreeIterator& operator++();
		RBTreeIterator& operator--();
		bool operator==(const RBTreeIterator& rhs)
		{
			//ignore the equality of Tree
			return (ptr.is_end && rhs.ptr.is_end) || ptr.node == rhs.ptr.node;
		}
		bool operator!=(const RBTreeIterator& rhs)
		{
			return !(*this == rhs);
		}
		const T& operator*()
		{
			return ptr.node.get()->value;
		}
		RBTreeIterator prev(const int n = 1)
		{
			auto iter(*this);
			for(int i = 0; i < n; i++)
				--iter;
			return iter;
		}
		RBTreeIterator next(const int n = 1)
		{
			auto iter(*this);
			for(int i = 0; i < n; i++)
				++iter;
			return iter;
		}
		friend RBTree<T, Compare>;
};

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

template<typename T, class Compare = less<T>>
class RBTree
{
	using Node = RBNode<T>;
	using Ptr = shared_ptr<Node>;
	using Iterator = RBTreeIterator<T, Compare>;
	private:
		struct ND
		{
			Ptr node;
			size_t depth;
		};
		Compare compare;
		Ptr root;
		RBTree(const Ptr& root_):root(root_){}
		
		static Ptr getMaxNode(Ptr ptr)
		{
			while(ptr->child[Side::Right])
				ptr = ptr->child[Side::Right];
			return ptr;
		}
		
		template<T compare>
		inline T __bound(const T& x) const
		{
			Ptr node=root, pre;
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

		Ptr rotate(const Ptr&, const Side);
		Ptr findNode(const T&) const;

		size_t depth() const; //the number of black nodes in a way from root to a leaf
		void _remove(Ptr);
		static Ptr joinLeft(const ND&, const T&, const ND&);
		static Ptr joinRight(const ND&, const T&, const ND&);
		static ND join(const ND&, const T&, const ND&);
		static ND join2(const ND&, const ND&);
		static tuple<ND, bool, ND> split(ND , const T&);
		static ND _merge(const ND&, const ND&);
		static ND _intersect(const ND&, const ND&);
		static ND _diff(const ND&, const ND&);
		void checkInvariant();

	public:
		RBTree():root(Ptr()){}
		RBTree(const initializer_list<const T>&); //Construct balanced rb-tree
		void add(const T&); //O(log n)
		void remove(const T& x) //O(log n)
		{
			auto node = findNode(x);
			if(node->value != x) return; //x is not found
			_remove(node);
		}
		
		static RBTree<T, Compare> merge(RBTree<T, Compare>&&, RBTree<T, Compare>&&); //O(m log(n/m + 1))
		RBTree<T, Compare>& merge(RBTree<T, Compare>&&);
		static RBTree<T, Compare> intersect(RBTree<T, Compare>&&, RBTree<T, Compare>&&); //O(m log(n/m + 1))
		RBTree<T, Compare>& intersect(RBTree<T, Compare>&&);
		static RBTree<T, Compare> diff(RBTree<T, Compare>&&, RBTree<T, Compare>&&); //O(m log(n/m + 1))
		RBTree<T, Compare>& diff(RBTree<T, Compare>&&);
		void debug_print() const;
		void print_svg(const string&) const;

		T upper_bound(const T& x) const //O(log n)
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
		T lower_bound(const T& x) const //O(log n)
		{
			return __bound<Compare>(x);
		}
		bool contain(const T& x) const //O(log n)
		{
			auto node = findNode(x);
			return node->value==x;
		}
		Iterator begin() const
		{
			if(!root) return end();
			Ptr ptr = root;
			while(ptr->child[Side::Left])
				ptr = ptr->child[Side::Left];
			return Iterator(*this, ptr);
		}
		Iterator end() const
		{
			return Iterator(*this);
		}

};

template<typename T>
class RBNode : public enable_shared_from_this<RBNode<T>>
{
	public:
		const T value;
		Color color;
		shared_ptr<RBNode<T>> child[2];
		weak_ptr<RBNode<T>> parent;

		RBNode(const T& val, Color c):value(val),color(c)
		{}

		shared_ptr<RBNode<T>> getBrother() const
		{
			auto par = parent.lock();
			if(!par) return shared_ptr<RBNode<T>>();
			auto side = getParentSide();
			return par->child[!side];
		}
		Side getParentSide() const
		{
			auto par = parent.lock();
			auto side = par->child[Side::Left] == this->shared_from_this() ? Side::Left : Side::Right;
			return side;
		}
		inline void setChild(Side s, shared_ptr<RBNode<T>> child)
		{
			this->child[s] = child;
			if(child) child->parent = this->shared_from_this();
		}
		inline static Color getColor(shared_ptr<RBNode<T>> node) //treat null as a black node
		{
			return !node ? Black : node->color;
		}
		
		static shared_ptr<RBNode<T>> rotate(const shared_ptr<RBNode<T>>&, const Side);
};

template<typename T>
shared_ptr<RBNode<T>> RBNode<T>::rotate(const shared_ptr<RBNode<T>>& ptr, const Side s)
{
	auto top = ptr->child[!s];
	if(auto par = ptr->parent.lock())
		par->setChild(ptr->getParentSide(), top);
	else
		top->parent = shared_ptr<RBNode<T>>();
	ptr->setChild(!s, top->child[s]);
	top->setChild(s, ptr);
	return top;
}

template<typename T, class Compare>
RBTreeIterator<T, Compare>& RBTreeIterator<T, Compare>::operator++()
{
	auto&& node = ptr.node;
	if(node->child[Side::Right])
	{
		node = node->child[Side::Right];
		while(node->child[Side::Left])
			node = node->child[Side::Left];
		return *this;
	}
	while(auto par = node->parent.lock())
	{
		if(node->getParentSide() == Side::Left)
		{
			node = par;
			return *this;
		}
		else
			node = par;
	}
	// Reach to the end
	ptr.is_end = true;
	return *this;
}

template<typename T, class Compare>
RBTreeIterator<T, Compare>& RBTreeIterator<T, Compare>::operator--()
{
	auto&& node = ptr.node;
	if(ptr.is_end)
	{
		node = tree.root;
		while(node->child[Side::Right])
			node = node->child[Side::Right];
		ptr.is_end = false;
		return *this;
	}

	if(node->child[Side::Left])
	{
		node = node->child[Side::Left];
		while(node->child[Side::Right])
			node = node->child[Side::Right];
		return *this;
	}
	while(auto par = node->parent.lock())
	{
		if(node->getParentSide() == Side::Right)
		{
			node = par;
			return *this;
		}
		else
			node = par;
	}
	return *this;
}

template<typename T, class Compare>
shared_ptr<RBNode<T>> RBTree<T, Compare>::rotate(const Ptr& ptr, const Side s)
{
	auto top = Node::rotate(ptr, s);
	if(root == ptr)
	{
		root = top;
		root->parent = weak_ptr<Node>();
	}
	return top;
}

template<typename T, class Compare>
shared_ptr<RBNode<T>> RBTree<T, Compare>::findNode(const T& x) const
{
	Ptr node=root, pre;
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

//the number of black nodes in a way from root to a leaf
template<typename T, class Compare>
size_t RBTree<T, Compare>::depth() const
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

template<typename T, class Compare>
void RBTree<T, Compare>::_remove(Ptr node)
{
	if(node->child[Side::Right] && node->child[Side::Left])
	{
		auto rep = getMaxNode(node->child[Side::Left]);
		_remove(rep);
		//node->value = rep->value;

		for(Side s : {Left, Right})
			rep->setChild(s, node->child[s]);
		rep->parent = node->parent;
		if(auto par = node->parent.lock())
		{
			par->child[node->getParentSide()] = rep;
		}
		rep->color = node->color;
	}
	else
	{
		auto chi = node->child[Side::Left] ? node->child[Side::Left] : node->child[Side::Right];

		if(node->color == Color::Red || Node::getColor(chi) == Color::Red)
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
				if(chi)
				{
					chi->color = Color::Black;
					node->parent.lock()->setChild(s, chi);
				}
				else
				{
					node->parent.lock()->child[s] = Ptr();
				}
			}
			return;
		}
		
		//node has no child
		auto nodeptr = node;
		while(true)
		{
			//node->color == Black
			if(root == node) break;
			auto bros = node->getBrother();
			auto par = node->parent.lock();
			Side nodeside = node->getParentSide();
			if(bros->color == Color::Red) //bros must exist
			{
				par->color = Color::Red;
				bros->color = Color::Black;

				rotate(par, nodeside);
				bros = par->child[!nodeside];
				//continue; //is also acceptable
			}

			//bros->color == Black
			if((Node::getColor(bros->child[Side::Left]) == Color::Black) &&
				(Node::getColor(bros->child[Side::Right]) == Color::Black))
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
			
			if(Node::getColor(bros->child[nodeside]) == Color::Red
			  && Node::getColor(bros->child[!nodeside]) != Color::Red)
			{
				par->child[!nodeside] = rotate(bros, !nodeside);
				bros = par->child[!nodeside];
			}
			//bros = Black, bros->child[!nodeside] = Red ||
			//bros = Red, bros->child[!nodeside] = Black
			bros->color = par->color;
			bros->child[!nodeside]->color = Color::Black;
			par->color = Color::Black;
			par = rotate(par, nodeside);
			break;
		}


		if(nodeptr == root)
		{
			root = Ptr();
		}
		else
		{
			auto s = nodeptr->getParentSide();
			nodeptr->parent.lock()->child[s] = Ptr();
		}
	}
}

template<typename T>
inline int isBlack(const shared_ptr<RBNode<T>>& node)
{
	return node->color == Color::Black ? 1 : 0;
}

template<typename T, class Compare>
shared_ptr<RBNode<T>> RBTree<T, Compare>::joinLeft(const ND& l, const T& k, const ND& r)
{
	if(Node::getColor(r.node) == Color::Black && l.depth == r.depth)
	{
		auto node = make_shared<Node>(Node(k, Color::Red));
		node->setChild(Side::Left, l.node);
		node->setChild(Side::Right, r.node);
		return node;
	}
	//depthLeft < depthRight
	auto right = ND{r.node->child[Side::Left], r.depth - isBlack(r.node)};
	auto left = joinLeft(l, k, right);
	r.node->setChild(Side::Left, left);
	auto ll = left->child[Side::Left];
	if(r.node->color == Color::Black
		&& Node::getColor(ll) == Color::Red
		&& left->color == Color::Red)
	{
		ll->color = Color::Black;
		return Node::rotate(r.node, Side::Right);
	}
	return r.node;
}

template<typename T, class Compare>
shared_ptr<RBNode<T>> RBTree<T, Compare>::joinRight(const ND& l, const T& k, const ND& r)
{
	if(Node::getColor(l.node) == Color::Black && l.depth == r.depth)
	{
		auto node = make_shared<Node>(Node(k, Color::Red));
		node->setChild(Side::Left, l.node);
		node->setChild(Side::Right, r.node);
		return node;
	}
	//depthLeft > depthRight
	auto left = ND{l.node->child[Side::Right], l.depth - isBlack(l.node)};
	auto right = joinRight(left, k, r);
	l.node->setChild(Side::Right, right);
	auto rr = right->child[Side::Right];
	if(l.node->color == Color::Black
		&& Node::getColor(rr) == Color::Red
		&& right->color == Color::Red)
	{
		rr->color = Color::Black;
		return Node::rotate(l.node, Side::Left);
	}
	return l.node;
}

template<typename T, class Compare>
typename RBTree<T, Compare>::ND RBTree<T, Compare>::join(const ND& l, const T& k, const ND& r)
{
	if(l.depth == r.depth)
	{
		auto node = make_shared<Node>(Node(k, Color::Black));
		node->setChild(Side::Left, l.node);
		node->setChild(Side::Right, r.node);
		return ND{node, l.depth + 1};
	}
	else
	{
		Ptr node;
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

template<typename T, class Compare>
typename RBTree<T, Compare>::ND RBTree<T, Compare>::join2(const ND& l, const ND& r)
{
	if(!l.node) return r;
	auto node = getMaxNode(l.node);
	auto&& v = node->value;
	ND nd, _r;
	bool b;
	tie(nd, b, _r) = split(l, v);
	return join(nd, v, r);
}

template<typename T, class Compare>
tuple<typename RBTree<T, Compare>::ND, bool, typename RBTree<T, Compare>::ND> RBTree<T, Compare>::split(ND nd, const T& key)
{
	auto node = nd.node;
	if(!node)
		return make_tuple(ND{Ptr(), static_cast<size_t>(0)}, false, ND{Ptr(), static_cast<size_t>(0)});

	auto d = nd.depth - isBlack(node);
	if(node->value == key)
	{
		auto l = ND{node->child[Side::Left], d};
		auto r = ND{node->child[Side::Right], d};
		if(Node::getColor(l.node) == Color::Red)
		{
			l.node->color = Color::Black;
			l.depth++;
		}
		if(Node::getColor(r.node) == Color::Red)
		{
			r.node->color = Color::Black;
			r.depth++;
		}
		return make_tuple(l, true, r);
	}
	else if(node->value < key)
	{
		ND rl, right;
		bool b;
		tie(rl, b, right) = split(ND{node->child[Side::Right], d}, key);
		auto left = join(ND{node->child[Side::Left], d}, node->value, rl);
		return make_tuple(left, b, right);
	}
	else
	{
		ND left, lr;
		bool b;
		tie(left, b, lr) = split(ND{node->child[Side::Left], d}, key);
		auto right = join(lr, node->value, ND{node->child[Side::Right], d});
		return make_tuple(left, b, right);
	}
}

template<typename T, class Compare>
typename RBTree<T, Compare>::ND RBTree<T, Compare>::_merge(const ND& t0, const ND& t1)
{
	if(!t1.node) return t0;
	if(!t0.node) return t1;

	auto d = t1.depth - isBlack(t1.node);
	ND left0, right0;
	bool b;
	tie(left0, b, right0) = split(t0, t1.node->value);

	auto left = _merge(left0, ND{t1.node->child[Side::Left], d});
	auto right = _merge(right0, ND{t1.node->child[Side::Right], d});
	return join(left, t1.node->value, right);
}

template<typename T, class Compare>
typename RBTree<T, Compare>::ND RBTree<T, Compare>::_intersect(const ND& t0, const ND& t1)
{
	if(!t0.node || !t1.node) return ND{Ptr(), 0};

	auto d = t1.depth - isBlack(t1.node);
	ND left0, right0;
	bool b;
	tie(left0, b, right0) = split(t0, t1.node->value);

	auto left = _intersect(left0, ND{t1.node->child[Side::Left], d});
	auto right = _intersect(right0, ND{t1.node->child[Side::Right], d});

	if(b)
		return join(left, t1.node->value, right);
	else
		return join2(left, right);
}

template<typename T, class Compare>
typename RBTree<T, Compare>::ND RBTree<T, Compare>::_diff(const ND& t0, const ND& t1)
{
	if(!t0.node) return ND{Ptr(), 0};
	if(!t1.node) return t0;

	auto d = t1.depth - isBlack(t1.node);
	ND left0, right0;
	bool b;
	tie(left0, b, right0) = split(t0, t1.node->value);

	auto left = _diff(left0, ND{t1.node->child[Side::Left], d});
	auto right = _diff(right0, ND{t1.node->child[Side::Right], d});
	return join2(left, right);
}

template<typename T, typename Compare>
void RBTree<T, Compare>::checkInvariant()
{
	function<tuple<int,bool>(Ptr)> check = [&](Ptr node){
		if(!node) return make_tuple(0, true);
		auto [ld, lb] = check(node->child[Side::Left]);
		auto [rd, rb] = check(node->child[Side::Right]);
		assert(ld == rd);
		auto d = ld;
		auto b = lb && rb;
		if(node->color == Color::Black) d++;
		else
		{
			//node->color == Red
			for(auto&& s : {Side::Left, Side::Right})
				b &= node->child[s]->getColor() == Color::Black;
		}
		return make_tuple(d, b);
	};
	auto [_, b] = check(root);
	assert(b);
}

template<typename T, class Compare>
RBTree<T, Compare>::RBTree(const initializer_list<const T>& list)
{
	const size_t len = static_cast<size_t>(list.size());
	vector<Ptr> ptrs(len);
	size_t blacks = 1;
	while(blacks-1 <= len)
		blacks<<=1;
	blacks = (blacks>>1)-1;
	size_t j = blacks == len ? (((blacks+1)>>1)-1) : blacks;
	auto iter = list.begin();
	for(size_t i=0; i<len; i++)
	{
		Color c = j < blacks ? Color::Black : Color::Red;
		auto&& v = *iter;
		ptrs[j] = make_shared<Node>(Node(v, c));
		iter++;

		//ptr[j]->child[Side::Left] = ptr[(j<<1)+1]
		//ptr[j]->child[Side::Right] = ptr[(j<<1)+2]
		//Get next node index below
		if((j<<1)+2 < len)
		{
			j = (j<<1)+2;
			while((j<<1)+1 < len)
				j=(j<<1)+1;
		}
		else
		{
			while(j>0)
			{
				if(j % 2 == 1)
				{
					j=j>>1;
					break;
				}
				else
					j=(j-2)>>1;
			}
		}
	}
	for(size_t i=0; i<len; i++)
	{
		if((i<<1)+1 < len)
		{
			ptrs[i]->child[Side::Left] = ptrs[(i<<1)+1];
			ptrs[(i<<1)+1]->parent = ptrs[i];
		}
		if((i<<1)+2 < len)
		{
			ptrs[i]->child[Side::Right] = ptrs[(i<<1)+2];
			ptrs[(i<<1)+2]->parent = ptrs[i];
		}
	}
	root = ptrs[0];
}

template<typename T, class Compare>
void RBTree<T, Compare>::add(const T& x)
{
	if(!root)
	{
		root = make_shared<Node>(Node(x, Color::Black));
		return;
	}
	auto par = findNode(x);
	if(par->value == x) return;
	Side side = compare(x, par->value) ? Side::Left : Side::Right;
	auto node = make_shared<Node>(Node(x, Color::Red));
	par->setChild(side, node);
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

		if(Node::getColor(uncle) == Color::Red)
		{
			//par->color==Red, uncle==(Red || null), grandpar->color==Black
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

template<typename T, class Compare>
RBTree<T, Compare> RBTree<T, Compare>::merge(RBTree<T, Compare>&& t0, RBTree<T, Compare>&& t1)
{
	if(t0.size() == 0) return t1;
	if(t1.size() == 0) return t0;

	auto nd = _merge(ND{t0.root, t0.depth()}, ND{t1.root, t1.depth()});
	return RBTree(nd.node);
}

template<typename T, class Compare>
RBTree<T, Compare>& RBTree<T, Compare>::merge(RBTree<T, Compare>&& t)
{
	if(!root)
	{
		root = t.root;
	}
	else if(t.root)
	{
		auto nd = _merge(ND{root, depth()}, ND{t.root, t.depth()});
		root = nd.node;
	}
	// if(!t.root) then just return this itself
	return *this;
}

template<typename T, class Compare>
RBTree<T, Compare> RBTree<T, Compare>::intersect(RBTree<T, Compare>&& t0, RBTree<T, Compare>&& t1)
{
	auto nd = _intersect(ND{t0.root, t0.depth()}, ND{t1.root, t1.depth()});
	return RBTree(nd.node);
}

template<typename T, class Compare>
RBTree<T, Compare>& RBTree<T, Compare>::intersect(RBTree<T, Compare>&& t)
{
	auto nd = _intersect(ND{root, depth()}, ND{t.root, t.depth()});
	root = nd.node;
	return *this;
}

template<typename T, class Compare>
RBTree<T, Compare> RBTree<T, Compare>::diff(RBTree<T, Compare>&& t0, RBTree<T, Compare>&& t1)
{
	auto nd = _diff(ND{t0.root, t0.depth()}, ND{t1.root, t1.depth()});
	return RBTree<T>(nd.node);
}

template<typename T, class Compare>
RBTree<T, Compare>& RBTree<T, Compare>::diff(RBTree<T, Compare>&& t)
{
	auto nd = _diff(ND{root, depth()}, ND{t.root, t.depth()});
	root = nd.node;
	return *this;
}


template<typename T, class Compare>
void RBTree<T, Compare>::debug_print() const
{
	function<void(Ptr)> f = [&](Ptr ptr)
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
	if(root) f(root);
	cout<<endl;
}

template<typename T, class Compare>
void RBTree<T, Compare>::print_svg(const string& filename) const
{
	ofstream fout;
	fout.open(filename, ios::out);
	const double vert = 10, hori = 60;
	//double mx = 800, my=600;
	function<int(Ptr)> getDepth = [&](Ptr n)
	{
		if(!n) return 0;
		return max(getDepth(n->child[Side::Left]), getDepth(n->child[Side::Right])) + 1;
	};
	function<void(double, double, double, Ptr)> f = [&](double x, double y, double v, Ptr ptr)
	{
		//if(x+10 > mx) mx = x+10;
		//if(y+10 > my) my = y+10;
		if(ptr->child[Side::Left])
		{
			const double px = x-v, py = y+hori;
			fout<<"<line x1=\""<<x<<"\" y1=\""<<y<<"\" x2=\""
				<<px<<"\" y2=\""<<py<<"\" stroke=\"#000000\" />"<<endl;
			f(px, py, v/2, ptr->child[Side::Left]);
		}
		if(ptr->child[Side::Right])
		{
			const double px = x+v, py = y+hori;
			fout<<"<line x1=\""<<x<<"\" y1=\""<<y<<"\" x2=\""
				<<px<<"\" y2=\""<<py<<"\" stroke=\"#000000\" />"<<endl;
			f(px, py, v/2, ptr->child[Side::Right]);
		}
		fout<<"<circle cx=\""<<x<<"\" cy=\""<<y<<"\" r=\"5\" fill=\"";
		switch(ptr->color)
		{
			case Color::Red:
				fout<<"#ff0000";
				break;
			case Color::Black:
				fout<<"#000000";
				break;
		}
		fout<<"\" />"<<endl;
		fout<<"<text x=\""<<x+5<<"\" y=\""<<y<<"\">"<<ptr->value<<"</text>"<<endl;
	};
	if(root)
	{
		auto d = getDepth(root);
		double v = vert*(pow(2, d));
		fout<<"<svg width=\""<<v*4<<"\" height=\""<<hori * d + 50<<"\" xmlns=\"http://www.w3.org/2000/svg\">"<<endl;
		f(v*2, 20, v,root);
		fout<<"</svg>"<<endl;
	}
}


int main()
{
	mt19937 mt(0);
	
	RBTree<int> t0, t1;
	int n = read<int>();
	int m = read<int>();
	int k = read<int>();
	VI ary(n+k+m);
	iota(ALL(ary), 0);
	shuffle(ALL(ary), mt);
	REP(i,n+k)
		t0.add(ary[i]);
	REP(i,k+m)
		t1.add(ary[i+n]);

	t0.debug_print();
	t1.debug_print();

	auto print_tree = [](auto&& tree){
		for(auto&& i = tree.begin(); i != tree.end(); ++i)
			cout<<*i<<" ";
		cout<<endl;
	};
	print_tree(t0);
	print_tree(t1);
	RBTree<int> t2;
	REP(i,n+k)
		t2.add(ary[i]);
	REP(i,k)
		t2.remove(ary[i+n]);
	print_tree(t2);


	auto tree = t1.merge(move(t0));
	tree.debug_print();
	print_tree(tree);

	
	t0 = RBTree<int>();
	t1 = RBTree<int>();
	REP(i,n+k)
		t0.add(ary[i]);
	REP(i,k+m)
		t1.add(ary[i+n]);
		
	tree = RBTree<int>::intersect(move(t0), move(t1));
	tree.debug_print();
	print_tree(tree);

	t0 = RBTree<int>();
	t1 = RBTree<int>();
	REP(i,n+k)
		t0.add(ary[i]);
	REP(i,k+m)
		t1.add(ary[i+n]);
		
	tree = RBTree<int>::diff(move(t0), move(t1));
	tree.debug_print();
	print_tree(tree);
	tree.print_svg("test.svg");

	RBTree<int> t{0, 2, 4, 5, 10, 11};
	print_tree(t);
	print_tree(RBTree<int>{0,1,2,3,4,5,6});
	print_tree(RBTree<int>{0,1,2,3,4,5,6,7});
	print_tree(RBTree<int>{0,1,2,3,4,5,6,7,8});
}