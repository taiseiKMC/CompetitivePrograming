
#include <bits/stdc++.h>
#include <iomanip>
#define DEBUG 1

using namespace std;

typedef long long LL;
typedef long double LD;
typedef pair<int, int> PII;
typedef pair<LL, LL> PLL;
typedef pair<LD, LD> PLDLD;
typedef vector<int> VI;
typedef vector<LL> VLL;
typedef vector<char> VB;

#define FOR(i,a,b) for(int i=(a);i<(int)(b);++i)
#define REP(i,n) FOR(i,0,n)
#define RFOR(i,a,b) for(int i=(a)-1;i>=(int)(b);--i)
#define RREP(i,n) RFOR(i,n,0)
#define CLR(a) memset((a), 0 ,sizeof(a))
#define ALL(a) a.begin(),a.end()
#define UNQ(a) a.erase(std::unique(ALL(a)),a.end());
#define endl "\n"

#define BEGIN_STACK(size) \
	void *dummy = malloc(size); \
	void *org_stack; \
	char *my = (char *)alloca((1 + (int)(((long long)dummy) & 127)) * 16); \
	*my = 0; \
	asm volatile("mov %%rsp, %%rbx\n" \
		"mov %%rax, %%rsp" \
		: "=b"(org_stack) \
		: "a"((char *)dummy + (size)-1024));

#define END_STACK \
	asm volatile("mov %%rax, %%rsp" ::"a"(org_stack)); \
	free(dummy);

const LD EPS=1e-10;
const long long INFLL=(LL)(1e9)*(LL)(1e9);
const int INF=1e9+7;

template<class T>
void chmin(T& a, const T b)
{
	if(a>b)
		a=b;
}
template<class T>
void chmax(T& a, const T b)
{
	if(a<b)
		a=b;
}

const LL powLL(const LL p, const LL q)
{
	LL t=1;
	for(int i=0;i<q;i++)
		t*=p;
	return t;
}

template <typename T>
struct has_iter
{
	private:
		template <typename U>
		static constexpr true_type check(typename U::iterator*);
		template <typename U>
		static constexpr false_type check(...);

	public:
		static constexpr bool value = decltype(check<T>(nullptr))::value;
};


template<typename T, typename U = typename T::iterator>
void print(const T& container)
{
		auto&& first=begin(container), last=end(container);
		auto&& back=prev(last);
		for(auto e=first; e!=last; e=next(e))
			cout<<*e<<" \n"[e==back];
}


extern void* enabler;
template<typename Head, typename enable_if<!has_iter<Head>::value>::type*& = enabler>
void print(const Head& head)
{
	cout<<head<<endl;
}

template<> void print<string>(const string& container)
{
	cout<<container<<endl;
}

template<typename Head, typename... Tail>
void print(const Head& head, const Tail&... tail)
{
	cout<<head<<" ";
	print(tail...);
}

template<typename... Args>
void printd(const Args&... args)
{
	#ifdef DEBUG
		print(args...);
	#endif
}

template<typename Head>
void input(Head& head)
{
	cin>>head;
}

template<typename Head, typename... Tail>
void input(Head& head, Tail&... tail)
{
	cin>>head;
	input(tail...);
}

void io_speedup()
{
	cin.tie(0);
	cout.tie(0);
	ios::sync_with_stdio(false);
}

template<typename T>
istream& operator >> (istream& is, vector<T>& vec)
{
	for(T& x: vec) is >> x;
	return is;
}


template<typename T, typename U>
istream& operator >> (istream& is, pair<T, U>& t)
{
	is>>t.first>>t.second;
	return is;
}

template<int N, typename... Ts, typename enable_if<N == sizeof...(Ts)-1>::type*& = enabler>
void tuple_in(istream &is, tuple<Ts...> &t)
{
	is>>get<N>(t);
}
template<int N, typename... Ts, typename enable_if<N < sizeof...(Ts)-1>::type*& = enabler>
void tuple_in(istream &is, tuple<Ts...> &t)
{
	is>>get<N>(t);
	tuple_in<N+1, Ts...>(is, t);
}

template<typename... Ts>
istream& operator >> (istream& is, tuple<Ts...>& t)
{
	tuple_in<0, Ts...>(is, t);
	return is;
}


template<typename T, typename U>
ostream& operator << (ostream& os, const pair<T, U>& t)
{
	os<<'('<<t.first<<", "<<t.second<<')';
	return os;
}

template<int N, typename... Ts, typename enable_if<N == sizeof...(Ts)-1>::type*& = enabler>
void tuple_out(ostream &os,const tuple<Ts...> &t)
{
	os<<get<N>(t);
}
template<int N, typename... Ts, typename enable_if<N < sizeof...(Ts)-1>::type*& = enabler>
void tuple_out(ostream &os,const tuple<Ts...> &t)
{
	os<<get<N>(t)<<", ";
	tuple_out<N+1, Ts...>(os, t);
}

template<typename... Ts>
ostream& operator << (ostream& os, const tuple<Ts...>& t)
{
	os<<'(';
	tuple_out<0, Ts...>(os, t);
	os<<')';
	return os;
}

template<typename T>
vector<T> read(int n)
{
	vector<T> t(n);
	cin>>t;
	return t;
}

template<typename T>
T read()
{
	T t;
	cin>>t;
	return t;
}

///
/// Dice
///
enum Face
{
	Front,Up,Back,Down,Left,Right,
};
template<typename T>
struct Dice
{
	T pip[6];
	T& operator[] (const Face id)
	{
		return pip[id];
	}
	const T& operator[] (const Face id) const
	{
		return pip[id];
	}

	void rotate(Face f1, Face f2, Face f3, Face f4)
	{
		int tmp=pip[f1];
		pip[f1]=pip[f2];
		pip[f2]=pip[f3];
		pip[f3]=pip[f4];
		pip[f4]=tmp;
	}
	void rollx() { rotate(Up, Front, Down, Back); }
	void rollxi() { rotate(Up, Back, Down, Front); }
	void rolly() { rotate(Up, Left, Down, Right); }
	void rollyi() { rotate(Up, Right, Down, Left); }
	void rollz() { rotate(Back, Left, Front, Right); }
	void rollzi() { rotate(Back, Right, Front, Left); }
};

///
///Union Find
///

class UnionFind
{
    private:
        vector<int> parent;

    public:
        UnionFind(const int n):parent(vector<int>(n,-1))
        {}
        
        const int Find(const int p)
        {
            return parent[p] < 0 ? p : parent[p] = Find(parent[p]);
        }
        const void Merge(int p, int q);

        const bool Belong(const int p, const int q)
        {
            return Find(p) == Find(q);
        }
        const int GetSize(const int p)
        {
            return -parent[Find(p)];
        }
};

const void UnionFind::Merge(int p, int q)
{
    p=Find(p);
    q=Find(q);
    if(p==q) return;
    if(parent[p] < parent[q])
    {
        parent[p] += parent[q];
        parent[q]=p;
    }
    else
    {
        parent[q] += parent[p];
        parent[p]=q;
    }
}

//
// Kruskal algorithm
//
struct Edge{
int start,end,dis;
bool operator >(const Edge &b)const{return dis > b.dis;}
};
using EdgeQueue=priority_queue<Edge,vector<Edge> ,greater<Edge> >;

const EdgeQueue convertGraph(const vector<vector<Edge> > &g)
{
	EdgeQueue que;
	for(auto&& es:g)
		for(auto&& e:es)
			que.push(e);
	return que;
}

const vector<Edge> Kruskal(const int n, EdgeQueue que)
{
	UnionFind belong(n);
	
	Edge edge;
	vector<Edge> min_cost_tree;
	while(!que.empty())
	{
		edge=que.top();
		if(belong.Find(edge.start)!=belong.Find(edge.end))
		{
			belong.Merge(edge.start, edge.end);
					
			min_cost_tree.push_back(edge);
		}
		que.pop();
	}
	return min_cost_tree;
}

//
//  Bellman-Ford algorithm
//
int inf = INT_MAX / 2;
int edge[V] = {inf};
int V,E;//V is the number of edges. E is that of vertexes.

vector<PII> vertex[V];  
void BF()
{
	bool update=true;
	int to,dist;
	while(update)
	{
		update = false;
		REP(i,V)
		if(edge[i]!=inf)
			for(int j = 0;j < vertex[i].size();j++)
			{
				to = vertex[i][j].first;
				dist = vertex[i][j].second;
				if(edge[to] > edge[i] + dist)
				{
					edge[to] = edge[i] + dist;
					update = true;
				}
			}	
	}
}

bool BF_negative()
{
	bool update=true;
	int to,dist,count;
	while(update)
	{
		update = false;
		REP(i,V)
		if(edge[i]!=inf)
			for(int j = 0;j < vertex[i].size();j++)
			{
				to = vertex[i][j].first;
				dist = vertex[i][j].second;
				if(edge[to] > edge[i] + dist)
				{
					edge[to] = edge[i] + dist;
					update = true;
				}
			}
		
		count ++;
		if(count > V * E)
			return false;
	}
	return true;
}

/* Dijkstra
使い方
Distを好きな辺のコストの型に置換する
Graph型変数vに辺の情報を与える
(v[a].push_back(PID(b,c))でaからbへコストcの有向辺を張る)
dijkstra(i,v):iから任意の点への最小コストのベクター
dijkstra(i,j,v):iからjへの最小コスト
dijkstra(i,j,v,path):iからjへの最小コスト+pathにiからjへの最小パス
dijkstra(i,v,preVec):iから任意の点への最小コストのベクター
get_path(i,j,preVec)でiからjへの最小パスが求まる
*/

typedef int Dist;

struct Edge;
typedef vector<Edge> Edges;
typedef vector<Edges> Graph;
const int INF = 1e9;

struct Edge
{
    int to;
    Dist dist;
    Edge(int to_,Dist dist_):to(to_),dist(dist_)
    {}
    bool operator >(const Edge &ed)const
    {
        return dist > ed.dist;
    }
};

//Shortest cost from i to j.
vector<Dist> dijkstra(int i, const Graph &vertex, vector<int> &preVector)
{
    vector<int> pVector(vertex.size(), INF);
    vector<Dist> shortest(vertex.size(), INF);
    priority_queue<Edge, vector<Edge>, greater<Edge> > que;

    que.push(Edge(i, 0));
    shortest[i]=0;

    Edge state(0,0), tmp(0,0), e(0,0);
    while(!que.empty())
    {
        state = que.top();
        que.pop();
        if(shortest[state.to] < state.dist) continue;
        for(Edge e : vertex[state.to])
        {
            if(shortest[e.to] > shortest[state.to] + e.dist)
            {
                shortest[e.to] = shortest[state.to] + e.dist;
                pVector[e.to] = state.to;
                tmp = Edge(e.to, shortest[e.to]);
                que.push(tmp);
            }
        }
    }
    preVector = pVector;
    return shortest;
}

vector<int> get_path(int i, int j, const vector<int> &preVector)
{
    vector<int> rev;
    rev.push_back(j);
    int p=j;
    while(p!=i)
    {
        p = preVector[p];
        rev.push_back(p);
        if(p==INF)
            return vector<int>();
    }
    reverse(rev.begin(),rev.end());
    return rev;
}

vector<Dist> dijkstra(int i, const Graph &vertex)
{
  vector<int> preVector(vertex.size());
  return dijkstra(i, vertex, preVector);
}
Dist dijkstra(int i,int j, const Graph &vertex)
{
  return dijkstra(i, vertex)[j];
}
Dist dijkstra(int i,int j, const Graph &vertex, vector<int> &path)
{
  vector<int> preVector(vertex.size());
  vector<Dist> shortest(dijkstra(i, vertex, preVector));
  path = get_path(i, j, preVector);
  return shortest[j];
}

///
/// Ford-Fulkerson
///
const int INF=1e9;
struct Edge
{
	int from,to,cost,rev;
};
class FlowNetwork
{
    public:
        FlowNetwork(int n):graph(vector<vector<Edge>>(n)),used(vector<char>(n))
        {}
        void add_Edge(int from, int to, int cap);
        int ford_fulkerson(int from, int to);

    private:
        vector<vector<Edge>> graph;
        vector<char> used;
        int dfs(int from, int to, int cap);
};
void FlowNetwork::add_Edge(int from, int to, int cap)
{
	graph[from].push_back((Edge){from,to,cap,(int)graph[to].size()});
	graph[to].push_back((Edge){to,from,0,(int)graph[from].size()-1});
}
int FlowNetwork::dfs(int from, int to, int f)
{
	if(from == to) return f;
	int d;
	used[from]=true;
	for(int i=0;i<graph[from].size();i++)
	{
		Edge &e=graph[from][i];
		if(used[e.to] || e.cost<=0) continue;
		d = dfs(e.to, to, min(e.cost, f));
		if(d>0)
		{
			e.cost -= d;
			graph[e.to][e.rev].cost += d;
			return d;
		}
	}
	return 0;
}
int FlowNetwork::ford_fulkerson(int from, int to)
{
	int flow=0;
	while(1)
	{
        fill(used.begin(),used.end(),0);
		int f=dfs(from,to,INF);
		if(f==0)break;
		flow += f;
	}
	return flow;
}

///
/// Dinic
///
//set INF
using FLOW = long long;
struct Edge
{
    int to;
    FLOW cap;
    int rev;
};
class FlowNetwork
{
    public:
        FlowNetwork(int n):graph(vector<vector<Edge>>(n)),iter(vector<int>(n)),level(vector<int>(n))
        {}
        void add_Edge(int from, int to, FLOW cap);
        FLOW dinic(int from, int to);

    private:
        vector<vector<Edge>> graph;
        vector<int> iter, level;
        FLOW dfs(int from, int to, FLOW cap);
        void bfs(int from);
};
void FlowNetwork::add_Edge(int from, int to, FLOW cap)
{
    //cout<<from<<":"<<to<<" "<<cap<<endl;
	graph[from].push_back((Edge){to,cap,(int)graph[to].size()});
	graph[to].push_back((Edge){from,0,(int)graph[from].size()-1});
}

void FlowNetwork::bfs(int from)
{
    fill(level.begin(),level.end(),-1);
    queue<int> que;
    level[from]=0;
    que.push(from);
    while(!que.empty())
    {
        int v=que.front();
        que.pop();
        for(int i=0;i<graph[v].size();i++)
        {
            Edge &e=graph[v][i];
            if(e.cap>0 && level[e.to]<0)
            {
                level[e.to] = level[v]+1;
                que.push(e.to);
            }
        }
    }
}

FLOW FlowNetwork::dfs(int from, int to, FLOW f)
{
	if(from == to) return f;
	for(int &i=iter[from];i<graph[from].size();i++)
	{
		Edge &e=graph[from][i];
        if(e.cap > 0 && level[from] < level[e.to])
        {
            FLOW d = dfs(e.to, to, min(e.cap, f));
            if(d>0)
            {
                e.cap -= d;
                graph[e.to][e.rev].cap += d;
                return d;
            }
        }
	}
	return 0;
}
FLOW FlowNetwork::dinic(int from, int to)
{
    FLOW flow=0;
	while(1)
	{
        bfs(from);
        if(level[to]<0) return flow;
        fill(iter.begin(),iter.end(),0);
        FLOW f;
        while((f=dfs(from,to,INF))>0)
        {
            flow+=f;
        }
	}
}

///
/// Primal-Dual
///
//O(FElogV)
using Flow=int;
using Cost=int;
struct Edge{
	int to, rev;
	Flow cap;
	Cost cost;
};

using Edges=vector<Edge>;
using Graph=vector<Edges>;
class MinCostFlow
{
	public:
		vector<vector<Edge>> g;
		MinCostFlow(const int n):g(vector<vector<Edge>>(n))
		{}
		const void add_Edge(const int from, const int to, const Flow cap, const Cost cost)
		{
			g[from].push_back((Edge){to, (int)g[to].size(), cap, cost});
			g[to].push_back((Edge){from, (int)g[from].size()-1, 0, -cost});
		}
		const Cost primal_dual(const int s, const int t, Flow f);
	private:
		const Cost INF=1e9;
};

const Cost MinCostFlow::primal_dual(const int s, const int t, Flow f)
{
	int n=g.size();
	vector<Cost> h(n), dist(n);
	vector<int> prevv(n), preve(n);
	using P=pair<Cost, int>;

	Cost res=0;
	fill(ALL(h), 0);
	while(f>0)
	{
		priority_queue<P, vector<P>, greater<P>> que;
		fill(ALL(dist), INF);
		dist[s]=0;
		que.push(P(0, s));
		while(!que.empty())
		{
			P p=que.top();
			que.pop();
			int v=p.second;
			if(dist[v] < p.first) continue;
			for(int i=0;i<g[v].size();i++)
			{
				Edge &e=g[v][i];
				if(e.cap>0 && dist[e.to] > dist[v] + e.cost + h[v] - h[e.to])
				{
					dist[e.to] = dist[v] + e.cost + h[v] - h[e.to];
					prevv[e.to] = v;
					preve[e.to] = i;
					que.push(P(dist[e.to], e.to));
				}
			}
		}
		
		if(dist[t]==INF)
			return -1;
		for(int v=0;v<n;v++) h[v] += dist[v];

		Flow d=f;
		for(int v=t;v!=s;v=prevv[v])
		{
			d=min(d, g[prevv[v]][preve[v]].cap);
		}
		f-=d;
		res+=d*h[t];
		for(int v=t;v!=s;v=prevv[v])
		{
			Edge &e=g[prevv[v]][preve[v]];
			e.cap-=d;
			g[v][e.rev].cap+=d;
		}
	}
	return res;
}

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

///
/// BIT
///
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

///
/// LazySegmentTree
///
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

///
/// StarrySky
///
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

///
///	Treap
///

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

///
/// Trie
///
class Trie {
	static const size_t SIZE=64;
	static size_t ctoi(const char c)
	{
		return c-64;
	}
	
	public:
		unique_ptr<Trie> next[SIZE];
		Trie(){}
		void insert(const string &s, const size_t i)
		{
			if(i>=s.size())
				return;
			size_t k=ctoi(s[i]);
			if (!next[k])
				next[k] = make_unique<Trie>(Trie());
			next[k]->insert(s, i+1);
		}
		bool find(const string &s, const size_t i) const
		{
			if (i>=s.size())
				return true;
			size_t k=ctoi(s[i]);
			if (!next[k])
				return false;
			return next[k]->find(s, i+1);
		}
};

///
/// PatriciaTrie
///
class PatriciaTrie {
	public:
		bool end=0;
		map<string, unique_ptr<PatriciaTrie>> next;
		PatriciaTrie(bool end__=false):end(end__){}
		void insert(const string &s, const size_t i)
		{
			if(i==s.size())
			{
				end=true;
				return;
			}
			auto l=next.lower_bound(string(1, s[i]));
			auto&& ls=l->first;
			if(l==next.end() || ls[0]!=s[i])
			{
				next[s.substr(i)]=make_unique<PatriciaTrie>(true);
				return;
			}
			for(int j=0; j<ls.size(); j++)
			{
				if(i+j<s.size() && s[i+j]==ls[j])
					continue;

				//ノード分割
				string pref=ls.substr(0,j);
				string ss=s.substr(i+j);
				string lss=ls.substr(j);
				bool ssemp=ss=="";
				next[pref]=make_unique<PatriciaTrie>(ssemp);
				if(!ssemp)
					next[pref]->next[ss]=make_unique<PatriciaTrie>(true);
				next[pref]->next[lss]=move(l->second);
				next.erase(l->first);
				return;
			}
			l->second->insert(s, i+ls.size());
		}
		bool find(const string &s, const size_t i) const
		{
			if(i==s.size())
				return end;
			auto l=next.lower_bound(string(1, s[i]));
			auto&& ls=l->first;
			string ssub=s.substr(i);
			if(l==next.end() || ls[0]!=s[i])
				return false;
			
			for(int j=0;j<ls.size();j++)
				if(i+j>=s.size() || ls[j]!=s[i+j]) return false;
			return l->second->find(s, i+ls.size());
		}
		void debug_print(int ls) const
		{
			string space(ls, ' ');
			bool f=0;
			if(end)
			{
				cout<<"\\";
				f=1;
			}
			for(auto&& e:next)
			{
				if(f)
					cout<<endl<<space;
				cout<<e.first<<" ";
				e.second->debug_print(ls + e.first.size()+1);
				f=1;
			}
		}
};

///
/// Mod
///

class Mod
{
	public:
		using value_type = long long;
	private:
		static const value_type MODULO = 1e9+7;
		value_type value;

		constexpr value_type Normalize(value_type x) const
		{
			return x<0?(x%MODULO+MODULO):(x%MODULO);
		}

	public:
		constexpr Mod():value(0){}
		constexpr Mod(const value_type &val):value(Normalize(val)) {}
		
		explicit operator value_type () const
		{
			return value;
		}

		constexpr const Mod operator -() const
		{
			return Mod(MODULO - value);
		}
		constexpr const Mod operator +(const Mod &rhs) const
		{
			return Mod(value + rhs.value);
		}
		constexpr const Mod operator -(const Mod &rhs) const
		{
			return Mod(value + (-rhs).value);
		}
		constexpr const Mod operator *(const Mod &rhs) const
		{
			return Mod(value * rhs.value);
		}
		Mod &operator +=(const Mod &rhs)
		{
			return *this = *this + rhs;
		}
		Mod &operator -=(const Mod &rhs)
		{
			return *this = *this - rhs;
		}
		Mod &operator *=(const Mod &rhs)
		{
			return *this = *this * rhs;
		}


		Mod pow(value_type p) const;

		Mod inv() const
		{
			return pow(MODULO-2);
		}

		const Mod operator /(const Mod &rhs) const
		{
			return *this * rhs.inv();
		}
		Mod &operator /=(const Mod &rhs)
		{
			return *this = *this / rhs;
		}  
		constexpr bool operator ==(const Mod &rhs)
		{
			return value == rhs.value;
		}
};

Mod Mod::pow(value_type p) const
{
	Mod tmp=1, mult=*this;
	while(p)
	{
		if((p&1)>0) tmp*=mult;
		p>>=1;
		mult*=mult;
	}
	return tmp;
}

namespace std
{
	ostream& operator<<(ostream& os, const Mod mod)
	{
		os<<(typename Mod::value_type)mod;
		return os;
	}
};

class Factorial
{
	private:
		vector<Mod> ary;
	public:
		explicit Factorial(const size_t size):ary(vector<Mod>(size))
		{
			ary[0]=1;
			for(size_t i=1;i<size;i++)
				ary[i]=ary[i-1]*i;
		}

		size_t size() const {   return ary.size();  }

		Mod operator[] (const int id) const
		{
			return ary[id];
		}
};
class FactorialInv
{
	private:
		vector<Mod> ary;
	public:
		explicit FactorialInv(const Factorial &fact):ary(vector<Mod>(fact.size()))
		{
			for(size_t i=0;i<ary.size();i++)
				ary[i]=fact[i].inv();
		}

		//FactorialInv& operator=(FactorialInv&&)=default;

		Mod operator[] (const int id) const
		{
			return ary[id];
		}
};

class Combination
{
	private:
		const Factorial *fact;
		const FactorialInv *fact_inv;
	public:
		Combination(const Factorial &fact_, const FactorialInv &fact_inv_):fact(&fact_),fact_inv(&fact_inv_)
		{}

		Mod operator()(const int n, const int m) const
		{
			return (*fact)[n] * (*fact_inv)[m] * (*fact_inv)[n-m];
		}
};

///
/// Mods
///
using mod_type = long long;
template<mod_type MODULO=1000000007>
class Mod
{
	public:
		using value_type = mod_type;

	private:
		value_type value;
		constexpr value_type Normalize(value_type x) const
		{
			return x<0?(x%MODULO+MODULO):(x%MODULO);
		}

	public:
		constexpr Mod<MODULO>():value(0){}
		constexpr Mod<MODULO>(const value_type &val):value(Normalize(val)) {}
		
		explicit operator value_type () const
		{
			return value;
		}

		constexpr const Mod<MODULO> operator -() const
		{
			return Mod<MODULO>(MODULO - value);
		}
		constexpr const Mod<MODULO> operator +(const Mod<MODULO> &rhs) const
		{
			return Mod<MODULO>(value + rhs.value);
		}
		constexpr const Mod<MODULO> operator -(const Mod<MODULO> &rhs) const
		{
			return Mod<MODULO>(value + (-rhs).value);
		}
		constexpr const Mod<MODULO> operator *(const Mod<MODULO> &rhs) const
		{
			return Mod<MODULO>(value * rhs.value);
		}
		Mod<MODULO> &operator +=(const Mod<MODULO> &rhs)
		{
			return *this = *this + rhs;
		}
		Mod<MODULO> &operator -=(const Mod<MODULO> &rhs)
		{
			return *this = *this - rhs;
		}
		Mod<MODULO> &operator *=(const Mod<MODULO> &rhs)
		{
			return *this = *this * rhs;
		}


		Mod<MODULO> pow(value_type p) const;

		Mod<MODULO> inv() const
		{
			return pow(MODULO-2);
		}

		const Mod<MODULO> operator /(const Mod<MODULO> &rhs) const
		{
			return *this * rhs.inv();
		}
		Mod<MODULO> &operator /=(const Mod<MODULO> &rhs)
		{
			return *this = *this / rhs;
		}

		constexpr const bool operator <(const Mod<MODULO> &rhs) const
		{
			return value < rhs.value;
		}
		constexpr const bool operator ==(const Mod<MODULO> &rhs) const
		{
			return value == rhs.value;
		}
};

template<mod_type MODULO>
Mod<MODULO> Mod<MODULO>::pow(value_type p) const
{
	Mod<MODULO> tmp=1, mult=*this;
	while(p)
	{
		if((p&1)>0) tmp*=mult;
		p>>=1;
		mult*=mult;
	}
	return tmp;
}

namespace std
{
	template<mod_type MODULO>
	ostream& operator<<(ostream& os, const Mod<MODULO> mod)
	{
		os<<(typename Mod<MODULO>::value_type)mod;
		return os;
	}
};


//MODSが空の場合のみマッチ
template<mod_type... MODS>
struct Mods
{
	using value_type = Mod<>::value_type;
	constexpr Mods<MODS...>(){}
	constexpr Mods<MODS...>(value_type m){}

	constexpr const Mods<MODS...> operator -() const { return *this; }
	constexpr const Mods<MODS...> operator +(const Mods<MODS...> &rhs) const 
	{ return *this; }
	constexpr const Mods<MODS...> operator -(const Mods<MODS...> &rhs) const
	{ return *this; }
	constexpr const Mods<MODS...> operator *(const Mods<MODS...> &rhs) const
	{ return *this; }
	Mods<MODS...> &operator +=(const Mods<MODS...> &rhs)
	{ return *this; }
	Mods<MODS...> &operator -=(const Mods<MODS...> &rhs)
	{ return *this; }
	Mods<MODS...> &operator *=(const Mods<MODS...> &rhs)
	{ return *this; }
	Mods<MODS...> pow(value_type p) const
	{ return *this; }
	Mods<MODS...> inv() const
	{ return *this; }
	const Mods<MODS...> operator /(const Mods<MODS...> &rhs) const
	{ return *this; }
	Mods<MODS...> &operator /=(const Mods<MODS...> &rhs)
	{ return *this; }
	constexpr const bool operator ==(const Mods<MODS...> &rhs) const
	{ return true; }
};

template<mod_type MOD, mod_type... Tail>
struct Mods<MOD, Tail...>
{
	using value_type = Mod<>::value_type;
	Mod<MOD> m;
	Mods<Tail...> ms;
	constexpr Mods<MOD, Tail...>():m(0),ms(0){}
	constexpr Mods<MOD, Tail...>(value_type m_):m(m_),ms(m_){}
	constexpr Mods<MOD, Tail...>(Mod<MOD> m_, Mods<Tail...> ms_):m(m_),ms(ms_){}

	constexpr const Mods<MOD, Tail...> operator -() const
	{
		return Mods<MOD, Tail...>(-m, -ms);
	}
	constexpr const Mods<MOD, Tail...> operator +(const Mods<MOD, Tail...> &rhs) const
	{
		return Mods<MOD, Tail...>(m + rhs.m, ms + rhs.ms);
	}
	constexpr const Mods<MOD, Tail...> operator -(const Mods<MOD, Tail...> &rhs) const
	{
		return Mods<MOD, Tail...>(m - rhs.m, ms - rhs.ms);
	}
	constexpr const Mods<MOD, Tail...> operator *(const Mods<MOD, Tail...> &rhs) const
	{
		return Mods<MOD, Tail...>(m * rhs.m, ms * rhs.ms);
	}
	Mods<MOD, Tail...> &operator +=(const Mods<MOD, Tail...> &rhs)
	{
		return *this = *this + rhs;
	}
	Mods<MOD, Tail...> &operator -=(const Mods<MOD, Tail...> &rhs)
	{
		return *this = *this - rhs;
	}
	Mods<MOD, Tail...> &operator *=(const Mods<MOD, Tail...> &rhs)
	{
		return *this = *this * rhs;
	}

	Mods<MOD, Tail...> pow(value_type p) const
	{
		return Mods<MOD, Tail...>(m.pow(p), ms.pow(p));
	}
	Mods<MOD, Tail...> inv() const
	{
		return Mods<MOD, Tail...>(m.inv(), ms.inv());
	}

	const Mods<MOD, Tail...> operator /(const Mods<MOD, Tail...> &rhs) const
	{
		return *this * rhs.inv();
	}
	Mods<MOD, Tail...> &operator /=(const Mods<MOD, Tail...> &rhs)
	{
		return *this = *this / rhs;
	}
	constexpr const bool operator ==(const Mods<MOD, Tail...> &rhs) const
	{
		return m == rhs.m && ms == rhs.ms;
	}
};

///
/// Matrix
///
/*
~Matrix<T>~
行列の簡単な計算ができる c++11
vector iostream cmath cassert 必須
Matrix a(VV<T>), b(VV<T>);
行列式:a*b
スカラ積:l*a
和:a+b
差:a-b
転置:a.transport()
aのi行j列目:a.get(i,j)　
aのi行目j列目にkを代入:set(i,j,k)
もしくはa[i][j] (a[i][j]=k)
aのx乗:a.pow(x) (verified AOJ1327 One-Dimensional Cellular Automaton)
ランク:a.rank() (verified AOJ2564 Tree Reconstruction)
余因子:a.cofactor() 返り値は小数
行列式:a.det() 返り値は小数 (verified AOJ2060 Tetrahedra)
n*n基本行列E:Matrix(n)
m*n 0行列:Matrix(m,n)
m*n 全要素がpの行列:Matrix(m,n,p)
Row<T>型rowでの初期化(m*1行列):Matrix(row)

**(整数不可)**
三角化:a.triangulate()
逆行列:a.inverse()
//(逆行列のa.det()倍ならa.pre_inverse()で求まる 整数可)
ガウスの消去法:a.row_reduction()
//連立一次方程式が解ける
***
*/

template<class T>
using Row = vector<T>;
template<class T>
using VV = vector<Row<T> >;

typedef long double ld;
const ld EPS = 1e-11;

template<class T>
const bool is_zero(const T e) {
  return abs(e) < EPS;
}

template<class T>
struct Matrix {
  VV<T> matrix;
  int m, n;

  Matrix(const VV<T> &matrix_);
  explicit Matrix(int n_);
  explicit Matrix(const Row<T> &row);
  Matrix(int m_, int n_, T e = 0);

  const T get(const int i, const int j) const;
  void set(const int x, const int y, const T k);

  const Matrix<T> transport() const;

  const Matrix<T> operator + (const Matrix<T> &rhs) const;
  const Matrix<T> operator * (const Matrix<T> &rhs) const;
  const Matrix<T> operator - (const Matrix<T> &rhs) const;
  Matrix<T> &operator += (const Matrix<T> &rhs);
  Matrix<T> &operator *= (const Matrix<T> &rhs);
  Matrix<T> &operator -= (const Matrix<T> &rhs);

  Row<T> &operator[](const int x);
  template<class U> operator Matrix<U> () const;

  const Matrix<T> pow(long long x) const;
  const int rank() const;

  //逆行列が存在すれば、(行列式)*(逆行列)を返す
  //A:matrix,return det A * A^-1
  const Matrix<T> pre_inverse() const;

  const ld cofactor(int x, int y) const;
  const ld det() const;
  const Matrix<ld> triangulate() const;
  const Matrix<ld> inverse() const;
  const Matrix<ld> row_reduction() const;
};
template<class T>
const Matrix<T> operator * (const T lambda, const Matrix<T> &rhs) {
  Matrix<T> tmp(rhs);
  for (int i = 0; i < rhs.m; i++)
    for (int j = 0; j < rhs.n; j++)
      tmp.set(i, j, tmp.get(i, j) * lambda);
  return tmp;
}

template<class T>
Matrix<T>::Matrix(const VV<T> &matrix_): matrix(matrix_) {
  m = matrix_.size();
  if (m == 0) n = 0;
  else n = matrix_[0].size();
}

template<class T>
Matrix<T>::Matrix(int n_): m(n_), n(n_) {
  matrix = VV<T>(n, Row<T>(n, 0));
  for (int i = 0; i < n; ++i)
    set(i, i, 1);
}

template<class T>
Matrix<T>::Matrix(const Row<T> &row): m(1), n(row.size()), matrix(VV<T>(1, row)) {
  //sizeがmのvector<T>からmx1行列の生成
  (*this) = transport();
}

template<class T>
Matrix<T>::Matrix(int m_, int n_, T e): m(m_), n(n_) {
  matrix = VV<T>(m, Row<T>(n, e));
}

template<class T>
const T Matrix<T>::get(const int i, const int j) const {
  if (0 <= i && i < m && 0 <= j && j < n)
    return matrix[i][j];

  cerr << "get(" << i << "," << j << ")is not exist." << endl;
  throw;
}

template<class T>
void Matrix<T>::set(const int i, const int j, const T k) {
  if (0 <= i && i < m && 0 <= j && j < n) {
    *(matrix[i].begin() + j) = k;
    return;
  }
  cerr << "set(" << i << "," << j << ")is not exist." << endl;
  throw;
}

template<class T>
const Matrix<T> Matrix<T>::transport() const {
  VV<T> tmp;
  for (int i = 0; i < n; i++) {
    Row<T> row;
    for (int j = 0; j < m; j++)
      row.push_back(get(j, i));
    tmp.push_back(row);
  }
  return tmp;
}

template<class T>
const Matrix<T> Matrix<T>::operator + (const Matrix<T> &rhs) const {
  assert(m == rhs.m && n == rhs.n);

  Matrix<T> tmp(m, n, 0);
  for (int i = 0; i < m; i++) {
    for (int j = 0; j < n; j++) {
      tmp.set(i, j, get(i, j) + rhs.get(i, j));
    }
  }
  return tmp;
}

template<class T>
const Matrix<T> Matrix<T>::operator * (const Matrix<T> &rhs) const {
  assert(n == rhs.m);

  Matrix<T> tmp(m, rhs.n, 0);
  T sum;
  for (int i = 0; i < m; i++)
    for (int j = 0; j < rhs.n; j++) {
      sum = 0;
      for (int k = 0; k < n; k++) {
        sum += get(i, k) * rhs.get(k, j);
      }
      tmp.set(i, j, sum);
    }
  return tmp;
}

template<class T>
const Matrix<T> Matrix<T>::operator - (const Matrix<T> &rhs) const {
  return *this + ((T) - 1 * rhs);
}

template<class T>
Matrix<T> &Matrix<T>::operator += (const Matrix<T> &rhs) {
  return *this = *this + rhs;
}

template<class T>
Matrix<T> &Matrix<T>::operator *= (const Matrix<T> &rhs) {
  return *this = *this * rhs;
}

template<class T>
Matrix<T> &Matrix<T>::operator -= (const Matrix<T> &rhs) {
  return *this = *this - rhs;
}

template<class T>
Row<T> &Matrix<T>::operator[](const int x) {
  return matrix[x];
}

template<class T> template <class U>
Matrix<T>::operator Matrix<U> () const {
  Matrix<U> tmp(m, n);
  for (int i = 0; i < m; i++)
    for (int j = 0; j < n; j++)
      tmp.set(i, j, (U)get(i, j));
  return tmp;
}

template<class T>
const Matrix<T> Matrix<T>::pow(long long x) const {
  Matrix<T> tmp(*this), e(m);
  for (long long i = 1; i <= x; i <<= 1) {
    if ((x & i) > 0)
      e = e * tmp;
    tmp = tmp * tmp;
  }
  return e;
}

template<class T>
const int Matrix<T>::rank() const {
  Matrix<ld> tmp( ((Matrix<ld>)*this).triangulate());
  for (int i = min(tmp.m - 1, tmp.n - 1); i >= 0; i--) {
    for (int j = tmp.n - 1; j >= i; j--)
      if (is_zero(tmp.get(i, j)))
        continue;
      else
        return i + 1;
  }
  return 0;
}

template<class T>
const Matrix<T> Matrix<T>::pre_inverse() const {
  assert(m == n);

  Matrix<T> tmp(m, n, 0);
  for (int i = 0; i < m; i++)
    for (int j = 0; j < n; j++)
      tmp.set(i, j, ((i + j) % 2 == 0 ? 1 : -1)*cofactor(i, j));
  return tmp.transport();
}

template<class T>
const ld Matrix<T>::cofactor(int x, int y) const {
  VV<T> tmp;
  for (int i = 0; i < m; i++) {
    if (x == i) continue;
    Row<T> row;
    for (int j = 0; j < n; j++) {
      if (y == j) continue;
      row.push_back(get(i, j));
    }
    tmp.push_back(row);
  }
  return Matrix<T>(tmp).det();
}

/*余因子展開
template<class T>
const T Matrix<T>::det() const {
  assert(n == m);

  if (m == 1)
    return get(0, 0);
  T sum = 0;
  for (int i = 0; i < m; i++) {
    sum += ((i % 2 == 0 ? 1 : -1) * get(i, 0)) * cofactor(i, 0);
  }
  return sum;
}*/

template<class T>
const ld Matrix<T>::det() const {
  assert(n==m);

  Matrix<ld> tmp(triangulate());
  ld sum=1;
  REP(i,m)
    sum*=tmp.get(i,i);
  return sum;
}

template<> const Matrix<ld> Matrix<ld>::triangulate() const {
  Matrix<ld> tmp(*this);
  ld e;
  int p = 0;
  for (int i = 0; i < m && p < n; i++, p++) {
    if (is_zero(tmp.get(i, p))) {
      tmp.set(i, p, 0);
      bool flag = true;
      for (int j = i + 1; j < m; j++)
        if (!is_zero(tmp.get(j, p))) {
          for (int k = 0; k < n; k++)
            tmp.set(i, k, tmp.get(i, k) + tmp.get(j, k));
          //tmp[i].swap(tmp[j]);
          flag = false;
          break;
        }
      if (flag) {
        i--;
        continue;
      }
    }
    for (int j = i + 1; j < m; j++) {
      e = tmp.get(j, p) / tmp.get(i, p);
      for (int k = 0; k < n; k++)
        tmp.set(j, k, tmp.get(j, k) - tmp.get(i, k) * e);
    }
  }
  return tmp;
}

template<> const Matrix<ld> Matrix<ld>::row_reduction() const {
  Matrix<ld> tmp(*this);
  ld e;
  int p = 0;
  for (int i = 0; i < m && p < n; i++, p++) {
    if (is_zero(tmp.get(i, p))) {
      tmp.set(i, p, 0);
      bool flag = true;
      for (int j = i + 1; j < m; j++)
        if (!is_zero(tmp.get(j, p))) {
          for (int k = 0; k < n; k++)
            tmp.set(i, k, tmp.get(i, k) + tmp.get(j, k));
          //tmp[i].swap(tmp[j]);
          flag = false;
          break;
        }
      if (flag) {
        i--;
        continue;
      }
    }
    e = 1 / tmp.get(i, p);
    tmp.set(i, p, 1);
    for (int k = i + 1; k < n; k++)
      tmp.set(i, k, tmp.get(i, k)*e);
    for (int j = 0; j < m; j++) {
      if (i == j) continue;
      e = tmp.get(j, p);
      for (int k = 0; k < n; k++)
        tmp.set(j, k, tmp.get(j, k) - tmp.get(i, k) * e);
    }
  }
  return tmp;
}

/*O(n!)実装
template<> const Matrix<ld> Matrix<ld>::inverse() const {
  Matrix<ld> tmp(pre_inverse());
  ld e = det();

  assert(!is_zero(e));

  tmp = 1 / e * tmp;
  return tmp.transport();
}*/

template<> const Matrix<ld> Matrix<ld>::inverse() const {
  assert(m == n);

  Matrix<ld> tmp(m, n * 2), tmp2(m, n);
  for (int i = 0; i < m; i++)
    for (int j = 0; j < n; j++)
      tmp.set(i, j, get(i, j));
  for (int i = 0; i < m; i++)
    tmp.set(i, i + n, 1);

  tmp = tmp.row_reduction();

  //逆行列が存在するかどうかのチェック
  for (int i = 0; i < m; i++)
    assert(is_zero(tmp.get(i, i) - 1));

  for (int i = 0; i < m; i++)
    for (int j = 0; j < n; j++)
      tmp2.set(i, j, tmp.get(i, j + n));

  return tmp2;
}

///
/// FFT
///
/*
verified AtCoder FFT
*/
const bool is_pow2(const int p)
{
	int t=1;
	while(t<p)
		t<<=1;
	return t==p;
}

const double PI=atan2(0,-1);
using Complex=complex<double>;
class FT
{
    private:
        //FFTの順方向、逆方向をまとめた関数
		template<class T> 
		static const vector<Complex> FFT_with_flag(const vector<T>& ary, const bool inverse=false);
        
        //1要素のvector<Complex>を作成する
        //目的はTとcomplex<T>のマッチング
		template<class T> 
		static const vector<Complex> create_complex(const complex<T> x)
        {
            return vector<Complex>(1, Complex(x));
        } 
		template<class T> 
		static const vector<Complex> create_complex(const T x)
        {
            return vector<Complex>(1, Complex(x, 0));
        } 
    public:
        /*
            FFTの引数は2の冪で、配列の長さの半分まで正しく変換できることが保証される
            余った要素は0で埋める
        */
		template<class T> 
		static const vector<Complex> FFT(const vector<T>& ary)
        {
            return FFT_with_flag(ary,false);
        }
		template<class T> 
		static const vector<Complex> IFFT(const vector<T>& ary)
        {
            auto iary=FFT_with_flag(ary, true);
            for(int i=0;i<(int)iary.size();i++)
                iary[i]/=iary.size();
            return iary;
        }
    
};

template<class T> 
const vector<Complex> FT::FFT_with_flag(const vector<T>& ary, const bool inverse)
{
    int n=ary.size();
    if(n==1)
        return create_complex(ary[0]);

    //2の冪でなければいけない
    assert(is_pow2(n));

    //偶数成分のフーリエ変換
    vector<T> ev(n/2);
    for(int i=0;i<n/2;i++)
        ev[i]=ary[i*2];
    auto even=FFT_with_flag(ev, inverse);

    //奇数成分のフーリエ変換
    vector<T> od(n/2);
    for(int i=0;i<n/2;i++)
        od[i]=ary[i*2+1];
    auto odd = FFT_with_flag(od, inverse);

    //以下合性
    if(inverse)
        for(int i=0;i<n/2;i++)
            odd[i]*=Complex(cos(2*PI*i/n), sin(2*PI*i/n));
    else
        for(int i=0;i<n/2;i++)
            odd[i]*=Complex(cos(2*PI*i/n), -sin(2*PI*i/n));
    
    vector<Complex> ret(n);
    for(int i = 0; i < n/2; i++)
        ret[i] = even[i] + odd[i];
    for(int i = 0; i < n/2; i++)
        ret[i+n/2] = even[i] - odd[i];
    return ret;
}

const int least_pow2(const int p)
{
	int t=1;
	while(t<p)
		t<<=1;
	return t;
} 

//畳み込み計算
template<class T>
vector<Complex> convolution(vector<T> a, vector<T> b) {
    vector<Complex> p(least_pow2(a.size())*2), q(least_pow2(b.size())*2);
    copy(a.begin(), a.end(), p.begin());
    copy(b.begin(), b.end(), q.begin());

	auto pf = FT::FFT(p), qf = FT::FFT(q);
	for (int i = 0;i < pf.size();i++) pf[i] *= qf[i];
    return FT::IFFT(pf);
}

///
/// Garner
///
namespace Garner
{
    /* Garnerのアルゴリズム
    x = mods[i].first mod mods[i].second を満たすxを求める
    https://qiita.com/drken/items/ae02240cd1f8edfc86fd
    */
    using LL = long long;
	LL gcd(const LL a, const LL b)
	{
		return __gcd(a, b);
	}
    LL mod(const LL p, const LL m)
    {
        return (p<0)?(p%m+m):(p%m);
    }
    LL extgcd(const LL a, const LL b, LL &x, LL &y)
    {
        if(b==0)
        {
            x=1;
            y=0;
            return a;
        }
        LL d=extgcd(b, a%b, y, x);
        y-=a/b*x;
        return d;
    }
    LL modinv(const LL a, const LL m)
    {
        //a*x + m*y = gcd(a,m) = 1
        LL x,y;
        extgcd(a, m, x, y);
        return mod(x, m);
    }

	/*
	Garnerの前処理
	modが互いに素になるようにするor解が無いときは-1を返す
	*/
	vector<pair<LL,LL>> normalize(const vector<pair<LL, LL>> &mods)
	{
		vector<pair<LL, LL>> ret(mods);
		for(size_t i=0;i<ret.size();i++)
		for(size_t j=0;j<i;j++)
		{
			LL g=gcd(ret[i].second, ret[j].second);
			if((ret[i].first - ret[j].first)%g != 0) return vector<pair<LL, LL>>(0);
			ret[i].second/=g;
			ret[j].second/=g;
			LL gi = gcd(ret[i].second, g);
			LL gj = g/gi;
			g = gcd(gi, gj);
			while(g != 1)
			{
				gi*=g;
				gj/=g;
				g = gcd(gi, gj);
			}
			ret[i].second*=gi;
			ret[j].second*=gj;
			ret[i].first%=ret[i].second;
			ret[j].first%=ret[j].second;
		}
		return ret;
	}

    LL Garner(const vector<pair<LL, LL>> &mods, const LL m)
    {
        const int n=mods.size();
        vector<LL> modprod(n+1,1);
        vector<LL> p(n+1);
        for(int i=0;i<n;i++)
        {
            LL t=mod((mods[i].first - p[i]) * modinv(modprod[i], mods[i].second), mods[i].second);

            for(int j=i+1;j<n;j++)
            {
                p[j]+=t*modprod[j];
                p[j]%=mods[j].second;
                modprod[j]*=mods[i].second;
                modprod[j]%=mods[j].second;
            }
            p[n]+=t*modprod[n];
            p[n]%=m;
            modprod[n]*=mods[i].second;
            modprod[n]%=m;
        }
        return p[n];
    }

	LL solve(const vector<pair<LL, LL>> &mods, const LL m)
	{
		const vector<pair<LL,LL>> tmp(normalize(mods));
		if(tmp.size() == 0) return -1;
		return Garner(tmp, m);
	}
}

///
/// LCA-Doubling
///

const int N=100001;
vector<int> graph[N], parent[N], path;
int depth[N];
bool reach[N];
void dfs(int p, int dep)
{
	reach[p]=true;
	depth[p]=dep;
	path.push_back(p);

	for(int i=1;i<=path.size()-1;i<<=1)
		parent[p].push_back(path[path.size()-1-i]);
	REP(i,graph[p].size())
	{
		if(reach[graph[p][i]]) continue;
		dfs(graph[p][i], dep+1);
	}

	path.pop_back();
}

int lca(int p, int q)
{
	int dp=depth[p], dq=depth[q];
	int t, s, diff=abs(dp-dq);
	if(dp>dq)
	{
		t=p;
		s=q;
	}
	else
	{
		t=q;
		s=p;
	}
	for(int i=1,j=0;i<=diff;i<<=1,j++)
		if((i&diff)>0)
			t=parent[t][j];

	if(t==s) return t;

	for(int i=parent[t].size()-1;i>=0;i--)
		if(parent[t][i]!=parent[s][i])
		{
			t=parent[t][i];
			s=parent[s][i];
			i=min(i, (int)parent[t].size());
		}

	return parent[t][0];
}
//辺は双方向に張る

///
/// LCA-RMQ
///
const int N=100000;
vector<int> graph[N], rmq;
int id[N], depth[N];
bool reach[N];

//dfs before lca.
void dfs(int p, int dep)
{
	reach[p]=true;
	depth[p]=dep;
	id[p]=rmq.size();
	rmq.push_back(p);
	REP(i,graph[p].size())
	{
		if(reach[graph[p][i]]) continue;
		dfs(graph[p][i], dep+1);
		rmq.push_back(p);
	}
}
int lca(int p,int q)
{
	int i,j;
	if(id[p]<id[q])
	{
		i=id[p];
		j=id[q];
	}
	else
	{
		i=id[q];
		j=id[p];
	}
	
	int midep=N, lowest;
	for(int k=i;k<=j;k++)
		if(depth[rmq[k]] < midep)
		{
			midep = depth[rmq[k]];
			lowest = rmq[k];
		}

	return lowest;
}

int main() {
	dfs(1, 0);
}

///
/// LowLink
///
//UnionFind
struct Edge
{
    int to,from,cost;
    Edge(int to_,int from_,int weight_):to(to_),from(from_),cost(weight_)
    {}

};
typedef vector<Edge> Edges;
typedef vector<Edges> Graph;

const tuple<vector<int>,vector<int>> lowlink(const Graph &g)
{
	int n=g.size();
	vector<int> order(n), low(n);
	vector<char> reach(n);
	int cnt=0;
	function<int(const int,const int)> dfs=[&](const int v,const int pv)
	{
		if(reach[v]) return order[v];
		reach[v]=true;
		order[v]=cnt;
		int mlow=cnt;
		cnt++;
		for(auto&& e:g[v])
		{
			if(e.to==pv) continue;
			mlow=min(mlow, dfs(e.to, v));
		}
		return low[v]=mlow;
	};
	dfs(0, -1);
	return make_tuple(order, low);
	//order[e.from]<low[e.to]のとき、eは橋
}

//縮約後の頂点番号,頂点数
const tuple<vector<int>, int> biconnectedComponent(const Graph &g)
{
	int n=g.size();
	vector<int> ord, low;
	tie(ord, low)=lowlink(g);
	UnionFind uf(n);
	for(int i=0;i<n;i++)
		for(auto&& e:g[i])
			if(!(ord[e.from]<low[e.to] || ord[e.to]<low[e.from]))
				uf.Merge(e.from, e.to);
	vector<int> bc_map(n,-1);
	int k=0;
	REP(i,n)
	{
		int j=uf.Find(i);
		if(bc_map[j]==-1)
			bc_map[j]=k++;
		bc_map[i]=bc_map[j];
	}
	return make_tuple(bc_map, k);
}

const Graph tree_from_BC(int tree_size, const Graph &g, const vector<int> &bc_map)
{
    Graph ret(tree_size);
	for(int i=0;i<g.size();i++)
		for(auto&& e:g[i])
			if(bc_map[e.from]!=bc_map[e.to])
				ret[bc_map[e.from]].push_back(Edge(bc_map[e.to], bc_map[e.from], e.cost));
    return ret;
}

///
/// Slide最小値
///
/*
蟻本p300
*/
template<typename T, bool Cmp(T,T)>
class Slide
{
	private:
		vector<T> minT;
		int k;
	public:
		Slide(const vector<T> &ary, const int k_):k(k_)
		{
			minT=vector<T>(ary.size()+k-1);
			deque<size_t> deq;
			for(size_t i=0;i<ary.size()+k-1;i++)
			{
				if(i < ary.size())
				{
					//while(!deq.empty() && ary[deq.back()] >= ary[i])
					while(!deq.empty() && !Cmp(ary[deq.back()], ary[i]))
						deq.pop_back();
					deq.push_back(i);
				}

				minT[i] = ary[deq.front()];
				if(i-k+1 < 0) continue;
				if(deq.front()==i-k+1)
					deq.pop_front();
			}
		}
		
		//-(k-1) <= index && index < (int)minT.size()-(k-1)
		T operator[] (int index) const
		{
			index+=k-1;
			assert(0 <= index && index < (int)minT.size());
			return minT[index];
		}
};

template<typename T>
bool lt(const T lhs, const T rhs) { return lhs < rhs; }

template <typename T>
using SlideMin = Slide<T, lt>;

///
/// StronglyConnectedComponent
///
struct Edge
{
    int to,from,cost;
    Edge(int to_,int from_,int weight_):to(to_),from(from_),cost(weight_)
    {}

};
typedef vector<Edge> Edges;
typedef vector<Edges> Graph;

const Graph reverse_edge_Graph(const Graph& g)
{
    Graph rg(g.size());
    for(auto&& es : g)
        for(auto&& e : es)
            rg[e.to].push_back(Edge(e.from,e.to,e.cost));
    return rg;
}

const tuple<vector<int>,vector< vector<int>>> SCC(const Graph& g) //StronglyConnectedComponent
{
    const int n = g.size();
    Graph rg(reverse_edge_Graph(g));
    vector<int> vs;
    vector<char> reach(n);  //vector<bool>
    vector<vector<int> > scc;
    vector<int> scc_vectormap(n);

    function<void(const int)> dfs = [&](const int v)
    {
        reach[v]=1;
        for(auto&& e : g[v])
            if(!reach[e.to]) dfs(e.to);
        vs.push_back(v);
    };
    function<void(const int, const int)> rdfs = [&](const int v, const int k)
    {
        reach[v]=1;
        scc[k].push_back(v);
        scc_vectormap[v]=k;
        for(auto&& e : rg[v])
            if(!reach[e.to]) rdfs(e.to, k);
    };

    for(int i=0;i<n;i++)
        if(!reach[i]) dfs(i);
    fill(reach.begin(), reach.end(), 0);
    int k=0;
    for(int i=vs.size()-1;i>=0;i--)
        if(!reach[vs[i]]) 
        {
            scc.push_back(vector<int>());
            rdfs(vs[i], k++);
        }
    return tuple<vector<int>, vector<vector<int>> >(scc_vectormap, scc);
}

const Graph DAG_from_SCC(int DAG_size,Graph& g, vector<int>& scc_vectormap)
{
    Graph dag(DAG_size);
    for(auto&& es : g)
        for(auto&& e : es)
        {
            if(scc_vectormap[e.from]==scc_vectormap[e.to]) continue;
            dag[scc_vectormap[e.from]].push_back(Edge(scc_vectormap[e.to], scc_vectormap[e.from], 0));
        }
    return dag;
}

///
/// Suffix Array
///
///
///lower_bound(t)...tを含む最小のSAの添字
///upper_bound(t)...tを含む最大のSAの添字+1
///lcp() ... 高さ配列
///
class SA
{
    private:
        int k;
        
    public:
        const string s;
        vector<int> sa;
        const size_t length;
        
        SA(const string &_s);
        
        const int size() const { return (int)length; }
        const bool contain(const string &t) const;
        const int lower_bound(const string &t) const;
        const int upper_bound(const string &t) const;
        const vector<int> lcp() const;
};

SA::SA(const string &_s):
s(_s), sa(_s.size()+1), length(_s.size())
{
    iota(sa.begin(), sa.end(),0);
    
    vector<int> rank(size()+1);
    for(int i=0;i<size();i++)
        rank[i]=s[i];
    rank[size()]=-1;
    
    auto sa_comparator=[&](int i, int j)
    {
        if(rank[i]!=rank[j]) return rank[i]<rank[j];
        int ri=i+k<=size()?rank[i+k]:-1;
        int rj=j+k<=size()?rank[j+k]:-1;
        return ri<rj;
    };
    
    for(k=1;k<=size();k*=2)
    {
        sort(sa.begin(), sa.end(), sa_comparator);
        
        vector<int> tmp(size()+1);
        tmp[sa[0]]=0;
        for(int i=1;i<=size();i++)
        {
            tmp[sa[i]]=tmp[sa[i-1]]+(sa_comparator(sa[i-1],sa[i])?1:0);
        }
        rank=move(tmp);
    }
}
        
const bool SA::contain(const string &t) const
{
    int l=0,r=size()+1;
    while(r-l>1)
    {
        int mid=(l+r)/2;
        if(s.compare(sa[mid], t.size(), t)<0) l=mid;
        else r=mid;
    }
    return s.compare(sa[r], t.size(), t)==0;
}


const int SA::lower_bound(const string &t) const
{
    int l=0,r=size()+1;
    while(r-l>1)
    {
        int mid=(l+r)/2;
        if(s.compare(sa[mid], t.size(), t)<0) l=mid;
        else r=mid;
    }
    return r;
}

const int SA::upper_bound(const string &t) const
{
    int l=0,r=size()+1;
    while(r-l>1)
    {
        int mid=(l+r)/2;
        if(s.compare(sa[mid], t.size(), t)<=0) l=mid;
        else r=mid;
    }
    return r;
}

const vector<int> SA::lcp() const
{
    vector<int> rank(size()+1), lcp(size()+1);
    for(int i=0;i<=size();i++)
        rank[sa[i]]=i;
    
    int h=0;
    lcp[0]=0;
    for(int i=0;i<size();i++)
    {
        int j=sa[rank[i]-1];
        
        if(h>0) h--;
        for(; j+h<size()&&i+h<size(); h++)
            if(s[j+h] != s[i+h]) break;
            
        lcp[rank[i]-1]=h;
    }
    
    return lcp;
}
///
/// gcd
///

using LL=long long;
//蟻本版
LL extgcd(const LL a, const LL b, LL &x, LL &y)
{
    if(b==0)
    {
        x=1;
        y=0;
        return a;
    }
    LL d=extgcd(b, a%b, y, x);
    y-=a/b*x;
    return d;
}
//gcd(a,b)要らなくない？
pair<LL,LL> extgcd(const LL a, const LL b)
{
    if(b==0)
        return pair<LL,LL>(1, 0);
    LL x,y;
    tie(y,x)=extgcd(b, a%b);
    y-=a/b*x;
    return pair<LL,LL>(x, y);
}

//ループに展開してやるぜ
pair<LL, LL> extgcd(const LL a, const LL b)
{
    LL c0=a;
    LL c1=b;
    LL x0=1;
    LL x1=0;
    LL y0=0;
    LL y1=1;
    while (c1>0)
    {
        LL s=c0/c1;
        LL c2=c0%c1;
        LL x2=x0-s*x1;
        LL y2=y0-s*y1;
        c0=c1;
        c1=c2;
        x0=x1;
        x1=x2;
        y0=y1;
        y1=y2;
    }
    return pair<LL, LL>(x0, y0);
}
///
/// MaximumIndependentSet
///
int main()
{
	int n,m;
	cin>>n>>m;
	vector<VB> cmp(n,VB(n));
	REP(i,m)
	{
		int a,b;
		cin>>a>>b;
		a--;b--;
		cmp[a][b]=1;
		cmp[b][a]=1;
	}

	int m2=n/2;
	int m1=n-m2;
	int INFB=127;
	VB dp(1<<m1,INFB);
	function<bool(int)> dfs=[&](int p)
	{
		if(dp[p]!=INFB) return dp[p];
		REP(i,m1)
		{
			if(((1<<i)&p)>0)
			{
				bool f=dfs(p-(1<<i));
				if(dp[p]!=INFB) continue;
				REP(j, m1)
				{
					if(((1<<j)&p)==0) continue;
					if(i==j) continue;
					f&=!cmp[i][j];
				}
				dp[p]=f;
			}
		}
		return dp[p];
	};
	dfs((1<<m1)-1);


	VB dp2(1<<m2,INFB);
	function<bool(int)> dfs2=[&](int p)
	{
		if(dp2[p]!=INFB) return dp2[p];
		REP(i,m2)
		{
			if(((1<<i)&p)>0)
			{
				bool f=dfs2(p-(1<<i));
				if(dp2[p]!=INFB) continue;
				REP(j, m2)
				{
					if(((1<<j)&p)==0) continue;
					if(i==j) continue;
					f&=!cmp[i+m1][j+m1];
				}
				dp2[p]=f;
			}
		}
		return dp2[p];
	};
	dfs2((1<<m2)-1);


	VI dp3(1<<m2,-1);
	function<int(int)> dfs3=[&](int p)
	{
		if(dp3[p]!=-1) return dp3[p];
		int ma=0;
		if(dp2[p])
		{
			REP(i,m2)
			{
				if(((1<<i)&p)>0)
				{
					dfs3(p-(1<<i));
					ma++;
				}
			}
			return dp3[p]=ma;
		}
		REP(i,m2)
		{
			if(((1<<i)&p)>0)
			{
				int f=dfs3(p-(1<<i));
				chmax(ma, f);
			}
		}
		return dp3[p]=ma;
	};
	dfs3((1<<m2)-1);


	VI dp4((1<<m1), -1);
	function<int(int)> dfs4=[&](int p)
	{
		if(dp4[p]!=-1) return dp4[p];
		if(p==0) return dp4[p]=(1<<m2)-1;
		int ma=0;
		bool f=1;
		REP(i,m1)
		{
			if(((1<<i)&p)>0)
			{
				int v=dfs4(p-(1<<i));
				if(f)
				{
					REP(j,m2)
					{
						v&=cmp[i][j+m1]?~(1<<j):~0;
					}
					dp4[p]=v;
					ma=v;
					f=0;
				}
			}
		}
		return ma;
	};
	dfs4((1<<m1)-1);

	int ans=0;
	REP(i,1<<m1)
	{
		if(!dp[i]) continue;
		int m2s=dp3[dp4[i]];
		REP(j,m1)
		{
			if(((1<<j)&i)>0) 
				m2s++;
		}
		chmax(ans, m2s);
	}
	print(ans);
}

///
/// int128
///

class Bint
{
	public:
		using value_type = __int128;
	private:
		value_type value;
	public:
		constexpr Bint():value(0){}
		constexpr Bint(const value_type &val):value(val) {}
		constexpr Bint(const long long &val):value(val) {}
		constexpr Bint(const int &val):value(val) {}
		Bint(const string &s)
		{
			value = 0;
			for (size_t i = 0; i < s.length(); i++)
				if ('0' <= s[i] && s[i] <= '9')
					value = value * 10 + s[i] - '0';
		}
		
		template<typename T>
		explicit operator T () const
		{
			return (T)value;
		}

		constexpr const Bint operator -() const
		{
			return Bint(-value);
		}
		constexpr const Bint operator +(const Bint &rhs) const
		{
			return Bint(value + rhs.value);
		}
		constexpr const Bint operator -(const Bint &rhs) const
		{
			return Bint(value + (-rhs).value);
		}
		constexpr const Bint operator *(const Bint &rhs) const
		{
			return Bint(value * rhs.value);
		}
		constexpr const Bint operator /(const Bint &rhs) const
		{
			return Bint(value / rhs.value);
		}
		constexpr const Bint operator %(const Bint &rhs) const
		{
			return Bint(value % rhs.value);
		}
		Bint &operator +=(const Bint &rhs)
		{
			return *this = *this + rhs;
		}
		Bint &operator -=(const Bint &rhs)
		{
			return *this = *this - rhs;
		}
		Bint &operator *=(const Bint &rhs)
		{
			return *this = *this * rhs;
		}
		Bint &operator /=(const Bint &rhs)
		{
			return *this = *this / rhs;
		}
		Bint &operator %=(const Bint &rhs)
		{
			return *this = *this % rhs;
		}
		
		Bint &operator ++()
		{
			return *this += 1;
		}
		Bint operator ++(int)
		{
			auto tmp = *this;
			*this += 1;
			return tmp;
		}
		Bint &operator --()
		{
			return *this -= 1;
		}
		Bint operator --(int)
		{
			auto tmp = *this;
			*this -= 1;
			return tmp;
		}

		Bint pow(long long p) const
		{
			Bint tmp=1, mult=*this;
			while(p>0)
			{
				if((p&1)>0) tmp*=mult;
				p>>=1;
				mult*=mult;
			}
			return tmp;
		}

		constexpr bool operator ==(const Bint &rhs) const
		{
			return value == rhs.value;
		}
		constexpr bool operator !=(const Bint &rhs) const
		{
			return value != rhs.value;
		}
		constexpr bool operator >(const Bint &rhs) const
		{
			return value > rhs.value;
		}
		constexpr bool operator <(const Bint &rhs) const
		{
			return value < rhs.value;
		}
		constexpr bool operator >=(const Bint &rhs) const
		{
			return value >= rhs.value;
		}
		constexpr bool operator <=(const Bint &rhs) const
		{
			return value <= rhs.value;
		}

		string to_s() const
		{
			if(value==0)
			{
				return "0";
			}
			bool minus=value<0;
			char str[128];
			int ind=0;
			Bint tmp(*this);
			while(tmp!=0)
			{
				str[ind++] = abs((int)(tmp%10)) + '0';
				tmp/=10;
			}
			if(minus)
				str[ind++]='-';
			str[ind]='\0';
			for(int i=0;i<ind/2;i++)
				swap(str[i], str[ind-1-i]);
			return string(str);
		}
};

ostream& operator<<(ostream &os, Bint &value)
{
	os<<value.to_s();
	return os;
}

///
/// Next Combination
///
class NextCombination
{
	private:
		const int n,r;
		VB used;
		vector<int> point;
		int back=0;
		int c=0;
	public:
		NextCombination(const int n_, const int r_)
			:n(n_),r(r_),used(VB(n)),point(vector<int>(r)),back(0),c(1)
		{}
		bool end() const
		{
			return c<0;
		}
		vector<int> next()
		{
			c--;
			for(int j=n-1;j>=0;j--)
			{
				if(!used[j]) break;
				used[j]=false;
				c--;
				if(c<0) break;
				back=point[c]+1;
			}
			if(c<0) 
				return vector<int>(0);
			used[point[c]]=false;

			while(c<r)
			{
				point[c]=back;
				used[back++]=1;
				c++;
			}
			return point;
		}
};
class NextCombinationInt
{
	using LL=long long;
	private:
		const int n,r;
		LL used;
		vector<int> point;
		int back=0;
		int c=0;
	public:
		NextCombinationInt(const int n_, const int r_)
			:n(n_),r(r_),used(0),point(vector<int>(r)),back(0),c(1)
		{}
		bool end() const
		{
			return c<0;
		}
		LL next()
		{
			c--;
			for(int j=n-1;j>=0;j--)
			{
				if((used&(1<<j))==0) break;
				used&=~(((LL)1)<<j);
				c--;
				if(c<0) break;
				back=point[c]+1;
			}
			if(c<0) 
				return 0;
			used&=~(((LL)1)<<point[c]);

			while(c<r)
			{
				point[c]=back;
				used|=(1<<(back++));
				c++;
			}
			return used;
		}
};

///
/// asi1024_is_god
///
#include <bits/stdc++.h>

#define REP(i,n) for(int i=0;i<(int)(n);i++)
#define ALL(x) (x).begin(),(x).end()

using namespace std;

///
/// ASI1024 IS GOD
///

typedef long double ld;
typedef complex<ld> Point;
const ld eps = 1e-9, pi = acos(-1.0);

bool eq(ld a, ld b) {
  return (abs(a - b) < eps);
}

namespace std {
  bool operator<(const Point &lhs, const Point &rhs) {
    if (lhs.real() < rhs.real() - eps) return true;
    if (lhs.real() > rhs.real() + eps) return false;
    return lhs.imag() < rhs.imag();
  }
}

Point input_point() {
  ld x, y;
  cin >> x >> y;
  return Point(x, y);
}

ld dot(Point a, Point b) {
  return real(conj(a) * b);
}

ld cross(Point a, Point b) {
  return imag(conj(a) * b);
}

class Line {
public:
  Point a, b;
  Line () : a(Point(0, 0)), b(Point(0, 0)) {}
  Line (Point a, Point b) : a(a), b(b) {}
};

class Circle {
public:
  Point p;
  ld r;
  Circle () : p(Point(0, 0)), r(0) {}
  Circle (Point p, ld r) : p(p), r(r) {}
};

int ccw (Point a, Point b, Point c) {
  b -= a; c -= a;
  if (cross(b, c) > eps) return 1; //a,b,cが反時計回りの順に並ぶ
  if (cross(b, c) < -eps) return -1; //a,b,cが時計回りの順に並ぶ
  if (dot(b, c) < 0) return 2; //c,a,bの順に直線に並ぶ
  if (norm(b) < norm(c)) return -2; //a,b,cの順に直線に並ぶ
  return 0; //a,c,bの順に直線に並ぶ
}

//2直線の交差判定
bool isis_ll (Line l, Line m) {
  return !eq(cross(l.b - l.a, m.b - m.a), 0);
}

//直線と線分の交差判定
bool isis_ls (Line l, Line s) {
  return isis_ll(l, s) &&
    (cross(l.b - l.a, s.a - l.a) * cross(l.b - l.a, s.b - l.a) < eps);
}

bool isis_ss(Line s, Line t) {
  return ccw(s.a, s.b, t.a) * ccw(s.a, s.b, t.b) <= 0 &&
    ccw(t.a, t.b, s.a) * ccw(t.a, t.b, s.b) <= 0;
}

//点が直線状に存在するかの判定
bool isis_lp (Line l, Point p) {
  return (abs(cross(l.b - p, l.a - p)) < eps);
}

//点が線分上に存在するかの判定
bool isis_sp (Line s, Point p) {
  return (abs(s.a - p) + abs(s.b - p) - abs(s.b - s.a) < eps);
}

//点から直線に下ろす垂線の足
Point proj (Line l, Point p) {
  ld t = dot(p - l.a, l.a - l.b) / norm(l.a - l.b);
  return l.a + t * (l.a - l.b);
}

//直線と直線の交点
Point is_ll (Line s, Line t) {
  Point sv = s.b - s.a, tv = t.b - t.a;
  assert(cross(sv, tv) != 0);
  return s.a + sv * cross(tv, t.a - s.a) / cross(tv, sv);
}

ld dist_lp (Line l, Point p) {
  return abs(p - proj(l, p));
}

ld dist_ll (Line l, Line m) {
  return isis_ll(l, m) ? 0 : dist_lp(l, m.a);
}

ld dist_ls (Line l, Line s) {
  return isis_ls(l, s) ? 0 : min(dist_lp(l, s.a), dist_lp(l, s.b));
}

ld dist_sp (Line s, Point p) {
  Point r = proj(s, p);
  return isis_sp(s, r) ? abs(r - p) : min(abs(s.a - p), abs(s.b - p));
}

ld dist_ss (Line s, Line t) {
  if (isis_ss(s, t)) return 0;
  return min({dist_sp(s, t.a), dist_sp(s, t.b), dist_sp(t, s.a), dist_sp(t, s.b)});
}

//円と円の交点
vector<Point> is_cc (Circle c1, Circle c2){
  vector<Point> res;
  ld d = abs(c1.p - c2.p);
  ld rc = (d * d + c1.r * c1.r - c2.r * c2.r) / (2 * d);
  ld dfr = c1.r * c1.r - rc * rc;
  if (abs(dfr) < eps) dfr = 0.0;
  else if (dfr < 0.0) return res;
  ld rs = sqrt(dfr);
  Point diff = (c2.p - c1.p) / d;
  res.push_back(c1.p + diff * Point(rc, rs));
  if (dfr != 0.0) res.push_back(c1.p + diff * Point(rc, -rs));
  return res;
}

//円と直線の交点
vector<Point> is_lc (Circle c, Line l){
  vector<Point> res;
  ld d = dist_lp(l, c.p);
  if (d < c.r + eps){
    ld len = (d > c.r) ? 0.0 : sqrt(c.r * c.r - d * d);
    Point nor = (l.a - l.b) / abs(l.a - l.b);
    res.push_back(proj(l, c.p) + len * nor);
    res.push_back(proj(l, c.p) - len * nor);
  }
  return res;
}

vector<Point> is_sc(Circle c, Line l){
  vector<Point> v = is_lc(c, l), res;
  for (Point p : v)
    if (isis_sp(l, p)) res.push_back(p);
  return res;
}

//点を通る円の接戦
vector<Line> tangent_cp(Circle c, Point p) {
  vector<Line> ret;
  Point v = c.p - p;
  ld d = abs(v);
  ld l = sqrt(norm(v) - c.r * c.r);
  if (isnan(l)) { return ret; }
  Point v1 = v * Point(l / d,  c.r / d);
  Point v2 = v * Point(l / d, -c.r / d);
  ret.push_back(Line(p, p + v1));
  if (l < eps) return ret;
  ret.push_back(Line(p, p + v2));
  return ret;
}

//円と円の接戦
vector<Line> tangent_cc(Circle c1, Circle c2) {
  vector<Line> ret;
  if (abs(c1.p - c2.p) - (c1.r + c2.r) > -eps) {
    Point center = (c1.p * c2.r + c2.p * c1.r) / (c1.r + c2.r);
    ret = tangent_cp(c1, center);
  }
  if (abs(c1.r - c2.r) > eps) {
    Point out = (-c1.p * c2.r + c2.p * c1.r) / (c1.r - c2.r);
    vector<Line> nret = tangent_cp(c1, out);
    ret.insert(ret.end(), ALL(nret));
  }
  else {
    Point v = c2.p - c1.p;
    v /= abs(v);
    Point q1 = c1.p + v * Point(0,  1) * c1.r;
    Point q2 = c1.p + v * Point(0, -1) * c1.r;
    ret.push_back(Line(q1, q1 + v));
    ret.push_back(Line(q2, q2 + v));
  }
  return ret;
}

typedef vector<Point> Polygon;

ld area(const Polygon &p) {
  ld res = 0;
  int n = p.size();
  REP(j,n) res += cross(p[j], p[(j+1)%n]);
  return res / 2;
}

bool is_counter_clockwise (const Polygon &poly) {
  ld angle = 0;
  int n = poly.size();
  REP(i,n) {
    Point a = poly[i], b = poly[(i+1)%n], c = poly[(i+2)%n];
    angle += arg((c - b) / (b - a));
  }
  return angle > eps;
}

int is_in_polygon (const Polygon &poly, Point p) {
  ld angle = 0;
  int n = poly.size();
  REP(i,n) {
    Point a = poly[i], b = poly[(i+1)%n];
    if (isis_sp(Line(a, b), p)) return 1;
    angle += arg((b - p) / (a - p));
  }
  return eq(angle, 0) ? 0 : 2;
}

Polygon convex_hull(vector<Point> ps) {
  int n = ps.size();
  int k = 0;
  sort(ps.begin(), ps.end());
  Polygon ch(2 * n);
  for (int i = 0; i < n; ch[k++] = ps[i++])
    while (k >= 2 && ccw(ch[k - 2], ch[k - 1], ps[i]) <= 0) --k;
  for (int i = n - 2, t = k + 1; i >= 0; ch[k++] = ps[i--])
    while (k >= t && ccw(ch[k - 2], ch[k - 1], ps[i]) <= 0) --k;
  ch.resize(k - 1);
  return ch;
}

Polygon convex_cut(const Polygon &ps, Line l) {
  int n = ps.size();
  Polygon Q;
  REP(i,n) {
    Point A = ps[i], B = ps[(i+1)%n];
    Line m = Line(A, B);
    if (ccw(l.a, l.b, A) != -1) Q.push_back(A);
    if (ccw(l.a, l.b, A) * ccw(l.a, l.b, B) < 0 && isis_ll(l, m))
      Q.push_back(is_ll(l, m));
  }
  return Q;
}

void add_point(vector<Point> &ps, Point p) {
  for (Point q : ps) if (abs(q - p) < eps) return;
  ps.push_back(p);
}

typedef int Weight;

struct Edge { int from, to; Weight weight; };

typedef vector<Edge> Edges;
typedef vector<Edges> Graph;

void add_edge(Graph &g, int from, int to, Weight weight) {
  g[from].push_back((Edge){from, to, weight});
}

Graph segment_arrangement(const vector<Line> &s, const vector<Point> &p) {
  int n = p.size(), m = s.size();
  Graph g(n);
  REP(i,m) {
    vector<pair<ld,int>> vec;
    REP(j,n) if (isis_sp(s[i], p[j]))
      vec.emplace_back(abs(s[i].a - p[j]), j);
    sort(ALL(vec));
    REP(j,vec.size()-1) {
      int from = vec[j].second, to = vec[j+1].second;
      add_edge(g, from, to, abs(p[from] - p[to]));
    }
  }
  return g;
}

Graph circle_arrangement(const vector<Circle> &c, const vector<Point> &p) {
  int n = p.size(), m = c.size();
  Graph g(n);
  REP(i,m) {
    vector<pair<ld,int>> vec;
    REP(j,n) if (abs(abs(c[i].p - p[j]) - c[i].r) < eps)
      vec.emplace_back(arg(c[i].p - p[j]), j);
    sort(ALL(vec));
    REP(j,vec.size()-1) {
      int from = vec[j].second, to = vec[j+1].second;
      ld angle = vec[j+1].first - vec[j].first;
      add_edge(g, from, to, angle * c[i].r);
    }
    if (vec.size() >= 2) {
      int from = vec.back().second, to = vec.front().first;
      ld angle = vec.front().first - vec.back().first;
      add_edge(g, from, to, angle * c[i].r);
    }
  }
  return g;
}

vector<vector<int>> polygon;
vector<int> seg2p[1024][1024];

Graph dual_graph(const vector<Line> &s, const vector<Point> &p) {
  int N = p.size();
  polygon.clear();
  REP(i,1024) REP(j,1024) seg2p[i][j].clear();
  vector<vector<tuple<ld,int,bool>>> tup(N);
  REP(i,s.size()) {
    int a = -1, b = -1;
    REP(j,N) if (abs(s[i].a - p[j]) < eps) a = j;
    REP(j,N) if (abs(s[i].b - p[j]) < eps) b = j;
    assert(a >= 0 && b >= 0);
    tup[a].emplace_back(arg(s[i].b - s[i].a), b, false);
    tup[b].emplace_back(arg(s[i].a - s[i].b), a, false);
  }
  REP(i,N) sort(ALL(tup[i]));
  REP(i,N) {
    REP(j,tup[i].size()) {
      ld angle; int pos = j, from = i, to; bool flag;
      tie(angle, to, flag) = tup[i][j];
      if (flag) continue;
      vector<int> ps;
      while (!flag) {
        ps.push_back(from);
        get<2>(tup[from][pos]) = true;
        seg2p[from][to].push_back(polygon.size());
        seg2p[to][from].push_back(polygon.size());
        angle += pi + eps;
        if (angle > pi) angle -= 2 * pi;
        auto it = lower_bound(ALL(tup[to]), make_tuple(angle, 0, false));
        if (it == tup[to].end()) it = tup[to].begin();
        from = to; tie(angle, to, flag) = *it;
        pos = it - tup[from].begin();
      }
      polygon.push_back(ps);
    }
  }
  Graph g(polygon.size());
  REP(i,N) REP(j,i) {
    if (seg2p[i][j].size() == 2) {
      int from = seg2p[i][j][0], to = seg2p[i][j][1];
      g[from].push_back((Edge){from, to});
      g[to].push_back((Edge){to, from});
    }
  }
  return g;
}

const ld zoom = 25;
const ld centerX = 6;
const ld centerY = 5;

void change_color(int r, int g, int b) {
  fprintf(stderr, "c.strokeStyle = 'rgb(%d, %d, %d)';\n", r, g, b);
}

int cordx(Point p) { return 400 + zoom * (p.real() - centerX); }
int cordy(Point p) { return 400 - zoom * (p.imag() - centerY); }

#define cord(p) cordx(p),cordy(p)

void draw_point(Point p) {
  fprintf(stderr, "circle(%d, %d, %d)\n", cord(p), 2);
}

void draw_segment(Line l) {
  fprintf(stderr, "line(%d, %d, %d, %d)\n", cord(l.a), cord(l.b));
}

void draw_line(Line l) {
  Point v = l.b - l.a;
  Line m(l.a - v * Point(1e4, 0), l.b + v * Point(1e4, 0));
  fprintf(stderr, "line(%d, %d, %d, %d)\n", cord(m.a), cord(m.b));
}

void draw_polygon(const Polygon &p) {
  int n = p.size();
  REP(i,n) draw_segment(Line(p[i], p[(i+1)%n]));
}

void draw_circle(Circle c) {
  fprintf(stderr, "circle(%d, %d, %d)\n", cord(c.p), (int)(zoom * c.r));
}
