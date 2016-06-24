#include <bits/stdc++.h>

using namespace std;

typedef long long LL;
typedef pair<int, int> PII;
typedef long double LD;
typedef pair<LD, LD> PLDLD;

#define FOR(i,a,b) for(int i=(a);i<(b);++i)
#define REP(i,n)  FOR(i,0,n)
#define CLR(a) memset((a), 0 ,sizeof(a))

//
//  Union-Find Tree
//
vector<int> parent(N,-1);//N is size
int Find(int p)
{
	return parent[p] < 0 ? p : parent[p] = Find(parent[p]);
}
void Merge(int p,int q)
{
	p=Find(p);q=Find(q);
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
bool Belong(int p, int q)
{
	return Find(p) == Find(q);
}
int GetSize(int p)
{
	return -parent[Find(p)];
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


/* verified AOJ Highway Express Bus
*使い方
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


//
// Clustal algorithm
//

struct Edge{
int start,end,dis;
bool operator >(const Bdge &b)const{return dis > b.dis;}
};
int V;//The num of vertex.
int inf = INT_MAX/2;
priority_queue<Edge,vector<Edge> ,greater<Edge> > que;//Edges are stacked
vector<Edge> minTree;//the result will be push in 
int belong[V];

void Clustal()
{
	REP(i,V)
		belong[i]=i;
	
	Edge edge;
	while(!que.empty())
	{
		edge=que.top();
		if(belong[edge.start]!=belong[edge.end])
		{
			REP(i,V)
				if(belong[i]==belong[edge.end])
					belong[i]=belong[edge.start];
					
			minTree.push_back(edge);
		}
		que.pop();
	}
}

/* verified AOJ1327 One-Dimensional Cellular Automaton
~Matrix<T>~
行列の簡単な計算ができる c++11
vector iostream cmath 必須
Matrix a(VV<T>), b(VV<T>);
行列式:a*b
スカラ積:l*a
和:a+b
差:a-b
転置:a.transport()
余因子:a.cofactor()
行列式:a.det()
aのi行j列目:a.get(i,j)　
aのi行目j列目にkを代入:set(i,j,k)
もしくはa[i][j] (a[i][j]=k)
aのx乗:a.pow(x)
ランク:a.rank()
n*n基本行列E:Matrix(n)
m*n 0行列:Matrix(m,n)
m*n 全要素がpの行列:Matrix(m,n,p)
Row<T>型rowでの初期化(m*1行列):Matrix(row)

**(整数不可)**
三角化:a.triangulate()
逆行列:a.inverse()
(逆行列のa.det()倍ならa.pre_inverse()で求まる 整数可)
***
*/
template<class T>
using Row = vector<T>;
template<class T>
using VV = vector<Row<T> >;

typedef long double ld;
const ld EPS = 1e-11;


template<class T>
struct Matrix {
    VV<T> matrix;
    int n,m;

    Matrix(const VV<T> &matrix_);
    explicit Matrix(int n_);
    explicit Matrix(const Row<T> &row);
    Matrix(int m_, int n_, T e=0);

    const T get(const int i, const int j) const;
    void set(const int x, const int y, const T k);

    const Matrix<T> transport() const;

    const Matrix<T> operator + (const Matrix<T> &rhs) const;
    const Matrix<T> operator * (const Matrix<T> &rhs) const;
    const Matrix<T> operator - (const Matrix<T> &rhs) const;
    Matrix<T>& operator += (const Matrix<T> &rhs);
    Matrix<T>& operator *= (const Matrix<T> &rhs);
    Matrix<T>& operator -= (const Matrix<T> &rhs);

    Row<T>& operator[](const int x);
    template<class U> operator Matrix<U> () const;

    const Matrix<T> pow(int x) const;
    const Matrix<T> cofactor(int x, int y) const;
    const T det() const;
    const int rank() const;

    //逆行列が存在すれば、(行列式)*(逆行列)を返す
    //A:matrix,return det A * A^-1
    const Matrix<T> pre_inverse() const;

    /*template */const Matrix<ld> triangulate() const;
    /*template */const Matrix<ld> inverse() const;
};
template<class T>
const Matrix<T> operator * (const T lambda, const Matrix<T> &rhs)
{
    Matrix<T> tmp(rhs);
    for(int i=0; i<rhs.m; i++)
        for(int j=0; j<rhs.n; j++)
            tmp.set(i, j, tmp.get(i,j) * lambda);
    return tmp;
}

template<class T>
Matrix<T>::Matrix(const VV<T> &matrix_):matrix(matrix_)
{
    m=matrix_.size();
    if(m == 0) n = 0;
    else n = matrix_[0].size();
}

template<class T>
Matrix<T>::Matrix(int n_):m(n_),n(n_)
{
  matrix = VV<T>(n, Row<T>(n, 0));
  for (int i = 0; i < n; ++i)
    set(i,i,1);
}

template<class T>
Matrix<T>::Matrix(const Row<T> &row):m(1), n(row.size()), matrix(VV<T>(1, row))
{
  //sizeがmのvector<T>からmx1行列の生成
  (*this) = transport();
}

template<class T>
Matrix<T>::Matrix(int m_, int n_, T e):m(m_),n(n_)
{
  matrix = VV<T>(m, Row<T>(n, e));
}

template<class T>
const T Matrix<T>::get(const int i, const int j) const
{
    if(0<=i && i<m && 0<=j && j<n)
        return matrix[i][j];

    cerr << "get(" << i << "," << j<< ")is not exist." << endl;
    throw;
}

template<class T>
void Matrix<T>::set(const int x, const int y, const T k)
{
    if(0<=x && x<m && 0<=y && y<n)
    {
      *(matrix[x].begin()+y) = k;
      return;
    }
    cerr << "set(" << x << "," << y<< ")is not exist." << endl;
    throw;
}

template<class T>
const Matrix<T> Matrix<T>::transport() const
{
    VV<T> tmp;
    for(int i=0; i<n; i++)
    {
        Row<T> row;
        for(int j=0; j<m; j++)
            row.push_back(get(j,i));
        tmp.push_back(row);
    }
    return tmp;
}

template<class T>
const Matrix<T> Matrix<T>::operator + (const Matrix<T> &rhs) const
{
    if(m != rhs.m || n != rhs.n)
    {
        cerr << "Matrix Add can not calculate." << endl;
        throw;
    }

    Matrix<T> tmp(m, n, 0);
    for(int i=0; i<m; i++)
    {
        for(int j=0; j<n; j++)
        {
            tmp.set(i, j, get(i,j)+rhs.get(i,j));
        }
    }
    return tmp;
}

template<class T>
const Matrix<T> Matrix<T>::operator * (const Matrix<T> &rhs) const
{
    if(n != rhs.m)
    {
        cerr << "Matrix Product can not calculate." << endl;
        throw;
    }
    Matrix<T> tmp(m, rhs.n, 0);
    T sum;
    for(int i=0; i<m; i++)
        for(int j=0; j<rhs.n; j++)
        {
            sum=0;
            for(int k=0; k<n; k++)
            {
                sum += get(i,k)*rhs.get(k,j);
            }
            tmp.set(i, j, sum);
        }
    return tmp;
}

template<class T>
const Matrix<T> Matrix<T>::operator - (const Matrix<T> &rhs) const
{
    return *this+((T)-1 * rhs);
}

template<class T>
Matrix<T>& Matrix<T>::operator += (const Matrix<T> &rhs)
{
    return *this = *this + rhs;
}

template<class T>
Matrix<T>& Matrix<T>::operator *= (const Matrix<T> &rhs)
{
  return *this = *this * rhs;
}

template<class T>
Matrix<T>& Matrix<T>::operator -= (const Matrix<T> &rhs)
{
    return *this = *this - rhs;
}

template<class T>
Row<T>& Matrix<T>::operator[](const int x)
{
  if(0<=x && x<m)
    return matrix[x];

    //Matrix[x][y]でｘが配列外のときのみこのエラーを吐く
  cerr<<"Matrix[" << m <<"] is not exist." << endl;
  throw;
}

template<class T> template <class U>
Matrix<T>::operator Matrix<U> () const
{
  Matrix<U> tmp(m,n);
  for(int i=0;i<m;i++)
    for(int j=0;j<n;j++)
      tmp.set(i,j,(U)get(i,j));
  return tmp;
}

template<class T>
const Matrix<T> Matrix<T>::pow(int x) const
{
  Matrix<T> tmp(*this), e(m);
  for(int i=1;i<=x;i<<=1)
  {
    if((x&i)>0)
      e=e*tmp;
    tmp=tmp*tmp;
  }
  return e;
}

template<class T>
const Matrix<T> Matrix<T>::cofactor(int x, int y) const
{
  VV<T> tmp;
  for(int i=0;i<m;i++)
  {
    if(x==i) continue;
    Row<T> row;
    for(int j=0;j<n;j++)
    {
      if(y==j) continue;
      row.push_back(get(i,j));
    }
    tmp.push_back(row);
  }
  return Matrix<T>(tmp);
}

template<class T>
const T Matrix<T>::det() const
{
  if(n!=m)
  {
    cerr << "det can not calculate." << endl;
    throw;
  }
  if(m==1)
    return get(0,0);
  T sum=0;
  for(int i=0;i<m;i++)
  {
    sum += ((i%2==0?1:-1)*get(i,0))*Matrix<T>(cofactor(i,0)).det();
  }
  return sum;
}

template<class T>
const int Matrix<T>::rank() const
{
  Matrix<ld> tmp( ((Matrix<ld>)*this).triangulate());
  for(int i=tmp.m-1;i>=0;i--)
  {
    if(abs(tmp.get(i,tmp.n-1)) < EPS)
      continue;
    return i+1;
  }
  return 0;
}

template<class T>
const Matrix<T> Matrix<T>::pre_inverse() const
{
  if(m!=n)
  {
    cerr<<"inverse can not calculate." << endl;
    throw;
  }

  Matrix<T> tmp(m,n,0);
  for(int i=0;i<m;i++)
    for(int j=0;j<n;j++)
      tmp.set(i, j, ((i+j)%2==0?1:-1)*cofactor(i,j).det());
  return tmp.transport();
}

template<> const Matrix<ld> Matrix<ld>::triangulate() const
{
  Matrix<ld> tmp(*this);
  ld e;
  for(int i=0;i<m;i++)
  {
    if(abs(tmp.get(i,i)) < EPS)
    {
      tmp.set(i,i,0);
      continue;
    }
    for(int j=i+1;j<m;j++)
    {
      e = tmp.get(j,i) / tmp.get(i,i);
      for(int k=0;k<n;k++)
      tmp.set(j, k, tmp.get(j,k) - tmp.get(i,k) * e);
    }
  }
  return tmp;
}

template<> const Matrix<ld> Matrix<ld>::inverse() const
{
  Matrix<ld> tmp(pre_inverse());
  ld e = det();
  if(e==0)
  {
    cerr<<"0 div error in inverse()." << endl;
    throw;
  }
  tmp=1/e*tmp;
  return tmp.transport();
}


//Ford-folkerson

const int N=101;
const int INF=1e9;
struct Edge
{
	int from,to,cost,rev;
};
vector<Edge> graph[N];
bool used[101];
void add_Edge(int from, int to, int cap)
{
	graph[from].push_back((Edge){from,to,cap,(int)graph[to].size()});
	graph[to].push_back((Edge){to,from,0,(int)graph[from].size()-1});
}
int dfs(int from, int to, int f)
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
int ford_fulkerson(int from, int to)
{
	int flow=0;
	while(1)
	{
		memset(used,0,sizeof(used));
		int f=dfs(from,to,INF);
		if(f==0)break;
		flow += f;
	}
	return flow;
}



/*
usage:
SegmentTree<T> tree(array, PLUS) で(array, +, 1)のセグツリーができる
update: 更新
fold:   畳み込み
*/

template<typename T>
struct SegmentTree
{
    const int SIZE;
    int tree_size, leaf_number;
    const function<T(T,T)> op;
    const T e;
    vector<T> node;
    
    SegmentTree(const vector<T> &ary, const function<T(T,T)> f, const T e_);
    
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
    const void update(const int k, const T x);
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
SegmentTree<T>::SegmentTree(const vector<T> &ary, const function<T(T,T)> f, const T e_):SIZE(ary.size()),op(f),e(e_)
{
    tree_size=SIZE;
    leaf_number=least_square(tree_size);
    tree_size=leaf_number*2;
    
    node=vector<T>(tree_size-1);
    
    for(int i=0;i<leaf_number;i++)
        if(i<SIZE)
            node[i+leaf_number-1]=ary[i];
        else
            node[i+leaf_number-1]=e;
                
    init(0);
}
    
    
template<typename T>
const T SegmentTree<T>::init(const int k)
{
    if(k>=leaf_number-1) return node[k];
    return node[k]=op(init(child_l(k)),init(child_r(k)));
}
    
template<typename T>
const void SegmentTree<T>::update(const int k, const T x)
{
    int tmp = k+leaf_number-1;
    node[tmp]=x;
    while(tmp > 0)
    {
        tmp=parent(tmp);
        node[tmp]=op(node[child_l(tmp)], node[child_r(tmp)]);
    }
}
    
template<typename T>
const T SegmentTree<T>::fold(const int a, const int b, const int k, const int l, const int r) const
{
    if(r <= a || b <= l)
        return e;  //[a,b)と[l,r)が交わらない
    if(a <= l && r <= b) 
        return node[k];    //[a,b)が[l,r)を含む
    
    return op(sum(a,b,child_l(k),l,(l+r)/2), sum(a,b,child_r(k),(l+r)/2, r));
}

#define PLUS [](int p,int q){return p+q;},0
#define MULT [](int p,int q){return p*q;},1
#define MAX [](int p,int q){return max(p,q);},INT_MIN
#define MIN [](int p,int q){return min(p,q);},INT_MAX

//n種類の遅延変数に対応したい
//いや無理だしいらんでしょ 調整中
using namespace std;
template<typename T>
struct Lazy
{
    vector<T> lazy;
    T e;
    const function<T(T,T)> op;
    const function<T(T,T,int)> range_op;
    Lazy(const int size, const function<T(T,T)> f, const function<T(T,T,int)> g, const T e_)
    :lazy(vector<T>(size, e_)),op(f),range_op(g),e(e_)
    {}
    const void add_lazy(const T x, const int k)
    {
        lazy[k]=op(lazy[k], x);
    }
    const T evalute(const T x, const T y, const int size) const
    {
        return range_op(x,y,size);
    }
    const void reset(const int k)
    {
        lazy[k]=e;
    }
};

template<typename T>
struct LazySegmentTree
{
    const int SIZE;
    int tree_size, leaf_number;
    const function<T(T,T)> op;
    vector<Lazy<T> > lazies;
    const T e;
    vector<T> node;
    
    LazySegmentTree(const vector<T> &ary, const function<T(T,T)> f, const T e_);
    
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
    const void add_lazy(const function<T(T,T)> f, const function<T(T,T,int)> g, const T e_)
    {
        lazies.push_back(Lazy<T>(tree_size, f, g, e_));
    }
    
    const void update(const int a, const int b, const T x, const int index, const int k, const int l, const int r);
    const void update(const int a, const int b, const T x, const int index=0)
    {
        return update(a,b,x,index,0,0,leaf_number);
    }
    const T fold(const int a, const int b, const int k, const int l, const int r);
    const T fold(const int a,const int b)
    {
        return fold(a, b, 0, 0, leaf_number);
    }
    
    const void evalute_lazy(const int k, const int size);
    
    /*const void print() const
    {
        REP(i, tree_size)
        cout<<node[i]<<" \n"[i==tree_size-1];
    }*/
};

template<typename T>
LazySegmentTree<T>::LazySegmentTree(const vector<T> &ary, const function<T(T,T)> f, const T e_)
    :SIZE(ary.size()),op(f),e(e_)
{
    tree_size=SIZE;
    leaf_number=least_square(tree_size);
    tree_size=leaf_number*2;
    
    node=vector<T>(tree_size-1);
    
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
const void LazySegmentTree<T>::update(const int a, const int b, const T x, const int index, const int k, const int l, const int r)
{
    if(r <= a || b <= l)
        return;  //[a,b)と[l,r)が交わらない
    if(a <= l && r <= b) 
        lazies[index].add_lazy(x,k);    //[a,b)が[l,r)を含む
    else
    {
        node[k]=lazies[index].evalute(node[k], x, min(b,r)-max(a,l));
        update(a,b,x,index,child_l(k),l,(l+r)/2);
        update(a,b,x,index,child_r(k),(l+r)/2, r);
    }
}
    
template<typename T>
const T LazySegmentTree<T>::fold(const int a, const int b, const int k, const int l, const int r)
{
    if(r <= a || b <= l)
        return e;  //[a,b)と[l,r)が交わらない
        
    evalute_lazy(k,r-l);
    if(a <= l && r <= b) 
        return node[k];    //[a,b)が[l,r)を含む
    
    return op(fold(a,b,child_l(k),l,(l+r)/2), fold(a,b,child_r(k),(l+r)/2, r));
}

template<typename T>
const void LazySegmentTree<T>::evalute_lazy(const int k, const int size)
{
    for(int i=0;i<lazies.size();i++)
    {
        if(size>1)
        {
            lazies[i].add_lazy(lazies[i].lazy[k], child_l(k));
            lazies[i].add_lazy(lazies[i].lazy[k], child_r(k));
        }
        //cout<<k<<" "<<lazies[i].lazy[k]<<" "<<node[k]<<" ";
        node[k]=lazies[i].evalute(node[k], lazies[i].lazy[k], size);
        lazies[i].reset(k);
        //cout<<node[k]<<endl;
    }
}

#define PLUS [](int p,int q){return p+q;},0
#define MULT [](int p,int q){return p*q;},1
#define MAX [](int p,int q){return max(p,q);},INT_MIN
#define MIN [](int p,int q){return min(p,q);},INT_MAX
#define RANGEADD [](int p,int q){return p+q;},[](int p,int q,int r){return p+q*r;},0

//BIT
template<typename T>
struct BIT
{
    const int SIZE;
    const function<T(T,T)> op;
    const function<T(T)> inv;
    const T e;
    vector<T> node;
    
    BIT(const vector<T> &ary, const function<T(T,T)> f, const T e_, const function<T(T)> inv_);
    
    const int least_square(const int k) const
    {
        int tmp=k;
        for(int i=1; 64>i; i<<=1)
            tmp |= (tmp >> i);
        return tmp+1;
    }
    
    const void add(const int k, const T x);
    const T fold(const int k) const;
    const T fold(const int a,const int b) const
    {
        return op(fold(b), inv(a==0?e:fold(a-1)));
    }
    
    /*const void print() const
    {
        REP(i, SIZE)
        cout<<node[i]<<" \n"[i==tree_size-1];
    }*/
};

template<typename T>
BIT<T>::BIT(const vector<T> &ary, const function<T(T,T)> f, const T e_, const function<T(T)> inv_):SIZE(least_square(ary.size()+1)),op(f),e(e_),inv(inv_)
{
    node=vector<T>(SIZE, e);
    
    for(int i=1;i<=ary.size();i++)
        add(i, ary[i-1]);
}


template<typename T>
const void BIT<T>::add(const int k, const T x) //k...0 origin
{
    for(int j=k+1;j<=SIZE;j+=(j&-j))
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

