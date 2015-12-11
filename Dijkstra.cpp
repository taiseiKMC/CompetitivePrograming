/*
*使い方
Distを好きな辺のコストの型に置換する
vertexに辺の情報を与える 
(vertex[a].push_back(PID(b,c))でaからbへコストcの有向辺を張る)
dijkstra(i,j)でiからjへの最小コストを返す。
dijkstra(i)でshortestにiからの最小コストが格納される。
dijkstraを実行した後にget_pathを実行すると、パスが返ってくる
*/


#include <queue>

using namespace std;
typedef long double ld;
typedef double Dist;

const int INF = 1e9;
const ld EPS = 1e-11;

const int V=100000;//要素数 要初期化
struct Edge;
vector<vector<Edge> > vertex(V);
vector<Dist> shortest(V, INF);
vector<int> prev(V);


struct Status
{
	int pos; Dist dist;
	Status():pos(0),dist(0) 
	{}
	Status(int pos_,Dist dist_):pos(pos_),dist(dist_)
	{}
	void set(int pos_,Dist dist_)
	{
		pos = pos_;
		dist = dist_;
	}
	bool operator >(const Status &st)const
	{
		return dist > st.dist;
	}
};
struct Edge
{
	int to; Dist dist;
	Edge():to(0),dist(0) 
	{}
	Edge(int to_,Dist dist_):to(to_),dist(dist_)
	{}
	void set(int to_,Dist dist_)
	{
		to = to_;
		dist = dist_;
	}
	bool operator >(const Edge &ed)const
	{
		return dist > ed.dist;
	}
};
//Shortest cost from i to j.
int dijkstra(int i, int j)
{
	priority_queue<Status, vector<Status>, greater<Status> > que;
	que.push(Status(i, 0));
	for(int k=0;k<V;k++)
		shortest[k]=INF;
	shortest[i]=0;

	Status sta, tmp;
	int to;
	Dist dist;
	while(!que.empty())
	{
		sta = que.top();
		que.pop();
		if(shortest[sta.pos] < sta.dist) continue;
		for(int k=0;k<vertex[sta.pos].size();k++)
		{
			to = vertex[sta.pos][k].to;
			dist = vertex[sta.pos][k].dist;
			if(shortest[to] > shortest[sta.pos] + dist)
			{
				shortest[to] = shortest[sta.pos] + dist;
				prev[to] = sta.pos;
				tmp.set(to, shortest[to]);
				que.push(tmp);
			}
		}
		
	}
	
	return shortest[j];
}
void dijkstra(int i)
{
	dijkstra(i, 0);
}
vector<int> get_path(int i,int j)
{
	vector<int> rev;
	rev.push_back(j);
	int p=j;
	while(p!=i)
	{
		p = prev[p];
		rev.push_back(p);
	}
	reverse(rev.begin(),rev.end());
	return rev;
}
