#include <bits/stdc++.h>
using namespace std;

typedef long long LL;
typedef pair<int, int> PII;
typedef long double LD;

#define FOR(i,a,b) for(int i=(a);i<(b);++i)
#define REP(i,n)  FOR(i,0,n)
#define CLR(a) memset((a), 0 ,sizeof(a))

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
