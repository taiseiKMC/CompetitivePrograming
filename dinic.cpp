#include <bits/stdc++.h>

using namespace std;

typedef long long FLOW;
const FLOW INF=(FLOW)(1e6)*(FLOW)(1e6);

const int N=10000;
struct Edge
{
    int to;
    FLOW cap;
    int rev;
};
vector<Edge> graph[N];
int iter[N];
int level[N];
void add_Edge(int from, int to, FLOW cap)
{
    //cout<<from<<":"<<to<<" "<<cap<<endl;
	graph[from].push_back((Edge){to,cap,(int)graph[to].size()});
	graph[to].push_back((Edge){from,0,(int)graph[from].size()-1});
}

void bfs(int from)
{
    memset(level, -1, sizeof(level));
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

FLOW dfs(int from, int to, FLOW f)
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
FLOW dinic(int from, int to)
{
    FLOW flow=0;
	while(1)
	{
        bfs(from);
        if(level[to]<0) return flow;
		memset(iter, 0, sizeof(iter));
        FLOW f;
        while((f=dfs(from,to,INF))>0)
        {
            flow+=f;
        }
	}
}
