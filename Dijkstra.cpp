#include <cstdio>
#include <algorithm>
#include <iostream>
#include <stack>
#include <queue>
#include <vector>
#include <limits.h>
#include <math.h>
using namespace std;

typedef long long LL;
typedef pair<int, int> PII;

typedef int Dis;
typedef pair<int, Dis> PID;


#define FOR(i,a,b) for(int i=(a);i<(b);++i)
#define REP(i,n)  FOR(i,0,n)
#define CLR(a) memset((a), 0 ,sizeof(a))

//
// Dijkstra algorithm  
//


struct status
{
	int edge;Dis dis;
	status(){}
	status(int _edge,Dis _dis)
	{
		edge = _edge;
		dis = _dis;
	}
	void Set(int _edge,Dis _dis)
	{
		edge = _edge;
		dis = _dis;
	}
	bool operator >(const status &st)const
	{
		return dis > st.dis;
	}
};
int V;
int inf = INT_MAX/2;
vector<PID> vertex[V];
Dis dist[V];
priority_queue<status, vector<status>, greater<status> > q;
void Dikstra()
{
	//dist should be initialized!!
	//vertex should be informed of.
	//q have been pushed first para.
	//q.push(status(0,0));
	status sta,p;
	int to;Dis cos;
	while(!q.empty())
	{
		sta = q.top();
		q.pop();
		if(dist[sta.edge] < sta.dis) continue;
		REP(i,vertex[sta.edge].size())
		{
			to = vertex[sta.edge][i].first;
			cos = vertex[sta.edge][i].second;
			if(dist[to] > dist[sta.edge] + cos)
			{
				dist[to] = dist[sta.edge] + cos;
				p.Set(to,dist[to]);
				q.push(p);
			}
		}
		
	}
    
}


