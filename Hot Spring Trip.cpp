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


#define FOR(i,a,b) for(int i=(a);i<(b);++i)
#define REP(i,n)  FOR(i,0,n)
#define CLR(a) memset((a), 0 ,sizeof(a))

//
//  This code uses Dikstra algorithm.
//  
//

struct status
{
	int edge,cost,chicket;
	void Set(int _edge,int _cost ,int _chicket)
	{
		edge = _edge;
		cost = _cost;
		chicket = _chicket;
	}
	bool operator >(const status &st)const
	{
		return cost > st.cost;
	}
};
int inf = INT_MAX/2;
vector<PII> vertex[101];
int dist[101][2];
int main()
{
	while(true)
	{
		REP(i,101)
		{
			vertex[i].clear();
			dist[i][0]=inf;dist[i][1]=inf;
		}
		
		
    int n,m;
    scanf("%d%d", &n,&m);
    if(n==0) break;
    int a,b,c;
    REP(i,m)
    {
		scanf("%d%d%d",&a,&b,&c);
		vertex[a].push_back(PII(b,c));
		vertex[b].push_back(PII(a,c));
	}
    status sta,p;
    sta.Set(1,0,1);dist[1][1]=0;
    priority_queue<status, vector<status>, greater<status> > q;
    q.push(sta);
    
    int to,th,cos;
    while(!q.empty())
    {
		sta = q.top();
		REP(i,vertex[sta.edge].size())
		{
			to = vertex[sta.edge][i].first;
			cos = vertex[sta.edge][i].second;
			if(dist[to][sta.chicket] > dist[sta.edge][sta.chicket] + cos)
			{
				dist[to][sta.chicket] = dist[sta.edge][sta.chicket] + cos;
				p.Set(to,dist[to][sta.chicket],sta.chicket);
				q.push(p);
			}
			if(sta.chicket > 0)
			{
				REP(j,vertex[to].size())
				{
					th = vertex[to][j].first;
					if(th != sta.edge)
					{
						if(dist[th][sta.chicket-1] > dist[sta.edge][sta.chicket])
						{
							dist[th][sta.chicket-1] = dist[sta.edge][sta.chicket];
							p.Set(th, dist[th][sta.chicket-1], sta.chicket-1);
							q.push(p);
						}
					}
				}
			}
		}
		
		q.pop();
	}
    
    
	printf("%d\n", min(dist[n][0],dist[n][1]));
}
}
