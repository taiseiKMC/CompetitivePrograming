#include <cstdio>
#include <algorithm>
#include <iostream>
#include <stack>
#include <queue>
#include <vector>
#include <limits.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <cassert>
#include <map>
using namespace std;

typedef long long LL;
typedef pair<int, int> PII;
typedef long double LD;

#define FOR(i,a,b) for(int i=(a);i<(b);++i)
#define REP(i,n)  FOR(i,0,n)
#define CLR(a) memset((a), 0 ,sizeof(a))


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
