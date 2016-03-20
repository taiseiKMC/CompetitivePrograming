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

int main()
{

	dfs(1, 0);

}
