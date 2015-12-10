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



