#include <cstdio>
#include <algorithm>
#include <iostream>
#include <queue>
#include <vector>
#include <limits.h>
#include <math.h>
#include <stack>
using namespace std;

//
//  This code uses Bellman-Ford algorithm
//

int cost[101][11];
vector<pair<int,int> > root[101];
int inf = INT_MAX/2;
int main()
{
	while(true)
	{
	int c,n,m,s,d;
	int a,b,f;
	scanf("%d%d%d%d%d",&c,&n,&m,&s,&d);
	if(n==0) break;
	
	for(int i=0;i<101;i++)
	{
		//for(int j=0;j<101;j++)
		root[i].clear();
		for(int k=0;k<11;k++)
			cost[i][k] = inf;
	}
	
	for(int i=0;i<m;i++)
	{
		scanf("%d%d%d",&a,&b,&f);
		root[b].push_back(pair<int, int>(a,f));
		root[a].push_back(pair<int, int>(b,f));
	}
	cost[s][c]=0;
	
	bool update=true;
	int pur, cos;
	while(update)
	{
		update = false;
		
		for(int i=1;i<=n;i++)
			for(int j=c;j>=0;j--)
				if(cost[i][j]!=inf)
					for(int k=0;k<root[i].size();k++)
						if(root[i][k].second != inf)
						{
							pur = root[i][k].first;
							cos = root[i][k].second;
							if(cost[pur][j] > cost[i][j] + cos)
							{
								cost[pur][j] = cost[i][j] + cos;
								update = true;
							}
							if(j > 0 && cost[pur][j-1] > cost[i][j] + (cos / 2))
							{
								cost[pur][j-1] = cost[i][j] + (cos / 2);
								update = true;
							}
						}
			
		
		//printf("roop\n");
	}
		int mi=inf;
		for(int i=c;i>=0;i--)
			mi = min(mi, cost[d][i]);
	
	
		printf("%d\n",mi);
	}
}
