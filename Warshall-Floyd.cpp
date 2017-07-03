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
//  Warshall-Floyd algorithm  
//

int cost[V][V];
int V;
void WF()
{
	REP(k,V) 
	REP(i,V)
	REP(j,V)
	cost[i][j] = cost[i][k] + cost[k][j];
}

