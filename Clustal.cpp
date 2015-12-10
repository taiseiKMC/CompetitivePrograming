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
// Clustal algorithm
//

struct Edge{
int start,end,dis;
bool operator >(const Bdge &b)const{return dis > b.dis;}
};
int V;//The num of vertex.
int inf = INT_MAX/2;
priority_queue<Edge,vector<Edge> ,greater<Edge> > que;//Edges are stacked
vector<Edge> minTree;//the result will be push in 
int belong[V];

void Clustal()
{
	REP(i,V)
		belong[i]=i;
	
	Edge edge;
	while(!que.empty())
	{
		edge=que.top();
		if(belong[edge.start]!=belong[edge.end])
		{
			REP(i,V)
				if(belong[i]==belong[edge.end])
					belong[i]=belong[edge.start];
					
			minTree.push_back(edge);
		}
		que.pop();
	}
}
