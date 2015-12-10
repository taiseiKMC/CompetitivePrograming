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
//  Union-Find Tree
//

vector<int> parent(N,-1);//N is size
int Find(int p)
{
	return parent[p] < 0 ? p : parent[p] = Find(parent[p]);
}
void Marge(int p,int q)
{
	p=Find(p);q=Find(q);
	if(p==q) return;
	if(parent[p] < parent[q])
		parent[q]=p;
	else if(parent[q] < parent[p])
		parent[p]=q;
	else
	{
		parent[q]=p;
		parent[p]--;
	}

}
bool Belong(int p, int q)
{
	return Find(p) == Find(q);
}



int main()
{
	int n;
	scanf("%d", &n);
	    
	printf("%d\n", n);
}
