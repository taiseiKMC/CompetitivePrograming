#include <cstdio>
#include <iostream>
#include <algorithm>
#include <queue>
#include <limits.h>

using namespace std;

//
//  This code uses Clustal algorithm.
//


struct Bridge{
int s,e,c;
bool operator >(const Bridge &b)const{return c > b.c;}
};
int main()
{
  while(true)
  {
  int inf = INT_MAX/2;
  int n,m;
  scanf("%d%d",&n,&m);
  if(n==0)break;
  int a,b,c;
  Bridge brg;
  priority_queue<Bridge,vector<Bridge> ,greater<Bridge> > q;
  for(int i=0;i<m;i++)
  {
    scanf("%d%d%d",&a,&b,&c);
    brg.c=c;
    brg.s=a;
    brg.e=b;
    q.push(brg);
  }
  
  int cit[101];
  for(int i=0;i<101;i++)
  cit[i]=i;
  
  int p,cost=0;
  while(!q.empty())
  {
    brg=q.top();
    if(cit[brg.s]!=cit[brg.e])
    {
      p=cit[brg.e];
      for(int i=0;i<n;i++)
      {
        if(cit[i]==p)
          cit[i]=cit[brg.s];
      }
      cost += brg.c;
    }
    q.pop();
  }
  printf("%d\n",cost);
}
}
