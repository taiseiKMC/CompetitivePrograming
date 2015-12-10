#include <cstdio>
#include <algorithm>

int McC(int n)
{
	if(n>100) return n-10;
	return McC(McC(n+11));
}
int main()
{
	int n;
	scanf("%d",&n);
	printf("%d\n",McC(n));
}
