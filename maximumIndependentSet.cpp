//とりあえず
//code thanks fes 2017 G

int main()
{
	int n,m;
	cin>>n>>m;
	vector<VB> cmp(n,VB(n));
	REP(i,m)
	{
		int a,b;
		cin>>a>>b;
		a--;b--;
		cmp[a][b]=1;
		cmp[b][a]=1;
	}

	int m2=n/2;
	int m1=n-m2;
	int INFB=127;
	VB dp(1<<m1,INFB);
	function<bool(int)> dfs=[&](int p)
	{
		if(dp[p]!=INFB) return dp[p];
		REP(i,m1)
		{
			if(((1<<i)&p)>0)
			{
				bool f=dfs(p-(1<<i));
				if(dp[p]!=INFB) continue;
				REP(j, m1)
				{
					if(((1<<j)&p)==0) continue;
					if(i==j) continue;
					f&=!cmp[i][j];
				}
				dp[p]=f;
			}
		}
		return dp[p];
	};
	dfs((1<<m1)-1);


	VB dp2(1<<m2,INFB);
	function<bool(int)> dfs2=[&](int p)
	{
		if(dp2[p]!=INFB) return dp2[p];
		REP(i,m2)
		{
			if(((1<<i)&p)>0)
			{
				bool f=dfs2(p-(1<<i));
				if(dp2[p]!=INFB) continue;
				REP(j, m2)
				{
					if(((1<<j)&p)==0) continue;
					if(i==j) continue;
					f&=!cmp[i+m1][j+m1];
				}
				dp2[p]=f;
			}
		}
		return dp2[p];
	};
	dfs2((1<<m2)-1);


	VI dp3(1<<m2,-1);
	function<int(int)> dfs3=[&](int p)
	{
		if(dp3[p]!=-1) return dp3[p];
		int ma=0;
		if(dp2[p])
		{
			REP(i,m2)
			{
				if(((1<<i)&p)>0)
				{
					dfs3(p-(1<<i));
					ma++;
				}
			}
			return dp3[p]=ma;
		}
		REP(i,m2)
		{
			if(((1<<i)&p)>0)
			{
				int f=dfs3(p-(1<<i));
				chmax(ma, f);
			}
		}
		return dp3[p]=ma;
	};
	dfs3((1<<m2)-1);


	VI dp4((1<<m1), -1);
	function<int(int)> dfs4=[&](int p)
	{
		if(dp4[p]!=-1) return dp4[p];
		if(p==0) return dp4[p]=(1<<m2)-1;
		int ma=0;
		bool f=1;
		REP(i,m1)
		{
			if(((1<<i)&p)>0)
			{
				int v=dfs4(p-(1<<i));
				if(f)
				{
					REP(j,m2)
					{
						v&=cmp[i][j+m1]?~(1<<j):~0;
					}
					dp4[p]=v;
					ma=v;
					f=0;
				}
			}
		}
		return ma;
	};
	dfs4((1<<m1)-1);

	int ans=0;
	REP(i,1<<m1)
	{
		if(!dp[i]) continue;
		int m2s=dp3[dp4[i]];
		REP(j,m1)
		{
			if(((1<<j)&i)>0) 
				m2s++;
		}
		chmax(ans, m2s);
	}
	print(ans);
}