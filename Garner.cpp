namespace Garner
{
    /* Garnerのアルゴリズム
    x = mods[i].first mod mods[i].second を満たすxを求める
    https://qiita.com/drken/items/ae02240cd1f8edfc86fd
    */
    using LL = long long;
	LL gcd(const LL a, const LL b)
	{
		return __gcd(a, b);
	}
    LL mod(const LL p, const LL m)
    {
        return (p<0)?(p%m+m):(p%m);
    }
    LL extgcd(const LL a, const LL b, LL &x, LL &y)
    {
        if(b==0)
        {
            x=1;
            y=0;
            return a;
        }
        LL d=extgcd(b, a%b, y, x);
        y-=a/b*x;
        return d;
    }
    LL modinv(const LL a, const LL m)
    {
        //a*x + m*y = gcd(a,m) = 1
        LL x,y;
        extgcd(a, m, x, y);
        return mod(x, m);
    }

	/*
	Garnerの前処理
	modが互いに素になるようにするor解が無いときは-1を返す
	*/
	vector<pair<LL,LL>> normalize(const vector<pair<LL, LL>> &mods)
	{
		vector<pair<LL, LL>> ret(mods);
		for(size_t i=0;i<ret.size();i++)
		for(size_t j=0;j<i;j++)
		{
			LL g=gcd(ret[i].second, ret[j].second);
			if((ret[i].first - ret[j].first)%g != 0) return vector<pair<LL, LL>>(0);
			ret[i].second/=g;
			ret[j].second/=g;
			LL gi = gcd(ret[i].second, g);
			LL gj = g/gi;
			g = gcd(gi, gj);
			while(g != 1)
			{
				gi*=g;
				gj/=g;
				g = gcd(gi, gj);
			}
			ret[i].second*=gi;
			ret[j].second*=gj;
			ret[i].first%=ret[i].second;
			ret[j].first%=ret[j].second;
		}
		return ret;
	}

    LL Garner(const vector<pair<LL, LL>> &mods, const LL m)
    {
        const int n=mods.size();
        vector<LL> modprod(n+1,1);
        vector<LL> p(n+1);
        for(int i=0;i<n;i++)
        {
            LL t=mod((mods[i].first - p[i]) * modinv(modprod[i], mods[i].second), mods[i].second);

            for(int j=i+1;j<n;j++)
            {
                p[j]+=t*modprod[j];
                p[j]%=mods[j].second;
                modprod[j]*=mods[i].second;
                modprod[j]%=mods[j].second;
            }
            p[n]+=t*modprod[n];
            p[n]%=m;
            modprod[n]*=mods[i].second;
            modprod[n]%=m;
        }
        return p[n];
    }

	LL solve(const vector<pair<LL, LL>> &mods, const LL m)
	{
		const vector<pair<LL,LL>> tmp(normalize(mods));
		if(tmp.size() == 0) return -1;
		return Garner(tmp, m);
	}
}