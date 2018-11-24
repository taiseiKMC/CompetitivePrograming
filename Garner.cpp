namespace Garner
{
    /* Garnerのアルゴリズム
    x = mods[i].first mod mods[i].second を満たすxを求める
    https://qiita.com/drken/items/ae02240cd1f8edfc86fd
    */
    using LL = long long;
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
}