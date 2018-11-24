
using LL=long long;
//蟻本版
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
//gcd(a,b)要らなくない？
pair<LL,LL> extgcd(const LL a, const LL b)
{
    if(b==0)
        return pair<LL,LL>(1, 0);
    LL x,y;
    tie(y,x)=extgcd(b, a%b);
    y-=a/b*x;
    return pair<LL,LL>(x, y);
}

//ループに展開してやるぜ
pair<LL, LL> extgcd(const LL a, const LL b)
{
    LL c0=a;
    LL c1=b;
    LL x0=1;
    LL x1=0;
    LL y0=0;
    LL y1=1;
    while (c1>0)
    {
        LL s=c0/c1;
        LL c2=c0%c1;
        LL x2=x0-s*x1;
        LL y2=y0-s*y1;
        c0=c1;
        c1=c2;
        x0=x1;
        x1=x2;
        y0=y1;
        y1=y2;
    }
    return pair<LL, LL>(x0, y0);
}