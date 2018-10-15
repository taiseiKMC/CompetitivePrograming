///
///lower_bound(t)...tを含む最小のSAの添字
///upper_bound(t)...tを含む最大のSAの添字+1
///lcp() ... 高さ配列
///
class SA
{
    private:
        int k;
        
    public:
        const string s;
        vector<int> sa;
        const size_t length;
        
        SA(const string &_s);
        
        const int size() const { return (int)length; }
        const bool contain(const string &t) const;
        const int lower_bound(const string &t) const;
        const int upper_bound(const string &t) const;
        const vector<int> lcp() const;
};

SA::SA(const string &_s):
s(_s), sa(_s.size()+1), length(_s.size())
{
    iota(sa.begin(), sa.end(),0);
    
    vector<int> rank(size()+1);
    for(int i=0;i<size();i++)
        rank[i]=s[i];
    rank[size()]=-1;
    
    auto sa_comparator=[&](int i, int j)
    {
        if(rank[i]!=rank[j]) return rank[i]<rank[j];
        int ri=i+k<=size()?rank[i+k]:-1;
        int rj=j+k<=size()?rank[j+k]:-1;
        return ri<rj;
    };
    
    for(k=1;k<=size();k*=2)
    {
        sort(sa.begin(), sa.end(), sa_comparator);
        
        vector<int> tmp(size()+1);
        tmp[sa[0]]=0;
        for(int i=1;i<=size();i++)
        {
            tmp[sa[i]]=tmp[sa[i-1]]+(sa_comparator(sa[i-1],sa[i])?1:0);
        }
        rank=move(tmp);
    }
}
        
const bool SA::contain(const string &t) const
{
    int l=0,r=size()+1;
    while(r-l>1)
    {
        int mid=(l+r)/2;
        if(s.compare(sa[mid], t.size(), t)<0) l=mid;
        else r=mid;
    }
    return s.compare(sa[r], t.size(), t)==0;
}


const int SA::lower_bound(const string &t) const
{
    int l=0,r=size()+1;
    while(r-l>1)
    {
        int mid=(l+r)/2;
        if(s.compare(sa[mid], t.size(), t)<0) l=mid;
        else r=mid;
    }
    return r;
}

const int SA::upper_bound(const string &t) const
{
    int l=0,r=size()+1;
    while(r-l>1)
    {
        int mid=(l+r)/2;
        if(s.compare(sa[mid], t.size(), t)<=0) l=mid;
        else r=mid;
    }
    return r;
}

const vector<int> SA::lcp() const
{
    vector<int> rank(size()+1), lcp(size()+1);
    for(int i=0;i<=size();i++)
        rank[sa[i]]=i;
    
    int h=0;
    lcp[0]=0;
    for(int i=0;i<size();i++)
    {
        int j=sa[rank[i]-1];
        
        if(h>0) h--;
        for(; j+h<size()&&i+h<size(); h++)
            if(s[j+h] != s[i+h]) break;
            
        lcp[rank[i]-1]=h;
    }
    
    return lcp;
}