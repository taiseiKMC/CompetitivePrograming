
class UnionFind
{
    private:
        vector<int> parent;

    public:
        UnionFind(const int n):parent(vector<int>(n,-1))
        {}
        
        const int Find(const int p)
        {
            return parent[p] < 0 ? p : parent[p] = Find(parent[p]);
        }
        const void Merge(int p, int q);

        const bool Belong(const int p, const int q)
        {
            return Find(p) == Find(q);
        }
        const int GetSize(const int p)
        {
            return -parent[Find(p)];
        }
};

const void UnionFind::Merge(int p, int q)
{
    p=Find(p);
    q=Find(q);
    if(p==q) return;
    if(parent[p] < parent[q])
    {
        parent[p] += parent[q];
        parent[q]=p;
    }
    else
    {
        parent[q] += parent[p];
        parent[p]=q;
    }
}
