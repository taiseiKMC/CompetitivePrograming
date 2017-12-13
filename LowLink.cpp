/*
verified by AOJ2172 Escape
LowLink
*/

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


struct Edge
{
    int to,from,cost;
    Edge(int to_,int from_,int weight_):to(to_),from(from_),cost(weight_)
    {}

};
typedef vector<Edge> Edges;
typedef vector<Edges> Graph;

const tuple<vector<int>,vector<int>> lowlink(const Graph &g)
{
	int n=g.size();
	vector<int> order(n), low(n);
	vector<char> reach(n);
	int cnt=0;
	function<int(const int,const int)> dfs=[&](const int v,const int pv)
	{
		if(reach[v]) return order[v];
		reach[v]=true;
		order[v]=cnt;
		int mlow=cnt;
		cnt++;
		for(auto&& e:g[v])
		{
			if(e.to==pv) continue;
			mlow=min(mlow, dfs(e.to, v));
		}
		return low[v]=mlow;
	};
	dfs(0, -1);
	return make_tuple(order, low);
	//order[e.from]<low[e.to]のとき、eは橋
}

//縮約後の頂点番号,頂点数
const tuple<vector<int>, int> biconnectedComponent(const Graph &g)
{
	int n=g.size();
	vector<int> ord, low;
	tie(ord, low)=lowlink(g);
	UnionFind uf(n);
	for(int i=0;i<n;i++)
		for(auto&& e:g[i])
			if(!(ord[e.from]<low[e.to] || ord[e.to]<low[e.from]))
				uf.Merge(e.from, e.to);
	vector<int> bc_map(n,-1);
	int k=0;
	REP(i,n)
	{
		int j=uf.Find(i);
		if(bc_map[j]==-1)
			bc_map[j]=k++;
		bc_map[i]=bc_map[j];
	}
	return make_tuple(bc_map, k);
}

const Graph tree_from_BC(int tree_size, const Graph &g, const vector<int> &bc_map)
{
    Graph ret(tree_size);
	for(int i=0;i<g.size();i++)
		for(auto&& e:g[i])
			if(bc_map[e.from]!=bc_map[e.to])
				ret[bc_map[e.from]].push_back(Edge(bc_map[e.to], bc_map[e.from], e.cost));
    return ret;
}