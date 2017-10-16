
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


//
// Kruskal algorithm
//

struct Edge{
int start,end,dis;
bool operator >(const Edge &b)const{return dis > b.dis;}
};
using EdgeQueue=priority_queue<Edge,vector<Edge> ,greater<Edge> >;

const EdgeQueue convertGraph(const vector<vector<Edge> > &g)
{
	EdgeQueue que;
	for(auto&& es:g)
		for(auto&& e:es)
			que.push(e);
	return que;
}

const vector<Edge> Kruskal(const int n, EdgeQueue que)
{
	UnionFind belong(n);
	
	Edge edge;
	vector<Edge> min_cost_tree;
	while(!que.empty())
	{
		edge=que.top();
		if(belong.Find(edge.start)!=belong.Find(edge.end))
		{
			belong.Merge(edge.start, edge.end);
					
			min_cost_tree.push_back(edge);
		}
		que.pop();
	}
	return min_cost_tree;
}