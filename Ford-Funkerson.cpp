const int INF=1e9;
struct Edge
{
	int from,to,cost,rev;
};
class FlowNetwork
{
    public:
        FlowNetwork(int n):graph(vector<vector<Edge>>(n)),used(vector<char>(n))
        {}
        void add_Edge(int from, int to, int cap);
        int ford_fulkerson(int from, int to);

    private:
        vector<vector<Edge>> graph;
        vector<char> used;
        int dfs(int from, int to, int cap);
};
void FlowNetwork::add_Edge(int from, int to, int cap)
{
	graph[from].push_back((Edge){from,to,cap,(int)graph[to].size()});
	graph[to].push_back((Edge){to,from,0,(int)graph[from].size()-1});
}
int FlowNetwork::dfs(int from, int to, int f)
{
	if(from == to) return f;
	int d;
	used[from]=true;
	for(int i=0;i<graph[from].size();i++)
	{
		Edge &e=graph[from][i];
		if(used[e.to] || e.cost<=0) continue;
		d = dfs(e.to, to, min(e.cost, f));
		if(d>0)
		{
			e.cost -= d;
			graph[e.to][e.rev].cost += d;
			return d;
		}
	}
	return 0;
}
int FlowNetwork::ford_fulkerson(int from, int to)
{
	int flow=0;
	while(1)
	{
        fill(used.begin(),used.end(),0);
		int f=dfs(from,to,INF);
		if(f==0)break;
		flow += f;
	}
	return flow;
}
