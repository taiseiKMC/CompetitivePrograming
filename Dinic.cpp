//set INF
using FLOW = long long;
struct Edge
{
    int to;
    FLOW cap;
    int rev;
};
class FlowNetwork
{
    public:
        FlowNetwork(int n):graph(vector<vector<Edge>>(n)),iter(vector<int>(n)),level(vector<int>(n))
        {}
        void add_Edge(int from, int to, FLOW cap);
        FLOW dinic(int from, int to);

    private:
        vector<vector<Edge>> graph;
        vector<int> iter, level;
        FLOW dfs(int from, int to, FLOW cap);
        void bfs(int from);
};
void FlowNetwork::add_Edge(int from, int to, FLOW cap)
{
    //cout<<from<<":"<<to<<" "<<cap<<endl;
	graph[from].push_back((Edge){to,cap,(int)graph[to].size()});
	graph[to].push_back((Edge){from,0,(int)graph[from].size()-1});
}

void FlowNetwork::bfs(int from)
{
    fill(level.begin(),level.end(),-1);
    queue<int> que;
    level[from]=0;
    que.push(from);
    while(!que.empty())
    {
        int v=que.front();
        que.pop();
        for(int i=0;i<graph[v].size();i++)
        {
            Edge &e=graph[v][i];
            if(e.cap>0 && level[e.to]<0)
            {
                level[e.to] = level[v]+1;
                que.push(e.to);
            }
        }
    }
}

FLOW FlowNetwork::dfs(int from, int to, FLOW f)
{
	if(from == to) return f;
	for(int &i=iter[from];i<graph[from].size();i++)
	{
		Edge &e=graph[from][i];
        if(e.cap > 0 && level[from] < level[e.to])
        {
            FLOW d = dfs(e.to, to, min(e.cap, f));
            if(d>0)
            {
                e.cap -= d;
                graph[e.to][e.rev].cap += d;
                return d;
            }
        }
	}
	return 0;
}
FLOW FlowNetwork::dinic(int from, int to)
{
    FLOW flow=0;
	while(1)
	{
        bfs(from);
        if(level[to]<0) return flow;
        fill(iter.begin(),iter.end(),0);
        FLOW f;
        while((f=dfs(from,to,INF))>0)
        {
            flow+=f;
        }
	}
}