//O(FElogV)
using Flow=int;
using Cost=int;
struct Edge{
	int to, rev;
	Flow cap;
	Cost cost;
};

using Edges=vector<Edge>;
using Graph=vector<Edges>;
class MinCostFlow
{
	public:
		vector<vector<Edge>> g;
		MinCostFlow(const int n):g(vector<vector<Edge>>(n))
		{}
		const void add_Edge(const int from, const int to, const Flow cap, const Cost cost)
		{
			g[from].push_back((Edge){to, (int)g[to].size(), cap, cost});
			g[to].push_back((Edge){from, (int)g[from].size()-1, 0, -cost});
		}
		const Cost primal_dual(const int s, const int t, Flow f);
	private:
		const Cost INF=1e9;
};

const Cost MinCostFlow::primal_dual(const int s, const int t, Flow f)
{
	int n=g.size();
	vector<Cost> h(n), dist(n);
	vector<int> prevv(n), preve(n);
	using P=pair<Cost, int>;

	Cost res=0;
	fill(ALL(h), 0);
	while(f>0)
	{
		priority_queue<P, vector<P>, greater<P>> que;
		fill(ALL(dist), INF);
		dist[s]=0;
		que.push(P(0, s));
		while(!que.empty())
		{
			P p=que.top();
			que.pop();
			int v=p.second;
			if(dist[v] < p.first) continue;
			for(int i=0;i<g[v].size();i++)
			{
				Edge &e=g[v][i];
				if(e.cap>0 && dist[e.to] > dist[v] + e.cost + h[v] - h[e.to])
				{
					dist[e.to] = dist[v] + e.cost + h[v] - h[e.to];
					prevv[e.to] = v;
					preve[e.to] = i;
					que.push(P(dist[e.to], e.to));
				}
			}
		}
		
		if(dist[t]==INF)
			return -1;
		for(int v=0;v<n;v++) h[v] += dist[v];

		Flow d=f;
		for(int v=t;v!=s;v=prevv[v])
		{
			d=min(d, g[prevv[v]][preve[v]].cap);
		}
		f-=d;
		res+=d*h[t];
		for(int v=t;v!=s;v=prevv[v])
		{
			Edge &e=g[prevv[v]][preve[v]];
			e.cap-=d;
			g[v][e.rev].cap+=d;
		}
	}
	return res;
}