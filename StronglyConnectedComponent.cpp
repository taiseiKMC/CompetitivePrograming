/*
verified by ARC030_C
Strongly Connected Component
*/

#include <bits/stdc++.h>
using namespace std;

struct Edge
{
    int to,from,cost;
    Edge(int to_,int from_,int weight_):to(to_),from(from_),cost(weight_)
    {}

};
typedef vector<Edge> Edges;
typedef vector<Edges> Graph;

const Graph reverse_edge_Graph(const Graph& g)
{
    Graph rg(g.size());
    for(auto&& es : g)
        for(auto&& e : es)
            rg[e.to].push_back(Edge(e.from,e.to,e.cost));
    return rg;
}

const tuple<vector<int>,vector< vector<int>>> SCC(const Graph& g) //StronglyConnectedComponent
{
    const int n = g.size();
    Graph rg(reverse_edge_Graph(g));
    vector<int> vs;
    vector<char> reach(n);  //vector<bool>
    vector<vector<int> > scc;
    vector<int> scc_vectormap(n);

    function<void(const int)> dfs = [&](const int v)
    {
        reach[v]=1;
        for(auto&& e : g[v])
            if(!reach[e.to]) dfs(e.to);
        vs.push_back(v);
    };
    function<void(const int, const int)> rdfs = [&](const int v, const int k)
    {
        reach[v]=1;
        scc[k].push_back(v);
        scc_vectormap[v]=k;
        for(auto&& e : rg[v])
            if(!reach[e.to]) rdfs(e.to, k);
    };

    for(int i=0;i<n;i++)
        if(!reach[i]) dfs(i);
    fill(reach.begin(), reach.end(), 0);
    int k=0;
    for(int i=vs.size()-1;i>=0;i--)
        if(!reach[vs[i]]) 
        {
            scc.push_back(vector<int>());
            rdfs(vs[i], k++);
        }
    return tuple<vector<int>, vector<vector<int>> >(scc_vectormap, scc);
}

const Graph DAG_from_SCC(int DAG_size,Graph& g, vector<int>& scc_vectormap)
{
    Graph dag(DAG_size);
    for(auto&& es : g)
        for(auto&& e : es)
        {
            if(scc_vectormap[e.from]==scc_vectormap[e.to]) continue;
            dag[scc_vectormap[e.from]].push_back(Edge(scc_vectormap[e.to], scc_vectormap[e.from], 0));
        }
    return dag;
}
