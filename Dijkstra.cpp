/* verified AOJ Highway Express Bus
*使い方
Distを好きな辺のコストの型に置換する
Graph型変数vに辺の情報を与える
(v[a].push_back(PID(b,c))でaからbへコストcの有向辺を張る)
dijkstra(i,v):iから任意の点への最小コストのベクター
dijkstra(i,j,v):iからjへの最小コスト
dijkstra(i,j,v,path):iからjへの最小コスト+pathにiからjへの最小パス
dijkstra(i,v,preVec):iから任意の点への最小コストのベクター
get_path(i,j,preVec)でiからjへの最小パスが求まる
*/


#include <bits/stdc++.h>

using namespace std;

typedef int Dist;

struct Edge;
typedef vector<Edge> Edges;
typedef vector<Edges> Graph;
const int INF = 1e9;

struct Edge
{
    int to;
    Dist dist;
    Edge(int to_,Dist dist_):to(to_),dist(dist_)
    {}
    bool operator >(const Edge &ed)const
    {
        return dist > ed.dist;
    }
};

//Shortest cost from i to j.
vector<Dist> dijkstra(int i, const Graph &vertex, vector<int> &preVector)
{
    vector<Dist> shortest(vertex.size(), INF);
    priority_queue<Edge, vector<Edge>, greater<Edge> > que;

    que.push(Edge(i, 0));
    shortest[i]=0;

    Edge state(0,0), tmp(0,0), e(0,0);
    while(!que.empty())
    {
        state = que.top();
        que.pop();
        if(shortest[state.to] < state.dist) continue;
        for(Edge e : vertex[state.to])
        {
            if(shortest[e.to] > shortest[state.to] + e.dist)
            {
                shortest[e.to] = shortest[state.to] + e.dist;
                preVector[e.to] = state.to;
                tmp = Edge(e.to, shortest[e.to]);
                que.push(tmp);
            }
        }
    }
    return shortest;
}
vector<int> get_path(int i, int j, const vector<int> &preVector)
{
    vector<int> rev;
    rev.push_back(j);
    int p=j;
    while(p!=i)
    {
        p = preVector[p];
        rev.push_back(p);
    }
    reverse(rev.begin(),rev.end());
    return rev;
}

vector<Dist> dijkstra(int i, const Graph &vertex)
{
  vector<int> preVector(vertex.size());
  return dijkstra(i, vertex, preVector);
}
Dist dijkstra(int i,int j, const Graph &vertex)
{
  return dijkstra(i, vertex)[j];
}
Dist dijkstra(int i,int j, const Graph &vertex, vector<int> &path)
{
  vector<int> preVector(vertex.size());
  vector<Dist> shortest(dijkstra(i, vertex, preVector));
  path = get_path(i, j, preVector);
  return shortest[j];
}
