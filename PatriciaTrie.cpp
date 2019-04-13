class PatriciaTrie {
	public:
		bool end=0;
		map<string, unique_ptr<PatriciaTrie>> next;
		PatriciaTrie(bool end__=false):end(end__){}
		void insert(const string &s, const size_t i)
		{
			if(i==s.size())
			{
				end=true;
				return;
			}
			auto l=next.lower_bound(string(1, s[i]));
			auto&& ls=l->first;
			if(l==next.end() || ls[0]!=s[i])
			{
				next[s.substr(i)]=make_unique<PatriciaTrie>(true);
				return;
			}
			for(int j=0; j<ls.size(); j++)
			{
				if(i+j<s.size() && s[i+j]==ls[j])
					continue;

				//ノード分割
				string pref=ls.substr(0,j);
				string ss=s.substr(i+j);
				string lss=ls.substr(j);
				bool ssemp=ss=="";
				next[pref]=make_unique<PatriciaTrie>(ssemp);
				if(!ssemp)
					next[pref]->next[ss]=make_unique<PatriciaTrie>(true);
				next[pref]->next[lss]=move(l->second);
				next.erase(l->first);
				return;
			}
			l->second->insert(s, i+ls.size());
		}
		bool find(const string &s, const size_t i) const
		{
			if(i==s.size())
				return end;
			auto l=next.lower_bound(string(1, s[i]));
			auto&& ls=l->first;
			string ssub=s.substr(i);
			if(l==next.end() || ls[0]!=s[i])
				return false;
			
			for(int j=0;j<ls.size();j++)
				if(i+j>=s.size() || ls[j]!=s[i+j]) return false;
			return l->second->find(s, i+ls.size());
		}
		void debug_print(int ls) const
		{
			string space(ls, ' ');
			bool f=0;
			if(end)
			{
				cout<<"\\";
				f=1;
			}
			for(auto&& e:next)
			{
				if(f)
					cout<<endl<<space;
				cout<<e.first<<" ";
				e.second->debug_print(ls + e.first.size()+1);
				f=1;
			}
		}
};
