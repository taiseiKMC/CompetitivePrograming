class Trie {
	static const size_t SIZE=64;
	static size_t ctoi(const char c)
	{
		return c-64;
	}
	
	public:
		unique_ptr<Trie> next[SIZE];
		Trie(){}
		void insert(const string &s, const size_t i)
		{
			if(i>=s.size())
				return;
			size_t k=ctoi(s[i]);
			if (!next[k])
				next[k] = make_unique<Trie>(Trie());
			next[k]->insert(s, i+1);
		}
		bool find(const string &s, const size_t i) const
		{
			if (i>=s.size())
				return true;
			size_t k=ctoi(s[i]);
			if (!next[k])
				return false;
			return next[k]->find(s, i+1);
		}
};
