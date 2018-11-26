class NextCombination
{
	private:
		const int n,r;
		VB used;
		vector<int> point;
		int back=0;
		int c=0;
	public:
		NextCombination(const int n_, const int r_)
			:n(n_),r(r_),used(VB(n)),point(vector<int>(r)),back(0),c(1)
		{}
		bool end() const
		{
			return c<0;
		}
		vector<int> next()
		{
			c--;
			for(int j=n-1;j>=0;j--)
			{
				if(!used[j]) break;
				used[j]=false;
				c--;
				if(c<0) break;
				back=point[c]+1;
			}
			if(c<0) 
				return vector<int>(0);
			used[point[c]]=false;

			while(c<r)
			{
				point[c]=back;
				used[back++]=1;
				c++;
			}
			return point;
		}
};
class NextCombinationInt
{
	using LL=long long;
	private:
		const int n,r;
		LL used;
		vector<int> point;
		int back=0;
		int c=0;
	public:
		NextCombinationInt(const int n_, const int r_)
			:n(n_),r(r_),used(0),point(vector<int>(r)),back(0),c(1)
		{}
		bool end() const
		{
			return c<0;
		}
		LL next()
		{
			c--;
			for(int j=n-1;j>=0;j--)
			{
				if((used&(1<<j))==0) break;
				used&=~(((LL)1)<<j);
				c--;
				if(c<0) break;
				back=point[c]+1;
			}
			if(c<0) 
				return 0;
			used&=~(((LL)1)<<point[c]);

			while(c<r)
			{
				point[c]=back;
				used|=(1<<(back++));
				c++;
			}
			return used;
		}
};