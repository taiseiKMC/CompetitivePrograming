/*
蟻本p300
*/
template<typename T, bool Cmp(T,T)>
class Slide
{
	private:
		vector<T> minT;
		int k;
	public:
		Slide(const vector<T> &ary, const int k_):k(k_)
		{
			minT=vector<T>(ary.size()+k-1);
			deque<size_t> deq;
			for(size_t i=0;i<ary.size()+k-1;i++)
			{
				if(i < ary.size())
				{
					//while(!deq.empty() && ary[deq.back()] >= ary[i])
					while(!deq.empty() && !Cmp(ary[deq.back()], ary[i]))
						deq.pop_back();
					deq.push_back(i);
				}

				minT[i] = ary[deq.front()];
				if(i-k+1 < 0) continue;
				if(deq.front()==i-k+1)
					deq.pop_front();
			}
		}
		
		//-(k-1) <= index && index < (int)minT.size()-(k-1)
		T operator[] (int index) const
		{
			index+=k-1;
			assert(0 <= index && index < (int)minT.size());
			return minT[index];
		}
};

template<typename T>
bool lt(const T lhs, const T rhs) { return lhs < rhs; }

template <typename T>
using SlideMin = Slide<T, lt>;