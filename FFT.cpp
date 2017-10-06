/*
verified AtCoder FFT
*/

#include <bits/stdc++.h>

using namespace std;

const bool is_pow2(const int p)
{
	int t=1;
	while(t<p)
		t<<=1;
	return t==p;
}

const double PI=atan2(0,-1);
using Complex=complex<double>;
class FT
{
    private:
        //FFTの順方向、逆方向をまとめた関数
		template<class T> 
		static const vector<Complex> FFT_with_flag(const vector<T>& ary, const bool inverse=false);
        
        //1要素のvector<Complex>を作成する
        //目的はTとcomplex<T>のマッチング
		template<class T> 
		static const vector<Complex> create_complex(const complex<T> x)
        {
            return vector<Complex>(1, Complex(x));
        } 
		template<class T> 
		static const vector<Complex> create_complex(const T x)
        {
            return vector<Complex>(1, Complex(x, 0));
        } 
    public:
        /*
            FFTの引数は2の冪で、配列の長さの半分まで正しく変換できることが保証される
            余った要素は0で埋める
        */
		template<class T> 
		static const vector<Complex> FFT(const vector<T>& ary)
        {
            return FFT_with_flag(ary,false);
        }
		template<class T> 
		static const vector<Complex> IFFT(const vector<T>& ary)
        {
            auto iary=FFT_with_flag(ary, true);
            for(int i=0;i<(int)iary.size();i++)
                iary[i]/=iary.size();
            return iary;
        }
    
};

template<class T> 
const vector<Complex> FT::FFT_with_flag(const vector<T>& ary, const bool inverse)
{
    int n=ary.size();
    if(n==1)
        return create_complex(ary[0]);

    //2の冪でなければいけない
    assert(is_pow2(n));

    //偶数成分のフーリエ変換
    vector<T> ev(n/2);
    for(int i=0;i<n/2;i++)
        ev[i]=ary[i*2];
    auto even=FFT_with_flag(ev, inverse);

    //奇数成分のフーリエ変換
    vector<T> od(n/2);
    for(int i=0;i<n/2;i++)
        od[i]=ary[i*2+1];
    auto odd = FFT_with_flag(od, inverse);

    //以下合性
    if(inverse)
        for(int i=0;i<n/2;i++)
            odd[i]*=Complex(cos(2*PI*i/n), sin(2*PI*i/n));
    else
        for(int i=0;i<n/2;i++)
            odd[i]*=Complex(cos(2*PI*i/n), -sin(2*PI*i/n));
    
    vector<Complex> ret(n);
    for(int i = 0; i < n/2; i++)
        ret[i] = even[i] + odd[i];
    for(int i = 0; i < n/2; i++)
        ret[i+n/2] = even[i] - odd[i];
    return ret;
}

const int least_pow2(const int p)
{
	int t=1;
	while(t<p)
		t<<=1;
	return t;
} 

//畳み込み計算
template<class T>
vector<Complex> convolution(vector<T> a, vector<T> b) {
    vector<Complex> p(least_pow2(a.size())*2), q(least_pow2(b.size())*2);
    copy(a.begin(), a.end(), p.begin());
    copy(b.begin(), b.end(), q.begin());

	auto pf = FT::FFT(p), qf = FT::FFT(q);
	for (int i = 0;i < pf.size();i++) pf[i] *= qf[i];
    return FT::IFFT(pf);
}