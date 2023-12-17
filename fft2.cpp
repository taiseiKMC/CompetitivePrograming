/*
FFT Library
written by nagisa
verified AOJ 2560:Point Distance

*/

#include <bits/stdc++.h>

using namespace std;

const double PI=atan2(0,-1);
typedef complex<double> Comp;

const bool is_pow_of2(const int n) 
{
	int a = 1;
	while (n >= a) 
    {
		if (n == a)
			return true;
		a <<= 1;
	}
	return false;
}

const int least_exp_of2(const int p)
{
    int tmp=1;
    while(p>tmp)
        tmp<<=1;
    return tmp;
}

const vector<Comp> fft(const vector<Comp> &p) 
{
    vector<Comp> v(p);
	int n = v.size();
	assert(is_pow_of2(n));

	for (int j = 1, i = 0; j < n - 1; j++) 
    {
		for (int k = n >> 1; k >(i ^= k); k >>= 1);
		if (j < i) 
            swap(v[i], v[j]);
	}

	for (int m = 2; m <= n; m <<= 1) 
    {
		double deg = (-1) * 2 * PI / m;
		Comp r(cos(deg), sin(deg));
		for (int i = 0;i < n;i += m) 
        {
			Comp w(1, 0);
			for (int j = i, k = i + m / 2;k < i + m;j++, k++) 
            {
				Comp t1 = v[j], t2 = w*v[k];
				v[j] = t1 + t2, v[k] = t1 - t2;
				w *= r;
			}
		}
	}
	return v;
}

const vector<Comp> ifft(const vector<Comp> &p) 
{
    vector<Comp> v(p);
	int n = v.size();
	assert(is_pow_of2(n));
	for (int j = 1, i = 0; j < n - 1; j++) 
    {
		for (int k = n >> 1; k >(i ^= k); k >>= 1);
		if (j < i) 
            swap(v[i], v[j]);
	}
	for (int m = 2; m <= n; m <<= 1) 
    {
		double deg = 2 * PI / m;
		Comp r(cos(deg), sin(deg));
		for (int i = 0;i < n;i += m) 
        {
			Comp w(1, 0);
			for (int j = i, k = i + m / 2;k < i + m;j++, k++) 
            {
				Comp t1 = v[j], t2 = w*v[k];
				v[j] = t1 + t2, v[k] = t1 - t2;
				w *= r;
			}
		}
	}
	for (int i = 0; i < n;++i) 
        v[i] *= 1.0 / n;
	return v;
}

const vector<Comp> convolution(const vector<Comp> &P, const vector<Comp> &Q)
{
    vector<Comp> p(least_exp_of2(P.size())*2), q(least_exp_of2(Q.size())*2);
    copy(P.begin(), P.end(), p.begin());
    copy(Q.begin(), Q.end(), q.begin());
	p=fft(p);
    q=fft(q);
    vector<Comp> tmp(p.size());
	for (int i = 0;i < tmp.size();i++) 
        tmp[i] = p[i] * q[i];
	return ifft(tmp);
}
