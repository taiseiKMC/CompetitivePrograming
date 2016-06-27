/* verified AOJ1327 One-Dimensional Cellular Automaton
~Matrix<T>~
行列の簡単な計算ができる c++11
vector iostream cmath 必須
Matrix a(VV<T>), b(VV<T>);
行列式:a*b
スカラ積:l*a
和:a+b
差:a-b
転置:a.transport()
余因子:a.cofactor()
行列式:a.det()
aのi行j列目:a.get(i,j)　
aのi行目j列目にkを代入:set(i,j,k)
もしくはa[i][j] (a[i][j]=k)
aのx乗:a.pow(x)
ランク:a.rank()
n*n基本行列E:Matrix(n)
m*n 0行列:Matrix(m,n)
m*n 全要素がpの行列:Matrix(m,n,p)
Row<T>型rowでの初期化(m*1行列):Matrix(row)

**(整数不可)**
三角化:a.triangulate()
逆行列:a.inverse()
(逆行列のa.det()倍ならa.pre_inverse()で求まる 整数可)
***
*/

#include <vector>
#include <iostream>
#include <cmath>
using namespace std;

template<class T>
using Row = vector<T>;
template<class T>
using VV = vector<Row<T> >;

typedef long double ld;
const ld EPS = 1e-11;


template<class T>
struct Matrix {
    VV<T> matrix;
    int n,m;

    Matrix(const VV<T> &matrix_);
    explicit Matrix(int n_);
    explicit Matrix(const Row<T> &row);
    Matrix(int m_, int n_, T e=0);

    const T get(const int i, const int j) const;
    void set(const int x, const int y, const T k);

    const Matrix<T> transport() const;

    const Matrix<T> operator + (const Matrix<T> &rhs) const;
    const Matrix<T> operator * (const Matrix<T> &rhs) const;
    const Matrix<T> operator - (const Matrix<T> &rhs) const;
    Matrix<T>& operator += (const Matrix<T> &rhs);
    Matrix<T>& operator *= (const Matrix<T> &rhs);
    Matrix<T>& operator -= (const Matrix<T> &rhs);

    Row<T>& operator[](const int x);
    template<class U> operator Matrix<U> () const;

    const Matrix<T> pow(int x) const;
    const Matrix<T> cofactor(int x, int y) const;
    const T det() const;
    const int rank() const;

    //逆行列が存在すれば、(行列式)*(逆行列)を返す
    //A:matrix,return det A * A^-1
    const Matrix<T> pre_inverse() const;

    /*template */const Matrix<ld> triangulate() const;
    /*template */const Matrix<ld> inverse() const;
};
template<class T>
const Matrix<T> operator * (const T lambda, const Matrix<T> &rhs)
{
    Matrix<T> tmp(rhs);
    for(int i=0; i<rhs.m; i++)
        for(int j=0; j<rhs.n; j++)
            tmp.set(i, j, tmp.get(i,j) * lambda);
    return tmp;
}

template<class T>
Matrix<T>::Matrix(const VV<T> &matrix_):matrix(matrix_)
{
    m=matrix_.size();
    if(m == 0) n = 0;
    else n = matrix_[0].size();
}

template<class T>
Matrix<T>::Matrix(int n_):m(n_),n(n_)
{
  matrix = VV<T>(n, Row<T>(n, 0));
  for (int i = 0; i < n; ++i)
    set(i,i,1);
}

template<class T>
Matrix<T>::Matrix(const Row<T> &row):m(1), n(row.size()), matrix(VV<T>(1, row))
{
  //sizeがmのvector<T>からmx1行列の生成
  (*this) = transport();
}

template<class T>
Matrix<T>::Matrix(int m_, int n_, T e):m(m_),n(n_)
{
  matrix = VV<T>(m, Row<T>(n, e));
}

template<class T>
const T Matrix<T>::get(const int i, const int j) const
{
    if(0<=i && i<m && 0<=j && j<n)
        return matrix[i][j];

    cerr << "get(" << i << "," << j<< ")is not exist." << endl;
    throw;
}

template<class T>
void Matrix<T>::set(const int x, const int y, const T k)
{
    if(0<=x && x<m && 0<=y && y<n)
    {
      *(matrix[x].begin()+y) = k;
      return;
    }
    cerr << "set(" << x << "," << y<< ")is not exist." << endl;
    throw;
}

template<class T>
const Matrix<T> Matrix<T>::transport() const
{
    VV<T> tmp;
    for(int i=0; i<n; i++)
    {
        Row<T> row;
        for(int j=0; j<m; j++)
            row.push_back(get(j,i));
        tmp.push_back(row);
    }
    return tmp;
}

template<class T>
const Matrix<T> Matrix<T>::operator + (const Matrix<T> &rhs) const
{
    if(m != rhs.m || n != rhs.n)
    {
        cerr << "Matrix Add can not calculate." << endl;
        throw;
    }

    Matrix<T> tmp(m, n, 0);
    for(int i=0; i<m; i++)
    {
        for(int j=0; j<n; j++)
        {
            tmp.set(i, j, get(i,j)+rhs.get(i,j));
        }
    }
    return tmp;
}

template<class T>
const Matrix<T> Matrix<T>::operator * (const Matrix<T> &rhs) const
{
    if(n != rhs.m)
    {
        cerr << "Matrix Product can not calculate." << endl;
        throw;
    }
    Matrix<T> tmp(m, rhs.n, 0);
    T sum;
    for(int i=0; i<m; i++)
        for(int j=0; j<rhs.n; j++)
        {
            sum=0;
            for(int k=0; k<n; k++)
            {
                sum += get(i,k)*rhs.get(k,j);
            }
            tmp.set(i, j, sum);
        }
    return tmp;
}

template<class T>
const Matrix<T> Matrix<T>::operator - (const Matrix<T> &rhs) const
{
    return *this+((T)-1 * rhs);
}

template<class T>
Matrix<T>& Matrix<T>::operator += (const Matrix<T> &rhs)
{
    return *this = *this + rhs;
}

template<class T>
Matrix<T>& Matrix<T>::operator *= (const Matrix<T> &rhs)
{
  return *this = *this * rhs;
}

template<class T>
Matrix<T>& Matrix<T>::operator -= (const Matrix<T> &rhs)
{
    return *this = *this - rhs;
}

template<class T>
Row<T>& Matrix<T>::operator[](const int x)
{
  if(0<=x && x<m)
    return matrix[x];

    //Matrix[x][y]でｘが配列外のときのみこのエラーを吐く
  cerr<<"Matrix[" << m <<"] is not exist." << endl;
  throw;
}

template<class T> template <class U>
Matrix<T>::operator Matrix<U> () const
{
  Matrix<U> tmp(m,n);
  for(int i=0;i<m;i++)
    for(int j=0;j<n;j++)
      tmp.set(i,j,(U)get(i,j));
  return tmp;
}

template<class T>
const Matrix<T> Matrix<T>::pow(int x) const
{
  Matrix<T> tmp(*this), e(m);
  for(int i=1;i<=x;i<<=1)
  {
    if((x&i)>0)
      e=e*tmp;
    tmp=tmp*tmp;
  }
  return e;
}

template<class T>
const Matrix<T> Matrix<T>::cofactor(int x, int y) const
{
  VV<T> tmp;
  for(int i=0;i<m;i++)
  {
    if(x==i) continue;
    Row<T> row;
    for(int j=0;j<n;j++)
    {
      if(y==j) continue;
      row.push_back(get(i,j));
    }
    tmp.push_back(row);
  }
  return Matrix<T>(tmp);
}

template<class T>
const T Matrix<T>::det() const
{
  if(n!=m)
  {
    cerr << "det can not calculate." << endl;
    throw;
  }
  if(m==1)
    return get(0,0);
  T sum=0;
  for(int i=0;i<m;i++)
  {
    sum += ((i%2==0?1:-1)*get(i,0))*Matrix<T>(cofactor(i,0)).det();
  }
  return sum;
}

template<class T>
const int Matrix<T>::rank() const
{
  Matrix<ld> tmp( ((Matrix<ld>)*this).triangulate());
  for(int i=tmp.m-1;i>=0;i--)
  {
    if(abs(tmp.get(i,tmp.n-1)) < EPS)
      continue;
    return i+1;
  }
  return 0;
}

template<class T>
const Matrix<T> Matrix<T>::pre_inverse() const
{
  if(m!=n)
  {
    cerr<<"inverse can not calculate." << endl;
    throw;
  }

  Matrix<T> tmp(m,n,0);
  for(int i=0;i<m;i++)
    for(int j=0;j<n;j++)
      tmp.set(i, j, ((i+j)%2==0?1:-1)*cofactor(i,j).det());
  return tmp.transport();
}

template<> const Matrix<ld> Matrix<ld>::triangulate() const
{
  Matrix<ld> tmp(*this);
  ld e;
  for(int i=0;i<m;i++)
  {
    if(abs(tmp.get(i,i)) < EPS)
    {
      tmp.set(i,i,0);
      continue;
    }
    for(int j=i+1;j<m;j++)
    {
      e = tmp.get(j,i) / tmp.get(i,i);
      for(int k=0;k<n;k++)
      tmp.set(j, k, tmp.get(j,k) - tmp.get(i,k) * e);
    }
  }
  return tmp;
}

template<> const Matrix<ld> Matrix<ld>::inverse() const
{
  Matrix<ld> tmp(pre_inverse());
  ld e = det();
  if(e==0)
  {
    cerr<<"0 div error in inverse()." << endl;
    throw;
  }
  tmp=1/e*tmp;
  return tmp;
}
