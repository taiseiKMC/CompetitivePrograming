/*
~Matrix~
行列の簡単な計算ができる
vector iostream cmath cassert 必須
Matrix a(VV), b(VV);
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
aのx乗:a.pow(x) (verified AOJ1327 One-Dimensional Cellular Automaton)
n*n基本行列E:Matrix(n)
m*n 0行列:Matrix(m,n)
m*n 全要素がpの行列:Matrix(m,n,p)
Row型rowでの初期化(m*1行列):Matrix(row)

**(整数不可)**
三角化:a.triangulate()
ランク:a.rank()
逆行列:a.inverse()
//(逆行列のa.det()倍ならa.pre_inverse()で求まる 整数可)
ガウスの消去法:a.row_reduction()
//連立一次方程式が解ける
***
*/

#include <vector>
#include <iostream>
#include <cmath>
#include <cassert>
using namespace std;

typedef double Elem;
typedef vector<Elem> Row;
typedef vector<Row> VV;

typedef long double ld;
const ld EPS = 1e-11;

const bool is_zero(const Elem e) {
  return abs(e) < EPS;
}

struct Matrix {
  VV matrix;
  int n, m;

  Matrix(const VV &matrix_);
  explicit Matrix(int n_);
  explicit Matrix(const Row &row);
  Matrix(int m_, int n_);
  Matrix(int m_, int n_, Elem e);

  const Elem get(const int i, const int j) const;
  void set(const int x, const int y, const Elem k);

  const Matrix operator + (const Matrix &rhs) const;
  const Matrix operator * (const Matrix &rhs) const;
  const Matrix operator - (const Matrix &rhs) const;
  Matrix &operator += (const Matrix &rhs);
  Matrix &operator *= (const Matrix &rhs);
  Matrix &operator -= (const Matrix &rhs);

  Row &operator[](const int x);

  const Matrix transport() const;
  const Matrix pow(int x) const;
  const Matrix cofactor(int x, int y) const;
  const Elem det() const;

  const Matrix triangulate() const;
  const int rank() const;

  //逆行列が存在すれば、(行列式)*(逆行列)を返す
  //A:matrix,return det A * A^-1
  const Matrix pre_inverse() const;
  const Matrix inverse() const;
  const Matrix row_reduction() const;
};

const Matrix operator * (const Elem lambda, const Matrix &rhs) {
  Matrix tmp(rhs);
  for (int i = 0; i < rhs.m; i++)
    for (int j = 0; j < rhs.n; j++)
      tmp.set(i, j, tmp.get(i, j) * lambda);
  return tmp;
}

Matrix::Matrix(const VV &matrix_): matrix(matrix_) {
  m = matrix_.size();
  if (m == 0) n = 0;
  else n = matrix_[0].size();
}
Matrix::Matrix(int n_): m(n_), n(n_) {
  matrix = VV(n, Row(n, 0));
  for (int i = 0; i < n; ++i)
    set(i, i, 1);
}
Matrix::Matrix(const Row &row): m(1), n(row.size()), matrix(VV(1, row)) {
  //sizeがmのvector<Elem>からmx1行列の生成
  (*this) = transport();
}
Matrix::Matrix(int m_, int n_): m(m_), n(n_) {
  matrix = VV(m, Row(n, 0));
}
Matrix::Matrix(int m_, int n_, Elem e): m(m_), n(n_) {
  matrix = VV(m, Row(n, e));
}

const Elem Matrix::get(const int i, const int j) const {
  if (0 <= i && i < m && 0 <= j && j < n)
    return matrix[i][j];

  cerr << "get(" << i << "," << j << ")is not exist." << endl;
  throw;
}
void Matrix::set(const int i, const int j, const Elem k) {
  if (0 <= i && i < m && 0 <= j && j < n) {
    *(matrix[i].begin() + j) = k;
    return;
  }
  cerr << "set(" << i << "," << j << ")is not exist." << endl;
  throw;
}

const Matrix Matrix::operator + (const Matrix &rhs) const {
  assert(m == rhs.m && n == rhs.n);

  Matrix tmp(m, n, 0);
  for (int i = 0; i < m; i++) {
    for (int j = 0; j < n; j++) {
      tmp.set(i, j, get(i, j) + rhs.get(i, j));
    }
  }
  return tmp;
}

const Matrix Matrix::operator * (const Matrix &rhs) const {
  assert(n == rhs.m);

  Matrix tmp(m, rhs.n, 0);
  Elem sum;
  for (int i = 0; i < m; i++)
    for (int j = 0; j < rhs.n; j++) {
      sum = 0;
      for (int k = 0; k < n; k++) {
        sum += get(i, k) * rhs.get(k, j);
      }
      tmp.set(i, j, sum);
    }
  return tmp;
}

const Matrix Matrix::operator - (const Matrix &rhs) const {
  return *this + ((Elem) - 1 * rhs);
}

Matrix &Matrix::operator += (const Matrix &rhs) {
  return *this = *this + rhs;
}

Matrix &Matrix::operator *= (const Matrix &rhs) {
  return *this = *this * rhs;;
}

Matrix &Matrix::operator -= (const Matrix &rhs) {
  return *this = *this - rhs;
}

Row &Matrix::operator[](const int x) {
  return matrix[x];
}

const Matrix Matrix::transport() const {
  VV tmp;
  for (int i = 0; i < n; i++) {
    Row row;
    for (int j = 0; j < m; j++)
      row.push_back(get(j, i));
    tmp.push_back(row);
  }
  return tmp;
}

const Matrix Matrix::pow(int x) const {
  Matrix tmp(*this), e(m);
  for (int i = 1; i <= x; i <<= 1) {
    if ((x & i) > 0)
      e = e * tmp;
    tmp = tmp * tmp;
  }
  return e;
}

const Matrix Matrix::cofactor(int x, int y) const {
  VV tmp;
  for (int i = 0; i < m; i++) {
    if (x == i) continue;
    Row row;
    for (int j = 0; j < n; j++) {
      if (y == j) continue;
      row.push_back(get(i, j));
    }
    tmp.push_back(row);
  }
  return Matrix(tmp);
}

const Elem Matrix::det() const {
  assert(n == m);

  if (m == 1)
    return get(0, 0);
  Elem sum = 0;
  for (int i = 0; i < m; i++) {
    sum += ((i % 2 == 0 ? 1 : -1) * get(i, 0)) * Matrix(cofactor(i, 0)).det();
  }
  return sum;
}

const Matrix Matrix::triangulate() const {
  Matrix tmp(*this);
  Elem e;
  int p = 0;
  for (int i = 0; i < m && p < n; i++, p++) {
    if (is_zero(tmp.get(i, p))) {
      tmp.set(i, p, 0);
      bool flag = true;
      for (int j = i + 1; j < m; j++)
        if (!is_zero(tmp.get(j, p))) {
          for (int k = 0; k < n; k++)
            tmp.set(i, k, tmp.get(i, k) + tmp.get(j, k));
          //tmp[i].swap(tmp[j]);
          flag = false;
          break;
        }
      if (flag) {
        i--;
        continue;
      }
    }
    for (int j = i + 1; j < m; j++) {
      e = tmp.get(j, p) / tmp.get(i, p);
      for (int k = 0; k < n; k++)
        tmp.set(j, k, tmp.get(j, k) - tmp.get(i, k) * e);
    }
  }
  return tmp;
}

const int Matrix::rank() const {
  Matrix tmp(triangulate());
  for (int i = min(tmp.m - 1, tmp.n - 1); i >= 0; i--) {
    for (int j = tmp.n - 1; j >= i; j--)
      if (is_zero(tmp.get(i, j)))
        continue;
      else
        return i + 1;
  }
  return 0;
}

const Matrix Matrix::pre_inverse() const {
  assert(m == n);

  Matrix tmp(m, n, 0);
  for (int i = 0; i < m; i++)
    for (int j = 0; j < n; j++)
      tmp.set(i, j, ((i + j) % 2 == 0 ? 1 : -1)*cofactor(i, j).det());
  return tmp.transport();
}

/*O(n!)実装
const Matrix Matrix::inverse() const {
  Matrix tmp(pre_inverse());
  Elem e = det();

  assert(!is_zero(e));

  tmp = 1 / e * tmp;
  return tmp.transport();
}*/

const Matrix Matrix::inverse() const {
  assert(m == n);

  Matrix tmp(m, n * 2), tmp2(m, n);
  for (int i = 0; i < m; i++)
    for (int j = 0; j < n; j++)
      tmp.set(i, j, get(i, j));
  for (int i = 0; i < m; i++)
    tmp.set(i, i + n, 1);

  tmp = tmp.row_reduction();

  //逆行列が存在するかどうかのチェック
  for (int i = 0; i < m; i++)
    assert(is_zero(tmp.get(i, i) - 1));

  for (int i = 0; i < m; i++)
    for (int j = 0; j < n; j++)
      tmp2.set(i, j, tmp.get(i, j + n));

  return tmp2;
}

const Matrix Matrix::row_reduction() const {
  Matrix tmp(*this);
  Elem e;
  int p = 0;
  for (int i = 0; i < m && p < n; i++, p++) {
    if (is_zero(tmp.get(i, p))) {
      tmp.set(i, p, 0);
      bool flag = true;
      for (int j = i + 1; j < m; j++)
        if (!is_zero(tmp.get(j, p))) {
          for (int k = 0; k < n; k++)
            tmp.set(i, k, tmp.get(i, k) + tmp.get(j, k));
          //tmp[i].swap(tmp[j]);
          flag = false;
          break;
        }
      if (flag) {
        i--;
        continue;
      }
    }
    e = 1 / tmp.get(i, p);
    tmp.set(i, p, 1);
    for (int k = i + 1; k < n; k++)
      tmp.set(i, k, tmp.get(i, k)*e);
    for (int j = 0; j < m; j++) {
      if (i == j) continue;
      e = tmp.get(j, p);
      for (int k = 0; k < n; k++)
        tmp.set(j, k, tmp.get(j, k) - tmp.get(i, k) * e);
    }
  }
  return tmp;
}
