#pragma once
#include <cmath>
#include <cstdlib>
#include <algorithm>
#include <iostream>
#include <map>
#include <random>
#include <string>

#define ChzF(a, b, c) for(int a = (b); a <= (c); a++)

namespace ChzEigen
{
#define MatrixTemplate Matrix<_Scalar, _Rows, _Cols, _MaxRows, _MaxCols>
#define MatrixTemplate_Trans Matrix<_Scalar, _Cols, _Rows, _MaxCols, _MaxRows>
#define DefMatrixTemplate template<typename _Scalar, int _Rows, int _Cols, int _MaxRows, int _MaxCols>

	template<typename _Scalar, int _Rows, int _Cols, int _MaxRows, int _MaxCols>
	class Matrix
	{
	private:
		int m, n;
		_Scalar data[_MaxRows][_MaxCols];
		void _Resetmn();
	public:
		Matrix();
		Matrix(int n1);
		Matrix(int m1, int n1);
		Matrix(double d1, double d2);
		Matrix(double d1, double d2, double d3);
		Matrix(double d1, double d2, double d3, double d4);
		int cols();
		int rows();

		static Matrix Identity();
		static Matrix Identity(int n1);
		static Matrix Identity(int m1, int n1);
		static Matrix Ones();
		static Matrix Ones(int n1);
		static Matrix Ones(int m1, int n1);
		static Matrix Zero();
		static Matrix Zero(int n1);
		static Matrix Zero(int m1, int n1);
		bool resize(int m1, int n1);
		bool resize(int n1);
		void setIdentity();
		void setOnes();
		void setZero();
		int size();

		MatrixTemplate_Trans transpose();
		MatrixTemplate_Trans inverse();

		DefMatrixTemplate friend Matrix operator+(Matrix m1, Matrix m2);
		DefMatrixTemplate friend Matrix operator+=(Matrix m1, Matrix m2);
		DefMatrixTemplate friend Matrix operator-(Matrix m1);
		DefMatrixTemplate friend Matrix operator-(Matrix m1, Matrix m2);
		DefMatrixTemplate friend Matrix operator-=(Matrix m1, Matrix m2);
		template<typename _Scalar, int _Rows1, int _Cols1, int _MaxRows1, int _MaxCols1, int _Rows2, int _Cols2, int _MaxRows2, int _MaxCols2>
		friend Matrix<_Scalar, _Rows1, _Cols2, _MaxRows1, _MaxCols2> operator*(Matrix<_Scalar, _Rows1, _Cols1, _MaxRows1, _MaxCols1> m1, Matrix<_Scalar, _Rows2, _Cols2, _MaxRows2, _MaxCols2> m2);
		template<typename _Scalar, int _Rows1, int _Cols1, int _MaxRows1, int _MaxCols1, int _Rows2, int _Cols2, int _MaxRows2, int _MaxCols2>
		friend Matrix<_Scalar, _Rows1, _Cols2, _MaxRows1, _MaxCols2> operator*=(Matrix<_Scalar, _Rows1, _Cols1, _MaxRows1, _MaxCols1> m1, Matrix<_Scalar, _Rows2, _Cols2, _MaxRows2, _MaxCols2> m2);
		DefMatrixTemplate friend Matrix operator*(Matrix m1, double d);
		DefMatrixTemplate friend Matrix operator*(double d, Matrix m1);
		DefMatrixTemplate friend Matrix operator*=(Matrix m1, double d);
		DefMatrixTemplate friend std::ostream& operator<<(std::ostream& os, MatrixTemplate m1);
		DefMatrixTemplate friend Matrix& operator<<(Matrix& m1, double d);
		DefMatrixTemplate friend Matrix& operator,(Matrix& m1, double d);

		_Scalar& operator()(int n1);
		_Scalar& operator()(int m1, int n1);
	};

	const int Dynamic = -1;
	const int MAX_MAT_DIM = 40;

	typedef Matrix<double, Dynamic, Dynamic, MAX_MAT_DIM, MAX_MAT_DIM> MatrixXd;
	typedef Matrix<double, 2, 2, 2, 2> Matrix2d;
	typedef Matrix<double, 3, 3, 3, 3> Matrix3d;
	typedef Matrix<double, 4, 4, 4, 4> Matrix4d;
	typedef Matrix<double, 6, 6, 6, 6> Matrix6d;

	typedef Matrix<double, Dynamic, 1, MAX_MAT_DIM, 1> VectorXd;
	typedef Matrix<double, 2, 1, 2, 1> Vector2d;
	typedef Matrix<double, 3, 1, 3, 1> Vector3d;
	typedef Matrix<double, 4, 1, 4, 1> Vector4d;
	typedef Matrix<double, 6, 1, 6, 1> Vector6d;

	extern int mInputpos[2];

	DefMatrixTemplate void MatrixTemplate::_Resetmn()
	{
		if (_Rows == -1) m = 0; else m = _Rows;
		if (_Cols == -1) n = 0; else n = _Cols;
	}

	DefMatrixTemplate MatrixTemplate::Matrix()
	{
		_Resetmn();
	}
	DefMatrixTemplate MatrixTemplate::Matrix(int n1)
	{
		_Resetmn();
		bool b = 0;
		if (_Rows == 1 && _Cols == -1) n = n1, b = 1;
		if (_Rows == 1 && _Cols == n1) b = 1;
		if (_Cols == 1 && _Rows == -1) m = n1, b = 1;
		if (_Cols == 1 && _Rows == n1) b = 1;
		if (!b) printf("Initialization failed with one input.\n");
	}
	DefMatrixTemplate MatrixTemplate::Matrix(int m1, int n1)
	{
		_Resetmn();
		bool b = 0;
		if ((_Rows == -1 || _Rows == m1) && (_Cols == -1 || _Cols == n1)) m = m1, n = n1, b = 1;
		if (!b) printf("Initialization failed with two input.\n");
	}
	DefMatrixTemplate MatrixTemplate::Matrix(double d0, double d1)
	{
		_Resetmn();
		bool b = 0;
		if (_Rows == 1 && _Cols == -1) n = 2, b = 1;
		if (_Rows == 1 && _Cols == 2) b = 1;
		if (_Cols == 1 && _Rows == -1) m = 2, b = 1;
		if (_Cols == 1 && _Rows == 2) b = 1;
		(*this)(0) = d0, (*this)(1) = d1;
		if (!b) printf("Initialization failed with two inputs.\n");
	}
	DefMatrixTemplate MatrixTemplate::Matrix(double d0, double d1, double d2)
	{
		_Resetmn();
		bool b = 0;
		if (_Rows == 1 && _Cols == -1) n = 3, b = 1;
		if (_Rows == 1 && _Cols == 3) b = 1;
		if (_Cols == 1 && _Rows == -1) m = 3, b = 1;
		if (_Cols == 1 && _Rows == 3) b = 1;
		(*this)(0) = d0, (*this)(1) = d1, (*this)(2) = d2;
		if (!b) printf("Initialization failed with 3 inputs.\n");
	}
	DefMatrixTemplate MatrixTemplate::Matrix(double d0, double d1, double d2, double d3)
	{
		_Resetmn();
		bool b = 0;
		if (_Rows == 1 && _Cols == -1) n = 4, b = 1;
		if (_Rows == 1 && _Cols == 4) b = 1;
		if (_Cols == 1 && _Rows == -1) m = 4, b = 1;
		if (_Cols == 1 && _Rows == 4) b = 1;
		(*this)(0) = d0, (*this)(1) = d1, (*this)(2) = d2, (*this)(3) = d3;
		if (!b) printf("Initialization failed with 4 inputs.\n");
	}
	
	DefMatrixTemplate inline int MatrixTemplate::cols() { return n; }
	DefMatrixTemplate inline int MatrixTemplate::rows() { return m; }

	DefMatrixTemplate inline MatrixTemplate MatrixTemplate::Identity()
	{
		MatrixTemplate mTemp;
		if (_Rows == -1 || _Cols == -1) { printf("Identity() failed with dynamic size.\n"); return mTemp; }
		if (_Rows != _Cols) { printf("Identity() failed with non-square type.\n"); return mTemp; }
		ChzF(i, 0, mTemp.rows() - 1) ChzF(j, 0, mTemp.cols() - 1) if (i == j) mTemp(i, j) = 1.0; else mTemp(i, j) = 0.0;
		return mTemp;
	}
	DefMatrixTemplate inline MatrixTemplate MatrixTemplate::Identity(int n1)
	{
		MatrixTemplate mTemp; if (!mTemp.resize(n1, n1)) return mTemp;
		ChzF(i, 0, mTemp.rows() - 1) ChzF(j, 0, mTemp.cols() - 1) if (i == j) mTemp(i, j) = 1.0; else mTemp(i, j) = 0.0;
		return mTemp;
	}
	DefMatrixTemplate inline MatrixTemplate MatrixTemplate::Identity(int m1, int n1)
	{
		MatrixTemplate mTemp; if (!mTemp.resize(m1, n1)) return mTemp;
		ChzF(i, 0, mTemp.rows() - 1) ChzF(j, 0, mTemp.cols() - 1) if (i == j) mTemp(i, j) = 1.0; else mTemp(i, j) = 0.0;
		return mTemp;
	}
	DefMatrixTemplate inline MatrixTemplate MatrixTemplate::Ones()
	{
		MatrixTemplate mTemp; 
		if (_Rows == -1 || _Cols == -1) { printf("Ones() failed with dynamic size.\n"); return mTemp; }
		ChzF(i, 0, mTemp.rows() - 1) ChzF(j, 0, mTemp.cols() - 1) mTemp(i, j) = 1.0;
		return mTemp;
	}
	DefMatrixTemplate inline MatrixTemplate MatrixTemplate::Ones(int n1)
	{
		MatrixTemplate mTemp; if (!mTemp.resize(n1)) return mTemp;
		ChzF(i, 0, mTemp.rows() - 1) ChzF(j, 0, mTemp.cols() - 1) mTemp(i, j) = 1.0;
		return mTemp;
	}
	DefMatrixTemplate inline MatrixTemplate MatrixTemplate::Ones(int m1, int n1)
	{
		MatrixTemplate mTemp; if (!mTemp.resize(m1, n1)) return mTemp;
		ChzF(i, 0, mTemp.rows() - 1) ChzF(j, 0, mTemp.cols() - 1) mTemp(i, j) = 1.0;
		return mTemp;
	}
	DefMatrixTemplate inline MatrixTemplate MatrixTemplate::Zero()
	{
		MatrixTemplate mTemp;
		if (_Rows == -1 || _Cols == -1) { printf("Ones() failed with dynamic size.\n"); return mTemp; }
		ChzF(i, 0, mTemp.rows() - 1) ChzF(j, 0, mTemp.cols() - 1) mTemp(i, j) = 0.0;
		return mTemp;
	}
	DefMatrixTemplate inline MatrixTemplate MatrixTemplate::Zero(int n1)
	{
		MatrixTemplate mTemp; if (!mTemp.resize(n1)) return mTemp;
		ChzF(i, 0, mTemp.rows() - 1) ChzF(j, 0, mTemp.cols() - 1) mTemp(i, j) = 0.0;
		return mTemp;
	}
	DefMatrixTemplate inline MatrixTemplate MatrixTemplate::Zero(int m1, int n1)
	{
		MatrixTemplate mTemp; if (!mTemp.resize(m1, n1)) return mTemp;
		ChzF(i, 0, mTemp.rows() - 1) ChzF(j, 0, mTemp.cols() - 1) mTemp(i, j) = 0.0;
		return mTemp;
	}

	DefMatrixTemplate inline bool MatrixTemplate::resize(int m1, int n1)
	{
		if ((_Rows != -1 && _Rows != m1) || (_Cols != -1 && _Cols != n1)) { printf("Resize failed with non-dynamic size.\n"); return false; }
		m = m1; n = n1;
		return true;
	}
	DefMatrixTemplate inline bool MatrixTemplate::resize(int n1)
	{
		if ((_Rows == -1 || _Rows == n1) && _Cols == 1) { m = n1; return true; }
		if (_Rows == 1 && (_Cols == -1 || _Cols == n1)) { n = n1; return true; }
		printf("Resize failed with size %d.\n", n1);
		return false;
	}

	DefMatrixTemplate inline void MatrixTemplate::setIdentity()
	{
		if (m != n) { printf("SetIdentity failed with non-square matrix.\n"); return; }
		ChzF(i, 0, rows() - 1) ChzF(j, 0, cols() - 1) if (i == j) data[i][j] = 1.0; else data[i][j] = 0.0;
	}
	DefMatrixTemplate inline void MatrixTemplate::setOnes()
	{
		ChzF(i, 0, rows() - 1) ChzF(j, 0, cols() - 1) data[i][j] = 1.0;
	}
	DefMatrixTemplate inline void MatrixTemplate::setZero()
	{
		ChzF(i, 0, rows() - 1) ChzF(j, 0, cols() - 1) data[i][j] = 0.0;
	}
	DefMatrixTemplate inline int MatrixTemplate::size()
	{
		return m * n;
	}

	DefMatrixTemplate inline MatrixTemplate_Trans MatrixTemplate::transpose()
	{
		MatrixTemplate_Trans mTemp;
		mTemp.resize(n, m);
		ChzF(i, 0, m - 1) ChzF(j, 0, n - 1) mTemp(j, i) = data[i][j];
		return mTemp;
	}
	DefMatrixTemplate inline MatrixTemplate_Trans MatrixTemplate::inverse()
	{
		if (m != n)
		{
			printf("Inverse failed with non-square matrix.\n");
			return this->transpose();
		}
		auto mBak = *this;
		auto mTemp = MatrixTemplate_Trans::Identity(n);
		double dtemp;
		ChzF(j, 0, m - 1)
		{
			int pos = -1;
			ChzF(i, j, m - 1) 
				if (abs(mBak(i, j)) > 1e-16)
				{ 
					pos = i; 
					break; 
				}
			if (pos == -1) { printf("Inverse failed with irreversible matrix.\n"); mTemp.setZero(); return mTemp; }
			if (pos != j) ChzF(k, 0, n - 1)
			{
				dtemp = mBak(j, k); mBak(j, k) = mBak(pos, k); mBak(pos, k) = dtemp;
				dtemp = mTemp(j, k); mTemp(j, k) = mTemp(pos, k); mTemp(pos, k) = dtemp;
			}
			//std::cout << mBak << mTemp << std::endl;
			dtemp = mBak(j, j); ChzF(k, j, n - 1) mBak(j, k) /= dtemp; ChzF(k, 0, n - 1) mTemp(j, k) /= dtemp;
			ChzF(i, j + 1, m - 1)
			{
				dtemp = mBak(i, j) / mBak(j, j);
				ChzF(k, j, n - 1) mBak(i, k) -= dtemp * mBak(j, k);
				ChzF(k, 0, n - 1) mTemp(i, k) -= dtemp * mTemp(j, k);
			}
			//std::cout << mBak << mTemp << std::endl;
		}
		for (int j = m - 1; j >= 0; j--)
		{
			ChzF(i, 0, j - 1)
			{
				dtemp = mBak(i, j) / mBak(j, j);
				ChzF(k, j, n - 1) mBak(i, k) -= dtemp * mBak(j, k);
				ChzF(k, 0, n - 1) mTemp(i, k) -= dtemp * mTemp(j, k);
			}
		}
		return mTemp;
	}

	DefMatrixTemplate inline MatrixTemplate operator+(MatrixTemplate m1, MatrixTemplate m2)
	{
		if (m1.rows() != m2.rows() || m1.cols() != m2.cols())
		{
			printf("+ error. rows()/cols() do not match.\n");
			m1.setZero();
			return m1;
		}
		ChzF(i, 0, m1.rows() - 1) ChzF(j, 0, m1.cols() - 1) m1(i, j) += m2(i, j);
		return m1;
	}
	DefMatrixTemplate inline MatrixTemplate operator+=(MatrixTemplate m1, MatrixTemplate m2)
	{
		return m1 + m2;
	}
	DefMatrixTemplate inline MatrixTemplate operator-(MatrixTemplate m1)
	{
		ChzF(i, 0, m1.rows() - 1) ChzF(j, 0, m1.cols() - 1) m1(i, j) = -m1(i, j);
		return m1;
	}
	DefMatrixTemplate inline MatrixTemplate operator-(MatrixTemplate m1, MatrixTemplate m2) { return m1 + (-m2); }
	template<typename _Scalar, int _Rows1, int _Cols1, int _MaxRows1, int _MaxCols1, int _Rows2, int _Cols2, int _MaxRows2, int _MaxCols2>
	Matrix<_Scalar, _Rows1, _Cols2, _MaxRows1, _MaxCols2> operator*(Matrix<_Scalar, _Rows1, _Cols1, _MaxRows1, _MaxCols1> m1, Matrix<_Scalar, _Rows2, _Cols2, _MaxRows2, _MaxCols2> m2)
	{
		Matrix<_Scalar, _Rows1, _Cols2, _MaxRows1, _MaxCols2> mTemp;
		if (m1.cols() != m2.rows()) { printf("* error. rows()/cols() do not match.\n"); mTemp.setZero(); return mTemp; }
		mTemp.resize(m1.rows(), m2.cols());
		ChzF(i, 0, m1.rows() - 1) ChzF(j, 0, m2.cols() - 1)
		{
			mTemp(i, j) = 0.0;
			ChzF(k, 0, m1.cols() - 1) mTemp(i, j) += m1(i, k) * m2(k, j);
		}
		return mTemp;
	}

	DefMatrixTemplate MatrixTemplate operator*(MatrixTemplate m1, double d)
	{
		ChzF(i, 0, m1.rows() - 1) ChzF(j, 0, m1.cols() - 1) m1(i, j) *= d; return m1;
	}
	DefMatrixTemplate MatrixTemplate operator*(double d, MatrixTemplate m1) { return m1 * d; }

	DefMatrixTemplate std::ostream& operator<<(std::ostream& os, MatrixTemplate m)
	{
		using namespace std;
		cout << m.rows() << " " << m.cols() << endl;
		ChzF(i, 0, m.rows() - 1)
		{
			ChzF(j, 0, m.cols() - 1) printf("%4.8f ", m(i, j));
			printf("\n");
		}
		return os;
	}
	DefMatrixTemplate MatrixTemplate& operator<<(MatrixTemplate& m, double d)
	{
		m(0, 0) = d;
		mInputpos[0] = mInputpos[1] = 0;
		return m;
	}

	DefMatrixTemplate MatrixTemplate& operator,(MatrixTemplate& m, double d)
	{
		mInputpos[1]++;
		if (mInputpos[1] >= m.cols()) mInputpos[1] = 0, mInputpos[0]++;
		if (mInputpos[0] >= m.rows()) { printf("Input failed with too many input values.\n"); return m; }
		m(mInputpos[0], mInputpos[1]) = d;
		return m;
	}

	DefMatrixTemplate inline _Scalar& MatrixTemplate::operator()(int n1)
	{
		if (m != 1 && n != 1) { printf("Operator () failed with one input.\n"); return data[0][0]; }
		else if (m == 1)
		{
			if (n1 >= n) { printf("Operator () failed. n >= Cols().\n"); return data[0][0]; }
			else return data[0][n1];
		}
		else
		{
			if (n1 >= m) { printf("Operator () failed. n >= Rows().\n"); return data[0][0]; }
			else return data[n1][0];
		}
	}
	DefMatrixTemplate inline _Scalar& MatrixTemplate::operator()(int m1, int n1)
	{
		if (m1 >= m) { printf("Operator () failed. m >= Rows().\n"); return data[0][0]; }
		else if (n1 >= n) { printf("Operator () failed. n >= Cols().\n"); return data[0][0]; }
		else return data[m1][n1];
	}
}