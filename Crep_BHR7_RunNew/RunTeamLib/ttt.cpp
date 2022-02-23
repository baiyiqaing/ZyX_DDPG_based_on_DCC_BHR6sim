#include "Eigen\Dense"

using namespace Eigen;

void main()
{
	Matrix<double, 19, 19> A;
	auto B = A.inverse()+A;
}