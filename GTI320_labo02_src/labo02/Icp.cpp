#include "Icp.h"

#include "Matrix.h"
#include "Vector.h"
#include "Operators.h"
#include "SVD.h"

using namespace gti320;

/**
 * Calcul du déterminant 3x3
 * TODO
 */
double gti320::determinant(const Matrix3d& M)
{
    return 0.0;
}

/**
 * Calcul de l'erreur entre deux nuages de points
 * TODO
 */
double Icp::error(const Points3d& A, const Points3d& B)
{
    assert(A.cols() == B.cols());
    return DBL_MAX;       
}


/**
 * Index du point de A qui minimise la distance à p
 * TODO
 */
int Icp::nearestNeighbor(const Vector3d& p, const Points3d& A)
{
    return 0;
}

/**
 * Meilleure transformation rigide pour amener les points de B sur ceux A
 * TODO
 */
Matrix4d Icp::bestFitTransform(const Points3d& A, const Points3d& B)
{
    Matrix4d T;
    T.setIdentity();

    return T;
}
