#pragma once

/**
 * @file SVD.hpp
 *
 * @brief Singular value decomposition.
 *
 * Nom:
 * Code permanent :
 * Email :
 *
 */

#include "Matrix.h"
#include "Math3D.h"
#include "Operators.h"

#include <algorithm>
#include <cstdlib>
#include <cstdio>
#include <cmath>

namespace gti320
{

    //
    //  An implementation of SVD from "Numerical Recipes in C". 
    //  http://numerical.recipes/webnotes/nr3web2.pdf
    // 
    template<typename _Scalar>
    static inline _Scalar sign(const _Scalar a, const _Scalar b)
    {
        return (b > 0.0) ? std::abs(a) : -std::abs(a);
    }

    template<typename _Scalar>
    static inline _Scalar sq(const _Scalar a)
    {
        return (a * a);
    }

    // calculates sqrt( a^2 + b^2 ) 
    template<typename _Scalar>
    static inline _Scalar pythag(_Scalar a, _Scalar b)
    {
        return std::sqrt(a*a + b * b);
    }

    template<typename _Scalar, int _Rows = Dynamic, int _Cols = Dynamic, int _Storage = ColumnStorage>
    class SVD
    {
    public:

        explicit SVD(const Matrix<_Scalar, _Rows, _Cols, _Storage>& _A) :
            m_U(_A), m_V(_A.cols(), _A.cols()), m_S(_A.cols())
        {
        }

        const Matrix<_Scalar, _Rows, _Cols>& getU() const { return m_U; }

        const Matrix<_Scalar, _Cols, _Cols>& getV() const { return m_V; }

        const Vector<_Scalar, _Cols>& getSigma() const { return m_S; }

        void decompose()
        {
            bool flag;
            int i, its, j, jj, k, l, nm;
            _Scalar anorm, c, f, g, h, s, scale, x, y, z;

            const int ncols = m_U.cols();
            const int nrows = m_U.rows();
            Vector<_Scalar> rv1(ncols);

            // Householder reduction to bidiagonal form
            //
            g = scale = anorm = 0.0;
            for (i = 0; i < ncols; ++i)
            {
                l = i + 1;
                rv1(i) = scale * g;
                g = s = scale = 0.0;
                if (i < nrows)
                {
                    for (k = i; k < nrows; ++k)
                    {
                        scale += std::abs(m_U(k, i));
                    }

                    if (scale != 0.0)
                    {
                        for (k = i; k < nrows; ++k)
                        {
                            m_U(k, i) /= scale;
                            s += m_U(k, i) * m_U(k, i);
                        }
                        f = m_U(i, i);
                        g = -sign(std::sqrt(s), f);
                        h = f * g - s;
                        m_U(i, i) = f - g;
                        for (j = l; j < ncols; ++j)
                        {
                            for (s = 0.0, k = i; k < nrows; ++k)
                                s += m_U(k, i) * m_U(k, j);
                            f = s / h;
                            for (k = i; k < nrows; ++k)
                                m_U(k, j) += f * m_U(k, i);
                        }
                        for (k = i; k < nrows; ++k)
                            m_U(k, i) *= scale;
                    }
                }
                m_S(i) = scale * g;
                g = s = scale = 0.0;
                if (i < nrows && i != ncols - 1)
                {
                    for (k = l; k < ncols; ++k)
                        scale += std::abs(m_U(i, k));

                    if (scale)
                    {
                        for (k = l; k < ncols; ++k)
                        {
                            m_U(i, k) /= scale;
                            s += m_U(i, k) * m_U(i, k);
                        }
                        f = m_U(i, l);
                        g = -sign(std::sqrt(s), f);
                        h = f * g - s;
                        m_U(i, l) = f - g;
                        for (k = l; k < ncols; ++k)
                        {
                            rv1(k) = m_U(i, k) / h;
                        }
                        for (j = l; j < nrows; ++j)
                        {
                            for (s = 0.0, k = l; k < ncols; ++k)
                                s += m_U(j, k) * m_U(i, k);
                            for (k = l; k < ncols; ++k)
                                m_U(j, k) += s * rv1(k);
                        }
                        for (k = l; k < ncols; ++k)
                            m_U(i, k) *= scale;
                    }
                }
                const _Scalar tmp = (std::abs(m_S(i)) + std::abs(rv1(i)));
                anorm = std::max(anorm, tmp);

                //printf(".");
                //fflush(stdout);
            }

            // Accumulation of right-hand transformations
            //
            for (i = ncols - 1; i >= 0; --i)
            {
                if (i < ncols - 1)
                {
                    if (g) {
                        for (j = l; j < ncols; ++j)
                            m_V(j, i) = (m_U(i, j) / m_U(i, l)) / g;  // double division to avoid possible underflow

                        for (j = l; j < ncols; ++j)
                        {
                            for (s = 0.0, k = l; k < ncols; ++k)
                                s += m_U(i, k) * m_V(k, j);
                            for (k = l; k < ncols; ++k)
                                m_V(k, j) += s * m_V(k, i);
                        }
                    }
                    for (j = l; j < ncols; ++j)
                        m_V(i, j) = m_V(j, i) = 0.0;
                }
                m_V(i, i) = 1.0;
                g = rv1(i);
                l = i;
                //printf(".");
                //fflush(stdout);
            }

            // Accumulation of left-hand transformations
            //
            for (i = std::min(nrows, ncols) - 1; i >= 0; --i)
            {
                l = i + 1;
                g = m_S(i);
                for (j = l; j < ncols; ++j)
                    m_U(i, j) = 0.0;
                if (g)
                {
                    g = 1.0 / g;
                    for (j = l; j < ncols; ++j)
                    {
                        for (s = 0.0, k = l; k < nrows; ++k)
                            s += m_U(k, i) * m_U(k, j);
                        f = (s / m_U(i, i)) * g;
                        for (k = i; k < nrows; ++k)
                            m_U(k, j) += f * m_U(k, i);
                    }
                    for (j = i; j < nrows; ++j)
                        m_U(j, i) *= g;
                }
                else
                    for (j = i; j < nrows; ++j)
                        m_U(j, i) = 0.0;
                m_U(i, i) = m_U(i, i) + 1.0;
                //printf(".");
                //fflush(stdout);
            }

            // Diagonalization of the bidiagonal form.
            // Loops over singular values using iterations.
            //
            for (k = ncols - 1; k >= 0; --k)
            {
                // Max iterations: 30
                for (its = 0; its < 30; ++its)
                {
                    flag = true;
                    for (l = k; l >= 0; --l)     // Test for splitting
                    {
                        nm = l - 1;
                        if ((std::abs(rv1(l)) + anorm) == anorm)
                        {
                            flag = false;
                            break;
                        }
                        if (std::abs(m_S(nm) + anorm) == anorm)
                            break;
                    }

                    if (flag)
                    {                           // Cancellation of rv1(l) if l > 0
                        c = 0.0;
                        s = 1.0;
                        for (i = l; i <= k; ++i)
                        {
                            f = s * rv1(i);
                            rv1(i) = c * rv1(i);
                            if ((std::abs(f) + anorm) == anorm)
                                break;

                            g = m_S(i);
                            h = pythag(f, g);
                            m_S(i) = h;
                            h = 1.0 / h;
                            c = g * h;
                            s = -f * h;
                            for (j = 0; j < nrows; ++j)
                            {
                                y = m_U(j, nm);
                                z = m_U(j, i);
                                m_U(j, nm) = y * c + z * s;
                                m_U(j, i) = z * c - y * s;
                            }
                        }
                    }
                    z = m_S(k);
                    if (l == k)         // Convergence.
                    {
                        if (z < 0.0)                // Compute non-negative singular values
                        {
                            m_S(k) = -z;
                            for (j = 0; j < ncols; ++j)
                            {
                                m_V(j, k) = -m_V(j, k); // Reverse bases direction 
                            }
                        }
                        break;
                    }

                    // Assertion when max iterations reached without convergence
                    assert(its < 29);

                    x = m_S(l);                     // Shift from bottom 2-by-2 minor
                    nm = k - 1;
                    y = m_S(nm);
                    g = rv1(nm);
                    h = rv1(k);
                    f = ((y - z) * (y + z) + (g - h) * (g + h)) / (2.0 * h * y);
                    g = pythag(f, 1.0);
                    f = ((x - z) * (x + z) + h * ((y / (f + sign(g, f))) - h)) / x;
                    c = s = 1.0;                    // Next QR transformation
                    for (j = l; j <= nm; ++j)
                    {
                        i = j + 1;
                        g = rv1(i);
                        y = m_S(i);
                        h = s * g;
                        g = c * g;
                        z = pythag(f, h);
                        rv1(j) = z;
                        c = f / z;
                        s = h / z;
                        f = x * c + g * s;
                        g = g * c - x * s;
                        h = y * s;
                        y *= c;
                        for (jj = 0; jj < ncols; ++jj)
                        {
                            x = m_V(jj, j);
                            z = m_V(jj, i);
                            m_V(jj, j) = x * c + z * s;
                            m_V(jj, i) = z * c - x * s;
                        }
                        z = pythag(f, h);
                        m_S(j) = z;                 // Rotation can be arbitrary if z == 0
                        if (z != 0.0)
                        {
                            z = 1.0 / z;
                            c = f * z;
                            s = h * z;
                        }
                        f = c * g + s * y;
                        x = c * y - s * g;
                        for (jj = 0; jj < nrows; ++jj)
                        {
                            y = m_U(jj, j);
                            z = m_U(jj, i);
                            m_U(jj, j) = y * c + z * s;
                            m_U(jj, i) = z * c - y * s;
                        }
                    }
                    rv1(l) = 0.0;
                    rv1(k) = f;
                    m_S(k) = x;
                }
                //printf(".");
                //fflush(stdout);
            }
            //printf("\n");

            reorder();
        }

    private:

        void reorder() {
            int s, inc = 1;
            _Scalar sw;

            const int ncols = m_U.cols();
            const int nrows = m_U.rows();
            Vector<_Scalar> su(nrows), sv(ncols);
            do { inc *= 3; inc++; } while (inc <= ncols);   // Sort using Shell’s sort.

            do {
                inc /= 3;
                for (int i = inc; i < ncols; ++i) {
                    sw = m_S(i);
                    for (int k = 0; k < nrows; ++k) su(k) = m_U(k, i);
                    for (int k = 0; k < ncols; ++k) sv(k) = m_V(k, i);
                    int j = i;
                    while (m_S(j - inc) < sw) {
                        m_S(j) = m_S(j - inc);
                        for (int k = 0; k < nrows; ++k) m_U(k, j) = m_U(k, j - inc);
                        for (int k = 0; k < ncols; ++k) m_V(k, j) = m_V(k, j - inc);
                        j -= inc;
                        if (j < inc) break;
                    }
                    m_S(j) = sw;
                    for (int k = 0; k < nrows; ++k) m_U(k, j) = su(k);
                    for (int k = 0; k < ncols; ++k) m_V(k, j) = sv(k);
                }
            } while (inc > 1);

            for (int k = 0; k < ncols; ++k)
            {
                // Flip signs.
                s = 0;
                for (int i = 0; i < nrows; ++i) if (m_U(i, k) < 0.0) ++s;
                for (int j = 0; j < ncols; ++j) if (m_V(j, k) < 0.0) ++s;
                if (s > (nrows + ncols) / 2)
                {
                    for (int i = 0; i < nrows; ++i) m_U(i, k) = -m_U(i, k);
                    for (int j = 0; j < ncols; ++j) m_V(j, k) = -m_V(j, k);
                }
            }
        }


        Matrix<_Scalar, _Rows, _Cols, _Storage> m_U;
        Matrix<_Scalar, _Cols, _Cols> m_V;
        Vector<_Scalar, _Cols> m_S;
    };

}