#pragma once

/**
 * @file Icp.h
 *
 * @brief Fonction utilisée par l'algorithme ICP.
 *
 * Nom:
 * Code permanent :
 * Email :
 *
 */

#include <float.h>
#include "Math3D.h"

namespace gti320 
{

  /**
   * Type spécialisé pour les nuages de points.
   *
   * On stocke n points 3D dans un matrice de taille 3xn où chaque colonne
   * contient un point 3D.
   */
  typedef Matrix<double, 3, Dynamic, ColumnStorage> Points3d;

  /**
   * Calcul le déterminant d'une matrice 3x3.
   */
  static double determinant(const gti320::Matrix3d& M);

  /**
   * Classe Icp
   *
   * Conteient les fonction nécessaires à l'exécution de l'algorithme ICP.
   */
  class Icp
    {
  public:

    /**
     * Retourne l'index du point de A qui est le plus proche du point p.
     *
     * TODO
     *
     * @param p un point
     * @param A un nuage de points
     * @return l'index du point de A le plus près de p
     */
    static int nearestNeighbor(const Vector3d& p, const Points3d& A);

    /**
     * Étant donné deux nuages de points A et B, calcule la meilleure
     * transformation rigide pour aligner les deux nuages.
     *
     * TODO
     *
     * @param A le nuage de points de référence
     * @param B le nuage de points à aligner avec celui de référence
     * @return matrice de transformation rigide en coordonnées homogènes
     */
    static Matrix4d bestFitTransform(const Points3d& A, const Points3d& B);


    /**
     * Calcul l'erreur entre deux nuages de points.
     *
     * L'erreur est la somme des distances entre le i-ième point de A et le
     * i-ième point de B.
     *
     * TODO
     *
     * @param A nuage de points
     * @param B nuage de points
     * @return la somme des distances entre les points de A et ceux de B
     */
    static double error(const gti320::Points3d& A, const gti320::Points3d& B);

    };

}
