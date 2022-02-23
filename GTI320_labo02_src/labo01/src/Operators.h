#pragma once

/**
 * @file Operators.h
 *
 * @brief Opérateurs arithmétiques pour les matrices et les vecteurs.
 *
 * Nom:
 * Code permanent :
 * Email :
 *
 */

#include "Matrix.h"
#include "Vector.h"

 /**
  * Implémentation de divers opérateurs arithmétiques pour les matrices et les vecteurs.
  */
namespace gti320 {

	/**
	 * Multiplication : Matrice * Matrice (générique)
	 */
	template <typename _Scalar, int RowsA, int ColsA, int StorageA, int RowsB, int ColsB, int StorageB>
	Matrix<_Scalar, RowsA, ColsB> operator*(const Matrix<_Scalar, RowsA, ColsA, StorageA>& A, const Matrix<_Scalar, RowsB, ColsB, StorageB>& B)
	{
		// TODO implémenter
		return Matrix<_Scalar, RowsA, ColsB>();
	}

	/**
	 * Multiplication : Matrice (colonne) * Matrice (ligne)
	 *
	 * Spécialisation de l'opérateur de multiplication pour le cas où les matrices
	 * ont un stockage à taille dynamique et où la matrice de gauche utilise un
	 * stockage par colonnes et celle de droite un stockage par lignes.
	 */
	template <typename _Scalar>
	Matrix<_Scalar, Dynamic, Dynamic> operator*(const Matrix<_Scalar, Dynamic, Dynamic, ColumnStorage>& A, const Matrix<_Scalar, Dynamic, Dynamic, RowStorage>& B)
	{
		// TODO : implémenter
		return Matrix<_Scalar, Dynamic, Dynamic>();
	}

	/**
	 * Multiplication : Matrice (ligne) * Matrice (colonne)
	 *
	 * Spécialisation de l'opérateur de multiplication pour le cas où les matrices
	 * ont un stockage à taille dynamique et où la matrice de gauche utilise un
	 * stockage par lignes et celle de droite un stockage par colonnes.
	 */
	template <typename _Scalar>
	Matrix<_Scalar, Dynamic, Dynamic> operator*(const Matrix<_Scalar, Dynamic, Dynamic, RowStorage>& A, const Matrix<_Scalar, Dynamic, Dynamic, ColumnStorage>& B)
	{
		// TODO : implémenter
		return Matrix<_Scalar, Dynamic, Dynamic>();
	}


	/**
	 * Addition : Matrice + Matrice (générique)
	 */
	template <typename _Scalar, int Rows, int Cols, int StorageA, int StorageB>
	Matrix<_Scalar, Rows, Cols> operator+(const Matrix<_Scalar, Rows, Cols, StorageA>& A, const Matrix<_Scalar, Rows, Cols, StorageB>& B)
	{
		// TODO : implémenter
		return Matrix<_Scalar, Rows, Cols>();
	}

	/**
	 * Addition : Matrice (colonne) + Matrice (colonne)
	 *
	 * Spécialisation de l'opérateur d'addition pour le cas où les deux matrices
	 * sont stockées par colonnes.
	 */
	template <typename _Scalar>
	Matrix<_Scalar, Dynamic, Dynamic> operator+(const Matrix<_Scalar, Dynamic, Dynamic, ColumnStorage>& A, const Matrix<_Scalar, Dynamic, Dynamic, ColumnStorage>& B)
	{
		// TODO : implémenter
		return Matrix<_Scalar, Dynamic, Dynamic>();
	}

	/**
	 * Addition : Matrice (ligne) + Matrice (ligne)
	 *
	 * Spécialisation de l'opérateur d'addition pour le cas où les deux matrices
	 * sont stockées par lignes.
	 */
	template <typename _Scalar>
	Matrix<_Scalar, Dynamic, Dynamic, RowStorage> operator+(const Matrix<_Scalar, Dynamic, Dynamic, RowStorage>& A, const Matrix<_Scalar, Dynamic, Dynamic, RowStorage>& B)
	{
		// TODO : implémenter
		return Matrix<_Scalar, Dynamic, Dynamic, RowStorage>();
	}

	/**
	 * Multiplication  : Scalaire * Matrice (colonne)
	 *
	 * Spécialisation de l'opérateur de multiplication par un scalaire pour le
	 * cas d'une matrice stockée par colonnes.
	 */
	template <typename _Scalar, int _Rows, int _Cols>
	Matrix<_Scalar, _Rows, _Cols, ColumnStorage> operator*(const _Scalar& a, const Matrix<_Scalar, _Rows, _Cols, ColumnStorage>& A)
	{
		// TODO : implémenter
		return Matrix<_Scalar, Dynamic, Dynamic>();
	}

	/**
	 * Multiplication  : Scalaire * Matrice (ligne)
	 *
	 * Spécialisation de l'opérateur de multiplication par un scalaire pour le
	 * cas d'une matrice stockée par lignes.
	 */
	template <typename _Scalar, int _Rows, int _Cols>
	Matrix<_Scalar, _Rows, _Cols, RowStorage> operator*(const _Scalar& a, const Matrix<_Scalar, _Rows, _Cols, RowStorage>& A)
	{
		// TODO : implémenter
		return Matrix<_Scalar, Dynamic, Dynamic, RowStorage>();
	}

	/**
	 * Multiplication : Matrice (ligne) * Vecteur
	 *
	 * Spécialisation de l'opérateur de multiplication matrice*vecteur pour le
	 * cas où la matrice est représentée par lignes.
	 */
	template <typename _Scalar, int _Rows, int _Cols>
	Vector<_Scalar, _Rows> operator*(const Matrix<_Scalar, _Rows, _Cols, RowStorage>& A, const Vector<_Scalar, _Cols>& v)
	{
		// TODO : implémenter
		return Vector<_Scalar, _Rows>();
	}

	/**
	 * Multiplication : Matrice (colonne) * Vecteur
	 *
	 * Spécialisation de l'opérateur de multiplication matrice*vecteur pour le
	 * cas où la matrice est représentée par colonnes.
	 */
	template <typename _Scalar, int _Rows, int _Cols>
	Vector<_Scalar, _Rows> operator*(const Matrix<_Scalar, _Rows, _Cols, ColumnStorage>& A, const Vector<_Scalar, _Cols>& v)
	{
		// TODO : implémenter
		return Vector<_Scalar, _Rows>();
	}

	/**
	 * Multiplication : Scalaire * Vecteur
	 */
	template <typename _Scalar, int _Rows>
	Vector<_Scalar, _Rows> operator*(const _Scalar& a, const Vector<_Scalar, _Rows>& v)
	{
		// TODO : implémenter
		return Vector<_Scalar, _Rows>();
	}


	/**
	 * Addition : Vecteur + Vecteur
	 */
	template <typename _Scalar, int _RowsA, int _RowsB>
	Vector<_Scalar, _RowsA> operator+(const Vector<_Scalar, _RowsA>& a, const Vector<_Scalar, _RowsB>& b)
	{
		// TODO : implémenter
		return Vector<_Scalar, _RowsA>();
	}

	/**
	 * Soustraction : Vecteur - Vecteur
	 */
	template <typename _Scalar, int _RowsA, int _RowsB>
	Vector<_Scalar, _RowsA> operator-(const Vector<_Scalar, _RowsA>& a, const Vector<_Scalar, _RowsB>& b)
	{
		// TODO : implémenter
		return Vector<_Scalar, _RowsA>();
	}
}
