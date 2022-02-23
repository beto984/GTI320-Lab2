#pragma once

/**
 * @file MatrixBase.h
 *
 * @brief Classe contenant les éléments de base des matrices et des vecteurs.
 *
 * Nom:
 * Code permanent :
 * Email :
 *
 */

#include "DenseStorage.h"

namespace gti320
{

	/**
	 * Classe de base pour les matrices et vecteurs à taille fixe.
	 */
	template <typename _Scalar, int _Rows, int _Cols>
	class MatrixBase
	{
	protected:
		DenseStorage<_Scalar, _Rows* _Cols> m_storage;

	public:

		typedef _Scalar Scalar;

		/**
		 * Constructeur par défaut
		 */
		MatrixBase() : m_storage() { }

		/**
		 * Constructeur de copie
		 */
		MatrixBase(const MatrixBase& other) : m_storage(other.m_storage) { }

		explicit MatrixBase(int _rows, int _cols) : m_storage() { }

		/**
		 * Destructeur
		 */
		~MatrixBase() { }


		/**
		 * Redimensionne la matrice
		 */
		void resize(int _rows, int _cols)
		{
			// Ne rien faire.
		}

		/**
		 * Opérateur de copie
		 */
		MatrixBase& operator=(const MatrixBase& other)
		{
			if (this != &other)
			{
				m_storage = other.m_storage;
			}
			return *this;
		}

		inline void setZero() { m_storage.setZero(); }
		static inline int cols() { return _Cols; }
		static inline int rows() { return _Rows; }

		/**
		 * Accès à la donnée membre de stockage (en lecture seule)
		 */
		const DenseStorage<_Scalar, _Rows* _Cols>& storage() const
		{
			return m_storage;
		}

		/**
		 * Nombre d'éléments stockés dans le tampon.
		 */
		inline int size() const
		{
			return m_storage.size();
		}

		/**
		 * Accès au tampon de données ((lecture seule))
		 */
		const _Scalar* data() const
		{
			return m_storage.data();
		}

	};

	/**
	 * Classe de base pour les matrices avec un nombre de lignes dynamique et un
	 * nombre fixe de colonnes.
	 */
	template <typename _Scalar, int _Cols>
	class MatrixBase<_Scalar, Dynamic, _Cols>
	{
	protected:

		DenseStorage<_Scalar, Dynamic> m_storage;
		int m_rows;

	public:

		typedef _Scalar Scalar;

		/**
		 * Constructeur par défaut
		 */
		MatrixBase() : m_storage(), m_rows(0) { }

		explicit MatrixBase(int _rows, int _cols) : m_storage(_rows* _Cols), m_rows(_rows) { }

		/**
		 * Constructeur de copie
		 */
		MatrixBase(const MatrixBase& other) : m_storage(other.m_storage), m_rows(other.m_rows) { }

		/**
		 * Destructeur
		 */
		~MatrixBase() { }

		/**
		 * Opérateur de copie
		 */
		MatrixBase& operator=(const MatrixBase& other)
		{
			if (this != &other)
			{
				m_storage = other.m_storage;
				m_rows = other.m_rows;
			}
			return *this;
		}

		/**
		 * Redimensionne la matrice
		 */
		void resize(int _rows, int _cols)
		{
			assert(_cols == _Cols);
			m_storage.resize(_rows * _Cols);
			m_rows = _rows;
		}

		inline void setZero() { m_storage.setZero(); }

		static inline int cols() { return _Cols; }
		inline int rows() const { return m_rows; }

		/**
		 * Accès à la donnée membre de stockage (en lecture seule)
		 */
		const DenseStorage<_Scalar, Dynamic>& storage() const
		{
			return m_storage;
		}

		/**
		 * Nombre d'éléments stockés dans le tampon.
		 */
		inline int size() const
		{
			return m_storage.size();
		}

		/**
		 * Accès au tampon de données ((lecture seule))
		 */
		const _Scalar* data() const
		{
			return m_storage.data();
		}
	};

	/**
	 * Classe de base pour les matrices avec un nombre fixe de lignes et un
	 * nombre de colonnes dynamique.
	 */
	template <typename _Scalar, int _Rows>
	class MatrixBase<_Scalar, _Rows, Dynamic>
	{
	protected:

		DenseStorage<_Scalar, Dynamic> m_storage;
		int m_cols;

	public:

		typedef _Scalar Scalar;

		/**
		 * Constructeur par défaut
		 */
		MatrixBase() : m_storage() { }

		explicit MatrixBase(int _rows, int _cols) : m_storage(_rows* _cols), m_cols(_cols) { }

		/**
		 * Constructeur de copie
		 */
		MatrixBase(const MatrixBase& other) : m_storage(other.m_storage), m_cols(other.m_cols) { }

		/**
		 * Destructeur
		 */
		~MatrixBase() { }

		/**
		 * Opérateur de copie
		 */
		MatrixBase& operator=(const MatrixBase& other)
		{
			if (this != &other) {
				m_storage = other.m_storage;
				m_cols = other.m_cols;
			}

			return *this;
		}

		/**
		 * Redimensionne la matrice
		 */
		void resize(int _rows, int _cols)
		{
			assert(_rows == _Rows);
			m_storage.resize(_Rows * _cols);
			m_cols = _cols;
		}

		inline void setZero() { m_storage.setZero(); }

		inline int cols() const { return m_cols; }
		static inline int rows() { return _Rows; }

		/**
		 * Accès à la donnée membre de stockage (en lecture seule)
		 */
		const DenseStorage<_Scalar, Dynamic>& storage() const
		{
			return m_storage;
		}

		/**
		 * Nombre d'éléments stockés dans le tampon.
		 */
		inline int size() const
		{
			return m_storage.size();
		}

		/**
		 * Accès au tampon de données ((lecture seule))
		 */
		const _Scalar* data() const
		{
			return m_storage.data();
		}
	};

	/**
	 * Classe de base pour les matrices avec un nombre de lignes et de colonnes
	 * dynamiques.
	 */
	template <typename _Scalar>
	class MatrixBase<_Scalar, Dynamic, Dynamic>
	{
	protected:

		DenseStorage<_Scalar, Dynamic> m_storage;
		int m_cols;
		int m_rows;

	public:
		typedef _Scalar Scalar;

		/**
		 * Constructeur par défaut
		 */
		MatrixBase() : m_storage(), m_rows(0), m_cols(0) { }

		explicit MatrixBase(int _rows, int _cols) : m_storage(_rows* _cols), m_rows(_rows), m_cols(_cols) { }

		/**
		 * Constructeur de copie
		 */
		MatrixBase(const MatrixBase& other) : m_storage(other.m_storage), m_rows(other.m_rows), m_cols(other.m_cols) { }

		/**
		 * Destructeur
		 */
		~MatrixBase() { }

		/**
		 * Opérateur de copie
		 */
		MatrixBase& operator=(const MatrixBase& other)
		{
			if (this != &other)
			{
				resize(other.m_rows, other.m_cols);
				m_storage = other.m_storage;
			}
			return *this;
		}

		/**
		 * Redimensionne la matrice
		 */
		void resize(int _rows, int _cols)
		{
			m_storage.resize(_rows * _cols);
			m_rows = _rows;
			m_cols = _cols;
		}

		inline void setZero() { m_storage.setZero(); }

		inline int cols() const { return m_cols; }
		inline int rows() const { return m_rows; }

		/**
		 * Accès à la donnée membre de stockage (en lecture seule)
		 */
		const DenseStorage<_Scalar, Dynamic>& storage() const
		{
			return m_storage;
		}

		/**
		 * Nombre d'éléments stockés dans le tampon.
		 */
		inline int size() const
		{
			return m_storage.size();
		}

		/**
		 * Accès au tampon de données ((lecture seule))
		 */
		const _Scalar* data() const
		{
			return m_storage.data();
		}
	};
}
