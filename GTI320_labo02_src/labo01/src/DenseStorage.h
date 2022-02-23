#pragma once

/**
 * @file DenseStorage.h
 *
 * @brief Stockage dense pour des données à taille fixe ou dynamique.
 *
 * Nom:
 * Code permanent :
 * Email :
 *
 */

#include <cstring>
#include <cassert>

namespace gti320
{
	enum SizeType { Dynamic = -1 };

	/**
	 * Stockage à taille fixe.
	 *
	 * Le nombre de données à stocker est connu au moment de la compilation.
	 * Ce nombre est donné par le paramètre de patron : _Size
	 *
	 * Un tampon (tableau) de taille `_Size_` est alloué sur la pile d'exécution.
	 */
	template<typename _Scalar, int _Size>
	class DenseStorage
	{
	private:

		// TODO déclarer une variable m_data et allouer la mémoire pour y stocker _Size éléments
		_Scalar* m_data;  // <-- Ceci n'est pas bon, à modifier

	public:

		/**
		 * Constructeur par défaut
		 */
		DenseStorage() { }

		/**
		 * Constructeur de copie
		 */
		DenseStorage(const DenseStorage& other)
		{
			memcpy(m_data, other.m_data, sizeof(m_data));
		}

		/**
		 * Constructeur avec taille spécifiée.
		 * Doit être la même que la taille spécifiée dans le patron.
		 */
		explicit DenseStorage(int _size)
        {
            assert(_size > 0 && _size == _Size);
        }

		/**
		 * Constructeur avec taille (_size) et données initiales (_data).
		 */
		explicit DenseStorage(const _Scalar* _data, int _size)
		{
			assert(_size >= 0 && _size == _Size);
			memcpy(m_data, _data, sizeof(_Scalar) * _size);
		}

		/**
		 * Opérateur de copie
		 */
		DenseStorage& operator=(const DenseStorage& other)
		{
			if (this != &other)
			{
				assert(other.size() == _Size);
				memcpy(m_data, other.m_data, sizeof(m_data));
			}
			return *this;
		}

		static int size() { return _Size; }

		/**
		 * Redimensionne le stockage pour qu'il contienne `size` élément.
		 */
		void resize(int size)
		{
			// Ne rien faire. Invalide pour les matrices à taille fixe.
		}

		/**
		 * Mets tous les éléments à zéro.
		 */
		void setZero()
		{
			memset(m_data, 0, sizeof(_Scalar) * _Size);
		}

		/**
		 * Accès au tampon de données (en lecteur seulement)
		 */
		const _Scalar* data() const
		{
			return m_data;
		}

		/**
		 * Accès au tampon de données (pour lecture et écriture)
		 */
		_Scalar* data()
		{
			return m_data;
		}
	};



	/**
	 * Stockage à taille dynamique.
	 *
	 * Le nombre de données à stocker est déterminé à l'exécution.
	 * Un tampon de la taille demandée doit être alloué sur le tas via
	 * l'opérateur `new []` et la mémoire doit être libérée avec `delete[]`
	 */
	template<typename _Scalar>
	class DenseStorage<_Scalar, Dynamic>
	{
	private:
		_Scalar* m_data;
		int m_size;

	public:

		/**
		 * Constructeur par défaut
		 */
		DenseStorage() : m_data(nullptr), m_size(0) {}

		/**
		 * Constructeur avec taille spécifiée
		 */
		explicit DenseStorage(int _size) : m_data(nullptr), m_size(_size)
		{
			// TODO allouer un tampon pour stocker _size éléments de type _Scalar.

			// TODO initialiser ce tampon à zéro.
		}

		/**
		 * Constructeur de copie
		 */
		DenseStorage(const DenseStorage& other)
			: m_data(nullptr)
			, m_size(other.m_size)
		{
			// TODO allouer un tampon pour stocker _size éléments de type _Scalar.

			// TODO copier other.m_data dans m_data.

		}

		/**
		 * Opérateur de copie
		 */
		DenseStorage& operator=(const DenseStorage& other)
		{
			// TODO implémenter !
			return *this;
		}

		/**
		 * Destructeur
		 */
		~DenseStorage()
		{
			// TODO libérer la mémoire allouée

		}

		/**
		 * Retourne la taille du tampon
		 */
		inline int size() const { return m_size; }

		/**
		 * Redimensionne le tampon alloué pour le stockage.
		 * La mémoire qui n'est plus utilisée doit être libérée.
		 * 
		 * Note :​ Toutes opérations de redimensionnement entraînent une réallocation de mémoire.
		 * Il n’est pas pertinent de copier les données car le résultat serait de toute façon incohérent.
		 */
		void resize(int _size)
		{
			// TODO redimensionner la mémoire allouée

		}

		/**
		 * Met tous les éléments à zéro.
		 */
		void setZero()
		{
			// TODO implémenter !
		}

		/**
		 * Accès au tampon de données (en lecteur seulement)
		 */
		const _Scalar* data() const { return m_data; }

		/**
		 * Accès au tampon de données (pour lecture et écriture)
		 */
		_Scalar* data() { return m_data; }
	};

}
