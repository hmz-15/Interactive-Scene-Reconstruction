#ifndef _PGM_PAIR_SET_H
#define _PGM_PAIR_SET_H

#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace pgm
{

struct pairHash
{
	template <class T1, class T2>
	std::size_t operator () (std::pair<T1, T2> const &pair) const
	{
		std::size_t h1 = std::hash<T1>()(pair.first);
		std::size_t h2 = std::hash<T2>()(pair.second);

		return h1 ^ h2;
	}
};

template<class T1, class T2>
using PairSet = std::unordered_set<std::pair<T1, T2>, pairHash>;

template<class T1, class T2, class T3>
using PairMap = std::unordered_map<std::pair<T1, T2>, T3, pairHash>;

} // end of namespace pgm


#endif