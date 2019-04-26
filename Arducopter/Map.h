#pragma once
#include"RBTree.h"
using namespace std;
template<class K, class V>
class Map {
public:
	typedef pair<K, V> valueType;
	typedef typename RBTree<K, V>::Iterator Iterator;
public:
	Map()
		:_t()
	{}

	pair<Iterator, bool> Insert(const valueType& v)
	{
		return _t.InsertUnique(v);
	}

	bool Empty()const
	{
		return _t.Empty();
	}

	size_t Size()const //bug
	{
		return _t.Size();
	}

	V& operator[](const K& key)
	{
		Iterator ret = _t.InsertUnique(pair<K, V>(key, V())).first;
		return (*ret).second;
	}

	Iterator Begin()
	{
		if(Empty())
			return _t.End();
		else
			return _t.Begin();
	}

	Iterator End()
	{
		return _t.End();
	}

private:
	RBTree<K, V> _t;
};


