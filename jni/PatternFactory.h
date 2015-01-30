#pragma once
#include "Globals.h"

enum Type
{
	bj1,
	zg1,
};
class PatternFactory
{
public:
	PatternFactory();
	~PatternFactory();
	void buildPattern(Type t);
};

