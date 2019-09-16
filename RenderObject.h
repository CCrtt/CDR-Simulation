#pragma once
#include "BasicFunctions.h"

class RenderObject
{
protected:
	Point m_pos; // position du robot

public:
	RenderObject();
	RenderObject(float x, float y, float angle);
	~RenderObject();

	virtual void render(sf::RenderTarget& target) = 0;
};

