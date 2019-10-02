#include "stdafx.h"
#include "RenderObject.h"

RenderObject::RenderObject() : m_pos(0, 0, 0)
{
}

RenderObject::RenderObject(float x, float y, float angle) : m_pos(x, y, angle)
{
}

RenderObject::~RenderObject()
{
}

Point RenderObject::getPos() const
{
	return m_pos;
}
