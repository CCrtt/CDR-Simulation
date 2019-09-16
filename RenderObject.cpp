#include "stdafx.h"
#include "RenderObject.h"

RenderObject::RenderObject() : m_pos(xconv(250), yconv(705), aconv(90))
{
}

RenderObject::RenderObject(float x, float y, float angle) : m_pos(x, y, angle)
{
}

RenderObject::~RenderObject()
{
}
