#include"stdafx.h"

#include "Pathfinding.h"

Pathfinding::Pathfinding()
{
	sf::RectangleShape shape(sf::Vector2f(robotLength, robotWidth));
	shape.setOrigin(sf::Vector2f(robotLength / 2, robotWidth / 2));
	shape.setFillColor(sf::Color(70, 200, 70, 170));
	shape.setOutlineColor(sf::Color::Black);
	shape.setOutlineThickness(2);
	shape.setPosition(sf::Vector2f(-robotLength, -robotWidth));
	shape.setRotation(0);

	m_shapeDebug = shape;
}

Pathfinding::~Pathfinding()
{
}
