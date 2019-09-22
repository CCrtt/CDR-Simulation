#include"stdafx.h"
#include "Terrain.h"

Terrain::Terrain(sf::Vector2u size)
	: m_size(size)
{
	sprites.push_back(new sf::RectangleShape(sf::Vector2f((float)m_size.x, 10.f)));
	sprites[0]->setPosition(0.f, 0.f);
	sprites[0]->setFillColor(sf::Color(50, 50, 50, 150));
}

Terrain::~Terrain()
{
	for (size_t i = 0; i < sprites.size(); ++i)
	{
		delete sprites[i];
		sprites[i] = NULL;
	}
}

void Terrain::render(sf::RenderTarget& target) const
{
	for (size_t i = 0; i < sprites.size(); i++)
	{
		target.draw(*sprites[i]);
	}
}

bool Terrain::collision(sf::IntRect Hitbox) const
{
	for (size_t i = 0; i < sprites.size(); ++i)
	{
		if (sprites[i]->getTextureRect().intersects(Hitbox))
		{
			return true;
		}
	}

	return false;
}

sf::Vector2u Terrain::getSize() const
{
	return m_size;
}