#pragma once
class Terrain
{
private:
	const sf::Vector2u m_size;

	std::vector<sf::RectangleShape*> sprites;

public:
	Terrain(sf::Vector2u size);
	~Terrain();

	void render(sf::RenderTarget& target) const;

	bool collision(sf::IntRect Hitbox) const;
	sf::Vector2u getSize() const;
};

