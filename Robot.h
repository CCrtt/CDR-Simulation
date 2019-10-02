#ifndef ROBOT1_H
#define ROBOT1_H

#include"AlgoGen.h"
#include"Capteurs.h"
#include "Point.h"
#include"BasicFunctions.h"
#include "constantes.h"
#include "RenderObject.h"

class robot : public RenderObject
{
private:
	//Point m_pos; // position du robot
	Point* m_target;	// position visee
	int m_length;	// dimensions du rectangle représentant le robot
	int m_width;
	float m_leftSpeed; // vitesses des roues droite et gauche
	float m_rightSpeed;
	const float m_maxAcceleration;
	const float m_maxSpeed;
	sf::RectangleShape m_shape; // forme du robot pour le rendu graphique
	sf::RectangleShape m_shapeTest;
	sf::RectangleShape m_shapeTarget;
	float m_delay;
	const team m_team;

	sf::Sprite m_sprite;
	sf::Texture m_texture;

	Pathfinding* m_IAPthfinding;
	std::vector <Capteurs*> capteurs;
	std::vector<Point*> m_posOtherRobot;

	// Functions
	void actualise_position(float rightSpeed, float leftSpeed);
	void actualise_positionTarget(float rightSpeed, float leftSpeed);
	void frottements(float dt);

	void retarget();
	bool reachTarget();
	void setTarget(const Point target);
	void deleteTarget();

	bool delay();

	void updatePosOtherRobots(const std::vector<Point>& posRobots);

public:
	robot(int nbRobots, team team, sf::Vector2i taille);
	virtual ~robot();

	void update(const float dt);
	void updateClavier(float vitD, float vitG);
	void play(float time_available, const std::vector<Point>& posRobots);
	void render(sf::RenderTarget& target);
};

#endif // ROBOT1_H
