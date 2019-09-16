#ifndef ROBOT1_H
#define ROBOT1_H

#include"AlgoGen.h"
#include "Point.h"
#include"BasicFunctions.h"
#include "constantes.h"
#include "RenderObject.h"

class robot : public RenderObject
{
public:
	robot(int nbRobots, team team);
	robot(float x, float y, float angle, sf::Color color, int nbRobots);
	virtual ~robot();
	sf::RectangleShape draw();
	sf::RectangleShape drawTarget();
	sf::RectangleShape drawTest();
	void update(sf::Time time);
	void updateClavier(float vitD, float vitG);
	void actualise_position(float rightSpeed, float leftSpeed);
	void actualise_positionTarget(float rightSpeed, float leftSpeed);
	void frottements(int time);

	bool delay();

	void play(float time_available);

	void render(sf::RenderTarget& target);

	void retarget();
	bool reachTarget();

protected:

private:
	//Point m_pos; // position du robot
	Point m_target;	// position visee
	int m_length;	// dimensions du rectangle représentant le robot
	int m_width;
	float m_leftSpeed; // vitesses des roues droite et gauche
	float m_rightSpeed;
	const float m_maxAcceleration;
	const float m_maxSpeed;
	sf::RectangleShape m_shape; // forme du robot pour le rendu graphique
	sf::RectangleShape m_shapeTest;
	sf::RectangleShape m_shapeTarget;
	int m_delay;

	Pathfinding* m_IAPthfinding;
	std::vector<Point*> m_posOtherRobot;

};

#endif // ROBOT1_H
