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


	//genetique
	void startGen(float x, float y);
	void endGen();
	void actualisePositionTest(float rightSpeed, float leftSpeed, int i);
	float evalueSol() const;
	void swapSol(int sol1, int sol2);
	int chercheMeilleure(int indiceDepart);
	void genSol();
	void crossOver(int ind1, int ind2, int a);
	void mutate();
	void brake(int ind);
	void accel(int ind);
	void turnRight(int ind);
	void turnLeft(int ind);
	void retarget();

	float getX();
	float getY();
	Point getPos();

	void testCollisionTest();
	void testUpCollisionTest();
	void testBotCollisionTest();
	void testLeftCollisionTest();
	void testRightCollisionTest();

	void testCollisionEn(int i);

	void score(int indSol);
	void distSolTest(int indSol);
	int testDist();
	bool reachTarget();
	bool reachTargetTest(int i);

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

	//genetique
	Pathfinding* m_IAPthfinding;
	std::vector<Point*> m_posOtherRobot;


	float m_sol[NB_SOL + 1][NB_TOURS_SIMULES + 1][2]; //[n° de la sol (nb_sol+1 pour stocker la sol testee)]
														//[avancee dans le temps(NB_TOURS_SIMULES+1 pour stocker le score)][droite, gauche]
	int m_posEn[2][NB_TOURS_SIMULES]; // pos d'un robot ennemi
	bool m_collision; // =true si une collision a lieu lors de la simulation, faux sinon
	int m_malus; // malus utilisé pour noter les solutions generees
	int m_nbMutations; // utile pour des statistiques

	Point m_posTest; // utilisée pour simuler l'avancee du robot virtuellement sur plusieurs tours
	Point m_posDep; // position de depart a partir de laquelle on va rechercher la trajectoire optimale. Cette position doit rester fixe pendant toute la duree de la recherche
};

#endif // ROBOT1_H
