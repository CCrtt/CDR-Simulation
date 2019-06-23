#ifndef ROBOT1_H
#define ROBOT1_H

#include <SFML/Graphics.hpp>

#include "point.h"

class robot
{
    public:
        robot();
        robot(double x, double y, double angle, sf::Color color);
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


		//genetique
        void startGen(double x, double y);
		void endGen();
        void actualisePositionTest(float rightSpeed, float leftSpeed, int i);
        double evalueSol() const;
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

        double getX();
        double getY();

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
        Point m_pos;
        Point m_target;
        int m_targetNumber;
        int m_posEn[2][NB_TOURS_SIMULES];
        int m_length;
        int m_width;
        double m_leftSpeed;
        double m_rightSpeed;
        double m_maxAcceleration;
        sf::RectangleShape m_shape;
        sf::RectangleShape m_shapeTest;
        sf::RectangleShape m_shapeTarget;
        int m_paletCount;
        int m_delay;

        //genetique
        double m_sol[NB_SOL+1][NB_TOURS_SIMULES+1][2]; //[n° de la sol (nb_sol+1 pour stocker la sol testee)]
                                                            //[avancee dans le temps(NB_TOURS_SIMULES+1 pour stocker le score)][droite, gauche]
        bool m_collision;
        int m_malus;
        int id;
        int m_nbMutations;

        Point m_posTest;
        Point m_posDep;
};

#endif // ROBOT1_H
