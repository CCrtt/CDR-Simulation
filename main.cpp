#include <SFML/Graphics.hpp>
#include <ctime>
#include <cstdlib>
#include <iostream>

#include "constantes.h"
#include "robot.h"

using namespace std;

/**

Simulation du comportement du robot pour la coupe du monde de robotique avec AREM.

Autheur : Corentin COURTOT

**/

sf::RenderWindow window;


int main()
{
    //creation de la fenetre

    sf::ContextSettings settings;
    settings.antialiasingLevel = 8;

    window.create(sf::VideoMode(WINDOW_WIDTH, WINDOW_HEIGHT), "CDFR", sf::Style::Default, settings);
    //window.setFramerateLimit(FPS);
    window.setPosition(sf::Vector2i(100,0));

    sf::Sprite fond;
    sf::Texture texture;
    if (!texture.loadFromFile("Capture.jpeg")){}
    fond.setTexture(texture);

    //creation des objets
    robot robot1;
    robot robot2(700, 460, PI/4, sf::Color::Red);

	//creation des vitesses pour la commande au clavier
	float vitD = 0;
	float vitG = 0;

    sf::Clock clock;
    sf::Time time = clock.getElapsedTime();

    sf::Clock clockGen;
    sf::Time timeGen = clockGen.getElapsedTime();

    robot1.startGen(robot2.getX(), robot2.getY());
    robot2.startGen(robot1.getX(), robot1.getY());

    int gen = 0;

    //boucle principale

    while (window.isOpen())
    {
        //gere la fermeture  de la fenetre
        sf::Event event;
        while (window.pollEvent(event))
        {
            if (event.type == sf::Event::Closed)
            {
                window.close();
            }
        }

        window.clear();

        time = clock.getElapsedTime();
        timeGen = clockGen.getElapsedTime();
        while(time.asMilliseconds()<17) // 17 correspond au nombre de milisecondes qu'il faut attendre pour afficher 60 FPS
        {
            //utilisation de l'algo gen
            if (timeGen.asMilliseconds()<TEMPS_GEN) // TEMPS_GEN correspond au temps dont dispose le robot pour trouver une trajectoire avant de reactualiser ses entrees
            {
                for (int i=0; i<20; i++)
                {
                    robot1.genSol(); // generation d'une nouvelle solution
                    robot2.genSol();
                }
            }
            else
            {
                clockGen.restart();
				robot1.endGen(); // on termine la recherche
                robot1.startGen(robot2.getX(), robot2.getY()); // et on la relance avec les nouvelles entrees
				robot2.endGen();
                robot2.startGen(robot1.getX(), robot1.getY());
            }

            timeGen = clockGen.getElapsedTime();
            time = clock.getElapsedTime();
        }

		robot1.update(time); //update prend le temps en argument afin que l'affichage soit identique peu importe le nbr de fps
        robot2.update(time);
        clock.restart();

		// gestion des entrées clavier
		if (sf::Keyboard::isKeyPressed(sf::Keyboard::Z)) {
			vitD += 0.5;
			vitG += 0.5;
		}
		if (sf::Keyboard::isKeyPressed(sf::Keyboard::S)) {
			vitD -= 0.2;
			vitG -= 0.2;
		}
		if (sf::Keyboard::isKeyPressed(sf::Keyboard::Q)) {
			vitD += 0.2;
			vitG -= 0.2;
		}
		if (sf::Keyboard::isKeyPressed(sf::Keyboard::D)) {
			vitD -= 0.2;
			vitG += 0.2;
		}
		vitD = vitD * 0.9;
		vitG = vitG * 0.9;

		robot1.updateClavier(vitD, vitG);

        //dessine les entites
        window.draw(fond);
        window.draw(robot1.draw());
        window.draw(robot2.draw());
        //window.draw(robot1.drawTest()); //affiche la pos a laquelle arrivera le robot s'il applique la meilleure sol trouvee
        window.draw(robot1.drawTarget());
        window.draw(robot2.drawTarget());

        window.display();
    }

    return 0;
}
