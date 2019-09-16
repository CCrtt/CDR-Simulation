#include"stdafx.h"

#include "SimulationState.h"

void SimulationState::initKeybinds()
{
}

void SimulationState::initBackGround()
{
	this->backGround.setSize(
		sf::Vector2f(
			static_cast<float>(this->window->getSize().x),
			static_cast<float>(this->window->getSize().y)
		)
	);
	if (!this->backGroundTexture.loadFromFile("Capture.png"))
	{
		std::cout << "loading error" << std::endl;
	}
	
	this->backGround.setTexture(&backGroundTexture);
}

void SimulationState::initRobot()
{
	for (int i = 0; i < m_nbRobotPurple; i++)
	{
		m_robotTeamPurple.push_back(new robot(2, team::PURPLE));
	}
	for (int i = 0; i < m_nbRobotYellow; i++)
	{
		m_robotTeamPurple.push_back(new robot(2, team::YELLOW));
	}

	for (int i = 0; i < m_robotTeamPurple.size(); ++i)
	{
		m_robotTeamPurple[i]->startGen(0, 0);
	}

	for (int i = 0; i < m_robotTeamYellow.size(); ++i)
	{
		m_robotTeamYellow[i]->startGen(0, 0);
	}

	//this->robot1.startGen(robot2.getX(), robot2.getY());
	//this->robot2.startGen(robot1.getX(), robot1.getY());

	this->frameTime = frameClock.getElapsedTime();
	this->genTime = genClock.getElapsedTime();
}

SimulationState::SimulationState(sf::RenderWindow* window, std::map<std::string, int>* supportedKeys, std::stack<State*>* states)
	: State(window, supportedKeys, states), m_nbRobotPurple(1), m_nbRobotYellow(0) //robot1(), robot2(700, 460, PI / 4, sf::Color::Red)
{
	this->initBackGround();
	this->initRobot();
}

SimulationState::~SimulationState()
{
	for (int i = 0; i < m_robotTeamPurple.size(); ++i)
	{
		delete m_robotTeamPurple[i];
	}
	for (int i = 0; i < m_robotTeamYellow.size(); ++i)
	{
		delete m_robotTeamYellow[i];
	}
}

/*std::vector<Point> SimulationState::posRobots()
{
	std::vector<Point> ret;

	for (int i = 0; i < m_robotTeamPurple.size(); ++i)
	{
		ret.push_back(m_robotTeamPurple[i]->getPos());
	}

	for (int i = 0; i < m_robotTeamYellow.size(); ++i)
	{
		ret.push_back(m_robotTeamYellow[i]->getPos());
	}

	return ret;
}*/

void SimulationState::updateInput(const float& dt)
{
}

void SimulationState::update(const float& dt, const float wheelTicks)
{
	this->totalClock += dt;
	//std::cout << totalClock << std::endl;

	this->frameTime = this->frameClock.restart();

	if (totalClock <= 100.f)
	{
		for (int i = 0; i < m_robotTeamPurple.size(); ++i)
		{
			m_robotTeamPurple[i]->play(16.f / (m_nbRobotPurple + m_nbRobotYellow));
		}
		for (int i = 0; i < m_robotTeamYellow.size(); ++i)
		{
			m_robotTeamYellow[i]->play(16.f / (m_nbRobotPurple + m_nbRobotYellow));

		}

		/*while (frameTime.asMilliseconds() < 17) // 17 correspond au nombre de milisecondes qu'il faut attendre pour afficher 60 FPS
		{
			//utilisation de l'algo gen
			if (genTime.asMilliseconds() < TEMPS_GEN) // TEMPS_GEN correspond au temps dont dispose le robot pour trouver une trajectoire avant de reactualiser ses entrees
			{
				for (int i = 0; i < 20; i++)
				{
					for (int i = 0; i < m_robotTeamPurple.size(); ++i)
					{
						m_robotTeamPurple[i]->genSol();
					}

					for (int i = 0; i < m_robotTeamYellow.size(); ++i)
					{
						m_robotTeamYellow[i]->genSol();
					}
					//robot1.genSol(); // generation d'une nouvelle solution
					//robot2.genSol();
					//std::cout << "generation d'une nouvelle solution" << std::endl;
				}
			}
			else
			{
				genClock.restart();

				for (int i = 0; i < m_robotTeamPurple.size(); ++i)
				{
					m_robotTeamPurple[i]->endGen();
					m_robotTeamPurple[i]->startGen(0, 0);
				}

				for (int i = 0; i < m_robotTeamYellow.size(); ++i)
				{
					m_robotTeamYellow[i]->endGen();
					m_robotTeamYellow[i]->startGen(0, 0);
				}

				//robot1.endGen(); // on termine la recherche
				//robot1.startGen(robot2.getX(), robot2.getY()); // et on la relance avec les nouvelles entrees
				//robot2.endGen();
				//robot2.startGen(robot1.getX(), robot1.getY());
				std::cout << "startGen" << std::endl;
			}

			genTime = genClock.getElapsedTime();
			frameTime = frameClock.getElapsedTime();
		}*/

		//std::cout << frameTime.asMilliseconds() << std::endl;

		for (int i = 0; i < m_robotTeamPurple.size(); ++i)
		{
			m_robotTeamPurple[i]->update(frameTime);
			std::cout << "update" << std::endl;
		}

		for (int i = 0; i < m_robotTeamYellow.size(); ++i)
		{
			m_robotTeamYellow[i]->update(frameTime);
			std::cout << "update" << std::endl;
		}

		/*robot1.update(frameTime); //update prend le temps en argument afin que l'affichage soit identique peu importe le nbr de fps
		robot2.update(frameTime);*/

		frameClock.restart();
	}
	else
	{
		//this->states->push(new PauseState());
	}
}

void SimulationState::render(sf::RenderTarget* target)
{
	if (!target)
		target = this->window;

	target->draw(this->backGround);

	for (int i = 0; i < m_robotTeamPurple.size(); ++i)
	{
		/*target->draw(m_robotTeamPurple[i]->draw());
		target->draw(m_robotTeamPurple[i]->drawTarget());
		target->draw(m_robotTeamPurple[i]->drawTest());*/
		m_robotTeamPurple[i]->render(*target);
	}

	for (int i = 0; i < m_robotTeamYellow.size(); ++i)
	{
		/*target->draw(m_robotTeamYellow[i]->draw());
		target->draw(m_robotTeamYellow[i]->drawTarget());*/
		m_robotTeamYellow[i]->render(*target);
	}

	/*target->draw(robot1.draw());
	target->draw(robot2.draw());
	target->draw(robot1.drawTarget());
	target->draw(robot2.drawTarget());*/
}
