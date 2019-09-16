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
	if (!this->backGroundTexture.loadFromFile("Ressources/Images/Terrain.png"))
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
		m_robotTeamPurple[i]->render(*target);
	}

	for (int i = 0; i < m_robotTeamYellow.size(); ++i)
	{
		m_robotTeamYellow[i]->render(*target);
	}
}
