#pragma once

#include"State.h"
#include"Robot.h"
#include"constantes.h"

class SimulationState
	: public State
{
private:
	sf::RectangleShape backGround;
	sf::Texture backGroundTexture;

	float totalClock;

	sf::Clock frameClock;
	sf::Clock genClock;
	sf::Time frameTime;
	sf::Time genTime;

	const int m_nbRobotYellow;
	const int m_nbRobotPurple;
	std::vector <robot*> m_robotTeamYellow;
	std::vector <robot*> m_robotTeamPurple;
	//robot robot1;
	//robot robot2;

	virtual void initKeybinds();
	void initBackGround();
	void initRobot();

public:
	SimulationState(sf::RenderWindow* window, std::map<std::string, int>* supportedKeys, std::stack<State*>* states);
	~SimulationState();

	std::vector<Point> posRobots();

	virtual void updateInput(const float& dt);
	virtual void update(const float& dt, const float wheelTicks = 0);
	virtual void render(sf::RenderTarget* target = 0);
};

