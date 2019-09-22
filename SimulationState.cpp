#include"stdafx.h"

#include "SimulationState.h"

void SimulationState::initKeybinds()
{
	std::ifstream ifs("Config/gamestate_keybinds.ini");

	if (ifs.is_open())
	{
		std::string key = "";
		std::string key2 = "";

		while (ifs >> key >> key2)
		{
			this->keybinds[key] = this->supportedKeys->at(key2);
		}
	}

	ifs.close();
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

void SimulationState::initRobot(sf::RenderWindow* window)
{
	for (int i = 0; i < m_nbRobotPurple; i++)
	{
		m_robotTeamPurple.push_back(new robot(2, team::PURPLE, sf::Vector2i(27 * (int)cmToPx.x, 27 * (int)cmToPx.y)));
	}
	for (int i = 0; i < m_nbRobotYellow; i++)
	{
		m_robotTeamPurple.push_back(new robot(2, team::YELLOW, sf::Vector2i(27 * (int)cmToPx.x, 27 * (int)cmToPx.y)));
	}

	this->frameTime = frameClock.getElapsedTime();
	this->genTime = genClock.getElapsedTime();
}

void SimulationState::initPauseMenu()
{
	this->pauseMenu = NULL;
}

void SimulationState::initFont()
{
	if (!this->pauseMenuFont.loadFromFile("Ressources/Fonts/Dosis-Light.ttf"))
	{
		throw("ERROR::MAINMENUSTATE::COULD NOT LOAD FONT");
	}
}

void SimulationState::resetPauseMenu()
{
	if (pauseMenu)
		delete this->pauseMenu;

	this->pauseMenu = new PauseMenu(*this->window, this->pauseMenuFont);
}

void SimulationState::updatePauseMenuTextButtons()
{
	if (this->pauseMenu->isTextButtonPressed("QUIT"))
		this->endState();
}

void SimulationState::updatePosRobots()
{
	for (size_t i = 0; i < m_robotTeamPurple.size(); ++i)
	{
		//posRobots = m_robotTeamPurple[i]->getPos();
		//m_robotTeamPurple[i]->play(16.f / (m_nbRobotPurple + m_nbRobotYellow));
	}
	for (size_t i = 0; i < m_robotTeamYellow.size(); ++i)
	{
		m_robotTeamYellow[i]->play(16.f / (m_nbRobotPurple + m_nbRobotYellow));

	}
}

void SimulationState::updateRobots(const float& dt)
{
	if (totalClock <= 100.f)
	{
		for (size_t i = 0; i < m_robotTeamPurple.size(); ++i)
		{
			m_robotTeamPurple[i]->play(16.f / (m_nbRobotPurple + m_nbRobotYellow));
		}
		for (size_t i = 0; i < m_robotTeamYellow.size(); ++i)
		{
			m_robotTeamYellow[i]->play(16.f / (m_nbRobotPurple + m_nbRobotYellow));

		}

		//std::cout << frameTime.asMilliseconds() << std::endl;

		for (size_t i = 0; i < m_robotTeamPurple.size(); ++i)
		{
			m_robotTeamPurple[i]->update(dt);
		}

		for (size_t i = 0; i < m_robotTeamYellow.size(); ++i)
		{
			m_robotTeamYellow[i]->update(dt);
		}
	}
	else
	{
	}
}

// Constructor/Destructor
SimulationState::SimulationState(sf::RenderWindow* window, std::map<std::string, int>* supportedKeys, std::stack<State*>* states)
	: State(window, supportedKeys, states), terrain(window->getSize()), m_nbRobotPurple(1), m_nbRobotYellow(1), 
	cmToPx((float) window->getSize().x / 300.f, (float)window->getSize().y / 200.f)
{
	//this->posRobots = new Point[m_nbRobotPurple + m_nbRobotYellow];

	this->initFont();
	this->initKeybinds();
	this->initPauseMenu();
	this->initBackGround();
	this->initRobot(window);
}

SimulationState::~SimulationState()
{
	for (size_t i = 0; i < m_robotTeamPurple.size(); ++i)
	{
		delete m_robotTeamPurple[i];
	}
	for (size_t i = 0; i < m_robotTeamYellow.size(); ++i)
	{
		delete m_robotTeamYellow[i];
	}
}

// Functions
std::vector<Point> SimulationState::posRobots()
{
	std::vector<Point> ret;

	for (int i = 0; i < m_nbRobotPurple; i++)
	{
		ret.push_back(m_robotTeamPurple[i]->getPos());
	}

	for (int i = 0; i < m_nbRobotYellow; i++)
	{
		ret.push_back(m_robotTeamYellow[i]->getPos());
	}

	return ret;
}

void SimulationState::updateInput(const float& dt)
{
	if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key(this->keybinds.at("CLOSE"))) && this->getKeytime())
	{
		if (!this->paused)
		{
			resetPauseMenu();
			this->pauseState();
		}
		else
			this->unPauseState();
	}
}

void SimulationState::update(const float& dt, const float wheelTicks)
{
	this->totalClock += dt;
	this->updateKeytime(dt);

	this->updateMousePositions();
	this->updateInput(dt);

	if (!this->paused)
	{
		this->updateRobots(dt);
	}
	else
	{
		this->pauseMenu->update(this->mousePosView);
		this->updatePauseMenuTextButtons();
	}
}

void SimulationState::render(sf::RenderTarget* target)
{
	if (!target)
		target = this->window;

	target->draw(this->backGround);

	for (size_t i = 0; i < m_robotTeamPurple.size(); ++i) {
		m_robotTeamPurple[i]->render(*target);
	}
	for (size_t i = 0; i < m_robotTeamYellow.size(); ++i) {
		m_robotTeamYellow[i]->render(*target);
	}

	terrain.render(*window);

	if (this->paused) // Pause menu render
	{
		this->pauseMenu->render(*target);


		//REMOVE LATER
		sf::Text mouseText;
		mouseText.setPosition(this->mousePosView.x, this->mousePosView.y - 15);
		mouseText.setFont(this->pauseMenuFont);
		mouseText.setCharacterSize(12);
		std::stringstream ss;
		ss << this->mousePosView.x << " " << this->mousePosView.y;
		mouseText.setString(ss.str());
		target->draw(mouseText);
	}
}
