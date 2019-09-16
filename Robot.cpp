#include"stdafx.h"

#include "Robot.h"

using namespace std;


robot::robot(int nbRobots, team team) 
	: m_target(xconv(330), yconv(1200), aconv(0)), 
	m_maxAcceleration(0.5f), m_maxSpeed(10.f)
{
	switch (team)
	{
	case (PURPLE):
		m_pos.Setall(xconv(250), yconv(705), aconv(90));
		m_target.Setall(xconv(250), yconv(705), aconv(90));
		break;

	case(YELLOW):
		m_pos.Setall(xconv(2550), yconv(705), aconv(90));
		m_target.Setall(xconv(2500), yconv(705), aconv(90));
		break;

	default:
		break;
	}

	m_length = 110;
	m_width = 100;
	m_rightSpeed = 0.1;
	m_leftSpeed = 0;
	m_delay = 2000;

	sf::RectangleShape shape(sf::Vector2f(m_length, m_width));
	shape.setOrigin(sf::Vector2f(m_length / 2, m_width / 2));
	shape.setFillColor(sf::Color::Blue);
	shape.setOutlineColor(sf::Color::Black);
	shape.setOutlineThickness(2);
	shape.setPosition(sf::Vector2f(m_pos.getX(), m_pos.getY()));
	shape.setRotation(-degre(m_pos.getangle()) - 90);

	m_shape = shape;

	shape.setFillColor(sf::Color(20, 100, 20, 170));
	shape.setPosition(sf::Vector2f(m_pos.getX() + 100, m_pos.getY()));
	shape.setRotation(-degre(m_pos.getangle()) - 90);

	m_shapeTest = shape;

	shape.setFillColor(sf::Color(0, 0, 255, 150));
	shape.setPosition(sf::Vector2f(m_target.getX(), m_target.getY()));
	shape.setRotation(-degre(m_target.getangle()) - 90);

	m_shapeTarget = shape;

	m_IAPthfinding = new AlgoGen(nbRobots);
	m_posOtherRobot.push_back(new Point);
}

robot::robot(float x, float y, float angle, sf::Color color, int nbRobots) 
	: RenderObject(x, y, angle), 
	m_target(400, 100, PI / 2), 
	m_maxAcceleration(0.5f), m_maxSpeed(10.f)
{
	m_length = 110;
	m_width = 100;
	m_rightSpeed = 1;
	m_leftSpeed = 2;
	m_delay = 2000;

	sf::RectangleShape shape(sf::Vector2f(m_length, m_width));
	shape.setOrigin(sf::Vector2f(m_length / 2, m_width / 2));
	shape.setFillColor(color);
	shape.setOutlineColor(sf::Color::Black);
	shape.setOutlineThickness(2);
	shape.setPosition(sf::Vector2f(m_pos.getX(), m_pos.getY()));
	shape.setRotation(-degre(m_pos.getangle()) - 90);

	m_shape = shape;

	shape.setFillColor(sf::Color(255, 0, 0, 150));
	shape.setPosition(sf::Vector2f(m_target.getX(), m_target.getY()));
	shape.setRotation(-degre(m_target.getangle()) - 90);

	m_shapeTarget = shape;

	m_IAPthfinding = new AlgoGen(nbRobots);
}

robot::~robot()
{
	if (m_IAPthfinding)
		delete m_IAPthfinding;

	for (int i = 0; i < m_posOtherRobot.size(); ++i)
	{
		delete m_posOtherRobot[i];
	}
}


sf::RectangleShape robot::draw()
{
	return m_shape;
}

sf::RectangleShape robot::drawTest()
{
	return m_shapeTest;
}

sf::RectangleShape robot::drawTarget()
{
	return m_shapeTarget;
}

void robot::update(sf::Time time)
{
	m_delay = 0;
	if (m_delay == 0)
	{
		frottements(time.asMilliseconds());

		actualise_position(m_rightSpeed, m_leftSpeed);

		m_shape.setPosition(sf::Vector2f(m_pos.getX(), m_pos.getY()));
		m_shape.setRotation(-degre(m_pos.getangle()) - 90);
	}
	else
	{
		m_delay -= time.asMilliseconds();
		if (m_delay < 0)
			m_delay = 0;
	}
}

void robot::updateClavier(float vitD, float vitG) // utile au debug : permet de tester le comportement du robot en deplacant la target au clavier
{
	actualise_positionTarget(vitD, vitG);
	//actualise_position(vitD,vitG); // on peut aussi controler le robot au clavier en inversant les lignes commentees dans cette fonction

	m_shapeTarget.setPosition(sf::Vector2f(m_target.getX(), m_target.getY()));
	m_shapeTarget.setRotation(-degre(m_target.getangle()) - 90);
	//m_shape.setPosition(sf::Vector2f(m_pos.getX(), m_pos.getY()));
	//m_shape.setRotation(-degre(m_pos.getangle()) - 90);
}

void robot::frottements(int time)
{
	m_leftSpeed *= 1 - 0.005 * time * FPS / 1000;
	m_rightSpeed *= 1 - 0.005 * time * FPS / 1000;
}

void robot::actualise_position(float rightSpeed, float leftSpeed)
{
	// determination du cercle décrit par la trajectoire et de la vitesse du robot sur ce cercle

	if (rightSpeed != leftSpeed) {

		float R = 0; // rayon du cercle decrit par la trajectoire
		float d = 0; // vitesse du robot
		float cx = 0; // position du centre du cercle decrit par la trajectoire
		float cy = 0;

		R = ECART_ROUE / 2 * (rightSpeed + leftSpeed) / (leftSpeed - rightSpeed); // rayon du cercle
		cx = m_pos.getX() + R * sin(m_pos.getangle());
		cy = m_pos.getY() + R * cos(m_pos.getangle());
		d = (leftSpeed + rightSpeed) * 0.5;

		// mise à jour des coordonnées du robot
		if (leftSpeed + rightSpeed != 0) {
			m_pos.Setangle(m_pos.getangle() - d / R);//m_angle -= d/R;
		}
		else {
			m_pos.Setangle(m_pos.getangle() + rightSpeed * 2.0 / ECART_ROUE); //m_angle += rightSpeed*2.0 / ECART_ROUE;
		}

		if (m_pos.getangle() > PI)
		{
			m_pos.Setangle(m_pos.getangle() - 2 * PI); //m_angle -= 2*PI;
		}
		else if (m_pos.getangle() <= -PI)
		{
			m_pos.Setangle(m_pos.getangle() + 2 * PI); //m_angle += 2*PI;
		}

		m_pos.setX(cx - R * sin(m_pos.getangle())); //m_xPos = cx - R * sin(m_angle);
		m_pos.setY(cy - R * cos(m_pos.getangle())); //m_yPos = cy - R * cos(m_angle);
	}
	else if (leftSpeed == rightSpeed) { // cas où la trajectoire est une parfaite ligne droite

		m_pos.setX(m_pos.getX() + leftSpeed * cos(m_pos.getangle())); //m_xPos += leftSpeed * cos(m_angle);
		m_pos.setY(m_pos.getY() - rightSpeed * sin(m_pos.getangle())); //m_yPos -= rightSpeed * sin(m_angle);
	}
}

void robot::actualise_positionTarget(float rightSpeed, float leftSpeed)
{
	// determination du cercle décrit par la trajectoire et de la vitesse du robot sur ce cercle

	if (rightSpeed != leftSpeed) {

		float R = 0; // rayon du cercle decrit par la trajectoire
		float d = 0; // vitesse du robot
		float cx = 0; // position du centre du cercle decrit par la trajectoire
		float cy = 0;

		R = ECART_ROUE / 2 * (rightSpeed + leftSpeed) / (leftSpeed - rightSpeed); // rayon du cercle
		cx = m_target.getX() + R * sin(m_target.getangle());
		cy = m_target.getY() + R * cos(m_target.getangle());
		d = (leftSpeed + rightSpeed) * 0.5;

		// mise à jour des coordonnées du robot
		if (leftSpeed + rightSpeed != 0) {
			m_target.Setangle(m_target.getangle() - d / R);//m_angle -= d/R;
		}
		else {
			m_target.Setangle(m_target.getangle() + rightSpeed * 2.0 / ECART_ROUE); //m_angle += rightSpeed*2.0 / ECART_ROUE;
		}

		if (m_target.getangle() > PI)
		{
			m_target.Setangle(m_target.getangle() - 2 * PI); //m_angle -= 2*PI;
		}
		else if (m_target.getangle() <= -PI)
		{
			m_target.Setangle(m_target.getangle() + 2 * PI); //m_angle += 2*PI;
		}

		m_target.setX(cx - R * sin(m_target.getangle())); //m_xPos = cx - R * sin(m_angle);
		m_target.setY(cy - R * cos(m_target.getangle())); //m_yPos = cy - R * cos(m_angle);
	}
	else if (leftSpeed == rightSpeed) { // cas où la trajectoire est une parfaite ligne droite

		m_target.setX(m_target.getX() + leftSpeed * cos(m_target.getangle())); //m_xPos += leftSpeed * cos(m_angle);
		m_target.setY(m_target.getY() - rightSpeed * sin(m_target.getangle())); //m_yPos -= rightSpeed * sin(m_angle);
	}
}

bool robot::delay()
{
	if (m_delay > 0)
		return true;
	return false;
}

void robot::play(float time_available)
{
	std::cout << "running" << std::endl;
	m_IAPthfinding->run(time_available, m_rightSpeed, m_leftSpeed, m_pos, m_target, m_posOtherRobot);
	std::cout << "vitesses appliquees : " << m_rightSpeed << ", " << m_leftSpeed << std::endl;

	if (reachTarget())
	{
		cout << "target reached" << std::endl;
		retarget();
	}
}

void robot::render(sf::RenderTarget& target)
{
	target.draw(m_shape);
	target.draw(m_shapeTarget);
	//target.draw(m_shapeTest);

	m_IAPthfinding->render(target);
}

bool robot::reachTarget()
{
	if (dist(m_target.getX() - m_pos.getX(), m_target.getY() - m_pos.getY()) < 13 && val_abs(properAngleRad(m_target.getangle() - m_pos.getangle())) < 0.05 && val_abs(m_rightSpeed) < 0.1 && val_abs(m_leftSpeed) < 0.1)
		return true;
	return false;
}

void robot::retarget()
{
	float x = (float)(rand()) * 1150 / RAND_MAX + 200;
	float y = (float)(rand()) * 400 / RAND_MAX + 200;
	float ang = ((float)(rand()) / RAND_MAX - 0.5) * PI;

	m_target.Setall(x, y, ang);

	m_shapeTarget.setPosition(sf::Vector2f(m_target.getX(), m_target.getY()));
	m_shapeTarget.setRotation(-degre(m_target.getangle()) - 90);
}