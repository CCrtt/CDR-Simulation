#include "stdafx.h"
#include "PosOtherRobots.h"

PosOtherRobots::PosOtherRobots(int nbRobots, int nbTours)
	: m_nbOtherRobots(nbRobots- 1), m_nbTours(nbTours)
{
	m_timeLastSeen = new float[m_nbOtherRobots];
	for (int i = 0; i < m_nbOtherRobots; ++i)
	{
		m_timeLastSeen[i] = 0.f;
	}

	m_posOtherRobots = new Point[m_nbOtherRobots * nbTours];
	for (int i = 0; i < m_nbOtherRobots; ++i)
	{
		m_posOtherRobots[i] = Point();
	}
}

PosOtherRobots::~PosOtherRobots()
{
	delete[] m_posOtherRobots;
}

Point PosOtherRobots::getPos(int i, int j) const
{
	return m_posOtherRobots[i* m_nbOtherRobots + j];
}

void PosOtherRobots::update(const std::vector<Point*>& robot_pos, const Point& actualPos)
{
	// mise à jour de la pos des autres robots

	//je veux identifier chaque robot
	for (size_t i = 0; i < robot_pos.size(); ++i)
	{
		if (*robot_pos[i] != actualPos)
		{
			Point lastPos = m_posOther[i][0];

			m_posOther[i][0] = *robot_pos[i];

			for (int j = 1; j < NB_TOURS_SIMULES; j++)
			{
				m_posOther[i][j] = m_posOther[i][0] + (float)j * (m_posOther[i][0] - lastPos);
			}
		}
	}
}
