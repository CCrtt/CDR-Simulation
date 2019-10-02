#pragma once

#include"Point.h"
#include"BasicFunctions.h"

class PosOtherRobots
{
private:
	float* m_timeLastSeen;
	Point* m_posOtherRobots;
	int m_nbOtherRobots;
	int m_nbTours;

public:
	PosOtherRobots(int nbRobots, int nbTours);
	~PosOtherRobots();

	Point getPos(int i, int j) const;

	void update(const std::vector <Point*>& robot_pos, const Point& actualPos);

};

