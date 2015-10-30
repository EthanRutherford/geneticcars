#ifndef WORLD_H
#define WORLD_H

#include <algorithm>
#include "solver.h"

struct Car{
	Body* b;
	std::vector<Vector2D> vertices;
	double ws1, ws2, wd1, wd2, cd;
	int wv1, wv2;
	double distance;
	int health;
	bool alive;
};

inline bool operator<(Car a, Car b)
{
	return a.distance > b.distance;
}

class World{
	public:
		World();
		
		void createfloor();
		Body* addBox(double x, double y);
		void addCar(Car& c);
		Car& randomCar();
		void checklife();
		void newGen();
		Car& makebaby(Car a, Car b);
		Car& mutate(Car a);
		
		void drawBodies();
		void Solve(int time);
		double GetProfile(std::string s) {return solver.GetProfile(s);}
		
		Vector2D getPos();
		double best() {return bestDist;}
	private:
		void saveChamp(Car& c);
		Car Champion();
		Solver solver;
		std::vector<Car> cars;
		Vector2D pos;
		int dead;
		double bestDist;
};

#endif