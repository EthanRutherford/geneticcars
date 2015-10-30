#include "world.h"
#include <gl/freeglut.h>
#include <time.h>
#include <fstream>

double drand()
{
	return static_cast <double> (rand()) / static_cast <double> (RAND_MAX);
}

World::World()
{
	srand(time(NULL));
	createfloor();
	for (int i = 0; i < 20; i++)
		addCar(randomCar());
	pos = Vector2D(0, 0);
	dead = 0;
	bestDist = 0;
}

void World::createfloor()
{
	Body* last;
	Vector2D position(-5,0);
	for (int i = 0; i < 200; i++)
	{
		last = addBox(position.x, position.y);
		double angle = (drand() * 3 - 1.5) * 1.5 * i / 200.0;
		last->Orient(angle);
		last->position = position -
			last->transform * ((mPolygon*)last->shape[0])->pt[3];
		position = last->position + last->transform * ((mPolygon*)last->shape[0])->pt[2];
	}
}

Body* World::addBox(double x, double y)
{
	mPolygon poly;
	mShape* shape = &poly;
	poly.SetBox(.75,.075);
	Body* b = new Body(&shape, 1, x, y, 1, true);
	solver.addBody(b);
	b->filtergroup = 3;
	b->restitution = .3;
	b->Friction = .5;
	return b;
}

void World::addCar(Car& c)
{
	mPolygon poly[8];
	for (int i = 0; i < 8; i++)
	{
		Vector2D vertices[] = {
			Vector2D(0, 0),
			c.vertices[i],
			c.vertices[(i+1)%8]
		};
		poly[i].Set(vertices, 3);
	}
	mShape* shapes[] = {
		&poly[0],
		&poly[1],
		&poly[2],
		&poly[3],
		&poly[4],
		&poly[5],
		&poly[6],
		&poly[7]
	};
	Body* b0 = new Body(shapes, 8, 0, 2, c.cd, false);
	c.b = b0;
	mCircle circle1(c.ws1);
	mCircle circle2(c.ws2);
	mShape* shape = &circle1;
	Body* b1 = new Body(&shape, 1, c.vertices[c.wv1].x, c.vertices[c.wv1].y + 2, c.wd1, false);
	shape = &circle2;
	Body* b2 = new Body(&shape, 1, c.vertices[c.wv2].x, c.vertices[c.wv2].y + 2, c.wd2, false);
	solver.addBody(b1);
	solver.addBody(b2);
	solver.addBody(b0);
	b0->filtergroup = 2;
	b1->filtergroup = 2;
	b2->filtergroup = 2;
	b0->Friction = 10;
	b1->Friction = 1;
	b2->Friction = 1;
	double torque[2];
	double mass = b0->mass.m + b1->mass.m + b2->mass.m;
	torque[0] = mass * 10 / c.ws1;
	torque[1] = mass * 10 / c.ws2;
	RevJoint* j1 = new RevJoint(b0, b1, c.vertices[c.wv1], Vector2D(0, 0));
	RevJoint* j2 = new RevJoint(b0, b2, c.vertices[c.wv2], Vector2D(0, 0));
	j1->SetMotor(true, -20, torque[0]);
	j2->SetMotor(true, -20, torque[1]);
	solver.addJoint(j1);
	solver.addJoint(j2);
}

Car& World::randomCar()
{
	Car c;
	c.ws1 = drand() * .5 + .2;
	c.ws2 = drand() * .5 + .2;
	c.wd1 = drand() * 1 + .4;
	c.wd2 = drand() * 1 + .4;
	c.cd = drand() * 3 + .3;
	c.vertices.emplace_back(drand() * 1.1 + .1, 0);
	c.vertices.emplace_back(drand() * 1.1 + .1, drand() * 1.1 + .1);
	c.vertices.emplace_back(0, drand() * 1.1 + .1);
	c.vertices.emplace_back(-drand() * 1.1 - .1,drand() * 1.1 + .1);
	c.vertices.emplace_back(-drand() * 1.1 - .1, 0);
	c.vertices.emplace_back(-drand() * 1.1 - .1, -drand() * 1.1 - .1);
	c.vertices.emplace_back(0, -drand() * 1.1 - .1);
	c.vertices.emplace_back(drand() * 1.1 + .1, -drand() * 1.1 - .1);
	c.wv1 = rand()%8;
	c.wv2 = (c.wv1 + 1 + rand()%7)%8;
	c.distance = 0;
	c.health = 500;
	c.alive = true;
	cars.emplace_back(c);
	return cars.back();
}

void World::checklife()
{
	for (int i = 0; i < cars.size(); i++)
	{
		cars[i].health--;
		if (cars[i].alive and cars[i].b->position.x > cars[i].distance + .1)
		{
			cars[i].distance = cars[i].b->position.x;
			bestDist = std::max(cars[i].distance, bestDist);
			cars[i].health = std::max(cars[i].health, 150);
		}
		else if (cars[i].alive and cars[i].health == 0)
		{
			cars[i].alive = false;
			dead++;
		}
	}
	if (dead == 20)
		newGen();
}

void World::newGen()
{
	std::vector<Car> parents;
	std::sort(cars.begin(), cars.end());
	for (int i = 0; i < 4; i++)
		parents.emplace_back(cars[i]);
	if (parents[0].distance > Champion().distance)
		saveChamp(parents[0]);
	cars.clear();
	solver.Flush();
	pos.Set(0, 0);
	createfloor();
	bestDist = std::max(bestDist, parents[0].distance);
	parents[0].alive = true;
	parents[0].health = 500;
	parents[0].distance = 0;
	cars.emplace_back(parents[0]);
	addCar(cars.back());
	addCar(makebaby(parents[0], parents[1]));
	addCar(makebaby(parents[0], parents[2]));
	addCar(makebaby(parents[0], parents[3]));
	addCar(makebaby(parents[1], parents[2]));
	addCar(makebaby(parents[1], parents[3]));
	addCar(makebaby(parents[2], parents[3]));
	addCar(mutate(cars[1]));
	addCar(mutate(cars[2]));
	addCar(mutate(cars[3]));
	addCar(mutate(cars[4]));
	addCar(mutate(cars[5]));
	addCar(mutate(cars[6]));
	addCar(mutate(parents[0]));
	addCar(mutate(parents[1]));
	addCar(mutate(parents[2]));
	addCar(mutate(parents[3]));
	addCar(randomCar());
	addCar(randomCar());
	addCar(randomCar());
	// for (int i = 0; i < 20; i++)
		// addCar(randomCar());
	dead = 0;
}

Car& World::makebaby(Car a, Car b)
{
	Car ans;
	for (int i = 0; i < 8; i++)
		ans.vertices.emplace_back((a.vertices[i] + b.vertices[i])/2);
	ans.ws1 = (a.ws1 + b.ws1) / 2;
	ans.ws2 = (a.ws2 + b.ws2) / 2;
	ans.wd1 = (a.wd1 + b.wd1) / 2;
	ans.wd2 = (a.wd2 + b.wd2) / 2;
	ans.wv1 = (a.wv1 + b.wv1) / 2;
	ans.wv2 = (a.wv2 + b.wv2) / 2;
	ans.cd = (a.cd + b.cd) / 2;
	ans.distance = 0;
	ans.health = 500;
	ans.alive = true;
	cars.emplace_back(ans);
	return cars.back();
}

Car& World::mutate(Car a)
{
	Car ans = a;
	if (rand() % 10 == 0)
		ans.ws1 = drand() * .5 + .2;
	if (rand() % 10 == 0)
		ans.ws2 = drand() * .5 + .2;
	if (rand() % 10 == 0)
		ans.wd1 = drand() * 1 + .4;
	if (rand() % 10 == 0)
		ans.wd2 = drand() * 1 + .4;
	if (rand() % 10 == 0)
		ans.cd = drand() * 3 + .3;
	if (rand() % 10 == 0)
		ans.vertices[0].Set(drand() * 1.1 + .1, 0);
	if (rand() % 10 == 0)
		ans.vertices[1].Set(drand() * 1.1 + .1, drand() * 1.1 + .1);
	if (rand() % 10 == 0)
		ans.vertices[2].Set(0, drand() * 1.1 + .1);
	if (rand() % 10 == 0)
		ans.vertices[3].Set(-drand() * 1.1 - .1,drand() * 1.1 + .1);
	if (rand() % 10 == 0)
		ans.vertices[4].Set(-drand() * 1.1 - .1, 0);
	if (rand() % 10 == 0)
		ans.vertices[5].Set(-drand() * 1.1 - .1, -drand() * 1.1 - .1);
	if (rand() % 10 == 0)
		ans.vertices[6].Set(0, -drand() * 1.1 - .1);
	if (rand() % 10 == 0)
		ans.vertices[7].Set(drand() * 1.1 + .1, -drand() * 1.1 - .1);
	ans.distance = 0;
	ans.health = 500;
	ans.alive = true;
	cars.emplace_back(ans);
	return cars.back();
}

void World::drawBodies()
{
	solver.drawBodies(false);
}

void World::Solve(int time)
{
	solver.Solve(time);
	checklife();
}

Vector2D World::getPos()
{
	Vector2D ans(0, 0);
	for (int i = 0; i < cars.size(); i++)
	{
		if (cars[i].alive and cars[i].b and ans.x < cars[i].b->position.x)
			ans = cars[i].b->position;
	}
	Vector2D diff = pos - ans;
	pos.x -= diff.x * .05;
	pos.y -= diff.y * .05;
	return pos;
}

void World::saveChamp(Car& c)
{
	std::ofstream output("geneticcars/champ.txt");
	for (int i = 0; i < 8; i++)
		output << c.vertices[i].x << '\n' << c.vertices[i].y << '\n';
	output << c.ws1 << '\n';
	output << c.ws2 << '\n';
	output << c.wd1 << '\n';
	output << c.wd2 << '\n';
	output << c.cd << '\n';
	output << c.wv1 << '\n';
	output << c.wv2 << '\n';
	output << c.distance << '\n';
	output.close();
}

Car World::Champion()
{
	Car champ;
	champ.distance = 0;
	std::ifstream input("geneticcars/champ.txt");
	if (!input.good())
		return champ;
	for (int i = 0; i < 8; i++)
	{
		double a, b;
		input >> a >> b;
		champ.vertices.emplace_back(a, b);
	}
	input >> champ.ws1;
	input >> champ.ws2;
	input >> champ.wd1;
	input >> champ.wd2;
	input >> champ.cd;
	input >> champ.wv1;
	input >> champ.wv2;
	input >> champ.distance;
	input.close();
	champ.health = 500;
	champ.alive = true;
	return champ;
}

