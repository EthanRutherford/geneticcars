#include "solver.h"

Solver::~Solver()
{
	Flush();
}

void Solver::Solve(int time)
{
	if (applyg)
		applyg(body);
	else
		applyG();
	BPT.Start();
	bp.sort();
	BPT.End();
	NPT.Start();
	checkCol();
	NPT.End();
	applyForces(time);
	SLT.Start();
	solveVelocities(time);
	SLT.End();
	MVT.Start();
	solvePositions(time);
	MVT.End();
	clearForces();
}

void Solver::addBody(Body* b)
{
	body.push_back(b);
	for (int i = 0; i < b->shape.size(); i++)
		bp.addAABB(&b->shape[i]->aabb);
}

void Solver::addJoint(Joint* j)
{
	joint.emplace_back(j);
}

void Solver::drawBodies(bool debug)
{
	for (int i = 0; i < body.size(); i++)
	{
		if (i >= 200 and i % 3 == 1)
			glColor3f(0.1, 0.3, 0.7);
		else if (i >= 200)
			glColor3f(0.2, 0.2, 0.2);
		else
			glColor3f(0.6, 0.6, 0.6);
		if (!body[i]->nodraw)
		for (int j = 0; j < body[i]->shape.size(); j++)
		{
			glBegin(GL_POLYGON);
			body[i]->shape[j]->Draw();
			glEnd();
		}
		glColor3f(0.9, 0.9, 0.9);
		for (int j = 0; j < body[i]->shape.size(); j++)
		{
			glBegin(GL_LINE_STRIP);
			body[i]->shape[j]->Draw();
			glEnd();
		}
		if (debug)
		{
			glColor3f(0.9f, 0.2f, 0.2f);
			for (int j = 0; j < body[i]->shape.size(); j++)
			{
				glBegin(GL_LINE_STRIP);
				body[i]->shape[j]->DrawAABB();
				glEnd();
			}
			glColor3f(0.5f, 0.5f, 0.9f);
			glBegin(GL_POINTS);
			glVertex2d(body[i]->position.x, body[i]->position.y);
			glEnd();
		}
	}
	if (debug)
	{
		glColor3f(0, 1, 1);
		glBegin(GL_POINTS);
		glColor3f(0.2f, 0.9f, 0.2f);
		for (auto it = contacts.begin(); it != contacts.end(); it++)
			for (int j = 0; j < it->contact_count; j++)
				glVertex2d(it->contacts[j].x, it->contacts[j].y);
		glEnd();
	}
}

double Solver::GetProfile(std::string s)
{
	if (s == "bp")
		return BPT.Get();
	if (s == "np")
		return NPT.Get();
	if (s == "slv")
		return SLT.Get();
	if (s == "mv")
		return MVT.Get();
	return -1;
}

void Solver::Flush()
{
	for (int i = 0; i < body.size(); i++)
		delete body[i];
	body.clear();
	for (int i = 0; i < joint.size(); i++)
		delete joint[i];
	joint.clear();
	contacts.clear();
	bp.Flush();
}

void Solver::applyG()
{
	for (int i = 0; i < body.size(); i++)
		if (!body[i]->planet)
			body[i]->ApplyForce(Vector2D(0,-10) * body[i]->mass.m);
}

void Solver::applyForces(int time)
{
	for (int i = 0; i < body.size(); i++)
	{
		Body* b = body[i];
		b->velocity += (b->force * b->mass.iM) * (time / 1000.0);
		b->angVel += b->torque * b->mass.iI * (time / 1000.0);
	}
}

void Solver::clearForces()
{
	for (int i = 0; i < body.size(); i++)
	{
		Body* b = body[i];
		b->force.Set(0, 0);
		b->torque = 0;
	}
}

void Solver::solvePositions(int time)
{
	for (int i = 0; i < body.size(); i++)
	{
		Body* b = body[i];
		b->position += b->velocity * (time/1000.0);
		b->orient += b->angVel * (time/1000.0);
	}
	for (int j = 0; j < 3; j++)
	{
		for (auto it = contacts.begin(); it != contacts.end(); it++)
			it->PositionalCorrection();
		for (int i = 0; i < joint.size(); i++)
			joint[i]->PositionalCorrection();
	}
	for (int i = 0; i < body.size(); i++)
	{
		body[i]->Orient(body[i]->orient);
		for (int j = 0; j < body[i]->shape.size(); j++)
			body[i]->shape[j]->setAABB();
	}
}

void Solver::checkCol()
{
	std::unordered_set<BPPair> pair = bp.pair;
	for (auto it = contacts.begin(); it != contacts.end(); it++)
	{
		it->Solve();
		pair.erase(BPPair(it->A, it->B));
		if(!it->contact_count)
		{
			it->A->body->contact = false;
			it->B->body->contact = false;
			contacts.erase(it--);
		}
	}
	for (auto it = pair.begin(); it != pair.end(); it++)
	{
		Body* A = it->A->body;
		Body* B = it->B->body;
		if (A->mass.iM == 0 && B->mass.iM == 0)
			continue;
		if (A->filtergroup == B->filtergroup and A->filtergroup != 0)
			continue;
		if (A == B)
			continue;
		Manifold m(it->A, it->B);
		m.Solve();
		if (m.contact_count)
		{
			A->contact = true;
			B->contact = true;
			contacts.emplace_back(m);
		}
	}
}

void Solver::solveVelocities(int time)
{
	int size = joint.size();
	for (auto it = contacts.begin(); it != contacts.end(); it++)
		it->Initialize();
	for (auto it = contacts.begin(); it != contacts.end(); it++)
		it->WarmStart();
	for (int i = 0; i < size; i++)
		joint[i]->Initialize(time);
	for (int j = 0; j < 8; j++)
	{
		for (int i = 0; i < size; i++)
			joint[i]->ApplyImpulse(time);
		for (auto it = contacts.begin(); it != contacts.end(); it++)
			it->ApplyImpulse();
	}
}

