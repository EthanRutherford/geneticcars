#ifndef BODY_H
#define BODY_H

#include "base.h"
#include <gl/gl.h>
#include <vector>
#include <iostream>
#define MaxPolyVertexCount 64

struct mShape;
struct massData{
	double I;
	double iI;
	double m;
	double iM;
	Vector2D CoM;
	void Zero(){
		I = 0; iI = 0; m = 0; iM = 0; CoM.Set(0,0);
	}
};
struct Body{
	Body(mShape** s, int num, int x, int y, double density, bool p);
	~Body();
	void SetMass(double density);
	void applyG(double m, Vector2D dir, double dist);
	void ApplyForce(const Vector2D f) {force += f;}
	void ApplyTorque(double t) {torque += t;}
	void ApplyImpulse(Vector2D impulse, Vector2D contactVector);
	void SetStatic();
	void Orient(double radians);
	Vector2D position;
	Vector2D velocity;
	double angVel;
	double torque;
	double orient;
	Matrix2D transform;
	Vector2D force;
	massData mass;
	double Friction;
	double restitution;
	int filtergroup;
	bool planet;
	bool contact;				//is body collided?
	bool nodraw;
	std::vector<mShape*> shape;
};
struct AABB{
	Vector2D max, min;
	mShape* shape;
};
struct mShape{
	enum Type{
		_circle,
		_poly,
		_count
	};
	mShape() {}
	virtual mShape* Clone() const = 0;
	virtual massData ComputeMass(double density) = 0;
	virtual void Draw() = 0;
	void DrawAABB();
	virtual Type GetType() const = 0;
	virtual void setAABB() = 0;
	AABB* getAABB(){return &aabb;}
	
	Body* body;
	double radius;
	AABB aabb;
	int id;
};
struct mCircle : public mShape{
	mCircle(double r) {radius = r;}
	mShape* Clone() const {return new mCircle(radius);}
	massData ComputeMass(double density);
	void Draw();
	Type GetType() const {return _circle;}
	void setAABB();
};
struct mPolygon : public mShape{
	mShape* Clone() const;
	massData ComputeMass(double density);
	void Draw();
	Type GetType() const {return _poly;}
	void setAABB();
	void SetBox(double hw, double hh);
	void Set(Vector2D* vertices, int count);
	Vector2D GetSupport(const Vector2D& dir);
	int vertexCount;
	Vector2D pt[MaxPolyVertexCount];
	Vector2D norm[MaxPolyVertexCount];
};
bool AABBtest(AABB* a, AABB* b);

#endif