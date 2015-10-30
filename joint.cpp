#include "joint.h"

void RopeJoint::Initialize(int time)
{
	Vector2D cA = A->position;
	Vector2D cB = B->position;
	double aA = A->orient;
	double aB = B->orient;
	Vector2D rA = AnchorA;
	rotateV(aA, rA);
	Vector2D rB = AnchorB;
	rotateV(aB, rB);
	double mA = A->planet ? 0 : A->mass.iM;
	double mB = B->planet ? 0 : B->mass.iM;
	double iA = A->planet ? 0 : A->mass.iI;
	double iB = B->planet ? 0 : B->mass.iI;
	u = cB + rB - cA - rA;
	dist = mag(u);
	if (dist > .005)
		u *= 1 / dist;
	else
	{
		u.Set(0,0);
		mass = 0;
		impulse = 0;
		return;
	}
	double crA = cross(rA, u);
	double crB = cross(rB, u);
	double invMass = mA + iA * crA * crA + mB + iB * crB * crB;
	mass = invMass != 0 ? 1 / invMass : 0;
	if (true)
	{
		Vector2D P = u * impulse;
		A->velocity -= P * mA;
		A->angVel -= iA * cross(rA, P);
		B->velocity += P * mB;
		B->angVel += iB * cross(rB, P);
	}
	else impulse = 0;
}
void RopeJoint::ApplyImpulse(int time)
{
	Vector2D rA = AnchorA;
	rotateV(A->orient, rA);
	Vector2D rB = AnchorB;
	rotateV(B->orient, rB);
	Vector2D vA = A->velocity;
	Vector2D vB = B->velocity;
	double wA = A->angVel;
	double wB = B->angVel;
	double mA = A->planet ? 0 : A->mass.iM;
	double mB = B->planet ? 0 : B->mass.iM;
	double iA = A->planet ? 0 : A->mass.iI;
	double iB = B->planet ? 0 : B->mass.iI;
	Vector2D vpA = vA + cross(wA, rA);
	Vector2D vpB = vB + cross(wB, rB);
	double C = dist - maxDist;
	double Cdot = dot(u, vpB - vpA);
	if (C < 0)
		Cdot += 1/(time/1000.0) * C;
	double cimpulse = -mass * Cdot;
	double oldImpulse = impulse;
	impulse = std::min(0.0, impulse + cimpulse);
	cimpulse = impulse - oldImpulse;
	Vector2D P = u * cimpulse;
	vA -= P * mA;
	wA -= iA * cross(rA, P);
	vB += P * mB;
	wB += iB * cross(rB, P);
	A->velocity = vA;
	A->angVel = wA;
	B->velocity = vB;
	B->angVel = wB;
}
void RopeJoint::PositionalCorrection()
{
	Vector2D cA = A->position;
	Vector2D cB = B->position;
	A->Orient(A->orient);
	B->Orient(B->orient);
	double aA = A->orient;
	double aB = B->orient;
	Vector2D rA = A->transform * AnchorA;
	Vector2D rB = B->transform * AnchorB;
	double mA = A->planet ? 0 : A->mass.iM;
	double mB = B->planet ? 0 : B->mass.iM;
	double iA = A->planet ? 0 : A->mass.iI;
	double iB = B->planet ? 0 : B->mass.iI;
	double length = mag(u);
	u /= length;
	double C = length - maxDist;
	C = Clamp(C, 0, .2);
	double impulse = -mass * C;
	Vector2D P = u * impulse;
	A->position -= P * mA;
	A->orient -= iA * cross(rA, P);
	B->position += P * mB;
	B->orient += iB * cross(rB, P);
}
void RevJoint::Initialize(int time)
{
	Vector2D rA = A->transform * AnchorA;
	Vector2D rB = B->transform * AnchorB;
	double mA = A->planet ? 0 : A->mass.iM;
	double mB = B->planet ? 0 : B->mass.iM;
	double iA = A->planet ? 0 : A->mass.iI;
	double iB = B->planet ? 0 : B->mass.iI;
	mass.m[0][0] = mA + mB + rA.y * rA.y * iA + rB.y * rB.y * iB;
	mass.m[0][1] = -rA.y * rA.x * iA - rB.y * rB.x * iB;
	mass.m[0][2] = -rA.y * iA - rB.y * iB;
	mass.m[1][0] = mass.m[0][1];
	mass.m[1][1] = mA + mB + rA.x * rA.x * iA + rB.x * rB.x * iB;
	mass.m[1][2] = rA.x * iA + rB.x * iB;
	mass.m[2][0] = mass.m[0][2];
	mass.m[2][1] = mass.m[1][2];
	mass.m[2][2] = iA + iB;
	if (!motorEnabled)
		motorImpulse = 0;
	if (limitEnabled)
	{
		double curAngle = B->orient - A->orient - refAngle;
		if (curAngle <= lowerLimit)
		{
			if (state != _atLower)
				cumImpulse.z = 0;
			state = _atLower;
		}
		else if (curAngle >= upperLimit)
		{
			if (state != _atUpper)
				cumImpulse.z = 0;
			state = _atUpper;
		}
		else
		{
			state = _between;
			cumImpulse.z = 0;
		}
	}
	if (true)
	{
		Vector2D P(cumImpulse.x, cumImpulse.y);
		A->velocity -= P * mA;
		A->angVel -= iA * (cross(rA, P) + motorImpulse + cumImpulse.z);
		B->velocity += P * mB;
		B->angVel += iB * (cross(rB, P) + motorImpulse + cumImpulse.z);
	}
	else
	{
		cumImpulse.Set(0,0,0);
		motorImpulse = 0;
	}
}
void RevJoint::ApplyImpulse(int time)
{
	Vector2D rA = A->transform * AnchorA;
	Vector2D rB = B->transform * AnchorB;
	Vector2D vA = A->velocity;
	Vector2D vB = B->velocity;
	double wA = A->angVel;
	double wB = B->angVel;
	double mA = A->planet ? 0 : A->mass.iM;
	double mB = B->planet ? 0 : B->mass.iM;
	double iA = A->planet ? 0 : A->mass.iI;
	double iB = B->planet ? 0 : B->mass.iI;
	if (motorEnabled)
	{
		double Cdot = wB - wA - motorSpeed;
		double impulse = -1/(iA + iB) * Cdot;
		double oldImpulse = motorImpulse;
		double maxImpulse = time/1000.0 * motorMaxTorque;
		motorImpulse = Clamp(motorImpulse + impulse, -maxImpulse, maxImpulse);
		impulse = motorImpulse - oldImpulse;
		wA -= iA * impulse;
		wB += iB * impulse;
	}
	if (limitEnabled and state != _between)
	{
		Vector2D Cdot1 = vB + cross(wB, rB) - vA - cross(wA, rA);
		double Cdot2 = wB - wA;
		Vector3D Cdot(Cdot1.x, Cdot1.y, Cdot2);
		Vector3D impulse = -mass.solve3(Cdot);
		if (state == _atLower)
		{
			double newImpulse = cumImpulse.z + impulse.z;
			if (newImpulse < 0)
			{
				Vector2D rhs = -Cdot1 + Vector2D(mass.m[0][2], mass.m[1][2]) * cumImpulse.z;
				Vector2D reduced = mass.solve2(rhs);
				impulse.x = reduced.x;
				impulse.y = reduced.y;
				impulse.z = -cumImpulse.z;
				cumImpulse.x += reduced.x;
				cumImpulse.y += reduced.y;
				cumImpulse.z = 0;
			}
			else
				cumImpulse += impulse;
		}
		else
		{
			double newImpulse = cumImpulse.z + impulse.z;
			if (newImpulse > 0)
			{
				Vector2D rhs = -Cdot1 + Vector2D(mass.m[0][2], mass.m[1][2]) * cumImpulse.z;
				Vector2D reduced = mass.solve2(rhs);
				impulse.x = reduced.x;
				impulse.y = reduced.y;
				impulse.z = -cumImpulse.z;
				cumImpulse.x += reduced.x;
				cumImpulse.y += reduced.y;
				cumImpulse.z = 0;
			}
			else
				cumImpulse += impulse;
		}
		Vector2D P(impulse.x, impulse.y);
		vA -= P * mA;
		wA -= iA * (cross(rA, P) + impulse.z);
		vB += P * mB;
		wB += iB * (cross(rB, P) + impulse.z);
	}
	else
	{
		Vector2D Cdot = vB + cross(wB, rB) - vA - cross(wA, rA);
		Vector2D impulse = mass.solve2(-Cdot);
		cumImpulse.x += impulse.x;
		cumImpulse.y += impulse.y;
		vA -= impulse * mA;
		wA -= iA * cross(rA, impulse);
		vB += impulse * mB;
		wB += iB * cross(rB, impulse);
	}
	A->velocity = vA;
	A->angVel = wA;
	B->velocity = vB;
	B->angVel = wB;
}
void RevJoint::PositionalCorrection()
{
	A->Orient(A->orient);
	B->Orient(B->orient);
	Vector2D rA = A->transform * AnchorA;
	Vector2D rB = B->transform * AnchorB;
	Vector2D cA = A->position;
	Vector2D cB = B->position;
	double aA = A->orient;
	double aB = B->orient;
	double mA = A->planet ? 0 : A->mass.iM;
	double mB = B->planet ? 0 : B->mass.iM;
	double iA = A->planet ? 0 : A->mass.iI;
	double iB = B->planet ? 0 : B->mass.iI;
	if (limitEnabled and state != _between)
	{
		double angle = aB - aA - refAngle;
		double limitImpulse = 0;
		if (state == _atLower)
		{
			double C = angle - lowerLimit;
			C = Clamp(C + pi/90, -pi/22.5, 0);
			limitImpulse = -1/(iA + iB) * C;
		}
		else
		{
			double C = angle - upperLimit;
			C = Clamp(C - pi/90, 0, pi/22.5);
			limitImpulse = -1/(iA + iB) * C;
		}
		aA -= iA * limitImpulse;
		aB += iB * limitImpulse;
	}
	Vector2D C = cB + rB - cA - rA;
	Matrix2D K(mA + mB + iA * rA.y * rA.y + iB * rB.y * rB.y,
		-iA * rA.x * rA.y - iB * rB.x * rB.y,
		-iA * rA.x * rA.y - iB * rB.x * rB.y,
		mA + mB + iA * rA.x * rA.x + iB * rB.x * rB.x);
	Vector2D impulse = -K.solve(C);
	cA -= impulse * mA;
	aA -= iA * cross(rA, impulse);
	cB += impulse* mB;
	aB += iB * cross(rB, impulse);
	A->position = cA;
	A->orient = aA;
	B->position = cB;
	B->orient = aB;
}
void RevJoint::SetMotor(bool on, double speed, double tlimit)
{
	motorEnabled = on;
	motorSpeed = speed;
	motorMaxTorque = tlimit;
}
void RevJoint::SetLimit(bool on, double upper, double lower)
{
	limitEnabled = on;
	upperLimit = upper;
	lowerLimit = lower;
}
double RevJoint::GetAngle()
{
	return B->orient - A->orient - refAngle;
}
Vector2D RevJoint::GetAnchor()
{
	return A->transform * AnchorA + A->position;
}
void WheelJoint::Initialize(int time)
{
	Vector2D cA = A->position;
	Vector2D cB = B->position;
	Vector2D rA = AnchorA;
	rotateV(A->orient, rA);
	Vector2D rB = AnchorB;
	rotateV(B->orient, rB);
	double mA = A->planet ? 0 : A->mass.iM;
	double mB = B->planet ? 0 : B->mass.iM;
	double iA = A->planet ? 0 : A->mass.iI;
	double iB = B->planet ? 0 : B->mass.iI;
	Vector2D d = cB + rB - cA - rA;
	ay = cross(1, AxisFrdm);
	rotateV(A->orient,ay);
	sAy = cross(d + rA, ay);
	sBy = cross(rB, ay);
	mass = mA + mB + iA * sAy * sAy + iB * sBy * sBy;
	if (mass > 0)
		mass = 1 / mass;
	smass = 0;
	bias = 0;
	gamma = 0;
	if (springFreq > 0)
	{
		ax = AxisFrdm;
		rotateV(A->orient,ax);
		sAx = cross(d + rA, ax);
		sBx = cross(rB, ax);
		double invMass = mA + mB + iA * sAx * sAx + iB * sBx * sBx;
		if (invMass > 0)
		{
			smass = 1 / invMass;
			double C = dot(d, ax);
			double omega = 2 * pi * springFreq;
			double dm = 2 * smass * springDamp * omega;
			double k = smass * omega * omega;
			double h = time/1000.0;
			gamma = h * (dm + h * k);
			if (gamma > 0)
				gamma = 1 / gamma;
			bias = C * h * k * gamma;
			smass = invMass + gamma;
			if (smass > 0)
				smass = 1 / smass;
		}
	}
	if (true)
	{
		Vector2D P = ay * cumImpulse + ax * springImpulse;
		double LA = sAy * cumImpulse + sAx * springImpulse + motorImpulse;
		double LB = sBy * cumImpulse + sBx * springImpulse + motorImpulse;
		A->velocity -= P * mA;
		A->angVel -= iA * LA;
		B->velocity += P * mB;
		B->angVel += iB * LB;
	}
	else
	{
		cumImpulse = 0;
		springImpulse = 0;
		motorImpulse = 0;
	}
}
void WheelJoint::ApplyImpulse(int time)
{
	Vector2D rA = AnchorA;
	rotateV(A->orient, rA);
	Vector2D rB = AnchorB;
	rotateV(B->orient, rB);
	Vector2D vA = A->velocity;
	Vector2D vB = B->velocity;
	double wA = A->angVel;
	double wB = B->angVel;
	double mA = A->planet ? 0 : A->mass.iM;
	double mB = B->planet ? 0 : B->mass.iM;
	double iA = A->planet ? 0 : A->mass.iI;
	double iB = B->planet ? 0 : B->mass.iI;
	{//spring constant
		double Cdot = dot(ax, vB - vA) + sBx * wB - sAx * wA;
		double impulse = -smass * (Cdot + bias + gamma * springImpulse);
		springImpulse += impulse;
		Vector2D P = ax * impulse;
		double LA = impulse * sAx;
		double LB = impulse * sBx;
		vA -= P * mA;
		wA -= iA * LA;
		vB += P * mB;
		wB += iB * LB;
	}
	if (motorEnabled)
	{
		double Cdot = wB - wA - motorSpeed;
		double impulse = -1/(iA + iB) * Cdot;
		double oldImpulse = motorImpulse;
		double maxImpulse = time/1000.0 * motorMaxTorque;
		motorImpulse = Clamp(motorImpulse + impulse, -maxImpulse, maxImpulse);
		impulse = motorImpulse - oldImpulse;
		wA -= iA * impulse;
		wB += iB * impulse;
	}
	{//align to axis
		double Cdot = dot(ay, vB - vA) + sBy * wB - sAy * wA;
		double impulse = -mass * Cdot;
		cumImpulse += impulse;
		Vector2D P = ay * impulse;
		double LA = impulse * sAy;
		double LB = impulse * sBy;
		vA -= P * mA;
		wA -= iA * LA;
		vB += P * mB;
		wB += iB * LB;
	}
	A->velocity = vA;
	A->angVel = wA;
	B->velocity = vB;
	B->angVel = wB;
}
void WheelJoint::PositionalCorrection()
{
	A->Orient(A->orient);
	B->Orient(B->orient);
	Vector2D rA = A->transform * AnchorA;
	Vector2D rB = B->transform * AnchorB;
	Vector2D cA = A->position;
	Vector2D cB = B->position;
	double aA = A->orient;
	double aB = B->orient;
	Vector2D d = (cB - cA) + rB - rA;
	ay = cross(1, AxisFrdm);
	rotateV(A->orient,ay);
	sAy = cross(d + rA, ay);
	sBy = cross(rB, ay);
	double C = dot(d, ay);
	double mA = A->planet ? 0 : A->mass.iM;
	double mB = B->planet ? 0 : B->mass.iM;
	double iA = A->planet ? 0 : A->mass.iI;
	double iB = B->planet ? 0 : B->mass.iI;
	double k = mA + mB + iA * sAy * sAy + iB * sBy * sBy;
	double impulse;
	if (k != 0)
		impulse = - C / k;
	else
		impulse = 0;
	Vector2D P = ay * impulse;
	double LA = impulse * sAy;
	double LB = impulse * sBy;
	A->position -= P * mA;
	A->orient -= iA * LA;
	B->position += P * mB;
	B->orient += iB * LB;
}
void WheelJoint::SetMotor(bool on, double speed, double tlimit)
{
	motorEnabled = on;
	motorSpeed = speed;
	motorMaxTorque = tlimit;
}
IKJoint::IKJoint(RevJoint* j, Vector2D endPoint)
{
	joints.emplace_back(j);
	effRel = endPoint - j->B->mass.CoM;;
}
void IKJoint::AddJoint(RevJoint* j)
{
	joints.emplace_back(j);
}
void IKJoint::SetTarget(Vector2D t)
{
	targeted = true;
	target = t;
}
void IKJoint::SetTarget(bool on)
{
	targeted = on;
}
void IKJoint::Solve()
{
	if (!targeted)
	{
		for (int i = 0; i < joints.size(); i++)
			joints[i]->SetMotor(false, 0, 0);
		return;
	}
	effector = joints[0]->B->position + joints[0]->B->transform * effRel;
	for (int i = 0; i < joints.size(); i++)
		solve(i);
}
void IKJoint::solve(int i)
{
	Vector2D tangent = effector - joints[i]->GetAnchor();
	Vector2D normal = cross(1, tangent);
	Vector2D error = target - effector;
	normalize(normal);
	double m = mag(error);
	if (m > 10)
		error *= (10/m);
	double correction = dot(normal, error);
	correction = Clamp(correction, -5, 5);
	joints[i]->SetMotor(true, correction, 2000000);
}