#include "RigidBody.h"


RigidBody::RigidBody():
	body_scale(1,1,1),
	x(0,0,0),
	q(),
	P(0,0,0),
	L(0,0,0),
	v(0,0,0),
	omega(0,0,0),
	force(0,0,0),
	torque(0,0,0)
{
	initialize();
}

RigidBody::RigidBody(hduVector3Dd in_x, hduQuaternion in_q, hduVector3Dd in_body_scale)
{
	x = in_x;
	q = in_q;
	body_scale = in_body_scale;
	initialize();
}

void RigidBody::initialize()
{
	mass = 1;
	Ibody.makeIdentity();
	Ibodyinv = Ibody;
	Ibodyinv.invert();
	R.getRotation(q);
}

void RigidBody::addVertex(double x, double y, double z)
{
	hduVector3Dd v;
	v[0] = body_scale[0] / 2 * x;
    v[1] = body_scale[1] / 2 * y;
    v[2] = body_scale[2] / 2 * z;

	vertexList.push_back(v);
}


void RigidBody::createCube()
{
	addVertex(+1, +1, +1); // 0
    addVertex(-1, +1, +1); // 1
    addVertex(+1, -1, +1); // 2
    addVertex(-1, -1, +1); // 3
    addVertex(+1, +1, -1); // 4
    addVertex(-1, +1, -1); // 5
    addVertex(+1, -1, -1); // 6
    addVertex(-1, -1, -1); // 7

}