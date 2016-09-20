#include "World.h"


World::World()
{
	gravity = -9.81;
}

World::~World()
{
	deleteAllBodies(); //placeholder
}

//! Carichiamo tutti i nostri rigid bodies
void World::init(void)
{
	hduVector3Dd cube_pos(0,1,0);

	hduMatrix rot;
	rot.makeIdentity();
	hduQuaternion cube_q; // identity
	cube_q.fromRotationMatrix(rot);

	hduVector3Dd cube_scale(1,1,1);

	RigidBody* cube = new RigidBody(cube_pos,cube_q,cube_scale);
	body_list.push_back(cube);

}