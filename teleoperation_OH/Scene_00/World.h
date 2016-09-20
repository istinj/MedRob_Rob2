#include <HDU\hduMatrix.h>
#include <HDU\hduVector.h>

#include "RigidBody.h"


class World
{

public:
	World();
	~World();

	void init(void);
	void initSimulation();

	// For simulatoion
	double time;
	std::vector<RigidBody*> body_list;
private:
	double gravity;
	void deleteAllBodies();
};