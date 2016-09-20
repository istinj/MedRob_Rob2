#include <HL\hl.h>
#include <vector>
#include <HDU\hduMatrix.h>
#include <HDU\hduVector.h>
#include <HDU\hduQuaternion.h>

class RigidBody
{
public:
	RigidBody();
	RigidBody(hduVector3Dd in_x, hduQuaternion in_q, hduVector3Dd in_body_scale);
	~RigidBody();

	// Constant quantities
	int id;
	double mass;
	hduMatrix Ibody;
	hduMatrix Ibodyinv;

	// State variables
	hduVector3Dd x;
	hduQuaternion q;
	hduVector3Dd P;			//Linear Momentum
	hduVector3Dd L;			//Angular Momentum

	// Derived quantities
	hduMatrix Iinv;
	hduMatrix R;			//Rotation Matrix
	hduVector3Dd v;			//linera velocity
	hduVector3Dd omega;		//angular velocity

	// Computed quantites
	hduVector3Dd force;
	hduVector3Dd torque;



private:
	hduVector3Dd body_scale;
	std::vector<hduVector3Dd> vertexList;
	std::vector<hduVector3Dd> faceList;

	void addVertex(double x, double y, double z);
	// void addFace();
	void initialize();

	void createCube();
};

