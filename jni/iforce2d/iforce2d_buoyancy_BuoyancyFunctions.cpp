#include "iforce2d/iforce2d_buoyancy_BuoyancyFunctions.h"
#include "iforce2d/iforce2d_Buoyancy_functions.h"

/*
 * Class:     iforce2d_buoyancy_BuoyancyFunctions
 * Method:    doWork
 * Signature: (JJ)V
 */
JNIEXPORT void JNICALL Java_iforce2d_buoyancy_BuoyancyFunctions_doWork
(JNIEnv *env, jclass clazz, jlong fixA, jlong fixB) {

	b2Fixture* fixtureA = (b2Fixture*) fixA;
	b2Fixture* fixtureB = (b2Fixture*) fixB;

	vector<b2Vec2> intersectionPoints;
	if (findIntersectionOfFixtures(fixtureA, fixtureB, intersectionPoints)) {
		//find centroid
		float area = 0;
		b2Vec2 centroid = ComputeCentroid(intersectionPoints, area);

		//apply buoyancy stuff here...
		b2Vec2 gravity(0, -10);
		//apply buoyancy force (fixtureA is the fluid)
		float displacedMass = fixtureA->GetDensity() * area;
		b2Vec2 buoyancyForce = displacedMass * -gravity;
		fixtureB->GetBody()->ApplyForce(buoyancyForce, centroid);

		// Apply better drag
		//apply drag separately for each polygon edge
		for (int i = 0; i < intersectionPoints.size(); i++) {
			//the end points and mid-point of this edge
			b2Vec2 v0 = intersectionPoints[i];
			b2Vec2 v1 = intersectionPoints[(i+1)%intersectionPoints.size()];
			b2Vec2 midPoint = 0.5f * (v0+v1);

			//find relative velocity between object and fluid at edge midpoint
			b2Vec2 velDir = fixtureB->GetBody()->GetLinearVelocityFromWorldPoint( midPoint ) -
			fixtureA->GetBody()->GetLinearVelocityFromWorldPoint( midPoint );
			float vel = velDir.Normalize();

			b2Vec2 edge = v1 - v0;
			float edgeLength = edge.Normalize();
			b2Vec2 normal = b2Cross(-1,edge);//gets perpendicular vector

			float dragDot = b2Dot(normal, velDir);
			if ( dragDot < 0 )
			continue;//normal points backwards - this is not a leading edge

			float dragMag = dragDot * edgeLength * fixtureA->GetDensity() * vel * vel;
			b2Vec2 dragForce = dragMag * -velDir;
			fixtureB->GetBody()->ApplyForce( dragForce, midPoint);

			//apply simple angular drag
			float angularDrag = area * -fixtureB->GetBody()->GetAngularVelocity()*fixtureA->GetDensity();// *density is added by me
			fixtureB->GetBody()->ApplyTorque(angularDrag);

			//apply lift
			float liftDot = b2Dot(edge, velDir);
			float liftMag = (dragDot * liftDot) * edgeLength * fixtureA->GetDensity() * vel * vel;
			b2Vec2 liftDir = b2Cross(1,velDir);//gets perpendicular vector
			b2Vec2 liftForce = liftMag * liftDir;
			fixtureB->GetBody()->ApplyForce( liftForce, midPoint);
		}
	}
}
