#ifndef MODEL_H_INCLUDED
#define MODEL_H_INCLUDED

/* Define the mathematical model of the 2D cart-pendulum model which is used as a model for the ball-in-a-bowl model */
/* The cart-pendulum model is an approximation of the ball in a bowl model, which is equivalent to considering that
	the ball only slide and does not roll in the bowl, and that there is no friction between the bowl and the ball
*/
/*
	The equations that govern the motion of the damped (in pendulum) cart-pendulum model are:
		ddx ( m + M ) = - m l ddtheta cos(theta) + m l (dtheta)^2 * sin(theta) + Fa
		ddtheta = - ddx / l * cos(theta) - g / l * sin(theta) - b / (m * l * l) * dtheta

	where 
		m: mass of the pendulum
		M: mass of the cart
		l: length of the pendulum
		b: damping coefficient in the pendulum
		theta: angle of the pendulum (dtheta and ddtheta are the 1st and 2nd derivatives)
		x: linear position of the cart (dx and ddx are the 1st and 2nd derivatives)
		Fa: force applied by the user on the cart

	The force applied by the pendulum on the cart is 
		Fb = - m l ddtheta cos(theta) + m l (dtheta)^2 * sin(theta)
*/
#include "math.h"

class Model
{
	public:
		Model(double massOfPendulum, double lengthOfPendulum, double dampingInpendulum, double pendulumInitialAngle, double pendulumInitialVelocity, double gravityMagnitude = 9.81);

		~Model();

		double ComputePendulumForceOnCart(double cartAcceleration); // this is only the force along the direction of motion (1 component)
		double GetPendulumAngle();
		double GetPendulumAngularVelocity();
		double GetPendulumAngularAcceleration();
		void InitializeState(double pendulumInitialAngle, double pendulumInitialVelocity); // reset the angle/velocity/acceleration and Cartesian pos/vel for the next trial
		// Compute acceleration and integrate (RK4) to get new pendulum state
		void UpdatePendulumState(double cartAcceleration, double integrationTimeStep);

	private:
		// This is written for the HM axis, assuming X is the depth axis, Y is the horizontal axis and Z is the vertical axis
		// This is also written assuming that the zero height reference is at the bottom of the pendulum (and not at the cart height)
		void ComputePendulumCartesianPositionInCartFrame();
		void ComputePendulumCartesianVelocityInCartFrame();
		// Compute current pendulum acceleration from the cart motion
		double ComputePendulumAcceleration(double cartAcceleration, double pendAngle, double pendVel);

		double gravity;
		double pendulumLength;
		double pendulumMass;
		double pendulumDamping; 
		double pendulumAngle;
		double pendulumVelocity;
		double pendulumAcceleration;
		
};

#endif // MODEL_H_INCLUDED
