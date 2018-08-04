package PhysicsBodies;

import ConvexPolygon.ContactManifold;
import ConvexPolygon.Shape;
import PhysicsEngine.PhysicsEngine;
import Vectors.Vector2D;

import java.util.HashMap;
import java.util.Iterator;
import java.util.Map;

abstract public class RigidBody extends KinematicBody
{
  private static final double APPROX_ZERO = 1.0E-14d;

  //TODO set up adjusting sizes
  private Map<PhysicsBody, ContactManifold> mcCollisionBodyManifoldMap;

  Vector2D mcTranslationalForce,
           mcTranslationalForceConst;
  double mTorque,
         mTorqueConst;

  private double  mTranslationalDampening,
                  mRotationalDampening;

  boolean mbSleep,
          mbCanRotate,
          mbApplyGravity;
  //TODO add mbJustSeperate, to just seperate rigidbody, not apply physics

  public RigidBody (Shape cShape)
  {
    super (cShape);

    mcCollisionBodyManifoldMap = new HashMap<> (PhysicsBody.DEFAULT_SIZES);
    mcTranslationalForce = new Vector2D ();
    //mcTranslationalForceConst = PhysicsEngine.getInstance ().getGravity ();
    mcTranslationalForceConst = new Vector2D ();
    mTorque = mTorqueConst = 0;

    mTranslationalDampening = mRotationalDampening = 0.0d;

    mbSleep = false;
    mbCanRotate = true;
    mbApplyGravity = true;
  }

  abstract public void collide (PhysicsBody cBody, ContactManifold cManifold);

  public void setRotationalDampening (double dampening)
  {
    if (0 > dampening)
    {
      mRotationalDampening = 0;
    }
    else
    {
      mRotationalDampening = dampening;
    }
  }

  public void setTranslationalDampening (double dampening)
  {
    if (0 > dampening)
    {
      mRotationalDampening = 0;
    }
    else
    {
      mRotationalDampening = dampening;
    }
  }

  public void applyForce (Vector2D cForce)
  {
    mcTranslationalForce.add (cForce);
  }

  public void applyImpulse (Vector2D cImpulse, Vector2D cRelativePosition)
  {
    mcVelocity.subtract (cImpulse.getScaled (this.getInvMass ()));
    mAngularVelocity -= (cImpulse.mX * cRelativePosition.mY
                        - cImpulse.mY * cRelativePosition.mX)
                        * this.getInvInertia ();
  }

  //pos torque causes ccw rotation
  public void applyTorque (double torque)
  {
    mTorque += torque;
  }

  public void updatePhysics (double delta)
  {
    //add constant forces
    mcTranslationalForce.add (mcTranslationalForceConst);
    mTorque += mTorqueConst;

    //modify translational force && torque based on friction
    //this.calcFriction (delta);

    //apply translational force
    mcVelocity.add (mcTranslationalForce.scale (delta * this.getInvMass ()));

    //apply translational dampening
    Vector2D cVelocityMirror = mcVelocity.getMirror ();
    cVelocityMirror.scale (mTranslationalDampening);
    mcVelocity.add (cVelocityMirror);

    //apply torque
    mAngularVelocity += mTorque * delta * this.getInvInertia ();

    //apply rotational dampening
    mAngularVelocity -= mRotationalDampening * mAngularVelocity;

    //rotate and move based on velocities
    this.offSet (mcVelocity.getScaled (delta), mAngularVelocity * delta);

    this.clearForces ();
  }

  //TODO maybe add bCheckFriction, so as to maybe save some processing if disable friction
  private void calcFriction (double delta)
  {
    //total translational force on this body
    Vector2D cForceTotal = this.mcTranslationalForce;
    //total torque on this body
    double torqueTotal = this.mTorque;
    //velocities of this body
    Vector2D cVelocityA = this.mcVelocity;
    double angularVelocityA = this.mAngularVelocity;
    //position of this body
    Vector2D cPositionA = this.getPosition ();

    //TODO IMPORTANT: ensure contact manifold doesn't lose/gain energy (vs single contact point)
    for (Iterator<PhysicsBody> cBodies
         = mcCollisionBodyManifoldMap.keySet ().iterator ();
         cBodies.hasNext ();)
    {
      PhysicsBody cBody = cBodies.next ();
      ContactManifold cManifold = mcCollisionBodyManifoldMap.get (cBody);
      Vector2D caContactPointsA[] = cManifold.mcaContactManifoldA,
               caContactPointsB[] = cManifold.mcaContactManifoldB;

      //normal to collision surface
      Vector2D cNormal = new Vector2D (cManifold.mcNormalA).unit ();

      //calc tangent to normal in direction of F_total
      Vector2D cTangent = cNormal.getTangent (cForceTotal).unit ();

      //calc normal force, translational only here
      double forceNormal = cForceTotal.dot (cNormal);
      //calc tangential force, translational only here
      double forceTangential = cForceTotal.dot (cTangent);

      //velocities of other body
      Vector2D cVelocityB = cBody.mcVelocity;
      double angularVelocityB = cBody.mAngularVelocity;
      //position of other body
      Vector2D cPositionB = cBody.getPosition ();

      for (int i = 0; i < caContactPointsA.length; ++i)
      {
        //radius from body center to contact point
        Vector2D cRA = caContactPointsA[i].getSubtract (cPositionA),
                 cRB = caContactPointsB[i].getSubtract (cPositionB);

        //magnitude of radius from body center to contact point
        double rA = cRA.getMagnitude (),
               rB = cRB.getMagnitude ();

        //total velocity of contact point on bodies, taking angular velocity
        //into account
        Vector2D cTotalVelocityA = new Vector2D (cVelocityA.mX
                                                  - cRA.mY * angularVelocityA,
                                                 cVelocityA.mY
                                                  + cRA.mX * angularVelocityA),
                 cTotalVelocityB = new Vector2D (cVelocityB.mX
                                                   - cRB.mY * angularVelocityB,
                                                 cVelocityB.mY
                                                   + cRB.mX * angularVelocityB);

        //eval both objects current speeds relative to the tangent,
        //and use to solve for the relative speed of objects
        double relSpeed = cTotalVelocityA.dot (cTangent)
                          - cTotalVelocityB.dot (cTangent);

        //create vector of force from torque
        Vector2D cForceFromTorque = cRA.getNormalCW ().unit ().scale (
                                    torqueTotal / rA);

        //add tangential and normal force from torque to the total
        double forceNormalTotal = forceNormal + cForceFromTorque.dot (cNormal),
               forceTangentialTotal = forceTangential
                                    + cForceFromTorque.dot (cTangent);

        //frictional coefficient of objects, for F_fric = mu * F_normal
        double mu,
            muA,
            muB;
        boolean bIsDynamic = false;

        //if objects are not moving relative to each other, then static friction
        //is applied
        if (RigidBody.APPROX_ZERO > relSpeed)
        {
          muA = this.getStaticFrictionCoefficient ();
          muB = cBody.getStaticFrictionCoefficient ();
        }
        //else dynamic friction is applied
        else
        {
          muA = this.getDynamicFrictionCoefficient ();
          muB = cBody.getDynamicFrictionCoefficient ();
          bIsDynamic = true;
        }

        mu = Math.sqrt (muA * muA + muB * muB);

        //TODO COME BACK HERE, THIS IS WHERE YOU LEFT OFF, YOU PLANNED TO EVAL
        // THE RESULTING FRICTIONAL FORCE AND CONVERT THAT TO AN IMPULSE AT THAT
        // POINT, THEN APPLY THAT IMPULSE, BUT YOU GOT ALL SCREWY WITH THERE
        // BEING TWO CONTACT POINTS

        //max friction that can affect total friction in direction of the tangent
        double maxFriction = forceNormal * mu;

        //if tangental force is greater than max friction force,
        //then total force is equal to total force - maxFriction in direction
        //of tangental
        if (forceTangential >= maxFriction)
        {
          mcTranslationalForce.subtract (cTangent.scale (forceTangential
              - maxFriction));
        }
        //else set force in tangental direction to zero
        else
        {
          mcTranslationalForce.subtract (cTangent.scale (forceTangential));
        }
      }





    }

  }

  // this is object A, cBody is object B
  public void calcPhysics (PhysicsBody cBody, ContactManifold cManifold, double delta)
  {
    //TODO may need to add constant amount to impulse to take into account seperation depth
    //TODO make sure that contactpoints for edges, the points at a given index correspond to the closest point on the other contact point array

    //TODO IMPORTANT: ensure contact manifold doesn't gain energy (vs single contact point), may have to look in calcFriction as well

    Vector2D cPositionA = this.getPosition (),
             cPositionB = cBody.getPosition (),
             caContactPointsA[] = cManifold.mcaContactManifoldA,
             cNormalA = cManifold.mcNormalA,
             caContactPointsB[] = cManifold.mcaContactManifoldB,
             cNormalB = cManifold.mcNormalB;

    Vector2D cVelocityA = this.mcVelocity,
             cVelocityB = cBody.mcVelocity;
    double angularVelocityA = this.mAngularVelocity,
           angularVelocityB = cBody.mAngularVelocity;

    double elasticity = Math.min (this.getElasticity (),
                                  cBody.getElasticity ()),
           invMassA = this.getInvMass (),
           invInertiaA = this.getInvInertia (),
           invMassB = cBody.getInvMass (),
           invInertiaB = cBody.getInvInertia ();


    //it should be the case that caContactPointsA.length == caContactPointsB.length
    int numContactPoints = caContactPointsA.length;
    for (int i = 0; i < numContactPoints; ++i)
    {
      //radius from body center to contact point
      Vector2D cRA = caContactPointsA[i].getSubtract (cPositionA),
               cRB = caContactPointsB[i].getSubtract (cPositionB);

      double rAX = cRA.mX,
             rAXSqr = rAX * rAX,
             rAY = cRA.mY,
             rAYSqr = rAY * rAY,
             rBX = cRB.mX,
             rBXSqr = rBX * rBX,
             rBY = cRB.mY,
             rBYSqr = rBY * rBY;

      //calc velocity of contact points
      double vAX = cVelocityA.mX - rAY * angularVelocityA,
             vAY = cVelocityA.mY + rAX * angularVelocityA,
             vBX = cVelocityB.mX - rBY * angularVelocityB,
             vBY = cVelocityB.mY + rBX * angularVelocityB;

      //calc values used to solve for impulse
      double sumInvMass = invMassA + invMassB,
             kFactor = sumInvMass * sumInvMass
                        - invMassA * invInertiaA * (rAXSqr + rAYSqr)
                        - invMassA * invInertiaB * (rBXSqr + rBYSqr)
                        - invMassB * invInertiaA * (rAYSqr + rAXSqr)
                        - invMassB * invInertiaB * (rBXSqr + rBYSqr)
                        + invInertiaA * invInertiaB * (rAYSqr * rBXSqr
                        + rAXSqr * rBYSqr - 2 * rAX * rAY * rBX * rBY),
             elasticityFactor = (elasticity + 1) / kFactor,
             diffVelX = vAX - vBX,
             diffVelY = vAY - vBY,
             sumRadiusInertia = rAX * rAY * invInertiaA + rBX * rBY
                                * invInertiaB;

      //calc impulse
      double impulseX = elasticityFactor * (diffVelX * (sumInvMass - rAXSqr
                        * invInertiaA - rBXSqr * invInertiaB)
                        - diffVelY * sumRadiusInertia),
             impulseY = elasticityFactor * (-1 * diffVelX * sumRadiusInertia
                        + diffVelY * (sumInvMass - rAYSqr * invInertiaA
                        - rBYSqr * invInertiaB));

      //scale impulse based upon number of contact points since force is
      //evenly distributed across area of contact
      Vector2D cImpulse = new Vector2D (impulseX, impulseY).scale (
          (1 / (double) numContactPoints));


      double negInvDelta = -1 / delta;

      //add force and torque on this body to be applied at end of turn,
      //after friction is evaluated
      this.applyForce (cImpulse.getScaled (negInvDelta));
      this.applyTorque ((impulseX * rAY - impulseY * rAX) * negInvDelta);
      this.addCollidedBodyManifold (cBody, cManifold);

      //apply force and torque on colliding body iff it is a rigid body
      if (cBody instanceof RigidBody)
      {
        ((RigidBody) cBody).applyForce (cImpulse);
        ((RigidBody) cBody).applyTorque ((impulseX * rBY - impulseY * rBX)
                                          * negInvDelta);
        ((RigidBody) cBody).addCollidedBodyManifold (cBody, cManifold.getSwap ());
      }
    }
  }

  public void addCollidedBodyManifold (PhysicsBody cBody, ContactManifold cManifold)
  {
    mcCollisionBodyManifoldMap.put (cBody, cManifold);
  }

  public void removeCollidedBody (PhysicsBody cBody)
  {
    if (null != cBody)
    {
      mcCollisionBodyManifoldMap.remove (cBody);
    }

    super.removeCollidedBody (cBody);
  }

  public boolean hasCollidedBodyManifold (PhysicsBody cBody)
  {
    return mcCollisionBodyManifoldMap.containsKey (cBody);
  }

  //clears collision-contactmanifold map
  public void clearCollidingSet ()
  {
    mcCollisionBodyManifoldMap.clear ();
    super.clearCollidingSet ();
  }


  public void clearForces ()
  {
    mcTranslationalForce.scale (0);
    mTorque = 0;
  }

  //if turn is out of date then update and clear store colliding sets
  public boolean updateTurn (int turn)
  {
    if (super.updateTurn (turn))
    {
      clearForces ();
      return true;
    }

    return false;
  }

  //if overwriting this function must call super.update(delta) in order to
  //apply physics, else call this.updatePhysics(delta) in overwritten function
  public void update (double delta)
  {
    this.updatePhysics (delta);
  }
  

}




/* before adding in rotational component to friction calculation
private void calcFriction (double delta)

    //total force on this body
    Vector2D cForceTotal = mcTranslationalForce;
    //velocities of this body
    Vector2D cVelocityA = this.mcVelocity;

    //TODO IMPORTANT: ensure contact manifold doesn't lose/gain energy (vs single contact point)
    for (Iterator<PhysicsBody> cBodies
         = mcCollisionBodyManifoldMap.keySet ().iterator ();
         cBodies.hasNext ();)
    {
      PhysicsBody cBody = cBodies.next ();
      ContactManifold cManifold = mcCollisionBodyManifoldMap.get (cBody);

      //normal to collision surface
      Vector2D cNormal = cManifold.mcNormalA;

      //calc tangent to normal in direction of F_total
      Vector2D cTangent = cNormal.getTangent (cForceTotal);

      //calc normal force
      double forceNormal = cForceTotal.projection (cNormal);
      //calc tangental force
      double forceTangental = cForceTotal.projection (cTangent);

      Vector2D cVelocityB = cBody.mcVelocity;

      //eval both objects current speeds relative to the tangent,
      //and use to solve for the relative speed of objects
      double relSpeed = this.mcVelocity.projection (cTangent)
                      - cBody.mcVelocity.projection (cTangent);

      //frictional coefficient of objects, for F_fric = mu * F_normal
      double mu,
             muA,
             muB;
      boolean bIsDynamic = false;

      //if objects are not moving relative to each other, then static friction
      //is applied
      if (RigidBody.APPROX_ZERO > relSpeed)
      {
        muA = this.getStaticFrictionCoefficient ();
        muB = cBody.getStaticFrictionCoefficient ();
      }
      //else dynamic friction is applied
      else
      {
        muA = this.getDynamicFrictionCoefficient ();
        muB = cBody.getDynamicFrictionCoefficient ();
        bIsDynamic = true;
      }

      mu = Math.sqrt (muA * muA + muB * muB);

      //max friction that can affect total friction in direction of the tangent
      double maxFriction = forceNormal * mu;

      //if tangental force is greater than max friction force,
      //then total force is equal to total force - maxFriction in direction
      //of tangental
      if (forceTangental >= maxFriction)
      {
        mcTranslationalForce.subtract (cTangent.scale (forceTangental
                                                        - maxFriction));
      }
      //else set force in tangental direction to zero
      else
      {
        mcTranslationalForce.subtract (cTangent.scale (forceTangental));
      }

    }


 */