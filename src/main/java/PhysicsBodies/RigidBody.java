package PhysicsBodies;

import ConvexPolygon.ContactManifold;
import ConvexPolygon.MinTransVec;
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

  private Vector2D mcTranslationalForce,
                   mcTranslationalForceConst;
  
  
  private double mTorque,
                 mTorqueConst;

  private double  mTranslationalDampening,
                  mRotationalDampening;

  private boolean mbSleep,
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
    if (mbApplyGravity)
    {
      mcTranslationalForce.add (PhysicsEngine.getInstance ().getGravity ());
    }
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
    Vector2D cOffSet = mcVelocity.getScaled (delta);
    this.offSet (cOffSet, mAngularVelocity * delta);

    //apply additional position correction to fix
    this.additionalCorrection ();
    
    this.clearForces ();
  }
  
  private void additionalCorrection ()
  {
    final double allowance = 0.01d; //penetration allowance
    final double percent = -0.001d; //amount correct by -0.01 is decent
    
    for (Iterator<PhysicsBody> cBodies = this.mcCollisionBodyManifoldMap.keySet ().iterator ();
    cBodies.hasNext ();)
    {
      PhysicsBody cBody = cBodies.next ();
      ContactManifold cManifold = mcCollisionBodyManifoldMap.get (cBody);
      MinTransVec cMinTransVec = cManifold.mcMinTransVec;
  
      double penetrationDepth = cMinTransVec.mcMinTransVec.getMagnitude ();
      double correctionDepth = Math.max (penetrationDepth - allowance, 0.0d) / (this.getInvMass () + cBody.getInvMass ()) * percent;
      
      this.offSet (cManifold.mcNormalA.getScaled (correctionDepth));
      
      //only move other body if other body is a RigidBody
      if (cBody instanceof RigidBody)
      {
        cBody.offSet (cManifold.mcNormalB.getScaled (correctionDepth));
      }
    }
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
             caContactPointsB[] = cManifold.mcaContactManifoldB;

    Vector2D cVelocityA = this.mcVelocity,
             cVelocityB = cBody.mcVelocity;
    
    double angularVelocityA = this.mAngularVelocity,
           angularVelocityB = cBody.mAngularVelocity;

    
    //it should be the case that caContactPointsA.length == caContactPointsB.length
    int numContactPoints = caContactPointsA.length;
    
    //radius from body center to contact point
    Vector2D cRA,
             cRB;
    
    //TODO revise approximation
    //if there are two points of contact then the collision was between to
    //parallel edges, to simplify find effective contact point between the two
    if (2 == numContactPoints)
    {
      cRA = (caContactPointsA[0].getSubtract (cPositionA).add (
              caContactPointsA[0].getSubtract (cPositionA))).scale (0.5);
      cRB = (caContactPointsB[0].getSubtract (cPositionB).add (
              caContactPointsB[0].getSubtract (cPositionB))).scale (0.5);
    }
    else
    {
      cRA = caContactPointsA[0].getSubtract (cPositionA);
      cRB = caContactPointsB[0].getSubtract (cPositionB);
    }
    

    //calc velocity of contact points
    double vAX = cVelocityA.mX - cRA.mY * angularVelocityA,
           vAY = cVelocityA.mY + cRA.mX * angularVelocityA,
           vBX = cVelocityB.mX - cRB.mY * angularVelocityB,
           vBY = cVelocityB.mY + cRB.mX * angularVelocityB;

//    //if the contact points are going away from each other then do not calculate
//    //impulse needed to remove them
//    if (new Vector2D (vAX, vAY).dot (new Vector2D (vBX, vBY)) > 0)
//    {
//      return;
//    }
    
    //calc values used to solve for impulse
    Vector2D cRelativeVel = new Vector2D (vBX - vAX, vBY - vAY),
             cMinTransVec = cManifold.mcMinTransVec.mcMinTransVec.getUnit ();
    
    if (this.isShape (cManifold.mcMinTransVec.mcRecipient))
    {
      cMinTransVec.mirror ();
    }
  
    double elasticity = Math.min (this.getElasticity (),
                                  cBody.getElasticity ()),
           invMassA = this.getInvMass (),
           invInertiaA = this.getInvInertia (),
           invMassB = cBody.getInvMass (),
           invInertiaB = cBody.getInvInertia ();
    
    double elasticityFactor = -(elasticity + 1),
           sumInvMass = invMassA + invMassB,
           rACrossN = cRA.cross (cMinTransVec),
           rBCrossN = cRB.cross (cMinTransVec),
           sumInvInertia = rACrossN * rACrossN * invInertiaA + rBCrossN * rBCrossN * invInertiaB;
    
    double denominator = sumInvInertia + sumInvMass;
    
    //if inv inertia && mass are all zero an impulse value cannot
    //be calculated
    if (0 == denominator)
    {
      return;
    }
    
    //calc impulse
    double impulse = cRelativeVel.dot (cMinTransVec) * elasticityFactor / denominator;
    //want to apply impulse in the direction of the minimum translational vector
    //this.applyImpulse (cMinTransVec.scale (impulse), cRA);
    //to calc friction want to sum all forces, so instead of applying impulse
    //directly we will apply force and torque
    Vector2D cForce = cMinTransVec.scale (-impulse / delta);
    this.applyForce (cForce);
    this.applyTorque (cForce.cross (cRA));
    
    //add bodies to map with manifold for calculations at the end of the turn,
    //this will only add if the bodies have masses/inertias for further
    //calculations
    this.addCollidedBodyManifold (cBody, cManifold);

    //if other body is also rigid body, then apply impulse to it as well
    if (cBody instanceof RigidBody)
    {
      RigidBody cRigid = (RigidBody) cBody;
      //-1 because the impulse needs to go in the opposite direction
      cRigid.applyForce (cForce.scale (-1));
      cRigid.applyTorque (cForce.cross (cRB));
      cRigid.addCollidedBodyManifold (this, cManifold.getSwap ());
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