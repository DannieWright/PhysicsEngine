package PhysicsBodies;

import AABBTree.AABB;
import AABBTree.AABBObject;
import ConvexPolygon.ContactManifold;
import ConvexPolygon.MinTransVec;
import ConvexPolygon.Polygon;
import ConvexPolygon.Shape;
import PhysicsEngine.PhysicsEngine;
import Vectors.Vector2D;

import java.util.HashSet;
import java.util.Iterator;

abstract public class PhysicsBody extends AABBObject
{
  //bodies stick together, KE is not conserved
  public static final double INELASTIC = 0.0d;
  //bodies reflect, KE is conserved
  public static final double ELASTIC = 1.0d;
  protected static final int DEFAULT_SIZES = 20;

  private Shape mcShape;

  //TODO set up adjusting sizes
  private HashSet<PhysicsBody> mcNoCollideBodySet;
  private HashSet<Class> mcNoCollideClassSet;
  protected HashSet<PhysicsBody> mcCollidedSet; //if mbNoCheckCollide == false, don't add

  private Vector2D mcPosition;
  protected Vector2D mcVelocity;
  protected double mAngularVelocity;
  private double mMass,
                 mInvMass,
                 mInertia,
                 mInvInertia;

  private double mElasticity, //value from 0 to 1 inclusive
                 mStaticFrictionCoefficient,
                 mDynamicFrictionCoefficient;

  private int mTurn,
              mLastTurn;

  private boolean mbCanCollide,
                  mbCheckCollide,
                  mbForceUpdate;

  //TODO way to store constraint information

  //TODO additional constructor with boolean to determine if user wants to force prevent adding body to physics engine

  //TODO collision layer

  public PhysicsBody (Shape cShape)
  {
    super (cShape.getAABB ());

    mcShape = cShape;
    mcNoCollideBodySet = new HashSet<> (PhysicsBody.DEFAULT_SIZES);
    mcNoCollideClassSet = new HashSet<> (PhysicsBody.DEFAULT_SIZES);
    mcCollidedSet = new HashSet<> (PhysicsBody.DEFAULT_SIZES);

    mcPosition = cShape.getCenter ();
    mcVelocity = new Vector2D ();
    mAngularVelocity = 0;

    //these values can ONLY be set to a value directly here, by default
    //they represent an object with infinite mass and inertia
    mMass = 0;
    mInertia = 0;
    mInvMass = 0;
    mInvInertia = 0;

    mElasticity = PhysicsBody.ELASTIC;
    mStaticFrictionCoefficient = mDynamicFrictionCoefficient = 0.0d;

    mTurn = 0;
    mLastTurn = -1;

    //must call function to set this to ensure PhysicsEngine's AABBTree adds it
    this.setCanCollide (true);

    //can set this to false directly here to save processing instead of calling
    //this.setCheckCollide (), but can ONLY be done here and ONLY false
    mbCheckCollide = false;

    //adds additional chance for body to call update() without requiring
    //a check for collisions. when mbForceUpdate is true update() is called
    //after all collisions have been detected. update() is called twice if
    //mbCheckCollide = true. if body checks collisions && force updates, can
    //distinguished between each update() calls by if hasUpdate() == true
    //update() has already been called that turn
    mbForceUpdate = false;

    //add physics body to PhysicsEngine
    PhysicsEngine.getInstance ().markAdd (this);
  }

  //called when the collision with a body happens
  //users should NOT call move or rotate functions from this collide function
  //as the AABB of the object will not be updated until the end of their turn
  //but the Shape will immediately, which can effect collision detection
  abstract public void collide (PhysicsBody cBody, ContactManifold cManifold);


  //called if body checks for collisions and/or if body sets mbForceUpdate
  // == true
  abstract public void update (double delta);


  /*
  adds body to this bodies set of colliding bodies iff this body actively
  checks for collisions/updates. this is to prevent bodies with
  mbCheckCollide == false to have stale mcCollidedSet, since the set will not
  be cleared.
   */
  public void addCollidedBody (PhysicsBody cBody)
  {
    if (null != cBody && mbCheckCollide)
    {
      mcCollidedSet.add (cBody);
    }
  }

  public void addCollisionException (PhysicsBody cBody)
  {
    if (null != cBody)
    {
      mcNoCollideBodySet.add (cBody);
    }
  }

  public void addCollisionException (Class cClass)
  {
    if (null != cClass)
    {
      mcNoCollideClassSet.add (cClass);
    }
  }

  public boolean canCollide ()
  {
    return mbCanCollide;
  }

  public boolean canCollideBody (PhysicsBody cBody)
  {
    return null != cBody && cBody.canCollide ()
        && !this.mcNoCollideClassSet.contains (cBody.getClass ())
        && !this.mcNoCollideBodySet.contains (cBody)
        && !this.mcCollidedSet.contains (cBody);
  }

  public boolean checkCollide ()
  {
    return mbCheckCollide;
  }

  public void clearCollidingSet ()
  {
    this.mcCollidedSet.clear ();
  }

  public void clearCollisionExceptionBody ()
  {
    mcNoCollideBodySet.clear ();
  }

  public void clearCollisionExceptionClass ()
  {
    mcNoCollideClassSet.clear ();
  }

  public ContactManifold contactManifold (PhysicsBody cBody)
  {
    MinTransVec cMinTransVec = Shape.overlap (this.mcShape, cBody.mcShape);

    //if objects are not colliding, return null
    if (null == cMinTransVec)
    {
      return null;
    }

    return this.mcShape.contactManifold (cMinTransVec);
  }

  public boolean forceUpdate ()
  {
    return mbForceUpdate;
  }

  public double getDynamicFrictionCoefficient ()
  {
    return mDynamicFrictionCoefficient;
  }

  public double getElasticity ()
  {
    return mElasticity;
  }

  public double getInertia ()
  {
    return mInertia;
  }

  public double getInvInertia ()
  {
    return mInvInertia;
  }

  public double getInvMass ()
  {
    return mInvMass;
  }

  public int getLastTurn ()
  {
    return mLastTurn;
  }

  public double getMass ()
  {
    return mMass;
  }

  public Vector2D getPosition ()
  {
    return new Vector2D (mcPosition);
  }

  public double getStaticFrictionCoefficient ()
  {
    return mStaticFrictionCoefficient;
  }

  public int getTurn ()
  {
    return mTurn;
  }

  public boolean hasUpdated ()
  {
    return mTurn == mLastTurn;
  }

  public boolean hasBeenUpdated (int turn)
  {
    return turn == mTurn && turn == mLastTurn;
  }


  public boolean hasCollidedBody (PhysicsBody cBody)
  {
    if (null == cBody)
    {
      return false;
    }
    else
    {
      return mcCollidedSet.contains (cBody);
    }
  }

  /*
  body is added to a set to be removed from physics engine at end of turn
  this removes the physics engine knowing about the body and updating it.
  if body information was saved elsewhere the object will still exist

  since bodies are only deleted after the turn, this body will update this turn
  and be involved in normal collision detection
  */
  public void markDestroy ()
  {
    PhysicsEngine.getInstance ().markRemove (this);
  }

  public int numBodyExceptions ()
  {
    return this.mcNoCollideBodySet.size ();
  }

  public int numClassExceptions ()
  {
    return this.mcNoCollideClassSet.size ();
  }

  public int numCollidingBodies ()
  {
    return this.mcCollidedSet.size ();
  }

  public void offSet (double x, double y, double angle)
  {
    mcShape.offSet (x, y);
    mcShape.rotate (angle);
    mcPosition.offSet (x, y);
    mcAABB = mcShape.getAABB ();
    PhysicsEngine.getInstance ().updateAABB (this);
  }

  public void offSet (Vector2D cVec, double angle)
  {
    this.offSet (cVec.mX, cVec.mY, angle);
  }

  public void offSet (double x, double y)
  {
    mcShape.offSet (x, y);
    mcPosition.offSet (x, y);
    mcAABB.offSet (x, y);
    PhysicsEngine.getInstance ().updateAABB (this);
  }

  public void offSet (Vector2D cVec)
  {
    this.offSet (cVec.mX, cVec.mY);
  }

  //removes cBody from this bodies set of colliding bodies
  public void removeCollidedBody (PhysicsBody cBody)
  {
    if (null != cBody)
    {
      mcCollidedSet.remove (cBody);
    }
  }

  public void removeCollisionException (PhysicsBody cBody)
  {
    if (null != cBody)
    {
      mcNoCollideBodySet.remove (cBody);
    }
  }

  public void removeCollisionException (Class cClass)
  {
    if (null != cClass)
    {
      mcNoCollideClassSet.remove (cClass);
    }
  }

  //goes through all bodies this body is colliding with, and has them delete
  //this body from their colliding with set
  public void removeFromCollidedBodySet ()
  {
    for (Iterator<PhysicsBody> cBodies = mcCollidedSet.iterator ();
         cBodies.hasNext ();)
    {
      PhysicsBody cBody = cBodies.next ();

      cBody.removeCollidedBody (this);
    }
  }

  public void rotate (double angle)
  {
    mcShape.rotate (angle);
    mcAABB = mcShape.getAABB ();
    PhysicsEngine.getInstance ().updateAABB (this);
  }

  /*
  when setting mbCanCollide to true, Body is not added to AABBTree until
  end of current turn, and so collisions will not be detected until next turn

  this does NOT make it so this body is updated each turn or this body checks
  for collisions, only that other bodies can check to collide with this body.
  To allow this body to update each turn and check for collisions, set
  mcCheckCollide = true

  when setting mbCanCollide to false, Body remains in AABBTree until next turn
  so collisions are detected, but no response is taken for any collisions post
  function call. This means collisions are disabled immediately.
 */
  public void setCanCollide (boolean bCollide)
  {
    if (bCollide != mbCanCollide)
    {
      PhysicsEngine.getInstance ().markUpdateCanCollide (this);
      mbCanCollide = bCollide;
    }
  }

  /*
  body is added to a set to be placed in AABBTree at end of current turn.
  this allows the body to check for collisions and update accordingly.

  mbCheckCollide determines if a body actively seeks out knowledge about whether
  collisions with it have occurred, mbCheckCollide does not determine whether
  a body is able to collide with other bodies, for that use
  mbCanCollide = true

  because mbCheckCollide determines if objects should check for collides bodies
  with mbCheckCollide are more resource expensive and so this should only be
  true for bodies that need to update and move
 */
  public void setCheckCollide (boolean bCheckCollide)
  {
    if (bCheckCollide != mbCheckCollide)
    {
      PhysicsEngine.getInstance ().markUpdateCheckCollide (this);
      mbCheckCollide = bCheckCollide;
    }
  }

  public void setDynamicFrictionCoefficient (double frictionCoeff)
  {
    if (frictionCoeff >= 0)
    {
      mDynamicFrictionCoefficient = frictionCoeff;
    }
  }

  public void setElasticity (double elasticity)
  {
    if (elasticity < PhysicsBody.INELASTIC)
    {
      mElasticity = PhysicsBody.INELASTIC;
    }
    else if (elasticity > PhysicsBody.ELASTIC)
    {
      mElasticity = PhysicsBody.ELASTIC;
    }
    else
    {
      mElasticity = elasticity;
    }
  }

  /*
  setting mbForceUpdate to true means the PhysicsEngine calls update() for body
  after all collisions have been detected. update() is not called for bodies
  that check for collisions
   */
  public void setForceUpdate (boolean bForceUpdate)
  {
    if (bForceUpdate != mbForceUpdate)
    {
      mbForceUpdate = bForceUpdate;
      PhysicsEngine.getInstance ().markForceUpdate (this);
    }
  }

  public void setInertia (double inertia)
  {
    if (0 >= inertia)
    {
      mInertia = 0;
      mInvInertia = 0;
    }
    else
    {
      mInertia = inertia;
      mInvInertia = 1 / inertia;
    }
  }

  public void setMass (double mass)
  {
    if (0 >= mass)
    {
      mMass = 0;
      mInvMass = 0;
    }
    else
    {
      mMass = mass;
      mInvMass = 1 / mass;
    }
  }

  public void setPos (double x, double y)
  {
    this.offSet (x - mcPosition.mX, y - mcPosition.mY);
  }

  public void setPos (Vector2D cVec)
  {
    this.setPos (cVec.mX, cVec.mY);
  }

  //TODO may want to queue update shape until end of turn
  //updating mid turn may cause collision issues!!!!!!!!!!!!!!!!!
  public void setShape (Shape cShape)
  {
    mcShape = cShape;
    mcPosition = cShape.getCenter ();
    mcAABB = cShape.getAABB ();
    PhysicsEngine.getInstance ().updateAABB (this);
  }

  public void setStaticFrictionCoefficient (double frictionCoeff)
  {
    if (frictionCoeff >= 0)
    {
      mStaticFrictionCoefficient = frictionCoeff;
    }
  }

  //if turn is out of date then update and clear store colliding sets
  public boolean updateTurn (int turn)
  {
    if (turn != mTurn)
    {
      this.clearCollidingSet ();
      mTurn = turn;
      return true;
    }

    return false;
  }

  public void updateLastTurn (int turn)
  {
    mLastTurn = turn;
  }


  /*

  The following are meant for debugging purposes

   */

  public AABB getAABB ()
  {
    return new AABB (mcAABB);
  }


  //TODO add temp check for collision that adds the body to the AABBTree,
  //checks for collisions, then immediately removes it
}
