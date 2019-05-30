package PhysicsEngine;

import AABBTree.AABBTree;
import AABBTree.AABBObject;
import ConvexPolygon.ContactManifoldBasic;
import PhysicsBodies.PhysicsBody;
import PhysicsBodies.RigidBody;
import Vectors.Vector2D;

import java.util.*;

public class PhysicsEngine
{
  private static final int DEFAULT_SIZE = 20;
  private static final Vector2D cDEFAULT_GRAVITY = new Vector2D (0.0d,
                                                                -9.81d);

  private AABBTree mcAABBTree;

  //physicsbodies that check for collisions each turn
  private HashSet<PhysicsBody> mcCheckCollideSet;
  //physicsbodies that do not check for collisions each turn
  private HashSet<PhysicsBody> mcNoCheckCollideSet;
  //physicsbodies that do not check for collides but call update()
  private HashSet<PhysicsBody> mcForceUpdateSet;

  private HashSet<PhysicsBody> mcMarkedRemoveSet,
                               mcMarkedAddSet,
                               mcMarkUpdateCheckCollideSet,
                               mcMarkUpdateCanCollideSet,
                               mcMarkForceUpdateSet;

  private Vector2D mcGravity;

  private int mTurn,
              mSize;

  private boolean mbPreventRemoveLeaking;


  private static PhysicsEngine ourInstance = new PhysicsEngine ();

  public static PhysicsEngine getInstance ()
  {
    return ourInstance;
  }


  private PhysicsEngine ()
  {
    this.setSize (PhysicsEngine.DEFAULT_SIZE);

    mcGravity = new Vector2D (cDEFAULT_GRAVITY);

    //when body is removed, clears removed bodies knowledge of bodies still
    //apart of the engine, and clears removed body from bodies still in the
    //engine's memory
    mbPreventRemoveLeaking = true;
  }

  //clears all contained bodies
  public void setSize (int size)
  {
    if (size > 0)
    {
      mcAABBTree = new AABBTree (size);

      mcCheckCollideSet = new HashSet<> (size);
      mcNoCheckCollideSet = new HashSet<> (size);
      mcForceUpdateSet = new HashSet<> (size);
      mcMarkedRemoveSet = new HashSet<> (size);
      mcMarkedAddSet = new HashSet<> (size);
      mcMarkUpdateCheckCollideSet = new HashSet<> (size);
      mcMarkUpdateCanCollideSet = new HashSet<> (size);
      mcMarkForceUpdateSet = new HashSet<> (size);

      mTurn = 0;
      mSize = size;
    }
  }

  public int size ()
  {
    return mSize;
  }


  public void markAdd (PhysicsBody cBody)
  {
    if (null != cBody)
    {
      mcMarkedAddSet.add (cBody);
    }
  }

  public void markRemove (PhysicsBody cBody)
  {
    if (null != cBody)
    {
      mcMarkedRemoveSet.add (cBody);
    }
  }

  private void addBodies ()
  {
    for (Iterator<PhysicsBody> cBodies = mcMarkedAddSet.iterator ();
         cBodies.hasNext ();)
    {
      PhysicsBody cBody = cBodies.next ();

      if (cBody.checkCollide ())
      {
        mcCheckCollideSet.add (cBody);
      }
      else
      {
        mcNoCheckCollideSet.add (cBody);
      }

      if (cBody.canCollide ())
      {
        mcAABBTree.removeObject (cBody);
      }
    }

    //clear mcMarkedAddSet
    mcMarkedAddSet.clear ();
  }

  private void removeBodies ()
  {
    for (Iterator<PhysicsBody> cBodies = mcMarkedRemoveSet.iterator ();
         cBodies.hasNext ();)
    {
      PhysicsBody cBody = cBodies.next ();

      //remove body from all possible references
      mcMarkedAddSet.remove (cBody);
      mcCheckCollideSet.remove (cBody);
      mcNoCheckCollideSet.remove (cBody);
      mcAABBTree.removeObject (cBody);
      mcMarkUpdateCheckCollideSet.remove (cBody);
      mcMarkUpdateCanCollideSet.remove (cBody);
      mcMarkForceUpdateSet.remove (cBody);
      mcForceUpdateSet.remove (cBody);

      /*
      TODO may not need to do these, these just prevent leaking of information
      about physics bodies, but is EXPENSIVE
       */
      if (mbPreventRemoveLeaking)
      {
        //removes cBody from records of bodies it has previously collided with
        cBody.removeFromCollidedBodySet ();
        //clears set of bodies cBody has collided with to prevent leaked
        //information
        cBody.clearCollidingSet ();
      }
    }

    //clear mcMarkedRemoveSet
    mcMarkedRemoveSet.clear ();
  }

  public void markUpdateCanCollide (PhysicsBody cBody)
  {
    if (null != cBody)
    {
      mcMarkUpdateCanCollideSet.add (cBody);
    }
  }

  //must be run after add/remove bodies is updated
  private void setCanCollide ()
  {
    for (Iterator<PhysicsBody> cBodies
          = mcMarkUpdateCanCollideSet.iterator ();
          cBodies.hasNext ();)
    {
      PhysicsBody cBody = cBodies.next ();

      //attempt to remove body regardless of whether cBody was able to collide
      //or is being set to collide. This is done since cBody.mbCanCollide can
      //be updated more than once per turn, since cBody.mbCanCollide
      //is update immediately
      mcAABBTree.removeObject (cBody);

      if (cBody.canCollide ())
      {
        mcAABBTree.insertObject (cBody);
      }
    }

    mcMarkUpdateCanCollideSet.clear ();
  }

  public void markUpdateCheckCollide (PhysicsBody cBody)
  {
   if (null != cBody)
   {
     mcMarkUpdateCheckCollideSet.add (cBody);
   }
  }

  //TODO maybe set up booleans in PhysicsBodies the establish if mbCheckCollide and mbCanCollide have been edited this turn, with old value,
  //then if they try to set it back to original value the sets and AABBTree do not need to be updated
  //must be run after add/remove bodies is updated
  private void setCheckCollide ()
  {
    for (Iterator<PhysicsBody> cBodies
         = mcMarkUpdateCheckCollideSet.iterator ();
          cBodies.hasNext ();)
    {
      PhysicsBody cBody = cBodies.next ();

      //attempts to remove cBody from both the mcNoCheckCollideSet and
      //mcCheckCollideSet sets in case cBody updated
      mcNoCheckCollideSet.remove (cBody);
      mcCheckCollideSet.remove (cBody);

      if (cBody.checkCollide ())
      {
        mcCheckCollideSet.add (cBody);
        if (cBody.canCollide ())
        {
          mcAABBTree.insertObject (cBody);
        }
      }
      else
      {
        //clears bodies colliding body sets to prevent body from having
        //stale colliding body sets
        cBody.clearCollidingSet ();

        mcNoCheckCollideSet.add (cBody);
        if (cBody.canCollide ())
        {
          mcAABBTree.removeObject (cBody);
        }
      }

    }

    mcMarkUpdateCheckCollideSet.clear ();
  }

  public void markForceUpdate (PhysicsBody cBody)
  {
    if (null != cBody)
    {
      mcMarkForceUpdateSet.add (cBody);
    }
  }

  private void setForceUpdate ()
  {
    for (Iterator<PhysicsBody> cBodies = mcMarkForceUpdateSet.iterator ();
    cBodies.hasNext ();)
    {
      PhysicsBody cBody = cBodies.next ();

      mcForceUpdateSet.remove (cBody);

      if (cBody.forceUpdate ())
      {
        mcForceUpdateSet.add (cBody);
      }
    }

    mcMarkForceUpdateSet.clear ();
  }

  //TODO may need to change to queue updates
  public void updateAABB (PhysicsBody cBody)
  {
    mcAABBTree.updateObject (cBody);
  }


  public void setGravity (Vector2D cGravity)
  {
    mcGravity = cGravity;
  }

  public Vector2D getGravity ()
  {
    return new Vector2D (mcGravity);
  }

  public boolean preventLeaking ()
  {
    return mbPreventRemoveLeaking;
  }

  public void setPreventLeaking (boolean bPreventLeaking)
  {
    mbPreventRemoveLeaking = bPreventLeaking;
  }

  public void update (double delta)
  {
    for (Iterator<PhysicsBody> cCheckCollideBodies
         = mcCheckCollideSet.iterator ();
         cCheckCollideBodies.hasNext ();)
    {
      PhysicsBody cBodyA = cCheckCollideBodies.next ();

      //if cBodyA's turn is outdated, clear previous colliding sets
      cBodyA.updateTurn (mTurn);

      //get broadphase possible overlaps,
      ArrayList<AABBObject> clAABBObjects = mcAABBTree.queryOverlaps (cBodyA);

      //if not null, then there are possible overlaps
      if (null != clAABBObjects)
      {
        int numBodies = clAABBObjects.size ();

        for (int i = 0; i < numBodies; ++i)
        {
          PhysicsBody cBodyB = (PhysicsBody) clAABBObjects.get (i);

          //if the bodies cannot collide, or cBodyB has already been updated
          //this turn, move onto next candidate
          if (cBodyB.hasBeenUpdated (mTurn)
              || !(cBodyA.canCollideBody (cBodyB)
              && cBodyB.canCollideBody (cBodyA)))
          {
            continue;
          }

          //if cBodyB's turn is outdated, clear previous colliding sets
          cBodyB.updateTurn (mTurn);

          //generate contact manifold for bodies, where manifold A values
          //are for cBodyA and manifold B values are for cBodyB
          ContactManifoldBasic cManifold = cBodyA.contactManifold (cBodyB);

          //if bodies are not colliding, move onto next candidate
          if (null == cManifold)
          {
            continue;
          }

          //if cBodyA is RigidBody, must calc affect to bodies, this func
          //will pass on needed info to cBodyB if cBodyB is also a rigid body
          if (cBodyA instanceof RigidBody)
          {
            ((RigidBody) cBodyA).calcPhysics (cBodyB, cManifold, delta);
          }

          //adds bodies to each other's set of colliding bodies
          cBodyA.addCollidedBody (cBodyB);
          cBodyB.addCollidedBody (cBodyA);

          //call collide functions for both bodies to allow user effects
          cBodyA.collide (cBodyB, cManifold);
          cBodyB.collide (cBodyA, cManifold);
        }

        //TODO could add clear to cBodyA colliding sets here to prevent leak
        //into next turn
      }

      //call update function to allow for user effects
      cBodyA.update (delta);

      //update cBodyA's mLastTurn to signify it has been updated this turn,
      //this way if cBodyA collides with a new body during this turn (after
      //cBodyA's position has been updated this turn), the collision will
      //not be counted until next turn
      cBodyA.updateLastTurn (mTurn);
    }

    //calls update() for all bodies that force update
    for (Iterator<PhysicsBody> cBodies = mcForceUpdateSet.iterator ();
        cBodies.hasNext ();)
    {
      cBodies.next ().update (delta);
    }

    //update sets of queued bodies
    this.updateSets ();

    mTurn += 1;
  }

  private void updateSets ()
  {
    //delete queued bodies
    this.removeBodies ();

    //add queued bodies
    this.addBodies ();

    //delete/add checkCollide (must be after delete and add bodies because it
    //doesn't check those queues)
    this.setCanCollide ();

    //delete/add checkCollide (must be after delete and add bodies because it
    //doesn't check those queues)
    this.setCheckCollide ();

    //forceUpdate must be done after delete/add bodies because it doesn't check
    //those queues
    this.setForceUpdate ();
  }

  //TODO function that temp adds AABB to check for collisions




  /*

  The following are meant for debugging/testing

   */

  //returns true if body has been marked to add to engine, as well as already,
  //in it
  public boolean hasBody (PhysicsBody cBody)
  {
    return mcMarkedAddSet.contains (cBody)
        || mcCheckCollideSet.contains (cBody)
        || mcNoCheckCollideSet.contains (cBody);
  }

  public boolean isInTree (PhysicsBody cBody)
  {
    return mcAABBTree.hasObject (cBody);
  }

  public int numMarkedRemove ()
  {
    return mcMarkedRemoveSet.size ();
  }

  public boolean isMarkedRemoveEmpty ()
  {
    return 0 == this.numMarkedRemove ();
  }

  public int numMarkedAdd ()
  {
    return mcMarkedAddSet.size ();
  }

  public boolean isMarkedAddEmpty ()
  {
    return 0 == this.numMarkedAdd ();
  }

  public int numMarkedUpdateCanCollide ()
  {
    return mcMarkUpdateCanCollideSet.size ();
  }

  public boolean isMarkedUpdateCanCollideEmpty ()
  {
    return 0 == this.numMarkedUpdateCanCollide ();
  }

  public int numMarkedUpdateCheckCollide ()
  {
    return mcMarkUpdateCheckCollideSet.size ();
  }

  public boolean isMarkedUpdateCheckCollideEmpty ()
  {
    return 0 == this.numMarkedUpdateCheckCollide ();
  }

  public int numMarkedForceUpdate ()
  {
    return mcMarkForceUpdateSet.size ();
  }

  public boolean isMarkedForceUpdateEmpty ()
  {
    return mcMarkForceUpdateSet.isEmpty ();
  }

  public int numForceUpdate ()
  {
    return mcForceUpdateSet.size ();
  }

  public boolean isForceUpdateEmpty ()
  {
    return mcForceUpdateSet.isEmpty ();
  }

  public int numBodiesQueued ()
  {
    return this.numMarkedAdd () + this.numMarkedRemove ()
            + this.numMarkedUpdateCanCollide ()
            + this.numMarkedUpdateCheckCollide ()
            + this.numMarkedForceUpdate ();
  }

  public boolean isQueuedBodiesEmpty ()
  {
    return 0 == this.numBodiesQueued ();
  }

  public boolean isMarkedRemove (PhysicsBody cBody)
  {
    return mcMarkedRemoveSet.contains (cBody);
  }

  public boolean isMarkedAdd (PhysicsBody cBody)
  {
    return mcMarkedAddSet.contains (cBody);
  }

  public boolean isMarkedUpdateCheckCollide (PhysicsBody cBody)
  {
    return mcMarkUpdateCheckCollideSet.contains (cBody);
  }

  public boolean isMarkedUpdateCanCollide (PhysicsBody cBody)
  {
    return mcMarkUpdateCanCollideSet.contains (cBody);
  }

  public boolean isMarkedForceUpdate (PhysicsBody cBody)
  {
    return mcMarkForceUpdateSet.contains (cBody);
  }

  public int numNoCheckCollide ()
  {
    return mcNoCheckCollideSet.size ();
  }

  public int numCheckCollide ()
  {
    return mcCheckCollideSet.size ();
  }

  public boolean isNoCheckCollide (PhysicsBody cBody)
  {
    return mcNoCheckCollideSet.contains (cBody);
  }

  public boolean isCheckCollide (PhysicsBody cBody)
  {
    return mcCheckCollideSet.contains (cBody);
  }

  public boolean isForceUpdate (PhysicsBody cBody)
  {
    return mcForceUpdateSet.contains (cBody);
  }

  //only return the bodies already in engine, not those marked to be added
  public int numBodies ()
  {
    return this.numCheckCollide () + this.numNoCheckCollide ();
  }

  //return value is only based on bodies already in engine, not those marked
  //to be added
  public boolean isEmpty ()
  {
    return 0 == this.numBodies ();
  }

  public void empty ()
  {
    mcAABBTree.empty ();
    mcCheckCollideSet.clear ();
    mcNoCheckCollideSet.clear ();
    mcMarkedRemoveSet.clear ();
    mcMarkedAddSet.clear ();
    mcMarkUpdateCheckCollideSet.clear ();
    mcMarkUpdateCanCollideSet.clear ();
    mcMarkForceUpdateSet.clear ();
    mcForceUpdateSet.clear ();
  }

  public void safeEmpty ()
  {
    mcMarkedRemoveSet.clear ();

    for (Iterator<PhysicsBody> cBodies = mcCheckCollideSet.iterator ();
    cBodies.hasNext ();)
    {
      this.markRemove (cBodies.next ());
    }

    for (Iterator<PhysicsBody> cBodies = mcNoCheckCollideSet.iterator ();
         cBodies.hasNext ();)
    {
      this.markRemove (cBodies.next ());
    }

    this.removeBodies ();

    mcMarkedAddSet.clear ();
    mcMarkUpdateCheckCollideSet.clear ();
    mcMarkUpdateCanCollideSet.clear ();
    mcMarkForceUpdateSet.clear ();
    mcForceUpdateSet.clear ();
  }

}
