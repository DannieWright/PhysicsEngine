package PhysicsBodies;

import ConvexPolygon.Circle;
import ConvexPolygon.ContactManifold;
import ConvexPolygon.Rectangle;
import Vectors.Vector2D;
import org.junit.Test;

import static org.junit.Assert.*;

public class RigidBodyTest
{

  private class Box extends RigidBody
  {
    public Box (Vector2D cTopLeft, Vector2D cBottomRight)
    {
      super (new Rectangle (cTopLeft, cBottomRight));
    }

    @Override
    public void collide (PhysicsBody cBody, ContactManifold cManifold)
    {

    }
  }

  private class Ball extends RigidBody
  {
    public Ball (Vector2D cPos, double radius)
    {
      super (new Circle (cPos, radius));
    }

    public Ball (double xPos, double yPos, double radius)
    {
      super (new Circle (new Vector2D (xPos, yPos), radius));
    }


    @Override
    public void collide (PhysicsBody cBody, ContactManifold cManifold)
    {

    }
  }

  private class Wall extends StaticBody
  {
    public Wall (Vector2D cTopLeft, Vector2D cBottomRight)
    {
      super (new Rectangle (cTopLeft, cBottomRight));
    }

    @Override
    public void collide (PhysicsBody cBody, ContactManifold cManifold)
    {

    }

    @Override
    public void update (double delta)
    {

    }
  }

  @Test
  public void calcPhysics ()
  {
    Ball cBall = new Ball (0, 2, 2);
    Wall cWall = new Wall (new Vector2D (1.9,5), new Vector2D (4, 0));
    Vector2D cVelocity = new Vector2D (10, 0),
             cExpectedVelocity = cVelocity.getMirrorX ();
    final double DELTA = 1;

    cBall.setMass (10);
    cBall.mcVelocity = cVelocity;

    ContactManifold cManifold = cBall.contactManifold (cWall);

    assertNotNull ("cManifold != null", cManifold);

    cBall.calcPhysics (cWall, cManifold, DELTA);
    cBall.updatePhysics (DELTA);

    assertTrue ("cBall.mcVelocity gets mirrored", cExpectedVelocity.equals (cBall.mcVelocity));


    
  }
}