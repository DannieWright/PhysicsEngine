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
    /*
    Elastic collision between a ball (m = 10kg and v_i = 10m/s) and a
    static wall (infinite mass and no velocity)

    Expected Results:
    Ball is reflected, v_f = -v_i
    Wall has no change
     */
    Ball cBall = new Ball (0, 2, 2);
    Wall cWall = new Wall (new Vector2D (1.9,5), new Vector2D (4, 0));
    Vector2D cVelocity = new Vector2D (10, 0),
             cExpectedVelocity = cVelocity.getMirrorX ();
    final double DELTA = 1;

    cBall.setMass (10);
    cBall.setInertia (10);
    cBall.mcVelocity = cVelocity;

    ContactManifold cManifold = cBall.contactManifold (cWall);

    assertNotNull ("cManifold != null", cManifold);

    cBall.calcPhysics (cWall, cManifold, DELTA);
    System.out.println (cBall.mcVelocity.toString ());
    cBall.updatePhysics (DELTA);

    assertTrue ("cBall.mcVelocity gets mirrored", cExpectedVelocity.equals (cBall.mcVelocity));
    assertTrue ("cWall has no velocity", cWall.mcVelocity.equals (new Vector2D ()));


    /*
    Elastic collision between rectangular box (m = 10kg and v_i = 10 m/s) and
    static wall (infinite mass and no velocity). Collision occurs on a face,
    so there are two points of contact

    Expected Results:
    Box is reflected, v_f = -v_i
    Wall has no change
     */
    Box cBox = new Box (new Vector2D (0, 4), new Vector2D (2, 0));
    cBox.setMass (10);
    cBox.mcVelocity = new Vector2D (10, 0);
    cExpectedVelocity = cBox.mcVelocity.getMirrorX ();

    cManifold = cBox.contactManifold (cWall);

    assertNotNull ("cManifold != null", cManifold);

    cBox.calcPhysics (cWall, cManifold, DELTA);
    cBox.updatePhysics (DELTA);
    
    System.out.println (cBox.mcVelocity.toString ());

    assertTrue ("cBox.mcVelocity gets mirrored", cExpectedVelocity.equals (cBox.mcVelocity));
    assertTrue  ("cBox.mAngularVelocity == 0", 0 == cBox.mAngularVelocity);

    
    /*
    Elastic collision between rectangular box (m = 10kg and v_i = 10 m/s) and
    static wall (infinite mass and no velocity). Collision occurs on a face,
    so there are two points of contact. The points of contact are skewed from
    the center of mass which will cause rotational motion to occur, but total
    kinetic energy should remain preserved
    
    Expected Results:
    
     */
    
    cBox = new Box (new Vector2D (0, 4), new Vector2D (2, 0));
    cBox.setMass (10);
    cBox.setInertia (10);
    cBox.mcVelocity = new Vector2D (10, 0);
    cBox.rotate (0.001);
    double KEinit = cBox.kineticEnergy ();
    
    System.out.println ("KE_init: " + KEinit);
    
    cManifold = cBox.contactManifold (cWall);
  
    assertNotNull ("cManifold != null", cManifold);
  
    cBox.calcPhysics (cWall, cManifold, DELTA);
//
//    System.out.println ("F_trans: " + cBox.mcTranslationalForce);
//    System.out.println ("torque: " + cBox.mTorque);
    
    cBox.updatePhysics (DELTA);
    
    double KEfinal = cBox.kineticEnergy ();
    
    System.out.println ("KE_final: " + KEfinal);
    System.out.println ("v_final: " + cBox.mcVelocity.toString ());
    System.out.println ("omega_final: " + cBox.mAngularVelocity);
    
    
    

    
    Vector2D r1 = new Vector2D (-2, 1),
             r2 = new Vector2D (1, 1),
             r_eff = r1.getAdd (r2).scale (0.5);
  
    Vector2D n = new Vector2D (0, 1);
    
//    double F_o = 10,
//           F_eff = F_o * (n.projection (r1) + n.projection (r2)) /  n.projection (r_eff);
//
//
//    double F_trans = -F_o * (n.projection (r1) + n.projection (r2)),
//           torque = -F_o * (n.projection (r1.getNormalCW ()) * r1.getMagnitude ()
//                          + n.projection (r2.getNormalCW ()) * r2.getMagnitude ());
//    double F_trans_eff = -F_eff * (n.projection (r_eff)),
//           torque_eff = -F_eff * (n.projection (r_eff.getNormalCW ())) * r_eff.getMagnitude ();
//
//    System.out.println ("F_trans: " + F_trans);
//    System.out.println ("torque:  " + torque);
//    System.out.println ("F_trans_eff: " + F_trans_eff);
//    System.out.println ("torque_eff:  " + torque_eff);
//
    
    
  }
}