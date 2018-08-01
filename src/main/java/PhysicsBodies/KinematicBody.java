package PhysicsBodies;

import ConvexPolygon.ContactManifold;
import ConvexPolygon.Shape;

abstract public class KinematicBody extends PhysicsBody
{


  public KinematicBody (Shape cShape)
  {
    super (cShape);

    //turn collision checking on
    this.setCheckCollide (true);
  }

  abstract public void collide (PhysicsBody cBody, ContactManifold cManifold);

  //mbCheckCollide must be true for update to be called, in event of wanting no
  //collide but update called, set mbCanCollide == false and have
  //mbCheckCollide == true
  abstract public void update (double delta);
}
