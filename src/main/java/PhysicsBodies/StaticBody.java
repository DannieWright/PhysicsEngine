package PhysicsBodies;

import ConvexPolygon.ContactManifold;
import ConvexPolygon.Shape;

abstract public class StaticBody extends PhysicsBody
{
  public StaticBody (Shape cShape)
  {
    super (cShape);
  }

  abstract public void collide (PhysicsBody cBody, ContactManifold cManifold);
  abstract public void update (double delta);
}
