package PhysicsBodies;

import ConvexPolygon.ContactManifoldBasic;
import ConvexPolygon.Shape;

abstract public class StaticBody extends PhysicsBody
{
  public StaticBody (Shape cShape)
  {
    super (cShape);
  }

  abstract public void collide (PhysicsBody cBody, ContactManifoldBasic cManifold);
  abstract public void update (double delta);
}
