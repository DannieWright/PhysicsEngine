package ConvexPolygon;

import Vectors.Line2D;
import Vectors.Vector2D;
import org.junit.Test;

import static org.junit.Assert.*;

public class ContactManifoldTest {

  @Test
  public void find() {
    Circle cCircle1 = new Circle(new Vector2D(0,0), 4),
           cCircle2 = new Circle (cCircle1);

    Vector2D cStart = new Vector2D(0,0),
             cEnd = new Vector2D(1,1),
             cMinTransVec = new Vector2D(2,2);
    ContactEdge cCircle1Edge = new ContactEdge(new Line2D(cStart, cEnd), cMinTransVec),
                cCircle2Edge = new ContactEdge(cCircle1Edge);

    ContactManifold cManifold = new ContactManifold();

    cManifold.insert(cCircle1, cCircle1Edge);
    assertNotNull("Object not found", cManifold.find(cCircle1));
    assertNull("Object was found but should not be found",
                cManifold.find(cCircle2));

    cManifold.insert(cCircle2, cCircle2Edge);
    assertNotNull("Object not found", cManifold.find(cCircle1));
    assertNotNull("Object not found", cManifold.find(cCircle2));
  }
}