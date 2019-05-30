package ConvexPolygon;

import Vectors.Vector2D;
import org.junit.Test;

import static org.junit.Assert.*;

public class CircleTest {

  @Test
  public void contactManifold() {
    Shape cCirc1 = new Circle(0,0,2),
          cCirc2 = new Circle(-1,-1,1),
          cCirc3 = new Circle(3,3,3),
          cCirc4 = new Circle((Circle) cCirc3),
          cRect1 = new Rectangle(new Vector2D(1.5,3),
                   new Vector2D(3,1.75)),
          cTri1 = new Polygon(new Vector2D[] {new Vector2D(0.75,1),
                  new Vector2D(-1,2),
                  new Vector2D(-1,0.5)});
  }
}