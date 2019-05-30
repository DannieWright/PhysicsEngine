package ConvexPolygon;

import Vectors.Line2D;
import Vectors.Vector2D;
import org.junit.Test;

import static org.junit.Assert.*;

public class PolygonTest {


  @Test
  public void contactManifold() {
    Polygon cRect1 = new Rectangle(new Vector2D(0,2),
                     new Vector2D(2,0)),
            cRect2 = new Rectangle(new Vector2D(1.5,3),
                     new Vector2D(3,1.75)),
            cRect3 = new Rectangle((Rectangle) cRect1),
            cTri1 = new Polygon(new Vector2D[] {new Vector2D(0.75,1),
                    new Vector2D(-1,2),
                    new Vector2D(-1,0.5)}),
            cTri2 = new Polygon(new Vector2D[] {new Vector2D(0,2),
                    new Vector2D(1,3),
                    new Vector2D(-1,3)});

    //resulting contact edges
    Line2D cExpected_Line_Rect1_Rect2 = new Line2D(new Vector2D(2,2),
                                        new Vector2D(1.5,2)),
           cExpected_Line_Rect2_Rect1 = new Line2D(new Vector2D(1.5,1.75),
                                        new Vector2D(2,1.75)),
           cExpected_Line_Rect1_Tri1 = new Line2D(new Vector2D(0,1),
                                       new Vector2D(0,1)),
           cExpected_Line_Tri1_Rect1 = new Line2D(new Vector2D(0.75,1),
                                       new Vector2D(0.75,1));
    //resulting minimum translational vectors
    Vector2D cExpected_Rect1_Rect2_Trans = new Vector2D(0,0.25),
             cExpected_Rect2_Rect1_Trans = cExpected_Rect1_Rect2_Trans.getMirror(),
             cExpected_Rect1_Tri1_Trans = new Vector2D(-0.75,0),
             cExpected_Tri1_Rect1_Trans = cExpected_Rect1_Tri1_Trans.getMirror();

    //resulting contact manifold
    ContactManifold cManifold_Rect1_Rect2 = cRect1.contactManifold(cRect2),
                    cManifold_Rect2_Rect1 = cRect2.contactManifold(cRect1),
                    cManifold_Rect1_Tri1 = cRect1.contactManifold(cTri1),
                    cManifold_Tri1_Rect1 = cTri1.contactManifold(cRect1),
                    cManifold_Rect2_Tri1 = cRect2.contactManifold(cTri1),
                    cManifold_Tri1_Rect2 = cTri1.contactManifold(cRect2),
                    cManifold_Rect1_Tri2 = cRect1.contactManifold(cTri2),
                    cManifold_Tri2_Rect1 = cTri2.contactManifold(cRect1),
                    cManifold_Rect1_Rect3 = cRect1.contactManifold(cRect3);


    //validate Manifolds and ContactEdges
    assertNotNull(cManifold_Rect1_Rect2);             //rect 1 && 2
    assertNotNull(cManifold_Rect1_Rect2.find(cRect1));
    assertNotNull(cManifold_Rect1_Rect2.find(cRect2));
    assertNotNull(cManifold_Rect2_Rect1);             //ensure call order is irrelevant
    assertTrue(cManifold_Rect1_Rect2.equals(cManifold_Rect2_Rect1));
    assertNotNull(cManifold_Rect1_Tri1);              //rect 1 && tri 1
    assertNotNull(cManifold_Rect1_Tri1.find(cRect1));
    assertNotNull(cManifold_Rect1_Tri1.find(cTri1));
    assertNotNull(cManifold_Tri1_Rect1);              //ensure call order is irrelevant
    assertTrue (cManifold_Rect1_Tri1.equals(cManifold_Tri1_Rect1));
    //tri1 and rect2 do not make contact
    assertNull(cManifold_Rect2_Tri1);                 //rect 2 && tri 1
    assertNull (cManifold_Tri1_Rect2);
    //tri2 and rect1's edges touch, but do not overlap
    assertNull(cManifold_Rect1_Tri2);                 //rect 1 && tri 2
    assertNull(cManifold_Tri2_Rect1);

    //grab contact information
    ContactEdge cRect1_Rect2_Edge = cManifold_Rect1_Rect2.find(cRect1),
                cRect2_Rect1_Edge = cManifold_Rect1_Rect2.find(cRect2),
                cRect1_Tri1_Edge = cManifold_Rect1_Tri1.find(cRect1),
                cTri1_Rect1_Edge = cManifold_Rect1_Tri1.find(cTri1);
    Line2D cLine_Rect1_Rect2_ContactPoints = cRect1_Rect2_Edge.getEdge(),
           cLine_Rect2_Rect1_ContactPoints = cRect2_Rect1_Edge.getEdge(),
           cLine_Rect1_Tri1_ContactPoints = cRect1_Tri1_Edge.getEdge(),
           cLine_Tri1_Rect1_ContactPoints = cTri1_Rect1_Edge.getEdge();
    Vector2D cRect1_Rect2_MinTransVec = cRect1_Rect2_Edge.getMinTransVec(),
             cRect2_Rect1_MinTransVec = cRect2_Rect1_Edge.getMinTransVec(),
             cRect1_Tri1_MinTransVec = cRect1_Tri1_Edge.getMinTransVec(),
             cTri1_Rect1_MinTransVec = cTri1_Rect1_Edge.getMinTransVec();


    //validate contact information is as expected
    //rect1 && 2
    assertTrue(cExpected_Line_Rect1_Rect2.equalsUndirected(cLine_Rect1_Rect2_ContactPoints));
    assertTrue(cExpected_Line_Rect2_Rect1.equalsUndirected(cLine_Rect2_Rect1_ContactPoints));
    assertTrue(cExpected_Rect1_Rect2_Trans.equals(cRect1_Rect2_MinTransVec));
    assertTrue(cExpected_Rect2_Rect1_Trans.equals(cRect2_Rect1_MinTransVec));
    //rect1 && tri1
    assertTrue(cExpected_Line_Rect1_Tri1.equalsUndirected(cLine_Rect1_Tri1_ContactPoints));
    assertTrue(cExpected_Line_Tri1_Rect1.equalsUndirected(cLine_Tri1_Rect1_ContactPoints));
    assertTrue(cExpected_Rect1_Tri1_Trans.equals(cRect1_Tri1_MinTransVec));
    assertTrue(cExpected_Tri1_Rect1_Trans.equals(cTri1_Rect1_MinTransVec));
  }

  @Test
  public void closestPointOnEdge() {
    Polygon cRect1 = new Rectangle(new Vector2D(0,2),
                     new Vector2D(2,0)),
            cRect2 = new Rectangle(new Vector2D(1.5,3),
                     new Vector2D(3,1.75)),
            cTri1 = new Polygon(new Vector2D[] {new Vector2D(0.75,1),
                    new Vector2D(-1,2),
                    new Vector2D(-1,0.5)}),
            cTri2 = new Polygon(new Vector2D[] {new Vector2D(0,0),
                    new Vector2D(1,0),
                    new Vector2D(0,1)});
    //points to check
    Vector2D cRect1_LeftCorner = new Vector2D(0,2),
             cRect1_RightCorner = new Vector2D(3,3),
             cRect1_BottomMiddle = new Vector2D(1,.2),
             cRect2_RightEdge = new Vector2D(3.4,2),
             cRect2_LeftEdge = new Vector2D(1.5,2),
             cRect2_TopEdge = new Vector2D(2,4),
             cRect2_BottomEdge = new Vector2D(2.3,1.5);
             //cTri2_MiddleTopRightEdge = new Vector2D();
    //expected results
    Vector2D cRect1_LeftCorner_R = new Vector2D(0,2),
             cRect1_RightCorner_R = new Vector2D(2,2),
             cRect1_BottomMiddle_R = new Vector2D(1,0),
             cRect2_RightEdge_R = new Vector2D(3,2),
             cRect2_LeftEdge_R = new Vector2D(1.5,2),
             cRect2_TopEdge_R = new Vector2D(2,3),
             cRect2_BottomEdge_R = new Vector2D(2.3,1.75);

    assertTrue(cRect1.closestPointOnEdge(cRect1_LeftCorner).equals(cRect1_LeftCorner_R));
    assertTrue(cRect1.closestPointOnEdge(cRect1_RightCorner).equals(cRect1_RightCorner_R));
    assertTrue(cRect1.closestPointOnEdge(cRect1_BottomMiddle).equals(cRect1_BottomMiddle_R ));
    assertTrue(cRect2.closestPointOnEdge(cRect2_RightEdge).equals(cRect2_RightEdge_R));
    assertTrue(cRect2.closestPointOnEdge(cRect2_LeftEdge).equals(cRect2_LeftEdge_R));
    assertTrue(cRect2.closestPointOnEdge(cRect2_TopEdge).equals(cRect2_TopEdge_R));
    assertTrue(cRect2.closestPointOnEdge(cRect2_BottomEdge).equals(cRect2_BottomEdge_R));

  }

  @Test
  public void containsPoint() {
    Polygon cRect1 = new Rectangle(new Vector2D(0,2),
                                   new Vector2D(2,0)),
            cTri1 = new Polygon(new Vector2D[] {new Vector2D(0.75,1),
                                                new Vector2D(-1,2),
                                                new Vector2D(-1,0)});

    Vector2D cRect1_Center = new Vector2D(1,1),
             cRect1_TopEdge = new Vector2D(1,1.99999),//edges not included
             cRect1_RightOfCenter = new Vector2D(1.5,1.75),
             cTri1_Inside = new Vector2D(0,1),
             cTri1_RightCorner = new Vector2D(0.74,1),//edges not included
             cOutside1 = new Vector2D(5,5),
             cOutside2 = new Vector2D(-0.1,0.1);

    assertTrue(cRect1.containsPoint(cRect1_Center));
    assertTrue(cRect1.containsPoint(cRect1_TopEdge));
    assertTrue(cRect1.containsPoint(cRect1_RightOfCenter));
    assertTrue(cTri1.containsPoint(cTri1_Inside));
    assertTrue(cTri1.containsPoint(cTri1_RightCorner));

    assertFalse(cRect1.containsPoint(cOutside1));
    assertFalse(cRect1.containsPoint(cOutside2));
    assertFalse(cTri1.containsPoint(cOutside1));
    assertFalse(cTri1.containsPoint(cOutside2));

  }
}