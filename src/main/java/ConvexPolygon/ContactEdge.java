package ConvexPolygon;

import Vectors.Line2D;
import Vectors.Vector2D;

public class ContactEdge {
  final Line2D mcEdge;
  final Vector2D mcMinTransVec;

  public ContactEdge (Line2D cEdge, Vector2D cMinTransVec) {
    mcEdge = new Line2D (cEdge);
    mcMinTransVec = new Vector2D (cMinTransVec);
  }

  public ContactEdge (ContactEdge cOther) {
    mcEdge = new Line2D(cOther.mcEdge);
    mcMinTransVec = new Vector2D(cOther.mcMinTransVec);
  }

  public Line2D getEdge () {
    return new Line2D (mcEdge);
  }

  public Vector2D getMinTransVec () {
    return new Vector2D(mcMinTransVec);
  }

  public boolean equals (ContactEdge cOther) {
    if (null == cOther) {
      return false;
    }
    return mcEdge.equalsUndirected(cOther.mcEdge)
      && mcMinTransVec.equals(cOther.mcMinTransVec);
  }
}
