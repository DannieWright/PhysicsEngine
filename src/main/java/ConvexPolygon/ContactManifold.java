package ConvexPolygon;

import Vectors.Vector2D;
import java.util.HashMap;
import java.util.Set;


public class ContactManifold {
  private HashMap<Shape,ContactEdge> mContactEdgeMap;

  private static final int SIZE = 2;

  public ContactManifold () {
    mContactEdgeMap = new HashMap<>(ContactManifold.SIZE);
  }

  public ContactManifold (ContactManifold cOther) {
    mContactEdgeMap = new HashMap<>(ContactManifold.SIZE);
    for (Shape cKey : cOther.mContactEdgeMap.keySet()) {
      mContactEdgeMap.put(cKey, new ContactEdge(
        cOther.mContactEdgeMap.get(cKey)));
    }
  }


  public void insert (Shape cShape, ContactEdge cEdge) {
    if (mContactEdgeMap.size() < ContactManifold.SIZE) {
      mContactEdgeMap.put(cShape, cEdge);
    }
  }

  public ContactEdge find (Shape cShape) {
    ContactEdge cTemp = mContactEdgeMap.get(cShape);
    if (null != cTemp) {
      return new ContactEdge(cTemp);
    }
    return null;
  }

  public boolean equals (ContactManifold cOther) {
    if (cOther.mContactEdgeMap.size() != mContactEdgeMap.size()) {
      return false;
    }

    for (Shape cShape :  mContactEdgeMap.keySet()) {
      if (!mContactEdgeMap.get(cShape).equals(
          cOther.mContactEdgeMap.get(cShape))) {
        return false;
      }
    }

    return true;
  }
}
