package ConvexPolygon;

import Vectors.Vector2D;

/**
 * Implements class to represent a contact manifold for two polygons
 *
 * @author  Dannie Wright
 * @since   7/2/2018
 */

public class ContactManifold
{
  public Vector2D mcaContactManifoldA[],
                  mcNormalA,
                  mcaContactManifoldB[],
                  mcNormalB;
  public MinTransVec mcMinTransVec;

  /**
   * Initializes class to given values
   *
   * @param caContactManifoldA - contact points for first object
   * @param cNormalA           - normal to the surface at the contact points
   * @param caContactManifoldB - contact points for second object
   * @param cNormalB           - normal to the surface at the contact points
   */
  public ContactManifold (Vector2D caContactManifoldA[], Vector2D cNormalA,
                          Vector2D caContactManifoldB[], Vector2D cNormalB,
                          MinTransVec cMinTransVec)
  {
    mcaContactManifoldA = caContactManifoldA;
    mcNormalA = cNormalA;
    mcaContactManifoldB = caContactManifoldB;
    mcNormalB = cNormalB;
    mcMinTransVec = cMinTransVec;
  }

  public void swap ()
  {
    Vector2D caContactManifoldA[] = mcaContactManifoldA,
             cNormalA = mcNormalA;

    mcaContactManifoldA = mcaContactManifoldB;
    mcNormalA = mcNormalB;
    mcaContactManifoldB = caContactManifoldA;
    mcNormalB = cNormalA;
  }

  public ContactManifold getSwap ()
  {
    return new ContactManifold (mcaContactManifoldB, mcNormalB,
                                mcaContactManifoldA, mcNormalA,
                                mcMinTransVec);
  }

}
