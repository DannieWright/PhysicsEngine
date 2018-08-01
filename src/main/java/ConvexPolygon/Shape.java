package ConvexPolygon;

import AABBTree.AABB;
import Vectors.Line;
import Vectors.Vector2D;

/**
 * Abstract class for a shape with a center point in 2D space
 *
 * @author  Dannie Wright
 * @since   7/6/2018
 */

abstract public class Shape
{
  Vector2D mcCenter;

  Shape (Vector2D cCenter)
  {
    mcCenter = cCenter;
  }

  abstract Line projection (Vector2D cAxis);
  abstract Vector2D[] getNormals ();
  abstract public boolean equals (Shape cShape);
  abstract protected ContactManifold contactManifold (Shape cShapeA,
                                                      Vector2D cMinTrans);

  public ContactManifold contactManifold (MinTransVec cMinTransVec)
  {
    Shape cOrigin = cMinTransVec.mcOrigin,
          cRecipient = cMinTransVec.mcRecipient;

    ContactManifold cManifold = cRecipient.contactManifold (cOrigin,
                                           cMinTransVec.mcMinTransVec);

    //make this shape be shape A in the contact manifold
    if (this != cOrigin)
    {
      cManifold.swap ();
    }

    return cManifold;
  }

  //rotates ccw
  abstract public void rotate (double angle);
  abstract public AABB getAABB ();

  public Vector2D getCenter ()
  {
    return new Vector2D (mcCenter);
  }

  public void offSet (double x, double y)
  {
    mcCenter.offSet (x,y);
  }

  public void set (double x, double y)
  {
    this.offSet (x - mcCenter.mX, y - mcCenter.mY);
  }

  public void set (Vector2D cNewCenter)
  {
    this.set (cNewCenter.mX, cNewCenter.mY);
  }

  public void offSet (Vector2D cOffSet)
  {
    this.offSet (cOffSet.mX, cOffSet.mY);
  }

  /**
   * Returns the minimum translational vector of two shapes that overlap.
   * Note that if both shapes have normals that would result in the same
   * min trans vec, Shape A will be the origin
   *
   * @param cShapeA - first shape being checked for overlap
   * @param cShapeB - second shape being checked for overlap
   *
   * @return - minimum translational vector class on overlap, else null if
   *           there is no overlap
   */
  static public MinTransVec overlap (Shape cShapeA, Shape cShapeB)
  {
    //scalar overlap amount
    double overlap = 0;
    //smallest vector needed to separate these two polygons
    Vector2D cMinTranslationVector = null;
    Shape cOrigin = null,
          cRecipient = null;

    //obtain the normals to each shape's edges
    Vector2D caNormalsA[] = cShapeA.getNormals (),
             caNormalsB[] = cShapeB.getNormals ();

    //in the case of a circle the only normal axis used is from the center
    //of the circle to the closest vertex on the polygon
    if (cShapeA instanceof Circle)
    {
      caNormalsA = ((Circle) cShapeA).getNormals (cShapeB);
    }
    if (cShapeB instanceof Circle)
    {
      caNormalsB = ((Circle) cShapeB).getNormals (cShapeA);
    }

    Shape caShapes[] = {cShapeA, cShapeB};
    Vector2D caShapesNormals[][] = {caNormalsA, caNormalsB};
    final int NUM_SHAPES = 2;

    //check each shapes's overlap
    for (int i = 0;  i < NUM_SHAPES; ++i)//(Vectors.Vector2D[] caNormals : caShapesNormals)
    {
      Vector2D caNormals[] = caShapesNormals[i];
      Shape cOriginTemp = caShapes[i],
            cRecipientTemp = caShapes[(i + 1) % NUM_SHAPES];

      for (Vector2D cNormal : caNormals)
      {
        Line cProjection1 = cShapeA.projection (cNormal),
             cProjection2 = cShapeB.projection (cNormal);

        //if any projections do not overlap, then the shapes do not collide
        if (!cProjection1.overlap (cProjection2))
        {
          return null;
        }
        else
        {
          double newOverlap = cProjection1.getOverlap (cProjection2);

          //if one shape is fully contained in another must add additional
          //overlap for minimum translational vector
          if (cProjection1.contains (cProjection2)
              || cProjection2.contains (cProjection1))
          {
            double diffMin = Math.abs (cProjection1.mMin - cProjection2.mMin);
            double diffMax = Math.abs (cProjection1.mMax - cProjection2.mMax);
            if (diffMin < diffMax)
            {
              newOverlap += diffMin;
            }
            else
            {
              newOverlap += diffMax;
            }
          }

          //if translation vector has not been set yet, or if new vector is
          //smaller, set new values
          if (null == cMinTranslationVector || newOverlap < overlap)
          {
            overlap = newOverlap;
            cMinTranslationVector = cNormal;
            cOrigin = cOriginTemp;
            cRecipient = cRecipientTemp;
          }
        }
      }
    }

    if (null == cMinTranslationVector)
    {
      return null;
    }


    //set translation vector to point to new object with same values,
    //set magnitude to that of depth
    cMinTranslationVector.unit ().scale (overlap);

    //vector pointing from the origin shape to the recipient shape,
    //will use this to ensure min trans vec points from origin to recipient
    Vector2D cCenterDiff = cRecipient.mcCenter.getSubtract (cOrigin.mcCenter);
    boolean bDiffXNeg = cCenterDiff.mX < 0,
            bDiffYNeg = cCenterDiff.mY < 0,
            bMinTransXNeg = cMinTranslationVector.mX < 0,
            bMinTransYNeg = cMinTranslationVector.mY < 0;

    if (bDiffXNeg != bMinTransXNeg)
    {
      cMinTranslationVector.mirrorX ();
    }
    if (bDiffYNeg != bMinTransYNeg)
    {
      cMinTranslationVector.mirrorY ();
    }


    return new MinTransVec (cMinTranslationVector, cOrigin, cRecipient);
  }
}
