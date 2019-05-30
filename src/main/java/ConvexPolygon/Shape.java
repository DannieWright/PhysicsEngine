package ConvexPolygon;

import AABBTree.AABB;
import Vectors.Line;
import Vectors.Line2D;
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
    mcCenter = new Vector2D(cCenter);
  }

  Shape (double centerX, double centerY) {
    mcCenter = new Vector2D(centerX, centerY);
  }


  @Deprecated
  abstract Vector2D[] getNormals ();

  @Deprecated
  abstract protected ContactManifoldBasic contactManifold (Shape cShapeA,
                                                           MinTransVec cMinTransVec);
  //rotates ccw
  abstract public void rotate (double angle);
  abstract public AABB getAABB ();
  abstract Line projection (Vector2D cAxis);
  abstract public boolean equals (Shape cShape);


  /**
   * Returns the minimum translational vector's axis between this shape and
   * the given shape, where the vector is along potentially mirrored to the
   * actual minimum translational vector and has the same magnitude
   *
   * @param cOther - other shape being compared
   *
   * @return - minimum translational vector's axis and magnitude between
   * this shape and the given shape if the shapes overlap, else null
   */
  abstract protected Vector2D overlapVector (Shape cOther);

  /**
   * Returns the points on this shape that are cover the most area across
   * the given axis. That is, if this shape were to be projected onto the
   * given axis, the returned Line2D contains the points on this shape that
   * result in the that projection. The resulting Line2D points in direction
   * of axis
   *
   * @param cAxis - axis covering
   *
   * @return - points on this shape that would result in the greatest
   * projection across the given axis
   */
  abstract protected Line2D boundingPoints (Vector2D cAxis);

  /**
   * Returns the edge that has the normal the most in the direction of
   * the given axis
   *
   * @param cAxis - normal axis being compared to the normals of this
   *              shapes edges
   *
   * @return - the edge whose normal is the most in the direction of the
   * given axis
   */
  abstract protected Line2D bestEdge (Vector2D cAxis);

  /**
   * Returns the closest point on this shape's edges to the given point
   *
   * @param cPoint - point being compared to
   *
   * @return - point on the surface of this shape that is the closest to
   * the given point
   */
  abstract public Vector2D closestPointOnEdge (Vector2D cPoint);

  /**
   * Returns whether the given point is contained in this shape (edges not
   * necessarily included)
   *
   * @param cPoint - point being checked if it is in this shape
   *
   * @return - true if the point is contained in this shape, else false
   */
  abstract public boolean containsPoint (Vector2D cPoint);

  /**
   * Returns the closest point in this shape (with this shapes edges being
   * inclusively considered as apart of this shape)
   *
   * @param cPoint - point being compared to
   *
   * @return - point on the surface of this shape or in this shape that
   * is the closest to the given point
   */
  public Vector2D closestPointInShape (Vector2D cPoint) {
    //if this contains the point: just return that point
    if (this.containsPoint(cPoint)) {
      return new Vector2D(cPoint);
    }
    //else point is outside this shape, so return point on edge closest to
    //the point
    return this.closestPointOnEdge(cPoint);
  }

  /**
   * Generates the contact manifold between overlapping shapes
   *
   * @param cOther - other shape being compared to overlapping this shape
   *
   * @return - contact manifold between these shapes if shapes overlap,
   * else returns null
   */
  public ContactManifold contactManifold (Shape cOther) {
    Vector2D cThisMinTransVec = this.overlapVector(cOther),
             cOtherMinTransVec = cOther.overlapVector(this);

    //if shapes do not overlap, return null
    if (null == cThisMinTransVec || null == cOtherMinTransVec) {
      return null;
    }

    //if this shapes min separation distance is less than other's:
    //use this min trans vec to generate contact manifold
    if (cThisMinTransVec.getMagnitudeSqrd()
      < cOtherMinTransVec.getMagnitudeSqrd()) {
      return this.contactEdges(cOther, cThisMinTransVec);
    }
    //else use other's trans vec to generate contact manifold
    else {
      return cOther.contactEdges(this, cOtherMinTransVec);
    }
  }

  /**
   * Determines where the shapes contact each other given the minimum
   * translational vector for this shape
   *
   * @param cOther - other shape being checked for contact
   * @param cMinTransVec - minimum translational vector for this
   *
   * @return the contact information representing where this polygon is
   * in contact with cOther. null if there is no collision between the
   * shapes
   */
  protected ContactManifold contactEdges (Shape cOther, Vector2D cMinTransVec) {

    //determine if min trans vec points towards middle or not
    { //reorient min trans vec if needed
      Line2D cBoundPtsThis = this.boundingPoints(cMinTransVec),
             cBoundPtsOther = cOther.boundingPoints(cMinTransVec);
      Vector2D cMinThis = cBoundPtsThis.getStart(),
               cMaxThis = cBoundPtsThis.getEnd(),
               cMinOther = cBoundPtsOther.getStart(),
               cMaxOther = cBoundPtsOther.getEnd();

      //determine min direction shapes need to move to separate
      if (Math.abs(cMaxThis.getSubtract(cMinOther).dot(cMinTransVec))
        < Math.abs(cMaxOther.getSubtract(cMinThis).dot(cMinTransVec))) {
        //if min trans vec points into this shape: it should point out,
        //so mirror it
        if (mcCenter.getSubtract(cMaxThis).dot(cMinTransVec) > 0) {
          cMinTransVec.mirror();
        }
      }
      else {
        if (mcCenter.getSubtract(cMinThis).dot(cMinTransVec) > 0) {
          cMinTransVec.mirror();
        }
      }
    }

    //get min trans vec for other
    Vector2D cMinTransVecOther = cMinTransVec.getMirror();
    //get the best edges for both shapes
    Line2D cBestEdgeThis = this.bestEdge(cMinTransVec),
           cBestEdgeOther = cOther.bestEdge(cMinTransVecOther),
           cContactThis,
           cContactOther;

    //if the best edges are parallel: then the shapes have a contact surface
    //(a line over which they make contact)
    if (cBestEdgeThis.parallel(cBestEdgeOther)) {
      cContactThis = cBestEdgeThis.overlap(cBestEdgeOther);
      cContactOther = cBestEdgeOther.overlap(cBestEdgeThis);
    }
    //else the shapes only contact at a single point
    else {
      //if contact best edge for other is single point: use that point
      if (cBestEdgeOther.zeroMagnitudeSqrd()) {
        cContactOther = cBestEdgeOther;
        cContactThis = (new Line2D(cContactOther)).shift(cMinTransVec);
      }
      //if contact best edge for this is single point: use that point
      else if (cBestEdgeThis.zeroMagnitudeSqrd()) {
        cContactThis = cBestEdgeThis;
        cContactOther = (new Line2D(cContactThis)).shift (cMinTransVecOther);
      }
      //if best edges aren't parallel && neither are single points: then
      //the vertex on the best edge that is the most in the same direction as
      //the min trans vec is the best vertex
      else {
        Vector2D cVertex1,// = cBestEdgeThis.getStart(),
                 cVertex2,// = cBestEdgeThis.getEnd(),
                 cBestVertex;
        double dot1,
               dot2,
               product;
        boolean bUsedThisEdge;

        //if this shape's best edge is not perpendicular to the
        //min trans vec: then one vertex is deeper than the other
        if (!cBestEdgeThis.perpendicular(cMinTransVec)) {
          cVertex1 = cBestEdgeThis.getStart();
          cVertex2 = cBestEdgeThis.getEnd();
          dot1 = cVertex1.getSubtract(mcCenter).dot(cMinTransVec);
          dot2 = cVertex2.getSubtract(mcCenter).dot(cMinTransVec);
          bUsedThisEdge = true;
        }
        //else cOther's best edge must have a deepest vertex
        else {
          cVertex1 = cBestEdgeOther.getStart();
          cVertex2 = cBestEdgeOther.getEnd();
          dot1 = cVertex1.getSubtract(cOther.mcCenter).dot(cMinTransVecOther);
          dot2 = cVertex2.getSubtract(cOther.mcCenter).dot(cMinTransVecOther);
          bUsedThisEdge = false;
        }

        product = dot1 * dot2;

        //is one vertex in the direction of the min trans vec, and the other
        //is in the opposite direction (or one is zero)
        if (product <= 0) {
          //pick the vertex in the direction of the min trans vec
          cBestVertex = dot1 > dot2 ? cVertex1 : cVertex2;
        }
        //both the relative vertices are going in the same direction as the
        //min trans vec or both in the opposite direction
        else {
          //same direction
          if (dot1 > 0) {
            cBestVertex = dot1 > dot2 ? cVertex1 : cVertex2;
          }
          //opposite direction
          else {
            cBestVertex = dot1 < dot2 ? cVertex1 : cVertex2;
          }
        }

        //set the contact points for each shape
        if (bUsedThisEdge) {
          cContactThis = new Line2D(cBestVertex, cBestVertex);
          cContactOther = (new Line2D(cContactThis)).shift(cMinTransVecOther);
        }
        else {
          cContactOther = new Line2D(cBestVertex, cBestVertex);
          cContactThis = (new Line2D(cContactOther)).shift(cMinTransVec);
        }

      }
    }

    //generate manifold
    ContactManifold cManifold = new ContactManifold();
    cManifold.insert(this, new ContactEdge(cContactThis, cMinTransVec));
    cManifold.insert(cOther, new ContactEdge(cContactOther, cMinTransVecOther));

    return cManifold;
  }

  @Deprecated
  public ContactManifoldBasic contactManifold (MinTransVec cMinTransVec)
  {
    Shape cOrigin = cMinTransVec.mcOrigin,
          cRecipient = cMinTransVec.mcRecipient;

    ContactManifoldBasic cManifold = cRecipient.contactManifold (cOrigin,
                                           cMinTransVec);

    //make this shape be shape A in the contact manifold
    if (this != cOrigin)
    {
      cManifold.swap ();
    }

    return cManifold;
  }


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
  @Deprecated
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
