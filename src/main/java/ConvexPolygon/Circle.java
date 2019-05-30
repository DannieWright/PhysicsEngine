package ConvexPolygon;

import AABBTree.AABB;
import Vectors.IVector2D;
import Vectors.Line;
import Vectors.Line2D;
import Vectors.Vector2D;


/**
 * Implements class to store points representing a circle
 *
 * @author  Dannie Wright
 * @since   7/5/2018
 */

public class Circle extends Shape {

  private static final double APPROX_ZERO = 1.0E-14d;

  double mRadius;

  /**
   * Initializes class to given values
   *
   * @param cCenter - position of circle's center
   * @param radius - circles radius
   */
  public Circle (Vector2D cCenter, double radius)
  {
    super (cCenter);
    mRadius = radius;

    //TODO handle error
    if (0 >= radius)
    {

    }
  }

  /**
   * Initializes class to given values
   *
   * @param centerX - x position of circle's center
   * @param centerY - y position of circle's center
   * @param radius - radius of circle
   */
  public Circle (double centerX, double centerY, double radius)
  {
    super (centerX, centerY);
    mRadius = radius;

    //TODO handle error
    if (0 >= radius)
    {

    }
  }

  public Circle (Circle cOther) {
    super (cOther.mcCenter);
    mRadius = cOther.mRadius;
  }

  /**
   * Returns the projection of the circle onto the given axis
   *
   * @param cAxis - axis circle is being projected onto
   *
   * @return projection of circle onto axis
   */
  Line projection (Vector2D cAxis)
  {
    double center = mcCenter.projection (cAxis);
    return new Line (center - mRadius, center + mRadius);
  }

  /**
   * Since circle's have an infinite number of normals null is returned
   *
   * @return null
   */
  Vector2D[] getNormals ()
  {
    return null;
  }

  /**
   * Returns the vector pointing from this circle to the closest vertex/center
   * of the given shape
   *
   * @param cShape - shape being compared with this circle
   *
   * @return normal to circle's surface that points to a polygon's closest
   *         vertex, or to a circle's center, else null on failure
   */
  Vector2D[] getNormals (Shape cShape)
  {
    if (cShape instanceof Circle)
    {
      return new Vector2D[] {cShape.mcCenter.getSubtract (this.mcCenter)};
    }
    else if (cShape instanceof Polygon)
    {
      Polygon cPolygon = (Polygon) cShape;
      int numVertices = cPolygon.mNumVertices;
      Vector2D caVertices[] = cPolygon.mcaVertices;
      Vector2D cCenter = this.mcCenter,
               cClosestVertex = caVertices[0];
      double bestDistanceSquared = caVertices[0].distanceSquared (cCenter);

      for (int i = 1; i < numVertices; ++i)
      {
        Vector2D cVertex = caVertices[i];
        double distanceSquared = cVertex.distanceSquared (cCenter);

        if (distanceSquared < bestDistanceSquared)
        {
          //in the event the closest vertex is at the circle's center, the
          //normal returned would be (zero, zero)
          //instead return the Vector between the two sides connected to the
          //closest vertex as an approximation of the line between the circle's
          //center and other shape's closest vertex
          if (0 == distanceSquared)
          {
            Vector2D cPrev = caVertices[i - 1].getSubtract (cVertex),
                     cNext = caVertices[(i + 1) % numVertices].getSubtract (
                                cVertex);

            //want half of the angle between the two edges
            double angle = 0.5 * cPrev.angle (cNext);

            //if shape's vertex orientation is ccw then Vector between two
            //edges is cPrev rotated cw, else it is ccw
            if (IVector2D.VectorOrientation.COUNTERCLOCKWISE
                == ((Polygon) cShape).meOrientation)
            {
              angle *= -1;
            }

            cVertex = cPrev.rotate (angle);
            distanceSquared = this.mRadius * this.mRadius;
          }

          cClosestVertex = cVertex;
          bestDistanceSquared = distanceSquared;
        }
      }

      return new Vector2D[] {cClosestVertex.getSubtract (cCenter)};
    }

    return null;
  }

  /**
   * Returns the point on this circle and on the given shape where the shapes
   * may have collided based on the minimum translational vector that points
   * from the given shape to this circle
   *
   * @param cShapeA   - colliding shape that is the origin of the minimum
   *                    translational vector
   * @param cMinTransVec - minimum amount shapes would have to stop colliding as
   *                    normal to one shape's edge
   *
   * @return contact manifold for both shapes, else null on failure
   */
  public ContactManifoldBasic contactManifold (Shape cShapeA, MinTransVec cMinTransVec)
  {
    //the minimum translational vector points from shape A to shape B,
    //where this is shape B
    Vector2D cMinTrans = cMinTransVec.mcMinTransVec,
             cMinTransAxis = cMinTrans.getUnit (),
             cContactB = cMinTransAxis.getScaled (-1 * this.mRadius).add (
                          this.mcCenter),
             caContactB[] = {cContactB},
             caContactA[] = {cContactB.getAdd (cMinTrans)},
             cNormalA = new Vector2D (cMinTrans),
             cNormalB = cMinTrans.getMirror ();

    return new ContactManifoldBasic(caContactA, cNormalA, caContactB, cNormalB,
                                cMinTransVec);
  }

  /**
   * Rotates the circle counterclockwise
   *
   * @param angle - angle of rotation
   */
  public void rotate (double angle) {
  }

  public AABB getAABB ()
  {
    double x = mcCenter.mX,
           y = mcCenter.mY,
           radius = mRadius;

    return new AABB (x - radius, x + radius, y - radius,
                     y + radius);
  }

  @Override
  public boolean equals (Shape cShape)
  {
    if (cShape instanceof Circle)
    {
      Circle cCircle = (Circle) cShape;

      return this.mcCenter.equals (cCircle.mcCenter)
            && Math.abs (this.mRadius - cCircle.mRadius) < Circle.APPROX_ZERO;
    }
    return false;
  }


  /**
   * Returns the minimum translational vector's axis between this shape and
   * the given shape, where the vector is potentially mirrored to the
   * actual minimum translational vector but has the same magnitude
   *
   * @param cOther - other shape being compared
   *
   * @return - minimum translational vector's axis and magnitude between
   * this shape and the given shape if the shapes overlap, else null
   */
  protected Vector2D overlapVector (Shape cOther) {
    Vector2D cCenter = new Vector2D(mcCenter),
             cDeepestPoint = cOther.closestPointInShape(mcCenter),
             cDiff = cDeepestPoint.getSubtract(cCenter);

    //if point is outside radius of this circle: shape's don't overlap
    if (cDiff.getMagnitudeSqrd() > mRadius * mRadius) {
      return null;
    }

    //return the vector in between the deepestPoint in the other shape
    // and the edge of this circle that is closest to the deepestPoint
    return cDiff.getUnit().scale(mRadius).subtract(cDiff);
  }

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
  protected Line2D boundingPoints (Vector2D cAxis) {
    //normalize the axis, then scale it by the length of the radius
    Vector2D cAxisMag = cAxis.getUnit().scale(mRadius);
    //return the Line2D where it points along cAxis (so dot product would be
    //pos)
    return new Line2D (mcCenter.getSubtract(cAxisMag),
                       mcCenter.getAdd(cAxisMag));
  }

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
  protected Line2D bestEdge (Vector2D cAxis) {
    //normalize the axis, then scale it by the length of the radius
    Vector2D cAxisMag = cAxis.getUnit().scale(mRadius),
             cEdgePoint = mcCenter.getAdd(cAxisMag);
    //return the point most in the direction of the axis as an edge
    return new Line2D (cEdgePoint, cEdgePoint);
  }

  /**
   * Returns the closest point on this shape's edges to the given point
   *
   * @param cPoint - point being compared to
   *
   * @return - point on the surface of this shape that is the closest to
   * the given point
   */
  public Vector2D closestPointOnEdge (Vector2D cPoint) {
    return cPoint.getSubtract(mcCenter).unit().scale(mRadius); //maybe problems if at mcCenter
  }


  /**
   * Returns whether the given point is contained in this shape
   *
   * @param cPoint - point being checked if it is in this shape
   *
   * @return - true if the point is contained in this shape, else false
   */
  public boolean containsPoint (Vector2D cPoint) {
    return mcCenter.distanceSquared(cPoint) <= mRadius * mRadius;
  }
}
