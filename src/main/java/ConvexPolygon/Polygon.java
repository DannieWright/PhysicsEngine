package ConvexPolygon;

import AABBTree.AABB;
import Vectors.IVector2D;
import Vectors.Line;
import Vectors.Line2D;
import Vectors.Vector2D;


/**
 * Implements class to store points representing a simple convex polygon
 *
 * @author  Dannie Wright
 * @since   6/15/2018
 */

public class Polygon extends Shape
{

  Vector2D mcaVertices[];
  int mNumVertices;
  IVector2D.VectorOrientation meOrientation,
                              meNormalOrientation;

  /**
   * Initializes class with given values
   *
   * @param caVertices - vertices of non-overlapping convex polygon where the
   *                     vertices are oriented with specified orientation
   *
   */
  public Polygon (Vector2D[] caVertices)
  {
    super (centroid (caVertices));

    int numVertices = caVertices.length;
    mcaVertices = new Vector2D[numVertices];
    for (int i = 0; i < numVertices; ++i) {
      mcaVertices[i] = new Vector2D(caVertices[i]);
    }

    mNumVertices = caVertices.length;
    this.setOrientation();
  }

  /**
   * Initializes class with given values
   *
   * @param caVertices - vertices of non-overlapping convex polygon where the
   *                     vertices are oriented with specified orientation
   * @param bUseGiven - true to use the given array of vertices, false to
 *                       prevent leaking by copying the vectors to a new
   *                     array.
   *
   */
  public Polygon (Vector2D[] caVertices, boolean bUseGiven)
  {
    super (centroid (caVertices));

    if (bUseGiven) {
      mcaVertices = caVertices;
    }
    else {
      int numVertices = caVertices.length;
      mcaVertices = new Vector2D[numVertices];
      for (int i = 0; i < numVertices; ++i) {
        mcaVertices[i] = new Vector2D(caVertices[i]);
      }
    }

    mNumVertices = caVertices.length;
    this.setOrientation();
  }

  /**
   * Initializes class with given values. This ctor is mean to be used by
   * special case subclasses to improve performance, an incorrect orientation
   * can lead to unexpected errors.
   *
   * @param caVertices - vertices of non-overlapping convex polygon where the
   *                     vertices are oriented with specified orientation
   * @param bUseGiven - true to use the given array of vertices, false to
   *                     prevent leaking by copying the vectors to a new
   *                     array.
   * @param eOri - orientation of vertices.
   *
   */
  Polygon (Vector2D[] caVertices, boolean bUseGiven,
           IVector2D.VectorOrientation eOri)
  {
    super (centroid (caVertices));

    if (bUseGiven) {
      mcaVertices = caVertices;
    }
    else {
      int numVertices = caVertices.length;
      mcaVertices = new Vector2D[numVertices];
      for (int i = 0; i < numVertices; ++i) {
        mcaVertices[i] = new Vector2D(caVertices[i]);
      }
    }

    mNumVertices = caVertices.length;
    meOrientation = eOri;

    //normal vectors must point away from center, so calculating the
    //normals uses the opposite orientation of the polygon
    meNormalOrientation = IVector2D.VectorOrientation.CLOCKWISE == meOrientation
                          ? IVector2D.VectorOrientation.COUNTERCLOCKWISE
                          : IVector2D.VectorOrientation.CLOCKWISE;
  }

  public Polygon (Polygon cOther)
  {
    super (cOther.mcCenter);

    Vector2D[] caVertices = cOther.mcaVertices;
    int numVertices = caVertices.length;

    //set vertices equal to other's vertices
    mcaVertices = new Vector2D[cOther.mcaVertices.length];
    for (int i = 0; i < numVertices; ++i) {
      mcaVertices[i] = new Vector2D (caVertices[i]);
    }

    mNumVertices = numVertices;
    this.setOrientation();
  }

  private void setOrientation () {
    meOrientation = Polygon.orientation(mcaVertices);

    //normal vectors must point away from center, so calculating the
    //normals uses the opposite orientation of the polygon
    meNormalOrientation = IVector2D.VectorOrientation.CLOCKWISE == meOrientation
      ? IVector2D.VectorOrientation.COUNTERCLOCKWISE
      : IVector2D.VectorOrientation.CLOCKWISE;
  }

  public static IVector2D.VectorOrientation orientation (Vector2D[] caVertices) {

    int numVertices = caVertices.length;
    Vector2D cEdge = caVertices[0].getSubtract(caVertices[numVertices - 1]);
    Vector2D.VectorDirection eFirst = cEdge.getDirectionSub(),
                             eSecond = eFirst;

    for (int i = 0; i < numVertices - 1; ++i) {
      eSecond = cEdge.set (caVertices[i + 1]).subtract(caVertices[i]).getDirectionSub();
      if (eSecond != eFirst) {
        break;
      }
    }

    //checks if CW, defaults to CCW if something went wrong
    return eSecond == eFirst.second(IVector2D.VectorOrientation.CLOCKWISE) ?
          IVector2D.VectorOrientation.CLOCKWISE
          : IVector2D.VectorOrientation.COUNTERCLOCKWISE;
  }

  /**
   * Returns the center of mass for the polygon
   *
   * @param caVertices -
   * @return
   */
  public static Vector2D centroid (Vector2D[] caVertices)
  {
    if (1 == caVertices.length)
    {
      return new Vector2D (caVertices[0]);
    }

    double sideArea = 0,
           centerX = 0,
           centerY = 0;
    int numVertices = caVertices.length,
        lessVertices = numVertices - 1;

    for (int i = 0; i < numVertices; ++i)
    {
      Vector2D cPoint1 = caVertices[i],
               cPoint2 = caVertices[i < lessVertices ? i + 1 : 0];
      double x1 = cPoint1.mX,
             x2 = cPoint2.mX,
             y1 = cPoint1.mY,
             y2 = cPoint2.mY;

      double sideAreaCoeff = x1 * y2 - x2 * y1;
      centerX += (x1 + x2) * sideAreaCoeff;
      centerY += (y1 + y2) * sideAreaCoeff;
      sideArea += sideAreaCoeff;
    }

    double coeff = 1 / (sideArea * 3);
    centerX *= coeff;
    centerY *= coeff;

    return new Vector2D (centerX, centerY);
  }

  /**
   *
   * @param cAxis - axis projecting this line onto
   * @return
   */
    Line projection (Vector2D cAxis)
    {
      cAxis = cAxis.getUnit ();
      double min = mcaVertices[0].dot (cAxis),
             max = min;
      int numVertices = mNumVertices;

      for (int i = 1; i < numVertices; ++i)
      {
        double projection = mcaVertices[i].dot (cAxis);

        if (min > projection)
        {
          min = projection;
        }
        else if (max < projection)
        {
          max = projection;
        }
      }

      return new Line (min, max);
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
    Vector2D cAxis_Unit = cAxis.getUnit ();
    double min = mcaVertices[0].dot (cAxis_Unit),
           max = min,
           projection;
    int numVertices = mNumVertices,
        minIndex = 0,
        maxIndex = 0;

    for (int i = 1; i < numVertices; ++i)
    {
      projection = mcaVertices[i].dot (cAxis_Unit);

      if (min > projection) {
        min = projection;
        minIndex = i;
      }
      else if (max < projection) {
        max = projection;
        maxIndex = i;
      }
    }

    return new Line2D(mcaVertices[minIndex], mcaVertices[maxIndex]);
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
  @Override
  protected Line2D bestEdge(Vector2D cAxis) {

    Vector2D cAxisNorm = cAxis.getUnit(),
             cNormal = new Vector2D(0,0);
    Line2D[] caEdges = this.getEdges();
    Line2D cBestEdge = caEdges[0];
    double min = Math.abs(cBestEdge.dot(cAxisNorm)),
           temp;

    for (Line2D cEdge : caEdges) {
      cNormal = cEdge.getNormal(cNormal, meNormalOrientation);
      temp = cNormal.dot(cAxisNorm);

      //check if this edge's normal is more in the direction of cAxis then
      //the prev selected edge. dot and cross products take care of cases
      //of parallel axis
      if (temp >= min && temp >= 0 && cNormal.cross(cAxisNorm) >= 0) {
        cBestEdge = cEdge;
        min = temp;
      }
    }

    return cBestEdge;
  }

  //This function has been deprecated, but should stay added to the Polygon
  //class interface, it will just no longer be inherited from the Shape interface.
  //New scope should be protected
  Vector2D[] getNormals () {
    int numVertices = mcaVertices.length;
    Vector2D[] caNormals = new Vector2D[numVertices];

    for (int i = 0; i < numVertices - 1; ++i) {
      caNormals[i] = mcaVertices[i].getSubtract(mcaVertices[i + 1]);
      caNormals[i] = caNormals[i].getNormal(caNormals[i],meNormalOrientation);
    }
    //get wrap around
    --numVertices;
    caNormals[numVertices] = mcaVertices[numVertices].getSubtract(mcaVertices[0]);
    caNormals[numVertices].getNormal(caNormals[numVertices],meNormalOrientation);

    return caNormals;
    }

  /**
   *
   * @return
   */
  private Line2D[] getEdges () {
    int numVertices = mcaVertices.length;
    Line2D[] caEdges = new Line2D[numVertices];

    for (int i = 0; i < numVertices - 1; ++i) {
      caEdges[i] = new Line2D (mcaVertices[i], mcaVertices[i + 1]);
    }
    //get wrap around
    caEdges[numVertices - 1] = new Line2D (mcaVertices[numVertices - 1],
                                           mcaVertices[0]);

    return caEdges;
  }

  /**
   *
   * @param cMinTransAxis
   * @return
   */
  private int deepestVertex (Vector2D cMinTransAxis) //unit of mintransvec
  {
    Vector2D[] caVertices = mcaVertices;
    int numVertices = mNumVertices,
        bestVertexIndex = 0;
    double maxProjection = caVertices[0].dot (cMinTransAxis);

    for (int i = 1; i < numVertices; ++i)
    {
      double projection = caVertices[i].dot (cMinTransAxis);

      if (projection > maxProjection)
      {
        maxProjection = projection;
        bestVertexIndex = i;
      }
    }

    return bestVertexIndex;
  }

  //global positions
  protected Vector2D[] getVertices () {
    Vector2D[] caVertices = new Vector2D[mcaVertices.length];
    for (int i = 0; i < mcaVertices.length; i++) {
      caVertices[i] = new Vector2D(mcaVertices[i]);
    }
    return caVertices;
  }

  protected Vector2D[] getLocalVertices () {
    Vector2D[] caVertices = new Vector2D[mcaVertices.length];
    Vector2D cCenter = mcCenter;
    for (int i = 0; i < mcaVertices.length; i++) {
      caVertices[i] = new Vector2D(mcaVertices[i]);
      caVertices[i].subtract(cCenter);
    }
    return caVertices;
  }


  /**
   * Returns the point(s) on this polygon and on the given shape where the
   * shapes may have collided based on the minimum translational vector that
   * points from the given shape to this polygon
   *
   * @param cShapeA   - colliding shape that is the origin of the minimum
   *                    translational vector
   * @param cMinTransVec - minimum amount shapes would have to stop colliding as
   *                    normal to one shape's edge
   *
   * @return contact manifold for both shapes
   */
    public ContactManifoldBasic contactManifold (Shape cShapeA,
                                                 MinTransVec cMinTransVec)
    {
      Vector2D cMinTrans = cMinTransVec.mcMinTransVec;
      
      if (cShapeA instanceof Polygon)
      {
        Polygon cPolygonA = (Polygon) cShapeA;
        Vector2D cMinTransAxis = cMinTrans.getUnit (),
                 cMinTransAxisNeg = cMinTransAxis.getMirror ();

        int bestVertexA = cPolygonA.deepestVertex (cMinTransAxis),
            bestVertexB = this.deepestVertex (cMinTransAxisNeg);

        Line2D cBestEdgeA,
               cBestEdgeB;

        //determine best edges of both polygons
        cBestEdgeA = cPolygonA.bestEdge (cMinTransAxis, bestVertexA);
        cBestEdgeB = this.bestEdge (cMinTransAxisNeg, bestVertexB);

        Vector2D[] caContactA,
                   caContactB;
        Vector2D cNormalA,
                 cNormalB;

        //if polygon B's best edge is parallel to polygon A's best edge
        //then the contact manifold is an edge
        if (cBestEdgeA.parallel (cBestEdgeB))
        {
          Line2D cOverlapA = cBestEdgeA.overlap (cBestEdgeB),
                 cOverlapB = cBestEdgeB.overlap (cBestEdgeA);

          caContactA = cOverlapA.toArray();
          caContactB = cOverlapB.toArray();
        }
        //if polygon B's best edge is not parallel to polygon A's best edge
        //then assume polygon A's best vertex was contact point, and extrapolate
        //back by the min trans vec for contact point on polygon B
        else
        {
          Vector2D cBestVertexA = cPolygonA.mcaVertices[bestVertexA];
          caContactA = new Vector2D[] {cBestVertexA};
          caContactB = new Vector2D[] {cBestVertexA.getSubtract (cMinTrans)};
        }

        //determine the normals to the best edges
        cNormalA = cBestEdgeA.getNormal (cPolygonA.meNormalOrientation);
        cNormalB = cBestEdgeB.getNormal (this.meNormalOrientation);

        //give normal vectors magnitude of min trans vec
        double transMag = cMinTrans.getMagnitude ();
        cNormalA.unit ();
        cNormalA.scale (transMag);
        cNormalB.unit ();
        cNormalB.scale (transMag);

        return new ContactManifoldBasic(caContactA, cNormalA, caContactB, cNormalB,
                                    cMinTransVec);
      }
      else if (cShapeA instanceof Circle)
      {
        Circle cCircle = (Circle) cShapeA;
        Vector2D cMinTransAxis = cMinTrans.getUnit (),
                 cContactCircle = cMinTransAxis.getScaled (
                                  cCircle.mRadius).getAdd (cCircle.mcCenter),
                 cNormalPolygon = cMinTrans.getMirror (),
                 cNormalCircle = new Vector2D (cMinTrans);
        Vector2D[] caContactCircle = {cContactCircle},
                   caContactPolygon = {cContactCircle.getSubtract (cMinTrans)};

        return new ContactManifoldBasic(caContactCircle, cNormalCircle,
                                    caContactPolygon, cNormalPolygon,
                                    cMinTransVec);
      }

      return null;
    }

  /**
   *
   * @param cMinTransAxis
   * @param bestVertexIndex
   * @return
   */
    private Line2D bestEdge (Vector2D cMinTransAxis, int bestVertexIndex)
    {
      Vector2D[] caVertices = mcaVertices;
      int numVertices = mNumVertices;
      int prevIndex = 0 <= bestVertexIndex - 1 ? bestVertexIndex - 1
                                                : numVertices - 1,
          nextIndex = (bestVertexIndex + 1) % numVertices;

      Vector2D cBestVertex = caVertices[bestVertexIndex],
               cPrevVertex = caVertices[prevIndex],
               cNextVertex = caVertices[nextIndex];

      //grab two edges connected to best vertex
      Line2D cEdge1 = new Line2D (cPrevVertex, cBestVertex),
             cEdge2 = new Line2D (cNextVertex, cBestVertex),
             cEdge2Oriented = new Line2D (cBestVertex, cNextVertex);

      //return edge most perpendicular to the mintransvec
      return cEdge1.dot (cMinTransAxis) < cEdge2.dot (cMinTransAxis)
              ? cEdge1 : cEdge2Oriented;
    }

    public void rotate (double angle)
    {
      Vector2D[] caVertices = mcaVertices;
      Vector2D cCenter = mcCenter;
      int numVertices = this.mNumVertices;

      for (int i = 0; i < numVertices; ++i)
      {
        caVertices[i].subtract (cCenter);
        caVertices[i].rotate (angle);
        caVertices[i].add (cCenter);
      }
    }

  public void offSet (double x, double y)
  {
    super.offSet (x,y);
    int numVertices = mNumVertices;

    for (int i = 0; i < numVertices; ++i) {
      mcaVertices[i].offSet (x,y);
    }
  }

  public AABB getAABB ()
  {
    Vector2D[] caVertices = mcaVertices;
    Vector2D cVertex = caVertices[0];
    int numVertices = mNumVertices;
    double x = cVertex.mX,
           y = cVertex.mY,
           minX = x,
           minY = y,
           maxX = x,
           maxY = y;

    for (int i = 1; i < numVertices; ++i) {
      cVertex = caVertices[i];
      x = cVertex.mX;
      y = cVertex.mY;

      if (x < minX) {
        minX = x;
      }
      else if (x > maxX) {
        maxX = x;
      }

      if (y < minY) {
        minY = y;
      }
      else if (y > maxY) {
        maxY = y;
      }
    }

    return new AABB (minX, maxX, minY, maxY);
  }

  //only works if has same orientation and vertices are at the same index
  @Override
  public boolean equals (Shape cShape)
  {
    if (cShape instanceof Polygon)
    {
      Polygon cPolygon = (Polygon) cShape;
      int numVertices = this.mNumVertices;

      if (numVertices == cPolygon.mNumVertices)
      {
        Vector2D caVertices[] = this.mcaVertices,
                 caVerticesPoly[] = cPolygon.mcaVertices;

        for (int i = 0; i < numVertices; ++i)
        {
          if (!caVertices[i].equals (caVerticesPoly[i]))
          {
            return false;
          }
        }

        return true;
      }
    }

    return false;
  }


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
  protected Vector2D overlapVector (Shape cOther) {
    //get the edges for this shape
    Vector2D []caNormals = this.getNormals();

    //normal vector for which the min amount of overlap occurs
    Vector2D cMinNormal = null;

    //projections of shapes onto the given normal vector
    Line cProjection1,
         cProjection2;
    //scalar overlap amount of the projections for a given normal vector
    double overlap = 0,
           newOverlap;

    //for all edges' normals: find the overlap between the shapes on that axis
    for (Vector2D cNormal : caNormals) {

      //project both shapes onto the vector, so can check for overlaps
      cProjection1 = this.projection (cNormal);
      cProjection2 = cOther.projection (cNormal);

      //if any projections do not overlap, then the shapes do not collide
      if (!cProjection1.overlap (cProjection2)) {
        return null;
      }
      else {
        //determine how much the shapes overlap using this normal
        newOverlap = cProjection1.getOverlap (cProjection2);

        //if one shape is fully contained in another then we must add
        //additional overlap for minimum translational vector
        if (cProjection1.contains (cProjection2)
          || cProjection2.contains (cProjection1)) {

          double diffMin = Math.abs (cProjection1.mMin - cProjection2.mMin);
          double diffMax = Math.abs (cProjection1.mMax - cProjection2.mMax);
          if (diffMin < diffMax) {
            newOverlap += diffMin;
          }
          else {
            newOverlap += diffMax;
          }
        }

        //if translation vector has not been set yet, or if new vector is
        //smaller, set new values
        if (newOverlap < overlap || null == cMinNormal) {
          overlap = newOverlap;
          cMinNormal = cNormal;
        }
      }

    }

    //if no overlapping occurred, shapes do not collide
    if (null == cMinNormal) {
      return null;
    }
    //return min trans axis with scale for overlap amount.
    //this is just the axis for the overlap, if this shape has any parallel
    //edges, this vector may point in the wrong direction
    return cMinNormal.unit().scale(overlap);
  }


  /**
   * Returns the closest point on this shape's edges to the given point
   *
   * @param cPoint - point being compared to
   *
   * @return - point on the surface of this shape that is the closest to
   * the given point
   */
  @Override
  public Vector2D closestPointOnEdge (Vector2D cPoint) {
    double minDist = mcaVertices[0].distanceSquared(cPoint);

    int minIndex = 0,
        nextMinIndex,
        numVertices = this.mNumVertices;

    {//find closest vertex
      double temp;

      for (int i = 1; i < numVertices; ++i) {
        temp = mcaVertices[i].distanceSquared(cPoint);
        if (temp < minDist) {
          minIndex = i;
          minDist = temp;
        }
      }
    }

    { //find next closest vertex
      int indexL = minIndex == 0 ? numVertices - 1 : minIndex - 1,
          indexR = (minIndex + 1) % numVertices;
      nextMinIndex = mcaVertices[indexL].distanceSquared(cPoint)
                   < mcaVertices[indexR].distanceSquared(cPoint)
                   ? indexL : indexR;
    }

    //grab best vertex, and the lines for a triangle around best vertex
    Vector2D cBestVertex = mcaVertices[minIndex],
             //vector from best vertex to cPoint
             cY = cPoint.getSubtract(cBestVertex),
             //vector from best vertex to next best vertex
             cX_Unit = mcaVertices[nextMinIndex].getSubtract(cBestVertex).unit();

    /*
      Best point is found on line in between BestVertex and NextBestVertex
      (inclusive).

      Notation:
      cY = (cPoint - cBestVertex)
      cX = (cNextBestVertex - cBestVertex).unit()

      We can make a triangle with cY and cX, which will either be acute or obtuse(or right)

      if obtuse/right: then best point is cBestVertex;

      else acute: then best point is between cBestVertex and cNextBestVertex,
      so project cY onto cX (although I do the opposite since cX is unit can
      save processing) to find distance from cBestVertex to cBestPointOnEdge;
      Then cBestPointOnEdge is cBestVertex plus the vector in the direction
      (cNext - cBest) with the length given by the projection of cY onto cX
     */

    //obtuse/right: return cBest
    if (0 >= cX_Unit.projection(cY)) {
      return new Vector2D(cBestVertex);
    }

    //else acute triangle: return point on line in between cBest and cNextBest
    //with length given by projection of cY onto that line
    return cX_Unit.scale(cY.dot(cX_Unit)).add(cBestVertex);
  }





  /**
   * Returns whether the given point is contained in this shape (edges not
   * necessarily included)
   *
   * @param cPoint - point being checked if it is in this shape
   *
   * @return - true if the point is contained in this shape, else false
   */
  public boolean containsPoint (Vector2D cPoint) {
    //get the edge most in this direction from center to given point
    Line2D cEdge = this.bestEdge(cPoint.getSubtract(mcCenter));

    //check if the point is contained in the triangle made from this center
    //and the best edge
    return this.triangleContainsPoint(mcCenter, cEdge.getStart(),
                                      cEdge.getEnd(), cPoint);
  }


  /**
   * Determines if given point falls inside the triangle ABC using the
   * Barycentric Technique:
   * http://blackpawn.com/texts/pointinpoly/default.html
   *
   * @param cA - one vertex of triangle
   * @param cB - one vertex of triangle
   * @param cC - one vertex of triangle
   * @param cPoint - point being checked if inside triangle
   *
   * @return - true if the point is inside the triangle
   */
  protected boolean triangleContainsPoint (Vector2D cA, Vector2D cB,
                                           Vector2D cC, Vector2D cPoint) {
    //compute vectors
    Vector2D cV0 = cC.getSubtract(cA),
             cV1 = cB.getSubtract(cA),
             cV2 = cPoint.getSubtract(cA);

    //compute dot products
    double dot00 = cV0.dot(cV0),
           dot01 = cV0.dot(cV1),
           dot02 = cV0.dot(cV2),
           dot11 = cV1.dot(cV1),
           dot12 = cV1.dot(cV2);

    //compute barycentric coordinates
    double invDenom = 1 / (dot00 * dot11 - dot01 * dot01),
           u = (dot11 * dot02 - dot01 * dot12) * invDenom,
           v = (dot00 * dot12 - dot01 * dot02) * invDenom;

    return u >= 0 && v >= 0 && (u + v) < 1;
  }


}
