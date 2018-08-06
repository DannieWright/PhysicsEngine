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
   * @param eOrientation - orientation of vertices
   */
  public Polygon (Vector2D caVertices[],
                  IVector2D.VectorOrientation eOrientation)
  {
    super (centroid (caVertices));

    //TODO handle too small number of vertices
    if (caVertices.length < 1)
    {

    }

    mcaVertices = caVertices;
    mNumVertices = caVertices.length;
    meOrientation = eOrientation;

    //normal vectors must point away from center, so calculating the
    //normals uses the opposite orientation of the polygon
    meNormalOrientation = IVector2D.VectorOrientation.CLOCKWISE == eOrientation
                          ? IVector2D.VectorOrientation.COUNTERCLOCKWISE
                          : IVector2D.VectorOrientation.CLOCKWISE;
  }

  public Polygon (Polygon cPolygon)
  {
    super (new Vector2D (cPolygon.mcCenter));

    Vector2D caVertices[] = cPolygon.mcaVertices;
    int numVertices = caVertices.length;

    //TODO handle too small number of vertices
    if (numVertices < 1)
    {

    }

    mcaVertices = new Vector2D[numVertices];
    for (int i = 0; i < numVertices; ++i)
    {
      mcaVertices[i] = new Vector2D (caVertices[i]);
    }

    mNumVertices = numVertices;
    meOrientation = cPolygon.meOrientation;
    //normal vectors must point away from center, so calculating the
    //normals uses the opposite orientation of the polygon
    meNormalOrientation = IVector2D.VectorOrientation.CLOCKWISE == meOrientation
                        ? IVector2D.VectorOrientation.COUNTERCLOCKWISE
                        : IVector2D.VectorOrientation.CLOCKWISE;
  }

  /**
   * Returns the center of mass for the polygon
   *
   * @param caVertices -
   * @return
   */
  private static Vector2D centroid (Vector2D caVertices[])
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
   *
   * @return
   */
  Vector2D[] getNormals ()
    {
      int numVertices = mcaVertices.length;
      Vector2D caNormals[] = new Vector2D[numVertices];
      Vector2D cPoint1,
               cPoint2,
               cEdge;

      for (int i = 0; i < numVertices; ++i)
      {
        cPoint1 = mcaVertices[i];
        cPoint2 = i < numVertices - 1 ? mcaVertices[i + 1] : mcaVertices[0];
        cEdge = cPoint2.getSubtract (cPoint1);
        caNormals[i] = cEdge.getNormal (meNormalOrientation);
      }

      return caNormals;
    }

  /**
   *
   * @param cMinTransAxis
   * @return
   */
    private int deepestVertex (Vector2D cMinTransAxis) //unit of mintransvec
    {
      Vector2D caVertices[] = mcaVertices;
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
    public ContactManifold contactManifold (Shape cShapeA,
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

        Vector2D caContactA[],
                 caContactB[],
                 cNormalA,
                 cNormalB;

        //if polygon B's best edge is parallel to polygon A's best edge
        //then the contact manifold is an edge
        if (cBestEdgeA.parallel (cBestEdgeB))
        {
          Line2D cOverlapA = cBestEdgeA.overlap (cBestEdgeB),
                 cOverlapB = cBestEdgeB.overlap (cBestEdgeA);

          caContactA = new Vector2D[] {cOverlapA.mcStart, cOverlapA.mcEnd};
          caContactB = new Vector2D[] {cOverlapB.mcStart, cOverlapB.mcEnd};
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

        return new ContactManifold (caContactA, cNormalA, caContactB, cNormalB,
                                    cMinTransVec);
      }
      else if (cShapeA instanceof Circle)
      {
        Circle cCircle = (Circle) cShapeA;
        Vector2D cMinTransAxis = cMinTrans.getUnit (),
                 cContactCircle = cMinTransAxis.getScaled (
                                  cCircle.mRadius).getAdd (cCircle.mcCenter),
                 caContactCircle[] = {cContactCircle},
                 caContactPolygon[] = {cContactCircle.getSubtract (cMinTrans)},
                 cNormalPolygon = cMinTrans.getMirror (),
                 cNormalCircle = new Vector2D (cMinTrans);

        return new ContactManifold (caContactCircle, cNormalCircle,
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
      Vector2D caVertices[] = mcaVertices;
      int numVertices = mNumVertices;
      int prevIndex = 0 <= bestVertexIndex - 1 ? bestVertexIndex - 1
                                                : numVertices - 1,
          nextIndex = (bestVertexIndex + 1) % numVertices;

      Vector2D cBestVertex = new Vector2D (caVertices[bestVertexIndex]),
               cPrevVertex = new Vector2D (caVertices[prevIndex]),
               cNextVertex = new Vector2D (caVertices[nextIndex]);

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
      Vector2D caVertices[] = mcaVertices,
               cCenter = mcCenter;
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

    for (int i = 0; i < numVertices; ++i)
    {
      mcaVertices[i].offSet (x,y);
    }
  }

  public AABB getAABB ()
  {
    Vector2D caVertices[] = mcaVertices,
             cVertex = caVertices[0];
    int numVertices = mNumVertices;
    double x = cVertex.mX,
           y = cVertex.mY,
           minX = x,
           minY = y,
           maxX = x,
           maxY = y;

    for (int i = 1; i < numVertices; ++i)
    {
      cVertex = caVertices[i];
      x = cVertex.mX;
      y = cVertex.mY;

      if (x < minX)
      {
        minX = x;
      }
      else if (x > maxX)
      {
        maxX = x;
      }

      if (y < minY)
      {
        minY = y;
      }
      else if (y > maxY)
      {
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

}
