package Vectors;

/**
 * Implements class to represent one dimensional line existing in two
 * dimensional space
 *
 * @author  Dannie Wright
 * @since   6/22/2018
 */

public class Line2D implements IVector2D
{
  private Vector2D mcStart,
                   mcEnd,
                   mcDifference;

  /**
   * Initializes data members to given values
   *
   * @param cStart - starting point of line
   * @param cEnd - ending point of line
   */
  public Line2D (Vector2D cStart, Vector2D cEnd)
  {
    mcStart = new Vector2D(cStart);
    mcEnd = new Vector2D(cEnd);
    mcDifference = cEnd.getSubtract (cStart);
  }

  public Line2D (Line2D cOther) {
    this (new Vector2D (cOther.mcStart), new Vector2D(cOther.mcEnd));
  }

  /**
   * Returns the angle between this line and the given vector
   *
   * @param cVec - vector being compare to this line
   *
   * @return angle between this line and the given vector
   */
  public double angle (Vector2D cVec)
  {
    return mcDifference.angle (cVec);
  }

  /**
   * Returns the angle between this line and the given line
   *
   * @param cLine - Vectors.Line2D being compare to this line
   *
   * @return angle between this line and the given line
   */
  public double angle (Line2D cLine)
  {
    return this.mcDifference.angle (cLine.mcDifference);
  }

  /**
   * Returns the dot product between this line and the given vector
   *
   * @param cVec - vector being compared to this line
   *
   * @return the dot product between this line and the given vector
   */
  public double dot (Vector2D cVec)
  {
    return mcDifference.dot (cVec);
  }
  
  @Override
  public double cross (Vector2D cVec)
  {
    return mcDifference.cross (cVec);
  }
  
  /**
   * Returns the dot product between this line and the given line
   *
   * @param cLine - Vectors.Line2D being compare to this line
   *
   * @return the dot product between this line and the given line
   */
  public double dot (Line2D cLine)
  {
    return this.mcDifference.dot (cLine.mcDifference);
  }

  /**
   * Normalizes length of line
   */
  public Vector2D unit ()
  {
    mcDifference.unit ();
    mcEnd = mcStart.getAdd (mcDifference);

    return new Vector2D (mcDifference);
  }

  /**
   * Returns the normalized value of the line
   *
   * @return normalized value of the line
   */
  public Vector2D getUnit ()
  {
    return mcDifference.getUnit ();
  }

  /**
   * Returns the projection of this line onto the given vector
   *
   * @param cAxis - axis this line is being projected onto
   *
   * @return projection of this line onto the given vector
   */
  public double projection (Vector2D cAxis)
  {
    return mcDifference.projection (cAxis);
  }

  public boolean contains (Vector2D cPoint) {
    return mcDifference.contains(cPoint.getSubtract(mcStart));
  }



  public Line toLine () {
    return new Line (mcStart.getMagnitude(), mcEnd.getMagnitude());
  }

  public Line toLineSqrd () {
    return new Line (mcStart.getMagnitudeSqrd(), mcEnd.getMagnitudeSqrd());
  }

  /**
   * Returns the projection of this line onto the given line
   *
   * @param cLine - line this line is being projected onto
   *
   * @return projection of this line onto the given line
   */
  public double projection (Line2D cLine)
  {
    return this.mcDifference.projection (cLine.mcDifference);
  }

  /**
   * Returns the normal of this line given the orientation
   *
   * @param eOrientation - orientation of normal being returned
   *
   * @return vector orthogonal to this line with the given orientation
   */
  public Vector2D getNormal (IVector2D.VectorOrientation eOrientation)
  {
    return mcDifference.getNormal (eOrientation);
  }


  public Vector2D getNormal (Vector2D cChange, IVector2D.VectorOrientation eOrientation) {
    return mcDifference.getNormal(cChange, eOrientation);
  }

  /**
   * Returns the normal of this line with clockwise orientation
   *
   * @return returns vector orthogonal to this line with clockwise orientation
   */
  public Vector2D getNormalCW ()
  {
    return mcDifference.getNormalCW ();
  }


  public Vector2D getNormalCW (Vector2D cChange) {
    return mcDifference.getNormalCW (cChange);
  }

  /**
   * Returns the normal of this line with counterclockwise orientation
   *
   * @return returns vector orthogonal to this line with counterclockwise
   *         orientation
   */
  public Vector2D getNormalCCW ()
  {
    return mcDifference.getNormalCCW ();
  }


  public Vector2D getNormalCCW (Vector2D cChange) {
    return mcDifference.getNormalCCW(cChange);
  }

  public Vector2D getTangent (Vector2D cVec)
  {
    return mcDifference.getTangent (cVec);
  }

  /**
   * Returns whether this line is perpendicular to the given vector
   *
   * @param cAxis - vector being compared to this line
   *
   * @return true if this line and given vector are perpendicular, else false
   */
  public boolean perpendicular (Vector2D cAxis)
  {
    return mcDifference.perpendicular (cAxis);
  }

  /**
   * Returns whether this line is perpendicular to the given line
   *
   * @param cLine - line being compared to this line
   *
   * @return true if this line and given line are perpendicular, else false
   */
  public boolean perpendicular (Line2D cLine)
  {
    return this.mcDifference.perpendicular (cLine.mcDifference);
  }

  /**
   * Returns whether this line is parallel to the given vector
   *
   * @param cAxis - vector being compared to this line
   *
   * @return true if this line and given vector are parallel, else false
   */
  public boolean parallel (Vector2D cAxis)
  {
    return mcDifference.parallel (cAxis);
  }

  /**
   * Returns whether this line is parallel to the given line
   *
   * @param cLine - line being compared to this line
   *
   * @return true if this line and given line are parallel, else false
   */
  public boolean parallel (Line2D cLine)
  {
    return this.mcDifference.parallel (cLine.mcDifference);
  }

  //TODO comments

  public Vector2D add (Vector2D cVec)
  {
    this.mcDifference.add (cVec);
    this.mcEnd.add (cVec);

    return new Vector2D (mcDifference);
  }

  public Line2D shift (Vector2D cVec) {
    this.mcStart.add(cVec);
    this.mcEnd.add(cVec);
    return this;
  }

  /**
   * Returns the
   * @param cVec
   * @return
   */
  public Vector2D getAdd (Vector2D cVec)
  {
    return mcDifference.getAdd (cVec);
  }

  public Vector2D subtract (Vector2D cVec)
  {
    mcDifference.subtract (cVec);
    mcEnd.subtract (cVec);

    return new Vector2D (mcDifference);
  }

  public Vector2D getSubtract (Vector2D cVec)
  {
    return mcDifference.getSubtract (cVec);
  }

  public Vector2D set (Vector2D cVec)
  {
    mcStart.set (cVec);
    mcDifference = mcEnd.getSubtract (mcStart);

    return new Vector2D (mcDifference);
  }

  public Vector2D set (double x, double y)
  {
    mcStart.set (x,y);
    mcDifference = mcEnd.getSubtract (mcStart);

    return new Vector2D (mcDifference);
  }

  public void setEnd (double x, double y)
  {
    mcEnd.set (x,y);
    mcDifference = mcEnd.getSubtract (mcStart);
  }

  public Vector2D offSet (double x, double y)
  {
    mcStart.offSet (x,y);
    mcDifference = mcEnd.getSubtract (mcStart);

    return new Vector2D (mcDifference);
  }

  public void offSetEnd (double x, double y)
  {
    mcEnd.offSet (x,y);
    mcDifference = mcEnd.getSubtract (mcStart);
  }

  public Vector2D rotate (double angle)
  {
    mcDifference.rotate (angle);

    return new Vector2D (mcDifference);
  }

  public Vector2D getRotated (double angle)
  {
    return mcDifference.getRotated (angle);
  }

  public void rotateOrigin (double angle)
  {
    mcStart.rotate (angle);
    mcDifference.rotate (angle);
    mcEnd = mcStart.getAdd (mcDifference);
  }

  public Line2D getRotatedOrigin (double angle)
  {
    Vector2D cStart = mcStart.getRotated (angle),
             cDiff = mcDifference.getRotated (angle);

    return new Line2D (cStart, cStart.getAdd (cDiff));
  }

  public Vector2D mirrorX ()
  {
    mcDifference.mirrorX ();
    mcStart.mirrorX ();
    mcEnd = mcStart.getAdd (mcDifference);

    return new Vector2D (mcDifference);
  }

  public Vector2D mirrorY ()
  {
    mcDifference.mirrorY ();
    mcStart.mirrorY ();
    mcEnd = mcStart.getAdd (mcDifference);

    return new Vector2D (mcDifference);
  }

  public Vector2D getMirrorX ()
  {
    return mcDifference.getMirrorX ();
  }

  public Vector2D getMirrorY ()
  {
    return mcDifference.getMirrorY ();
  }

  public Vector2D mirror ()
  {
    this.mirrorX ();
    this.mirrorY ();

    return new Vector2D (mcDifference);
  }

  public Vector2D getMirror ()
  {
    return mcDifference.getMirror ();
  }

  public Vector2D getScaled (double scale)
  {
    return mcDifference.getScaled (scale);
  }

  public Vector2D scale (double scale)
  {
    mcDifference.scale (scale);
    mcEnd = mcStart.getAdd (mcDifference);

    return new Vector2D (mcDifference);
  }


  public Line2D overlap (Line2D cLine)
  {
    Vector2D cDifferenceAxis = this.mcDifference.getUnit ();
    //determine projection of starting and ending points onto
    //this.mcDifference
    double thisStart = this.mcStart.dot (cDifferenceAxis),
           thisEnd = this.mcEnd.dot (cDifferenceAxis),
           otherStart = cLine.mcStart.dot (cDifferenceAxis),
           otherEnd = cLine.mcEnd.dot (cDifferenceAxis);

    //create 1D lines based on projections
    Vectors.Line cThis = thisStart < thisEnd ? new Vectors.Line (thisStart,
                          thisEnd) : new Vectors.Line (thisEnd, thisStart),
                 cOther = otherStart < otherEnd ? new Vectors.Line (
                          otherStart, otherEnd) : new Vectors.Line (otherEnd,
                          otherStart);

    //obtain overlap between these lines
    Vectors.Line cOverlap = cThis.getOverlapLine (cOther);

    //if lines do not overlap return null
    if (null == cOverlap)
    {
      return null;
    }

    //if lines do overlap, transform 1D lines into 2D space relative to
    //this.mcStart to determine overlap of projection of other's line onto
    //this line
    Vector2D cStart = this.mcStart.getAdd (cDifferenceAxis.getScaled (
                                        cOverlap.mMin - thisStart)),
             cEnd = this.mcStart.getAdd (cDifferenceAxis.getScaled (
                                        cOverlap.mMax - thisStart));

    return new Line2D (cStart, cEnd);
  }

  public boolean equals (Vector2D cVec)
  {
    return mcDifference.equals (cVec);
  }

  public boolean equals (Line2D cOther) {
    return mcStart.equals(cOther.mcStart) && mcEnd.equals(cOther.mcEnd);
  }

  /**
   * Returns if the lines have the same endpoints but may point in opposite
   * directions
   *
   * @param cOther - other line being compared to this line
   *
   * @return - true if the lines have the same endpoints, false if either
   * endpoint is not equal to one of cOther's endpoints
   */
  public boolean equalsUndirected (Line2D cOther) {
    return (mcStart.equals(cOther.mcStart) && mcEnd.equals(cOther.mcEnd))
        || (mcEnd.equals(cOther.mcStart) && mcStart.equals(cOther.mcEnd));
  }

  public Vector2D getDifference ()
  {
    return new Vector2D (mcDifference);
  }

  public Vector2D[] toArray () {
    return new Vector2D[] {new Vector2D(mcStart), new Vector2D(mcEnd)};
  }

  public Vector2D getStart () {
    return new Vector2D (mcStart);
  }

  public Vector2D getEnd () {
    return new Vector2D (mcEnd);
  }

  @Override
  public double getMagnitude () {
    return mcDifference.getMagnitude();
  }

  @Override
  public double getMagnitudeSqrd () {
    return mcDifference.getMagnitudeSqrd();
  }

  @Override
  public boolean zeroMagnitude () {
    return mcDifference.zeroMagnitude();
  }

  @Override
  public boolean zeroMagnitudeSqrd () {
    return mcDifference.zeroMagnitudeSqrd();
  }

}