package Vectors;

/**
 * Implements class to handle two dimensional vectors
 *
 * @author  Dannie Wright
 * @since   6/15/2018
 */

//TODO add mbUnitUpToDate && Vector2D mcUnitVec, calc mcUnitVec if !mbUnitUpToDate else return mcUnitVec to save calculation cost for multiple access

public class Vector2D implements IVector2D
{
  public enum VectorDirection {
    N,
    NE,
    E,
    SE,
    S,
    SW,
    W,
    NW;



    //orientation if going from this direction to the given direction
    public Vector2D.VectorDirection next (IVector2D.VectorOrientation eOri) {
      //if clockwise
      if (IVector2D.VectorOrientation.CLOCKWISE == eOri) {
        //return next enum
        if (Vector2D.VectorDirection.NW != this) {
          return Vector2D.VectorDirection.values()[this.ordinal() + 1];
        }
        //handle wrapping
        return Vector2D.VectorDirection.N;
      }

      //if counterclockwise
      //return prev enum
      if (Vector2D.VectorDirection.N != this) {
        return Vector2D.VectorDirection.values()[this.ordinal() - 1];
      }
      //handle wrapping
      return Vector2D.VectorDirection.NW;
    }

    //orientation if going from this direction to the given direction
    public Vector2D.VectorDirection second (IVector2D.VectorOrientation eOri) {
      int ordinal = this.ordinal();
      //if clockwise
      if (IVector2D.VectorOrientation.CLOCKWISE == eOri) {
        //return next enum
        if (Vector2D.VectorDirection.W.ordinal() > ordinal) {
          return Vector2D.VectorDirection.values()[ordinal + 2];
        }
        //handle wrapping
        else if (Vector2D.VectorDirection.W.ordinal() == ordinal) {
          return Vector2D.VectorDirection.N;
        }
        return Vector2D.VectorDirection.NE;
      }

      //if counterclockwise
      //return prev enum
      if (Vector2D.VectorDirection.NE.ordinal() < ordinal) {
        return Vector2D.VectorDirection.values()[ordinal - 2];
      }
      //handle wrapping
      else if (Vector2D.VectorDirection.NE.ordinal() == ordinal) {
        return Vector2D.VectorDirection.NW;
      }
      return Vector2D.VectorDirection.W;
    }


  }
  public static final Vector2D[] caDIRECTIONS = {
    new Vector2D(0,1), new Vector2D(0.707,0.707),
    new Vector2D(1,0), new Vector2D(0.707,-0.707),
    new Vector2D(0,-1), new Vector2D(-0.707,-0.707),
    new Vector2D(-1,0), new Vector2D(-0.707,1)};

  public double mX,
                mY;
  
  
  public Vector2D ()
  {
    mX = mY = 0;
  }

  public Vector2D (double xValue, double yValue)
  {
    mX = xValue;
    mY = yValue;
  }

  public Vector2D (Vector2D cVec)
  {
    mX = cVec.mX;
    mY = cVec.mY;
  }
  
  @Override
  public double angle (Vector2D cVec)
  {
    Vector2D cUnitVec1 = this.getUnit (),
             cUnitVec2 = cVec.getUnit ();

    return Math.acos (cUnitVec1.dot (cUnitVec2));
  }
  
  @Override
  public double dot (Vector2D cVec)
  {
    return this.mX * cVec.mX + this.mY * cVec.mY;
  }

  @Override
  public double getMagnitude ()
  {
    return Math.sqrt (mX * mX + mY * mY);
  }

  @Override
  public double getMagnitudeSqrd () {
    return mX * mX + mY * mY;
  }

  @Override
  public boolean zeroMagnitude () {
    return IVector2D.APPROX_ZERO > this.getMagnitude();
  }

  @Override
  public boolean zeroMagnitudeSqrd () {
    return IVector2D.APPROX_ZERO > this.getMagnitudeSqrd();
  }

  @Override
  public Vector2D unit ()
  {
    double magnitude = this.getMagnitude ();

    mX /= magnitude;
    mY /= magnitude;

    return this;
  }
  
  @Override
  public Vector2D getUnit ()
  {
    double magnitude = this.getMagnitude ();

    return new Vector2D (mX / magnitude, mY / magnitude);
  }
  
  @Override
  public double projection (Vector2D cAxis)
  {
    return this.dot (cAxis.getUnit ());
  }

  /**
   * Returns true if the given point projected onto this vector falls in
   * between the beginning and end of this vector
   *
   * @param cPoint - point being checked if it is contained on this vector's
   *               line
   *
   * @return - true if the point is contained in this vector
   */
  public boolean contains (Vector2D cPoint) {
    return this.getMagnitudeSqrd() >= cPoint.getMagnitudeSqrd()
        && this.dot(cPoint) >= 0;
  }

  public double distance (Vector2D cVec)
  {
    double xDiff = this.mX - cVec.mX,
           yDiff = this.mY - cVec.mY;
    return Math.sqrt (xDiff * xDiff + yDiff * yDiff);
  }

  
  public double distanceSquared (Vector2D cVec)
  {
    double xDiff = this.mX - cVec.mX,
           yDiff = this.mY - cVec.mY;
    return xDiff * xDiff + yDiff * yDiff;
  }
  
  @Override
  public Vector2D getNormal (IVector2D.VectorOrientation eOrientation)
  {
    return IVector2D.VectorOrientation.CLOCKWISE == eOrientation ? this.getNormalCW ()
        : this.getNormalCCW ();
  }

  public Vector2D getNormal (Vector2D cChange, IVector2D.VectorOrientation eOrientation) {
    return IVector2D.VectorOrientation.CLOCKWISE == eOrientation ? this.getNormalCW (cChange)
      : this.getNormalCCW (cChange);
  }

  // clockwise convention
  @Override
  public Vector2D getNormalCW () {
    return new Vector2D (mY, -mX);
  }

  public Vector2D getNormalCW (Vector2D cChange) {
    return cChange.set(mY, -mX);
  }
  
  @Override
  public Vector2D getNormalCCW () {
    return new Vector2D (-mY, mX);
  }

  public Vector2D getNormalCCW (Vector2D cChange) {
    return cChange.set(-mY, mX);
  }


  //gets tangent vector in direction of cVec
  @Override
  public Vector2D getTangent (Vector2D cVec)
  {
    return this.getScaled (-this.dot (cVec)).add (cVec);
  }
  
  @Override
  public boolean perpendicular (Vector2D cAxis)
  {
    return IVector2D.APPROX_ZERO >= (1 - Math.abs(this.getUnit ().cross (cAxis.getUnit())));
  }
  
  @Override
  public boolean parallel (Vector2D cAxis)
  {
    return IVector2D.APPROX_ZERO >= (1 - Math.abs (this.getUnit ().projection (cAxis)));
  }
  
  @Override
  public Vector2D add (Vector2D cVec)
  {
   this.mX += cVec.mX;
   this.mY += cVec.mY;

    return this;
  }
  
  @Override
  public Vector2D getAdd (Vector2D cVec)
  {
    return new Vector2D (this.mX + cVec.mX, this.mY + cVec.mY);
  }
  
  @Override
  public Vector2D subtract (Vector2D cVec)
  {
    this.mX -= cVec.mX;
    this.mY -= cVec.mY;

    return this;
  }
  
  @Override
  public Vector2D getSubtract (Vector2D cVec)
  {
    return new Vector2D (this.mX - cVec.mX, this.mY - cVec.mY);
  }
  
  @Override
  public Vector2D set (Vector2D cVec)
  {
    this.mX = cVec.mX;
    this.mY = cVec.mY;

    return this;
  }
  
  @Override
  public Vector2D set (double x, double y)
  {
    mX = x;
    mY = y;

    return this;
  }
  
  @Override
  public Vector2D offSet (double x, double y)
  {
    mX += x;
    mY += y;

    return this;
  }

  //rotate ccw
  @Override
  public Vector2D rotate (double angle)
  {
    double cos = Math.cos (angle),
           sin = Math.sin (angle),
           x = mX * cos - mY * sin,
           y = mX * sin + mY * cos;

    mX = x;
    mY = y;

    return this;
  }
  
  @Override
  public Vector2D getRotated (double angle)
  {
    double cos = Math.cos (angle),
           sin = Math.sin (angle),
           x = mX * cos - mY * sin,
           y = mX * sin + mY * cos;

    return new Vector2D (x, y);
  }
  
  @Override
  public Vector2D mirrorX ()
  {
    mX *= -1;

    return this;
  }
  
  @Override
  public Vector2D mirrorY ()
  {
    mY *= -1;

    return this;
  }
  
  @Override
  public Vector2D mirror ()
  {
    this.mirrorX ();
    this.mirrorY ();

    return this;
  }
  
  @Override
  public Vector2D getMirrorX ()
  {
    return new Vector2D (-mX, mY);
  }
  
  @Override
  public Vector2D getMirrorY ()
  {
    return new Vector2D (mX, -mY);
  }
  
  @Override
  public Vector2D getMirror ()
  {
    return new Vector2D (-mX, -mY);
  }
  
  @Override
  public Vector2D getScaled (double scale)
  {
    return new Vector2D (this.mX * scale, this.mY * scale);
  }
  
  @Override
  public Vector2D scale (double scale)
  {
    this.mX *= scale;
    this.mY *= scale;

    return this;
  }
  
  @Override
  public String toString ()
  {
    return String.format ("(%f,%f)", mX, mY);
  }
  
  @Override
  public boolean equals (Vector2D cVec)
  {
    return Math.abs (this.mX - cVec.mX) < IVector2D.APPROX_ZERO
        && Math.abs (this.mY - cVec.mY) < IVector2D.APPROX_ZERO ;
  }
  
  @Override
  public double cross (Vector2D cVec)
  {
    return this.mX * cVec.mY - this.mY * cVec.mX;
  }

  public Vector2D setSameQuadrant (Vector2D cVec) {
    mX = Math.copySign(mX,cVec.mX);
    mY = Math.copySign(mY, cVec.mY);
    return this;
  }

  public Vector2D getSameQuadrant (Vector2D cVec) {
    return new Vector2D(Math.copySign(mX, cVec.mX), Math.copySign(mY, cVec.mY));
  }

  //defaults NE
  public Vector2D.VectorDirection getDirection () {
    Vector2D.VectorDirection eMain = this.getDirectionMain(),
                             eSub = this.getDirectionSub();
    double main = this.dot (Vector2D.caDIRECTIONS[eMain.ordinal()]),
           sub = this.dot (Vector2D.caDIRECTIONS[eSub.ordinal()]);

    return main > sub ? eMain : eSub;
  }

  //defaults N
  public Vector2D.VectorDirection getDirectionMain () {
    double north = this.dot(Vector2D.caDIRECTIONS[Vector2D.VectorDirection.N.ordinal()]),
           east = this.dot(Vector2D.caDIRECTIONS[Vector2D.VectorDirection.E.ordinal()]);

    //if vector is N or S
    if (Math.abs(north) >= Math.abs(east)) {
      if (north >= 0) {
        return Vector2D.VectorDirection.N;
      }
      return Vector2D.VectorDirection.S;
    }

    //if vector is E or W
    if (east >= 0) {
      return Vector2D.VectorDirection.E;
    }
    return Vector2D.VectorDirection.W;
  }

  public Vector2D.VectorDirection getDirectionSub () {
    double northE = this.dot(Vector2D.caDIRECTIONS[Vector2D.VectorDirection.NE.ordinal()]),
           southE = this.dot(Vector2D.caDIRECTIONS[Vector2D.VectorDirection.SE.ordinal()]);

    //if vector is NE or SW
    if (Math.abs(northE) >= Math.abs(southE)) {
      if (northE >= 0) {
        return Vector2D.VectorDirection.NE;
      }
      return Vector2D.VectorDirection.SW;
    }

    //if vector is SE or NW
    if (southE >= 0) {
      return Vector2D.VectorDirection.SE;
    }
    return Vector2D.VectorDirection.NW;
  }
}
