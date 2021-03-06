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

  public double getMagnitude ()
  {
    return Math.sqrt (mX * mX + mY * mY);
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

  // clockwise convention
  @Override
  public Vector2D getNormalCW ()
  {
    double newX = mY,
           newY = -mX;
    return new Vector2D (newX, newY);
  }
  
  @Override
  public Vector2D getNormalCCW ()
  {
    double newX = -mY,
           newY = mX;
    return new Vector2D (newX, newY);
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
    return IVector2D.APPROX_ZERO >= this.getUnit ().projection (cAxis);
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
}
