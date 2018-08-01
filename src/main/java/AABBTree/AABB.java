package AABBTree;

import java.lang.Math;

/**
 * Implements class to compare AABB collisions
 *
 * @author  Dannie Wright
 * @since   6/11/2018
 */

public class AABB
{
  private static final double APPROX_ZERO = 1.0E-14d;

  private double mMinX,mMaxX,
                 mMinY,mMaxY;
  protected double mSurfaceArea;

  /**
   * Initializes class to default values
   */
  public AABB ()
  {
    mMinX = mMaxX = mMinY = mMaxY = mSurfaceArea = 0;
  }

  /**
   * Initializes class to specified values
   *
   * @param minX - left most position value
   * @param maxX - right most position value
   * @param minY - lower most position value
   * @param maxY - upper most position value
   */
  public AABB (double minX, double maxX, double minY, double maxY)
  {
    mMinX = minX;
    mMaxX = maxX;
    mMinY = minY;
    mMaxY = maxY;
    mSurfaceArea = surfaceArea ();
  }

  /**
   * Initializes class to have same values as other
   *
   * @param cOther - class values being copied
   */
  public AABB (AABB cOther)
  {
    this.mMinX = cOther.mMinX;
    this.mMaxX = cOther.mMaxX;
    this.mMinY = cOther.mMinY;
    this.mMaxY = cOther.mMaxY;
    this.mSurfaceArea = surfaceArea ();
  }

  /**
   * Merges AABB with given AABB
   *
   * @param cOther - other AABB being merged with
   *
   * @return AABB such that this and cOther fit inside
   */
  public AABB merge (AABB cOther)
  {
    AABB cNew = new AABB (Math.min (cOther.mMinX, this.mMinX),
                          Math.max (cOther.mMaxX, this.mMaxX),
                          Math.min (cOther.mMinY, this.mMinY),
                          Math.max (cOther.mMaxY, this.mMaxY));

    return cNew;
  }

  /**
   * Returns if this AABB overlaps with given AABB
   *
   * @param cOther - other AABB being checked against
   *
   * @return true on overlap, else false
   */
  public boolean overlap (AABB cOther)
  {
    return this.mMaxX > cOther.mMinX
        && this.mMinX < cOther.mMaxX
        && this.mMaxY > cOther.mMinY
        && this.mMinY < cOther.mMaxY;
  }

  /**
   * Returns if other AABB is fully contained in this AABB
   *
   * @param cOther - other AABB beinc checked
   *
   * @return true if other AABB is fully contained inside this AABB,
   *          else false
   */
  public boolean encases (AABB cOther)
  {
    return this.mMaxX <= cOther.mMinX
        && this.mMaxX >= cOther.mMaxX
        && this.mMinY <= cOther.mMinY
        && this.mMaxY >= cOther.mMaxY;
  }

  /**
   * Returns the AABB that makes up the intersection between this and cOther
   *
   * @param cOther - other AABB being checked
   *
   * @return intersection of this and cOther
   */
  public AABB intersection (AABB cOther)
  {
    AABB cNew = new AABB (Math.max (cOther.mMinX, this.mMinX),
                          Math.min (cOther.mMaxX, this.mMaxX),
                          Math.max (cOther.mMinY, this.mMinY),
                          Math.min (cOther.mMaxY, this.mMaxY));

    return cNew;
  }

  /**
   * Returns surface area of AABB
   *
   * @return AABB surface area
   */
  public double surfaceArea ()
  {
    return (mMaxX - mMinX) * (mMaxY - mMinY);
  }

  /**
   * Returns whether the objects have the same values
   *
   * @param cAABB - other AABB being checked
   *
   * @return true if all the values are the same, else false
   */
  public boolean equals (AABB cAABB)
  {
    return Math.abs (this.mMinX - cAABB.mMinX) < AABB.APPROX_ZERO
        && Math.abs (this.mMinY - cAABB.mMinY) < AABB.APPROX_ZERO
        && Math.abs (this.mMaxX - cAABB.mMaxX) < AABB.APPROX_ZERO
        && Math.abs (this.mMaxY - cAABB.mMaxY) < AABB.APPROX_ZERO;
  }

  /**
   * Offsets AABB location in the x and y directions
   *
   * @param x - offset in the x-direction
   * @param y - offset in the y-direction
   */
  public void offSet (double x, double y)
  {
    mMinX += x;
    mMaxX += x;
    mMinY += y;
    mMaxY += y;
  }

  /**
   * Returns the data members in a string format
   *
   * @return data members in a string format
   */
  public String toString ()
  {
    return String.format ("{(%f,%f),(%f,%f)}", mMinX, mMinY, mMaxX, mMaxY);
  }
}
