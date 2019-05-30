package Vectors;

/**
 * Implements class to represent one dimensional line
 *
 * @author  Dannie Wright
 * @since   6/22/2018
 */

public class Line
{
  public double mMin,
                mMax;

  /**
   * Initializes class to given values
   *
   * @param min - smaller point on line
   * @param max - larger point on line
   */
  public Line (double min, double max)
  {
    mMin = min;
    mMax = max;
  }

  /**
   * Returns whether there is an overlap between the two lines
   *
   * @param cOther - other line being checked against this line
   *
   * @return true if there is an overlap, else false
   */
  public boolean overlap (Line cOther)
  {
    return this.mMax > cOther.mMin
        && this.mMin < cOther.mMax;
  }

  /**
   * Returns the amount the lines overlap
   *
   * @param cOther - other line being checked against this line
   *
   * @return amount the lines overlap if they do, else zero if they do not
   */
  public double getOverlap (Line cOther)
  {
    if (!this.overlap (cOther))
    {
      return 0.0d;
    }

    return Math.min (this.mMax, cOther.mMax)
         - Math.max (this.mMin, cOther.mMin);
  }

  /**
   * Returns the Line of overlap
   *
   * @param cOther - Line being compared with this Line
   *
   * @return Line object that is the overlap of these two lines, null if the
   *         lines do not overlap
   */
  public Line getOverlapLine (Line cOther)
  {
    if (!this.overlap (cOther))
    {
      return null;
    }

    return new Line (Math.min (this.mMax, cOther.mMax),
                    Math.max (this.mMin, cOther.mMin));
  }

  /**
   * Returns whether the other Line is fully contained in this Line
   *
   * @param cOther - Line being compared with this Line
   *
   * @return true if other Line is fully contained in this Line
   */
  public boolean contains (Line cOther)
  {
    return this.mMin <= cOther.mMin && this.mMax >= cOther.mMax;
  }

  /**
   * Returns whether the given point is contained in this Line
   *
   * @param cPoint - Point being checked if it is on this line
   *
   * @return - true if the point is in between this line's min and max values
   */
  public boolean contains (double cPoint) {
    return this.mMin <= cPoint && this.mMax >= cPoint;
  }

  /**
   * Returns data members as a string
   *
   * @return data members as a string
   */
  public String toString ()
  {
    return String.format ("(%f,%f)", mMin, mMax);
  }
}
