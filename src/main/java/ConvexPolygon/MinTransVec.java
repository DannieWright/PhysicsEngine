package ConvexPolygon;

import Vectors.Vector2D;

/**
 * Implements class to store a minimum translational vector to remove the
 * given shapes
 *
 * @author  Dannie Wright
 * @since   7/5/2018
 */

public class MinTransVec
{
  public Vector2D mcMinTransVec;
  public Shape mcOrigin,     //shape that min translational vector points from
               mcRecipient;  //shape that min translational vector points to

  /**
   * Initializes data members to given values
   *
   * @param cMinTransVec - minimum required vector to separate origin and
 *                         recipient shapes
   * @param cOrigin - shape minimum translational vector points from
   * @param cRecipient - shape minimum translational vector points towards
   */
  MinTransVec (Vector2D cMinTransVec, Shape cOrigin, Shape cRecipient)
  {
    mcMinTransVec = cMinTransVec;
    mcOrigin = cOrigin;
    mcRecipient = cRecipient;
  }

}
