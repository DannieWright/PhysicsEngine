package Vectors;

/**
 * Interface for a two dimensional vector
 *
 * @author  Dannie Wright
 * @since   7/2/2018
 */

public interface IVector2D
{
  enum VectorOrientation
  {
    CLOCKWISE,
    COUNTERCLOCKWISE
  }

  double APPROX_ZERO = 1.0E-14d;

  double angle (Vector2D cVec);
  double dot (Vector2D cVec);
  double getMagnitude ();
  double getMagnitudeSqrd ();
  boolean zeroMagnitude ();
  boolean zeroMagnitudeSqrd();
  double cross (Vector2D cVec);

  Vector2D unit ();
  Vector2D getUnit ();

  double projection (Vector2D cAxis);
  boolean contains (Vector2D cPoint);

  Vector2D getNormal (VectorOrientation eOrientation);
  Vector2D getNormal (Vector2D cChange, IVector2D.VectorOrientation eOrientation);
  Vector2D getNormalCW ();
  Vector2D getNormalCW (Vector2D cChange);
  Vector2D getNormalCCW ();
  Vector2D getNormalCCW (Vector2D cChange);
  Vector2D getTangent (Vector2D cVec);
  boolean perpendicular (Vector2D cAxis);
  boolean parallel (Vector2D cAxis);

  Vector2D add (Vector2D cVec);
  Vector2D getAdd (Vector2D cVec);
  Vector2D subtract (Vector2D cVec);
  Vector2D getSubtract (Vector2D cVec);

  Vector2D set (double x, double y);
  Vector2D set (Vector2D cVec);
  Vector2D offSet (double x, double y);
  Vector2D rotate (double angle); //counterclockwise
  Vector2D getRotated (double angle);
  Vector2D mirrorX ();
  Vector2D mirrorY ();
  Vector2D mirror ();
  Vector2D getMirrorX ();
  Vector2D getMirrorY ();
  Vector2D getMirror ();

  Vector2D getScaled (double scale);
  Vector2D scale (double scale);

  boolean equals (Vector2D cVec);
}
