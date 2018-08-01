package ConvexPolygon;

import Vectors.IVector2D;
import Vectors.Vector2D;


public class Rectangle extends Polygon
{
  //default cw orientation
  public static final IVector2D.VectorOrientation eORIENTATION
                        = IVector2D.VectorOrientation.CLOCKWISE;

  //starts with top left vertex
  public Rectangle (Vector2D cCenter, double width, double height)
  {
   super (new Vector2D[] {new Vector2D (cCenter.mX - width * 0.5,
                                        cCenter.mY + height * 0.5),
                         new Vector2D (cCenter.mX + width * 0.5,
                                        cCenter.mY + height * 0.5),
                         new Vector2D (cCenter.mX + width * 0.5,
                                        cCenter.mY - height * 0.5),
                         new Vector2D (cCenter.mX - width * 0.5,
                                        cCenter.mY - height * 0.5)},
       Rectangle.eORIENTATION);

   //TODO handle error
    if (0 >= width || 0 >= height)
    {

    }
  }

  //starts with top left vertex
  public Rectangle (Vector2D cTopLeft, Vector2D cBottomRight)
  {
    super (new Vector2D[] {cTopLeft,
                          new Vector2D (cBottomRight.mX, cTopLeft.mY),
                          cBottomRight,
                          new Vector2D (cTopLeft.mX, cBottomRight.mY)},
        Rectangle.eORIENTATION);

    //TODO handle error
    if (cTopLeft.mX > cBottomRight.mX || cTopLeft.mY < cBottomRight.mY)
    {

    }
  }

  public Rectangle (Rectangle cRectangle)
  {
    super (cRectangle);
  }


  public Rectangle (double centerX, double centerY, double width, double height)
  {
    super (new Vector2D[] {
    new Vector2D (centerX - 0.5 * width, centerY + 0.5 * height),
    new Vector2D (centerX + 0.5 * width, centerY + 0.5 * height),
    new Vector2D (centerX + 0.5 * width, centerY - 0.5 * height),
    new Vector2D (centerX - 0.5 * width, centerY - 0.5 * height)},
        IVector2D.VectorOrientation.CLOCKWISE);

    //TODO handle error
    if (0 >= width || 0 >= height)
    {

    }
  }

  //TODO overwrite getAABB() to make it more efficient

}
