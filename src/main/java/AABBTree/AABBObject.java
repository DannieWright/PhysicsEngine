package AABBTree;

/**
 * Implements class to store AABB data
 *
 * @author  Dannie Wright
 * @since   7/6/2018
 */

public class AABBObject
{
  public AABB mcAABB;

  public AABBObject (AABB cAABB)
  {
    mcAABB = cAABB;
  }

  public AABBObject (AABBObject cObject)
  {
    this.mcAABB = new AABB (cObject.mcAABB);
  }
}
