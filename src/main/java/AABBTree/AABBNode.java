package AABBTree;

/**
 * Implements class to compare AABB collisions
 *
 * @author  Dannie Wright
 * @since   6/11/2018
 */

public class AABBNode
{
  public static final int NULL_INDEX = -1;

  public AABBObject mcObject;
  public AABB mcAABB;
  public int mParentIndex,
             mLeftIndex,
             mRightIndex,
             mNextNodeIndex;

  /**
   * Initializes class to default values
   */
  public AABBNode ()
  {
    mParentIndex = mLeftIndex = mRightIndex = mNextNodeIndex
        = AABBNode.NULL_INDEX;
    mcObject = null;
    mcAABB = null;
  }

  /**
   * Initializes class with given AABBObject
   *
   * @param cObject - AABBObject for node
   */
  public AABBNode (AABBObject cObject)
  {
    mParentIndex = mLeftIndex = mRightIndex = mNextNodeIndex
        = AABBNode.NULL_INDEX;
    mcObject = cObject;
    mcAABB = cObject.mcAABB;
  }

  public AABBNode (AABBNode cAABBNode)
  {
    this.mParentIndex = cAABBNode.mParentIndex;
    this.mLeftIndex = cAABBNode.mLeftIndex;
    this.mRightIndex = cAABBNode.mRightIndex;
    this.mNextNodeIndex = cAABBNode.mNextNodeIndex;
    if (null != cAABBNode.mcObject)
    {
      this.mcObject = new AABBObject (cAABBNode.mcObject);
      this.mcAABB = this.mcObject.mcAABB;
    }
    else
    {
      this.mcObject = null;
      this.mcAABB = new AABB (cAABBNode.mcAABB);
    }
  }

  /**
   * Returns if the node is a leaf or not. Since a branch must have both a
   * left and right, only one must be checked for null
   *
   * @return true if the node is a leaf, else false
   */
  public boolean isLeaf ()
  {
    return AABBNode.NULL_INDEX == mLeftIndex;
  }
}
