package AABBTree;

/**
 * Implements class to compare AABB collisions
 *
 * @author  Dannie Wright
 * @since   6/11/2018
 */

import java.util.*;

public class AABBTree
{
  private static final int DEFAULT_SIZE = 10;

  /**
   * TODO Check if using Integer object is faster/slower than doing wrapper of
   * just class IntegerObject {public int value;}
   */
  private Map<AABBObject, Integer> mcObjectIndexMap;

  /**
   * TODO Determine faster method of doing a dynamic array since ArrayList has
   *  many O(N) computation times
   */
  private ArrayList<AABBNode> mlcNodes;

  private int mRootNodeIndex,
              mNumNodes,
              mMaxNumNodes,
              mGrowthSize,
              mNextFreeNodeIndex;

  /**
   * Initializes tree to default values
   */
  public AABBTree ()
  {
    this.setSize (AABBTree.DEFAULT_SIZE);
  }

  /**
   * Initializes tree based upon initial size
   *
   * @param initialSize - initial number of nodes in the tree, used as
   *                    growth rate as well
   */
  public AABBTree (int initialSize)
  {
    this.setSize (initialSize);
  }

  //removes access to all stored data
  public void setSize (int size)
  {
    mcObjectIndexMap = new HashMap<AABBObject, Integer> (size);
    mlcNodes = new ArrayList<AABBNode> (size);
    mRootNodeIndex = AABBNode.NULL_INDEX;
    mNumNodes = 0;
    //mNextFreeNodeIndex set in this.defaultNodeIndices
    mMaxNumNodes = mGrowthSize = size;

    //set each node's next value in the linked list of available nodes
    this.defaultNodeIndices (0, size);
  }

  private void defaultNodeIndices (int startIndex, int endIndex)
  {
    AABBNode cNode;
    for (int i = startIndex; i < endIndex - 1; ++i)
    {
      cNode = new AABBNode ();
      cNode.mNextNodeIndex = i + 1;
      mlcNodes.add (cNode);
    }
    cNode = new AABBNode ();
    cNode.mNextNodeIndex = AABBNode.NULL_INDEX;
    mlcNodes.add (cNode);

    this.mNextFreeNodeIndex = startIndex;
  }

  /**
   * Allocates new node and returns the index of the new
   * node
   *
   * @return index of new node in mlcNodes
   */
  private int allocateNode ()
  {
    if (mNextFreeNodeIndex == AABBNode.NULL_INDEX)
    {
      AABBNode cNode;

      //ensures no nodes are lost
      assert (mMaxNumNodes == mNumNodes) : "End of incomplete node list";

      //expand capacity
      mMaxNumNodes += mGrowthSize;
      mlcNodes.ensureCapacity (mMaxNumNodes);

      //set each node's next value in the linked list of available node
      this.defaultNodeIndices (mNumNodes, mMaxNumNodes);
    }

    int newIndex = mNextFreeNodeIndex;
    AABBNode cNewNode = mlcNodes.get (newIndex);
    //set relative node indices to null in the case the node is reused
    cNewNode.mParentIndex = cNewNode.mRightIndex
        = cNewNode.mLeftIndex = AABBNode.NULL_INDEX;
    mNextFreeNodeIndex = cNewNode.mNextNodeIndex;
    ++mNumNodes;

    return newIndex;
  }

  /**
   * Deallocates node at given index, allowing node to be used in the future
   *
   * @param index - index of node being deallocated
   */
  private void deallocateNode (int index)
  {
    AABBNode cNode = mlcNodes.get (index);
    cNode.mNextNodeIndex = mNextFreeNodeIndex;

    mNextFreeNodeIndex = index;
    --mNumNodes;
  }

  /**
   * Adds the node at the given index to the tree
   *
   * @param index - index of node to be added to tree
   */
  private void insertLeaf (int index)
  {
    AABBNode cLeaf = mlcNodes.get (index);

    //ensures node is a new leaf, not accessing leaf currently in use
    assert (AABBNode.NULL_INDEX == cLeaf.mParentIndex
         && AABBNode.NULL_INDEX == cLeaf.mLeftIndex
         && AABBNode.NULL_INDEX == cLeaf.mRightIndex)
          : "Node is not a new leaf";

    //if tree is empty new leaf becomes root
    if (AABBNode.NULL_INDEX == mRootNodeIndex)
    {
      mRootNodeIndex = index;
      return;
    }

    //determines best place to add leaf to tree
    int treeIndex = mRootNodeIndex;
    AABBNode cTree = mlcNodes.get (treeIndex);

    //while loop condition ensures only check if need to descend if on
    //a branch
    while (!cTree.isLeaf ())
    {
      int leftIndex = cTree.mLeftIndex,
          rightIndex = cTree.mRightIndex;
      AABBNode cLeftNode = mlcNodes.get (leftIndex),
               cRightNode = mlcNodes.get (rightIndex);
      AABB cCombinedAabb = cTree.mcAABB.merge (cLeaf.mcAABB);

      //cost of insertion is based upon surface area
      //cost of creating new branch to cover current branch and new leaf
      double costHere = 2.0f * cCombinedAabb.mSurfaceArea;
      //minimum cost of inserting leaf further down the tree
      double costPushDownMinimum = 2.0f * (cCombinedAabb.mSurfaceArea
                                    - cTree.mcAABB.mSurfaceArea);

      //calculate cost of descending the tree to branch children
      double costLeft;
      double costRight;
      if (cLeftNode.isLeaf ())
      {
        costLeft = cLeaf.mcAABB.merge (cLeftNode.mcAABB).mSurfaceArea
                    + costPushDownMinimum;
      }
      else
      {
        costLeft = (cLeaf.mcAABB.merge (cLeftNode.mcAABB).mSurfaceArea
                    - cLeftNode.mcAABB.mSurfaceArea) + costPushDownMinimum;
      }
      if (cRightNode.isLeaf ())
      {
        costRight = cLeaf.mcAABB.merge (cRightNode.mcAABB).mSurfaceArea
                    + costPushDownMinimum;
      }
      else
      {
        costRight = (cLeaf.mcAABB.merge (cRightNode.mcAABB).mSurfaceArea
                    - cRightNode.mcAABB.mSurfaceArea) + costPushDownMinimum;
      }

      //if insertion cost is less here than descending, then break to insert
      //leaf here
      if (costHere < costLeft && costHere < costRight)
      {
        break;
      }
      //else descend in cheapest direction
      else
      {
        treeIndex = costLeft < costRight ? leftIndex : rightIndex;
      }
      cTree = mlcNodes.get (treeIndex);
    }

    //make a new branch to attach new leaf and node found above to it,
    //new branch must fully encase new leaf and node found above
    int oldParentIndex = cTree.mParentIndex;
    int newParentIndex = this.allocateNode ();

    AABBNode cNewParent = mlcNodes.get (newParentIndex);
    cNewParent.mcAABB = cLeaf.mcAABB.merge (cTree.mcAABB);
    cNewParent.mParentIndex = oldParentIndex;
    cNewParent.mLeftIndex = treeIndex;
    cNewParent.mRightIndex = index;
    cLeaf.mParentIndex = cTree.mParentIndex = newParentIndex;

    //if old parent was root must make new branch the root
    if (AABBNode.NULL_INDEX == oldParentIndex)
    {
      mRootNodeIndex = newParentIndex;
    }
    //old parent was not root, so must update left or right index values
    else
    {
      AABBNode cOldParent = mlcNodes.get (oldParentIndex);
      if (cOldParent.mLeftIndex == treeIndex)
      {
        cOldParent.mLeftIndex = newParentIndex;
      }
      else
      {
        cOldParent.mRightIndex = newParentIndex;
      }
    }

    //TODO set up balancing for tree

    //walk up tree and fix heights and areas
    this.fixUpwardsTree (newParentIndex);
  }

  /**
   * Removes leaf at given index from the tree
   *
   * @param index - index of leaf being removed from the tree
   */
  private void removeLeaf (int index)
  {
    //if leaf is root
    if (index == mRootNodeIndex)
    {
      mRootNodeIndex = AABBNode.NULL_INDEX;
    }
    else
    {
      AABBNode cDelNode = mlcNodes.get (index);
      int parentIndex = cDelNode.mParentIndex;
      AABBNode cParent = mlcNodes.get (parentIndex);
      int siblingIndex = cParent.mLeftIndex == index
          ? cParent.mRightIndex : cParent.mLeftIndex;

      //ensure there exists a sibling node
      assert (AABBNode.NULL_INDEX != siblingIndex)
          : "Sibling index should not be null";
      AABBNode cSibling = mlcNodes.get (siblingIndex);

      //if leaf's parent is root
      if (parentIndex == mRootNodeIndex)
      {
        mRootNodeIndex = siblingIndex;
        cSibling.mParentIndex = AABBNode.NULL_INDEX;
        this.deallocateNode (parentIndex);
      }
      else
      {
        int grandParentIndex = cParent.mParentIndex;
        AABBNode cGrandParent = mlcNodes.get (grandParentIndex);

        cSibling.mParentIndex = grandParentIndex;
        if (cGrandParent.mLeftIndex == parentIndex)
        {
          cGrandParent.mLeftIndex = siblingIndex;
        }
        else
        {
          cGrandParent.mRightIndex = siblingIndex;
        }

        this.deallocateNode (parentIndex);
        this.fixUpwardsTree (grandParentIndex);
      }

    }

    this.deallocateNode (index);
  }

  /**
   * Ensures all ancestral branches from given index, including given index,
   * still contain all child nodes
   *
   * @param index - index of branch to start ensuring AABB covers all children
   */
  private void fixUpwardsTree (int index)
  {
    while (AABBNode.NULL_INDEX != index)
    {
      AABBNode cNode = mlcNodes.get (index);

      //each node should be a branch
      assert (AABBNode.NULL_INDEX != cNode.mRightIndex
          && AABBNode.NULL_INDEX != cNode.mLeftIndex)
          : "Parent node must be a branch";

      //ensure branch fully encases all subtrees
      AABBNode cLeftNode = mlcNodes.get (cNode.mLeftIndex);
      AABBNode cRightNode = mlcNodes.get (cNode.mRightIndex);
      cNode.mcAABB = cLeftNode.mcAABB.merge (cRightNode.mcAABB);

      //increment to parent branch
      index = cNode.mParentIndex;
    }
  }

  /**
   * Updates the AABB of the leaf at the given index
   *
   * @param index - index of the leaf being updated
   * @param cNewAabb - new AABB of the leaf
   */
  private void updateLeaf (int index, AABB cNewAabb)
  {
    AABBNode cNode = mlcNodes.get (index);

    /*
    TODO add additional AABB to nodes to take into account velocity, only
    update if the main AABB is outside of the velocity AABB
     */

    //if the node still fully contains the new AABB, then do not update
    if (cNode.mcAABB.encases (cNewAabb))
      return;

    this.removeLeaf (index);
    cNode.mcAABB = cNewAabb;

    //TODO maybe do quick allocate, had to change this to allocateNode because in insert leaf allocateNode is called which grabs index, so basically allocating index TWICE
    this.insertLeaf (this.allocateNode ());
    //this.insertLeaf (index);

  }

  /**
   * Adds object to member data
   *
   * @param cObject - object being added
   */
  public void insertObject (AABBObject cObject)
  {
    //if object is already in tree return
    if (null != mcObjectIndexMap.get (cObject))
    {
      return;
    }

    int index = this.allocateNode ();
    AABBNode cNode = mlcNodes.get (index);

    cNode.mcAABB = cObject.mcAABB;
    cNode.mcObject = cObject;
    this.insertLeaf (index);
    mcObjectIndexMap.put (cObject, index);
  }

  /**
   * Removes the given object from member data
   *
   * @param cObject - object being removed
   */
  public void removeObject (AABBObject cObject)
  {
    Integer index = mcObjectIndexMap.get (cObject);

    //if object is in tree, remove object
    if (null != index)
    {
      this.removeLeaf (index);
      mcObjectIndexMap.remove (cObject);
    }
  }

  /**
   * Updates the given object's related AABBNode
   *
   * @param cObject - object whose related node is having its AABB updated
   */
  public void updateObject (AABBObject cObject)
  {
    Integer index = mcObjectIndexMap.get (cObject);

    //if object is in tree, update leaf
    if (null != index)
    {
      this.updateLeaf (index, cObject.mcAABB);
    }
  }

  /**
   * Returns AABBObjects that overlap with the given object
   *
   * @param cObject - object having overlaps compared
   *
   * @return list of objects that overlap with the given object
   */
  public ArrayList<AABBObject> queryOverlaps (AABBObject cObject)
  {
    //if object is not in tree return null
    if (null == mcObjectIndexMap.get (cObject))
    {
      return null;
    }

    ArrayList<AABBObject> clOverlaps = new ArrayList<> ();
    Stack<Integer> cStack = new Stack<> ();
    AABB cAabb = cObject.mcAABB;
    Integer cIndex = mRootNodeIndex;
    int index;
    AABBNode cNode;

    cStack.push (cIndex);

    while (!cStack.empty ())
    {
      cIndex = cStack.pop ();
      index = cIndex;

      if (AABBNode.NULL_INDEX == index)
        continue;

      cNode = mlcNodes.get (index);
      if (cNode.mcAABB.overlap (cAabb))
      {
        if (cNode.isLeaf () && cNode.mcObject != cObject)
        {
          clOverlaps.add (cNode.mcObject);
        }
        else
        {
          cStack.push (cNode.mLeftIndex);
          cStack.push (cNode.mRightIndex);
        }
      }
    }

    return clOverlaps;
  }

  /**
   * Returns the number of nodes in the tree
   *
   * @return number of nodes in the tree
   */
  public int getNumNodes ()
  {
    return mNumNodes;
  }

  /**
   * Returns a copy of the node at the given index
   *
   * @param index - index of node being returned
   *
   * @return a copy of the node at the given index
   */
  public AABBNode getNode (int index)
  {
    return new AABBNode (mlcNodes.get (index));
  }

  /**
   * Returns a copy of the root node
   *
   * @return a copy of the root node
   */
  public AABBNode getRoot ()
  {
    return this.getNode (mRootNodeIndex);
  }



  public void empty ()
  {
    mcObjectIndexMap.clear ();

    for (int i = 0; i < mMaxNumNodes - 1; ++i)
    {
      mlcNodes.get (i).mNextNodeIndex = i + 1;
    }
    mlcNodes.get (mMaxNumNodes - 1).mNextNodeIndex = AABBNode.NULL_INDEX;

    mRootNodeIndex = AABBNode.NULL_INDEX;
    mNumNodes = 0;
    mNextFreeNodeIndex = 0;
  }

  public boolean hasObject (AABBObject cObject)
  {
    return mcObjectIndexMap.containsKey (cObject);
  }
}
