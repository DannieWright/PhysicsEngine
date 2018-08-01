package AABBTree;

import java.util.ArrayList;

import static org.junit.Assert.*;

public class AABBTreeTest
{
  public static void main (String[] args)
  {
    AABBTreeTest cTest = new AABBTreeTest ();
    cTest.testInsertObject ();
    cTest.testRemoveObject ();
    cTest.testQueryOverlaps ();
  }
  public void testInsertObject ()
  {
    AABBTree cTree = new AABBTree (10);
    AABB cAABB1 = new AABB (5, 10, 5, 10),
        cAABB2 = new AABB (1, 4, 1, 4),
        cAABB3 = new AABB (6,9,6,9),
        cAABBRoot,
        cAABBRootNew;

    AABBObject cAABBObj1 = new AABBObject (cAABB1),
        cAABBObj2 = new AABBObject (cAABB2),
        cAABBObj3 = new AABBObject (cAABB3);

    assertEquals ("tree size = 0", 0, cTree.getNumNodes ());

    //should add new AABB as root
    cTree.insertObject (cAABBObj1);
    assertEquals ("tree size = 1", 1, cTree.getNumNodes ());
    cAABBRoot = cTree.getRoot ().mcAABB;
    assertTrue (String.format ("%s = root AABB", cAABB1.toString ()),
        cAABB1.equals (cAABBRoot));

    //new AABB and old AABB do not overlap and should be made leaves of a new
    //AABB which will be the new root. The new AABB should be the merge of the
    //ones added here.
    cTree.insertObject (cAABBObj2);
    assertEquals ("tree size = 3", 3, cTree.getNumNodes ());
    cAABBRoot = cTree.getRoot ().mcAABB;
    assertTrue (String.format ("root %s = %s merge %s", cAABBRoot, cAABB1,
        cAABB2), cAABBRoot.equals (cAABB1.merge (cAABB2)));

    //new AABB will be encased by one of the older leaves, so a branch must be
    //added and the encasing leaf and the new one will be made children of a
    //new branch
    //root should remain the same
    cTree.insertObject (cAABBObj3);
    assertEquals ("tree size = 5", 5, cTree.getNumNodes ());
    cAABBRootNew = cTree.getRoot ().mcAABB;
    assertTrue (String.format ("%s equals %s", cAABBRoot.toString (),
        cAABBRootNew.toString ()), cAABBRoot.equals (cAABBRootNew));

  }

  public void testRemoveObject ()
  {
    AABBTree cTree = new AABBTree (10);
    AABB cAABB1 = new AABB (5, 10, 5, 10),
        cAABB2 = new AABB (1, 4, 1, 4),
        cAABB3 = new AABB (6,9,6,9);

    AABBObject cAABBObj1 = new AABBObject (cAABB1),
        cAABBObj2 = new AABBObject (cAABB2),
        cAABBObj3 = new AABBObject (cAABB3);

    assertEquals ("tree size = 0", 0, cTree.getNumNodes ());

    //should add new AABB as root
    cTree.insertObject (cAABBObj1);
    cTree.removeObject (cAABBObj1);
    assertEquals ("tree size = 0", 0, cTree.getNumNodes ());

    cTree.insertObject (cAABBObj1);
    cTree.insertObject (cAABBObj2);
    cTree.removeObject (cAABBObj2);
    assertEquals ("tree size = 1", 1, cTree.getNumNodes ());

    cTree.insertObject (cAABBObj2);
    cTree.insertObject (cAABBObj3);
    cTree.removeObject (cAABBObj3);
    assertEquals ("tree size = 3", 3, cTree.getNumNodes ());

    cTree.insertObject (cAABBObj3);
    cTree.removeObject (cAABBObj2);
    assertEquals ("tree size = 3", 3, cTree.getNumNodes ());
  }

  public void testQueryOverlaps ()
  {
    AABBTree cTree = new AABBTree (10);
    AABB cAABB1 = new AABB (5, 10, 5, 10),
        cAABB2 = new AABB (1, 4, 1, 4),
        cAABB3 = new AABB (6,9,6,9),
        cAABB4;

    AABBObject cAABBObj1 = new AABBObject (cAABB1),
        cAABBObj2 = new AABBObject (cAABB2),
        cAABBObj3 = new AABBObject (cAABB3),
        cAABBObj4;

    cTree.insertObject (cAABBObj1);
    cTree.insertObject (cAABBObj2);
    cTree.insertObject (cAABBObj3);

    ArrayList<AABBObject> clOverlaps1 = cTree.queryOverlaps (cAABBObj1),
        clOverlaps2 = cTree.queryOverlaps (cAABBObj2),
        clOverlaps3 = cTree.queryOverlaps (cAABBObj3),
        clOverlaps4;

    //cAABB1 should overlap cAABB3 only
    assertEquals (String.format ("%s has 1 overlap", cAABB1.toString ()),
        1, clOverlaps1.size ());
    assertTrue (String.format ("%s overlaps %s", cAABB1.toString (),
        cAABB3.toString ()), clOverlaps1.get (0).mcAABB.equals (cAABB3));

    //cAABB3 should overlap cAABB1 only
    assertEquals (String.format ("%s has 1 overlap", cAABB3.toString ()),
        1, clOverlaps3.size ());
    assertTrue (String.format ("%s overlaps %s", cAABB3.toString (),
        cAABB1.toString ()), clOverlaps3.get (0).mcAABB.equals (cAABB1));

    //cAABB2 should have no overlaps
    assertEquals (String.format ("%s has 0 overlap", cAABB2.toString ()),
        0, clOverlaps2.size ());


    cAABB4 = new AABB (5,12,0,4);
    cAABBObj4 = new AABBObject (cAABB4);
    cTree.insertObject (cAABBObj4);

    clOverlaps1 = cTree.queryOverlaps (cAABBObj1);
    clOverlaps2 = cTree.queryOverlaps (cAABBObj2);
    clOverlaps3 = cTree.queryOverlaps (cAABBObj3);
    clOverlaps4 = cTree.queryOverlaps (cAABBObj4);

    //cAABB1 should overlap cAABB3 only
    assertEquals (String.format ("%s has 1 overlap", cAABB1.toString ()),
        1, clOverlaps1.size ());
    assertTrue (String.format ("%s overlaps %s", cAABB1.toString (),
        cAABB3.toString ()), clOverlaps1.get (0).mcAABB.equals (cAABB3));

    //cAABB3 should overlap cAABB1 only
    assertEquals (String.format ("%s has 1 overlap", cAABB3.toString ()),
        1, clOverlaps3.size ());
    assertTrue (String.format ("%s overlaps %s", cAABB3.toString (),
        cAABB1.toString ()), clOverlaps3.get (0).mcAABB.equals (cAABB1));

    //cAABB2 should have no overlaps
    assertEquals (String.format ("%s has 0 overlap", cAABB2.toString ()),
        0, clOverlaps2.size ());

    //cAABB4 should have no overlaps
    assertEquals (String.format ("%s has 0 overlap", cAABB4.toString ()),
        0, clOverlaps4.size ());


    cTree.removeObject (cAABBObj1);
    cTree.removeObject (cAABBObj2);
    cTree.removeObject (cAABBObj3);
    cTree.removeObject (cAABBObj4);

    cAABBObj1.mcAABB = new AABB (0, 10, 0, 10);
    cAABBObj2.mcAABB = new AABB (-2, 2, -2, 2);
    cAABBObj3.mcAABB = new AABB (-1, 1, 6, 8);

    cTree.insertObject (cAABBObj1);
    cTree.insertObject (cAABBObj2);
    cTree.insertObject (cAABBObj3);

    clOverlaps1 = cTree.queryOverlaps (cAABBObj1);
    clOverlaps2 = cTree.queryOverlaps (cAABBObj2);
    clOverlaps3 = cTree.queryOverlaps (cAABBObj3);

    System.out.println (clOverlaps1.toString ());
    System.out.println (clOverlaps2.toString ());
    System.out.println (clOverlaps3.toString ());

  }
}