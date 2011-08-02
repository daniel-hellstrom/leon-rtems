/*
 * Copyright (c) 2010 Gedare Bloom.
 *
 *  The license and distribution terms for this file may be
 *  found in the file LICENSE in this distribution or at
 *  http://www.rtems.com/license/LICENSE.
 *
 *  $Id$
 */

#include <tmacros.h>
#include <rtems/rbtree.h>

int numbers[20] = {
52, 99, 0, 85, 43, 44, 10, 60, 50, 19, 8, 68, 48, 57, 17, 67, 90, 12, 77, 71};

int numbers_sorted[20] = {
  0, 8, 10, 12, 17, 19, 43, 44, 48, 50, 52, 57, 60, 67, 68, 71, 77, 85, 90, 99};

typedef struct {
  int              id;
  rtems_rbtree_node Node;
} test_node;

/* 
 * recursively checks tree. if the tree is built properly it should only 
 * be a depth of 7 function calls for 100 entries in the tree. 
 */
int rb_assert ( rtems_rbtree_node *root )
{
  int lh, rh;

  if ( root == NULL )
    return 1;
  else {
    rtems_rbtree_node *ln = rtems_rbtree_left(root);
    rtems_rbtree_node *rn = rtems_rbtree_right(root);

    /* Consecutive red links */
    if ( root->color == RBT_RED ) {
      if ((ln && ln->color == RBT_RED)  || (rn && rn->color == RBT_RED)) {
        puts ( "Red violation" );
        return -1;
      }
    }

      lh = rb_assert ( ln );
      rh = rb_assert ( rn );

    /* Invalid binary search tree */
    if ( ( ln != NULL && ln->value >= root->value )
        || ( rn != NULL && rn->value <= root->value ) )
    {
      puts ( "Binary tree violation" );
      return -1;
    }

    /* Black height mismatch */
    if ( lh != -1 && rh != -1 && lh != rh ) {
      puts ( "Black violation" );
      return -1;
    }

    /* Only count black links */
    if ( lh != -1 && rh != -1 )
      return ( root->color == RBT_RED ) ? lh : lh + 1;
    else
      return -1;
  }
}



rtems_task Init(
    rtems_task_argument ignored
    )
{
  rtems_rbtree_control  rbtree1;
  rtems_rbtree_node    *p;
  test_node            node1, node2;
  test_node            node_array[100];
  int                  id;
  int i;

  puts( "\n\n*** TEST OF RTEMS RBTREE API ***" );

  puts( "Init - Initialize rbtree empty" );
  rtems_rbtree_initialize_empty( &rbtree1 );
  
  /* verify that the rbtree insert work */
  puts( "INIT - Verify rtems_rbtree_insert with two nodes" );
  node1.id = 1;
  node1.Node.value = 1;
  node2.id = 2;
  node2.Node.value = 2;
  rtems_rbtree_insert( &rbtree1, &node1.Node );
  rtems_rbtree_insert( &rbtree1, &node2.Node );

  p = _RBTree_Insert_unprotected( &rbtree1, NULL );
  if (p != (void *)(-1))
    puts( "INIT - FAILED NULL NODE INSERT" );

  _RBTree_Rotate(NULL, RBT_LEFT);
  i = (node1.Node.parent == &node2.Node);
  _RBTree_Rotate( &node1.Node,
                  !node1.Node.child[RBT_LEFT] ? RBT_RIGHT : RBT_LEFT
                );
  if ( (node1.Node.parent == &node2.Node) != i )
    puts( "INIT - FAILED FALSE ROTATION" );

  if (!rb_assert(rbtree1.root) )
    puts( "INIT - FAILED TREE CHECK" );

  for ( p = rtems_rbtree_get_min(&rbtree1), id = 1 ; p ;
      p = rtems_rbtree_get_min(&rbtree1) , id++ ) {
    test_node *t = rtems_rbtree_container_of(p,test_node,Node);
    if ( id > 2 ) {
      puts( "INIT - TOO MANY NODES ON RBTREE" );
      rtems_test_exit(0);
    }
    if ( t->id != id ) {
      puts( "INIT - ERROR ON RBTREE ID MISMATCH" );
      rtems_test_exit(0);
    }

    if (!rb_assert(rbtree1.root) )
      puts( "INIT - FAILED TREE CHECK" );
  }
  if (id < 2) {
    puts("INIT - NOT ENOUGH NODES ON RBTREE");
    rtems_test_exit(0);
  }

  puts("INIT - Verify rtems_rbtree_insert with the same value twice");
  node2.Node.value = node1.Node.value;
  rtems_rbtree_insert(&rbtree1, &node1.Node);
  rtems_rbtree_insert(&rbtree1, &node2.Node);
  for ( p = rtems_rbtree_get_min(&rbtree1), id = 1 ; p ;
      p = rtems_rbtree_get_min(&rbtree1) , id++ ) {
    test_node *t = rtems_rbtree_container_of(p,test_node,Node);
    if ( id > 1 ) {
      puts( "INIT - TOO MANY NODES ON RBTREE" );
      rtems_test_exit(0);
    }
    if ( t->id != id ) {
      puts( "INIT - ERROR ON RBTREE ID MISMATCH" );
      rtems_test_exit(0);
    }

    if (!rb_assert(rbtree1.root) )
      puts( "INIT - FAILED TREE CHECK" );
  }
  if (id < 1) {
    puts("INIT - NOT ENOUGH NODES ON RBTREE");
    rtems_test_exit(0);
  }
  node2.Node.value = 2;

  /* verify that the rbtree is empty */
  puts( "INIT - Verify rtems_rbtree_is_empty" );
  if(!rtems_rbtree_is_empty(&rbtree1)) {
    puts( "INIT - TREE NOT EMPTY" );
    rtems_test_exit(0);
  }

  puts( "INIT - Verify rtems_XXX on an empty tree" );
  if(rtems_rbtree_get_min(&rbtree1)) {
    puts("INIT - get_min on empty returned non-NULL");
    rtems_test_exit(0);
  }
  if(rtems_rbtree_get_max(&rbtree1)) {
    puts("INIT - get_max on empty returned non-NULL");
    rtems_test_exit(0);
  }
  if(rtems_rbtree_peek_min(&rbtree1)) {
    puts("INIT - peek_min on empty returned non-NULL");
    rtems_test_exit(0);
  }
  if(rtems_rbtree_peek_max(&rbtree1)) {
    puts("INIT - peek_max on empty returned non-NULL");
    rtems_test_exit(0);
  }


  /* verify that the rbtree insert works after a tree is emptied */
  puts( "INIT - Verify rtems_rbtree_insert after empty tree" );
  node1.id = 2;
  node1.Node.value = 2;
  node2.id = 1;
  node2.Node.value = 1;
  rtems_rbtree_insert( &rbtree1, &node1.Node );
  rtems_rbtree_insert( &rbtree1, &node2.Node );

  puts( "INIT - Verify rtems_rbtree_peek_max/min, rtems_rbtree_extract" );
  if(rtems_rbtree_peek_max(&rbtree1)->value - 
      rtems_rbtree_peek_min(&rbtree1)->value != 1) {
    puts( "INIT - Peek Min - Max failed" );
    rtems_test_exit(0);
  }
  p = rtems_rbtree_peek_max(&rbtree1);
  rtems_rbtree_extract(&rbtree1, p);
  if (p->value != 2) {
    puts( "INIT - rtems_rbtree_extract failed");
    rtems_test_exit(0);
  }
  rtems_rbtree_insert(&rbtree1, p);

  for ( p = rtems_rbtree_get_min(&rbtree1), id = 1 ; p ;
      p = rtems_rbtree_get_min(&rbtree1) , id++ ) {
    test_node *t = rtems_rbtree_container_of(p,test_node,Node);
    if ( id > 2 ) {
      puts( "INIT - TOO MANY NODES ON RBTREE" );
      rtems_test_exit(0);
    }
    if ( t->id != id ) {
      puts( "INIT - ERROR ON RBTREE ID MISMATCH" );
      rtems_test_exit(0);
    }
  }
  
  puts( "INIT - Verify rtems_rbtree_insert with 100 nodes value [0,99]" );
  for (i = 0; i < 100; i++) {
    node_array[i].id = i;
    node_array[i].Node.value = i;
    rtems_rbtree_insert( &rbtree1, &node_array[i].Node );

    if (!rb_assert(rbtree1.root) )
      puts( "INIT - FAILED TREE CHECK" );
  }

  puts( "INIT - Removing 100 nodes" );

  for ( p = rtems_rbtree_get_min(&rbtree1), id = 0 ; p ;
      p = rtems_rbtree_get_min(&rbtree1) , id++ ) {
    test_node *t = rtems_rbtree_container_of(p,test_node,Node);
    if ( id > 99 ) {
      puts( "INIT - TOO MANY NODES ON RBTREE" );
      rtems_test_exit(0);
    }
    if ( t->id != id ) {
      puts( "INIT - ERROR ON RBTREE ID MISMATCH" );
      rtems_test_exit(0);
    }

    if (!rb_assert(rbtree1.root) )
      puts( "INIT - FAILED TREE CHECK" );
  }
  
  if(!rtems_rbtree_is_empty(&rbtree1)) {
    puts( "INIT - TREE NOT EMPTY" );
    rtems_test_exit(0);
  }

  puts( "INIT - Verify rtems_rbtree_insert with 100 nodes value [99,0]" );
  for (i = 0; i < 100; i++) {
    node_array[i].id = 99-i;
    node_array[i].Node.value = 99-i;
    rtems_rbtree_insert( &rbtree1, &node_array[i].Node );

    if (!rb_assert(rbtree1.root) )
      puts( "INIT - FAILED TREE CHECK" );
  }

  puts( "INIT - Removing 100 nodes" );

  for ( p = rtems_rbtree_get_min(&rbtree1), id = 0 ; p ;
      p = rtems_rbtree_get_min(&rbtree1) , id++ ) {
    test_node *t = rtems_rbtree_container_of(p,test_node,Node);
    if ( id > 99 ) {
      puts( "INIT - TOO MANY NODES ON RBTREE" );
      rtems_test_exit(0);
    }
    if ( t->id != id ) {
      puts( "INIT - ERROR ON RBTREE ID MISMATCH" );
      rtems_test_exit(0);
    }

    if (!rb_assert(rbtree1.root) )
      puts( "INIT - FAILED TREE CHECK" );
  }

  if(!rtems_rbtree_is_empty(&rbtree1)) {
    puts( "INIT - TREE NOT EMPTY" );
    rtems_test_exit(0);
  }

  /* testing rbtree_extract by adding 100 nodes then removing the 20 with
   * keys specified by the numbers array, then removing the rest */
  puts( "INIT - Verify rtems_rbtree_extract with 100 nodes value [0,99]" );
  for (i = 0; i < 100; i++) {
    node_array[i].id = i;
    node_array[i].Node.value = i;
    rtems_rbtree_insert( &rbtree1, &node_array[i].Node );

    if (!rb_assert(rbtree1.root) )
      puts( "INIT - FAILED TREE CHECK" );
  }

  puts( "INIT - Extracting 20 random nodes" );

  for (i = 0; i < 20; i++) {
    id = numbers[i];
    rtems_rbtree_extract( &rbtree1, &node_array[id].Node );
    if (!rb_assert(rbtree1.root) )
      puts( "INIT - FAILED TREE CHECK" );
  }

  puts( "INIT - Removing 80 nodes" );

  for ( p = rtems_rbtree_get_min(&rbtree1), id = 0, i = 0 ; p ;
      p = rtems_rbtree_get_min(&rbtree1) , id++ ) {
    test_node *t = rtems_rbtree_container_of(p, test_node, Node);

    while ( id == numbers_sorted[i] ) {
      /* skip if expected minimum (id) is in the set of extracted numbers */
      id++;
      i++;
    }

    if ( id > 99 ) {
      puts( "INIT - TOO MANY NODES ON RBTREE" );
      rtems_test_exit(0);
    }

    if ( t->id != id ) {
      puts( "INIT - ERROR ON RBTREE ID MISMATCH" );
      rtems_test_exit(0);
    }

    if (!rb_assert(rbtree1.root) )
      puts( "INIT - FAILED TREE CHECK" );
  }

  if(!rtems_rbtree_is_empty(&rbtree1)) {
    puts( "INIT - TREE NOT EMPTY" );
    rtems_test_exit(0);
  }

  /* Additional rtems_rbtree_extract test which removes a node
   * with two children while target node has a left child only. */
  for ( i = 0; i < 7; i++ ) {
    node_array[i].id = i;
    node_array[i].Node.value = i;
  }
  rtems_rbtree_insert( &rbtree1, &node_array[3].Node );
  rtems_rbtree_insert( &rbtree1, &node_array[1].Node );
  rtems_rbtree_insert( &rbtree1, &node_array[5].Node );
  rtems_rbtree_insert( &rbtree1, &node_array[0].Node );
  rtems_rbtree_insert( &rbtree1, &node_array[2].Node );
  rtems_rbtree_insert( &rbtree1, &node_array[4].Node );
  rtems_rbtree_insert( &rbtree1, &node_array[6].Node );
  rtems_rbtree_extract( &rbtree1, &node_array[2].Node );
  /* node_array[1] has now only a left child. */
  if ( !node_array[1].Node.child[RBT_LEFT] ||
        node_array[1].Node.child[RBT_RIGHT] )
     puts( "INIT - LEFT CHILD ONLY NOT FOUND" );
  rtems_rbtree_extract( &rbtree1, &node_array[3].Node );
  while( (p = rtems_rbtree_get_max(&rbtree1)) );

  puts( "INIT - Verify rtems_rbtree_get_max with 100 nodes value [99,0]" );
  for (i = 0; i < 100; i++) {
    node_array[i].id = 99-i;
    node_array[i].Node.value = 99-i;
    rtems_rbtree_insert( &rbtree1, &node_array[i].Node );

    if (!rb_assert(rbtree1.root) )
      puts( "INIT - FAILED TREE CHECK" );
  }

  puts( "INIT - Removing 100 nodes" );

  for ( p = rtems_rbtree_get_max(&rbtree1), id = 0 ; p ;
      p = rtems_rbtree_get_max(&rbtree1) , id++ ) {
    test_node *t = rtems_rbtree_container_of(p,test_node,Node);
    if ( id > 99 ) {
      puts( "INIT - TOO MANY NODES ON RBTREE" );
      rtems_test_exit(0);
    }
    if ( t->id != 99-id ) {
      puts( "INIT - ERROR ON RBTREE ID MISMATCH" );
      rtems_test_exit(0);
    }

    if (!rb_assert(rbtree1.root) )
      puts( "INIT - FAILED TREE CHECK" );
  }

  if(!rtems_rbtree_is_empty(&rbtree1)) {
    puts( "INIT - TREE NOT EMPTY" );
    rtems_test_exit(0);
  }

  puts( "INIT - Verify rtems_rbtree_get_max with 100 nodes value [0,99]" );
  for (i = 0; i < 100; i++) {
    node_array[i].id = i;
    node_array[i].Node.value = i;
    rtems_rbtree_insert( &rbtree1, &node_array[i].Node );

    if (!rb_assert(rbtree1.root) )
      puts( "INIT - FAILED TREE CHECK" );
  }

  puts( "INIT - Verify rtems_rbtree_find" );
  p = rtems_rbtree_find(&rbtree1, 30);
  if(rtems_rbtree_container_of(p,test_node,Node)->id != 30) {
    puts ("INIT - ERROR ON RBTREE ID MISMATCH");
    rtems_test_exit(0);
  }

  puts( "INIT - Verify rtems_rbtree_predecessor/successor");
  p = rtems_rbtree_predecessor(p);
  if(p && rtems_rbtree_container_of(p,test_node,Node)->id > 30) {
    puts ("INIT - ERROR ON RBTREE ID MISMATCH");
    rtems_test_exit(0);
  }
  p = rtems_rbtree_find(&rbtree1, 30);
  p = rtems_rbtree_successor(p);
  if(p && rtems_rbtree_container_of(p,test_node,Node)->id < 30) {
    puts ("INIT - ERROR ON RBTREE ID MISMATCH");
    rtems_test_exit(0);
  }

  p = rtems_rbtree_find(&rbtree1, 30);
  puts( "INIT - Verify rtems_rbtree_find_header" );
  if (rtems_rbtree_find_header(p) != &rbtree1) {
    puts ("INIT - ERROR ON RBTREE HEADER MISMATCH");
    rtems_test_exit(0);
  }

  if ( _RBTree_Sibling( NULL ) != NULL )
    puts ( "INIT - ERROR ON RBTREE NULL SIBLING MISMATCH" );
  if ( _RBTree_Sibling( rbtree1.root ) != NULL )
    puts ( "INIT - ERROR ON RBTREE NULL SIBLING MISMATCH" );
  if ( _RBTree_Grandparent( NULL ) != NULL )
    puts ( "INIT - ERROR ON RBTREE NULL GRANDPARENT MISMATCH" );

  puts( "INIT - Removing 100 nodes" );

  for ( p = rtems_rbtree_get_max(&rbtree1), id = 99 ; p ;
      p = rtems_rbtree_get_max(&rbtree1) , id-- ) {
    test_node *t = rtems_rbtree_container_of(p,test_node,Node);
    if ( id < 0 ) {
      puts( "INIT - TOO MANY NODES ON RBTREE" );
      rtems_test_exit(0);
    }
    if ( t->id != id ) {
      puts( "INIT - ERROR ON RBTREE ID MISMATCH" );
      rtems_test_exit(0);
    }

    if (!rb_assert(rbtree1.root) )
      puts( "INIT - FAILED TREE CHECK" );
  }

  if(!rtems_rbtree_is_empty(&rbtree1)) {
    puts( "INIT - TREE NOT EMPTY" );
    rtems_test_exit(0);
  }

  if (rtems_rbtree_find_header(p) != NULL) {
    puts ("INIT - ERROR ON RBTREE HEADER MISMATCH");
    rtems_test_exit(0);
  }
  if (rtems_rbtree_find_header(NULL) != NULL) {
    puts ("INIT - ERROR ON RBTREE HEADER MISMATCH");
    rtems_test_exit(0);
  }

  puts("INIT - Insert 20 random numbers");
  for (i = 0; i < 20; i++) {
    node_array[i].id = numbers[i];
    node_array[i].Node.value = numbers[i];
    rtems_rbtree_insert( &rbtree1, &node_array[i].Node );

    if (!rb_assert(rbtree1.root) )
      puts( "INIT - FAILED TREE CHECK" );
  }

  puts( "INIT - Removing 20 nodes" );

  for ( p = rtems_rbtree_get_min(&rbtree1), id = 0 ; p ;
      p = rtems_rbtree_get_min(&rbtree1) , id++ ) {
    test_node *t = rtems_rbtree_container_of(p,test_node,Node);
    if ( id > 19 ) {
      puts( "INIT - TOO MANY NODES ON RBTREE" );
      rtems_test_exit(0);
    }
    if ( t->id != numbers_sorted[id] ) {
      puts( "INIT - ERROR ON RBTREE ID MISMATCH" );
      rtems_test_exit(0);
    }

    if (!rb_assert(rbtree1.root) )
      puts( "INIT - FAILED TREE CHECK" );
  }

  if(!rtems_rbtree_is_empty(&rbtree1)) {
    puts( "INIT - TREE NOT EMPTY" );
    rtems_test_exit(0);
  }

  puts( "INIT - Verify rtems_rbtree_initialize with 100 nodes value [0,99]" );
  for (i = 0; i < 100; i++) {
    node_array[i].id = i;
    node_array[i].Node.value = i;
  }
  rtems_rbtree_initialize( &rbtree1, &node_array[0].Node, 100,
                           sizeof(test_node));

  puts( "INIT - Removing 100 nodes" );

  for ( p = rtems_rbtree_get_min(&rbtree1), id = 0 ; p ;
      p = rtems_rbtree_get_min(&rbtree1) , id++ ) {
    test_node *t = rtems_rbtree_container_of(p,test_node,Node);
    if ( id > 99 ) {
      puts( "INIT - TOO MANY NODES ON RBTREE" );
      rtems_test_exit(0);
    }

    if ( t->id != id ) {
      puts( "INIT - ERROR ON RBTREE ID MISMATCH" );
      rtems_test_exit(0);
    }

    if (!rb_assert(rbtree1.root) )
      puts( "INIT - FAILED TREE CHECK" );
  }

  if(!rtems_rbtree_is_empty(&rbtree1)) {
    puts( "INIT - TREE NOT EMPTY" );
    rtems_test_exit(0);
  }

  puts( "*** END OF RTEMS RBTREE API TEST ***" );
  rtems_test_exit(0);
}

/* configuration information */

#define CONFIGURE_APPLICATION_NEEDS_CONSOLE_DRIVER
#define CONFIGURE_APPLICATION_DOES_NOT_NEED_CLOCK_DRIVER

#define CONFIGURE_RTEMS_INIT_TASKS_TABLE
#define CONFIGURE_MAXIMUM_TASKS 1

#define CONFIGURE_INIT
#include <rtems/confdefs.h>

/* global variables */