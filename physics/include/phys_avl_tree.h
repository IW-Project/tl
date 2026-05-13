#pragma once

#include <tl_system.h>

template <typename T>
class phys_inplace_avl_tree_node
{
public:
  T *m_left;
  T *m_right;
  int m_balance;
  inline void init()
  {
    m_left = NULL;
    m_right = NULL;
    m_balance = 0;
  }
};

template <typename key_type, typename data_type, typename accessor>
class phys_inplace_avl_tree
{
private:
  data_type *m_tree_root;
  int avl_max( const int a, const int b )
  {
    if ( a <= b )
      return b;
    else
      return a;
  }
  inline void rotate_right( data_type *&root )
  {
    data_type *save_left = accessor::get_avl_node( root )->m_left;
    accessor::get_avl_node( root )->m_left = accessor::get_avl_node( save_left )->m_right;
    accessor::get_avl_node( save_left )->m_right = root;
    accessor::get_avl_node( root )->m_balance = avl_max( accessor::get_avl_node( accessor::get_avl_node( root )->m_left )->m_balance,
                                                         accessor::get_avl_node( accessor::get_avl_node( root )->m_right )->m_balance ) +
                                                1;
    accessor::get_avl_node( save_left )->m_balance =
        avl_max( accessor::get_avl_node( accessor::get_avl_node( save_left )->m_left )->m_balance,
                 accessor::get_avl_node( accessor::get_avl_node( save_left )->m_right )->m_balance ) +
        1;
    root = save_left;
  }
  void rotate_left( data_type *&root )
  {
    data_type *save_right = accessor::get_avl_node( root )->m_right;
    accessor::get_avl_node( root )->m_right = accessor::get_avl_node( save_right )->m_left;
    accessor::get_avl_node( save_right )->m_left = root;
    accessor::get_avl_node( root )->m_balance = avl_max( accessor::get_avl_node( accessor::get_avl_node( root )->m_left )->m_balance,
                                                         accessor::get_avl_node( accessor::get_avl_node( root )->m_right )->m_balance ) +
                                                1;
    accessor::get_avl_node( save_right )->m_balance =
        avl_max( accessor::get_avl_node( accessor::get_avl_node( save_right )->m_left )->m_balance,
                 accessor::get_avl_node( accessor::get_avl_node( save_right )->m_right )->m_balance ) +
        1;
    root = save_right;
  }

public:
  struct stack_item
  {
    data_type **m_node;
    int m_child;
  };
  phys_inplace_avl_tree() { remove_all(); }
  void remove_all() { m_tree_root = NULL; }
  data_type *get_root() { return m_tree_root; }
  data_type *find( const key_type &key )
  {
    // Start from the root
    data_type *cur = m_tree_root;

    // Search the tree
    while ( cur )
    {
      // Compare keys
      const key_type &nodeKey = accessor::get_avl_key( cur );

      if ( key == nodeKey )
      {
        // Found the node
        break;
      }

      // Navigate left or right based on key comparison
      if ( key >= nodeKey )
      {
        cur = accessor::get_avl_node( cur )->m_right;
      }
      else
      {
        cur = accessor::get_avl_node( cur )->m_left;
      }
    }

    return cur;
  }
  void add( const key_type &key, data_type *data )
  {
    stack_item the_stack[32];
    stack_item *cur_item = the_stack;
    the_stack[0].m_node = &m_tree_root;
    while ( *cur_item->m_node )
    {
      data_type *root = *cur_item->m_node;
      tlAssert( cur_item + 1 - the_stack < 32 );
      stack_item *next_item = cur_item + 1;
      if ( key >= accessor::get_avl_key( root ) )
      {
        tlAssert( key > accessor::get_avl_key( root ) );
        cur_item->m_child = 1;
        next_item->m_node = &accessor::get_avl_node( root )->m_right;
      }
      else
      {
        cur_item->m_child = -1;
        next_item->m_node = &accessor::get_avl_node( root )->m_left;
      }
      ++cur_item;
    }

    *cur_item->m_node = data;
    accessor::get_avl_node( data )->init();
    accessor::set_avl_key( data, key );
    while ( cur_item > the_stack )
    {
      --cur_item;
      data_type **root = cur_item->m_node;

      // Update balance factor
      accessor::get_avl_node( *root )->m_balance += cur_item->m_child;

      // Check if tree became unbalanced (left heavy)
      if ( accessor::get_avl_node( *root )->m_balance == -2 )
      {
        // Verify child balance factor
        if ( accessor::get_avl_node( accessor::get_avl_node( *root )->m_left )->m_balance != -1 &&
             accessor::get_avl_node( accessor::get_avl_node( *root )->m_left )->m_balance != 1 )
        {
          tlAssert( accessor::get_avl_node( accessor::get_avl_node( *root )->m_left )->m_balance == -1 ||
                    accessor::get_avl_node( accessor::get_avl_node( *root )->m_left )->m_balance == 1 );
        }

        // Left-Right case: double rotation needed
        if ( accessor::get_avl_node( accessor::get_avl_node( *root )->m_left )->m_balance == 1 )
        {
          rotate_left( accessor::get_avl_node( *root )->m_left );
        }

        // Left-Left case or after left-right rotation
        rotate_right( *root );

        // Verify balance is restored
        if ( accessor::get_avl_node( *root )->m_balance != 0 )
        {
          tlAssert( accessor::get_avl_node( *root )->m_balance == 0 );
        }
        else
        {
          tlDebugBreak();
          // If balance is 0, we don't need to continue up the tree
          break;
        }
      }
      // Check if tree became unbalanced (right heavy)
      else if ( accessor::get_avl_node( *root )->m_balance == 2 )
      {
        // Verify child balance factor
        if ( accessor::get_avl_node( accessor::get_avl_node( *root )->m_right )->m_balance != -1 &&
             accessor::get_avl_node( accessor::get_avl_node( *root )->m_right )->m_balance != 1 )
        {
          tlAssert( accessor::get_avl_node( accessor::get_avl_node( *root )->m_right )->m_balance == -1 ||
                    accessor::get_avl_node( accessor::get_avl_node( *root )->m_right )->m_balance == 1 );
        }

        // Right-Left case: double rotation needed
        if ( accessor::get_avl_node( accessor::get_avl_node( *root )->m_right )->m_balance == -1 )
        {
          rotate_right( accessor::get_avl_node( *root )->m_right );
        }

        // Right-Right case or after right-left rotation
        rotate_left( *root );

        // Verify balance is restored
        if ( accessor::get_avl_node( *root )->m_balance != 0 )
        {
          tlAssert( accessor::get_avl_node( *root )->m_balance == 0 );
        }
        else
        {
          // If balance is 0, we don't need to continue up the tree
          break;
        }
      }

      // If balance is 0, we don't need to continue up the tree
      if ( accessor::get_avl_node( *root )->m_balance == 0 )
      {
        break;
      }
    }
  }

  void remove( const key_type &key )
  {
    stack_item the_stack[32];
    stack_item *cur_item = the_stack;
    data_type **node_to_be_removed;

    // Initialize stack with root
    the_stack[0].m_node = &m_tree_root;

    // First phase: Find the node to remove
    while ( true )
    {
      data_type *root = *cur_item->m_node;
      tlAssert( root );
      tlAssert( cur_item + 1 - the_stack < 32 );

      stack_item *next_item = cur_item + 1;

      // Compare keys
      const key_type &nodeKey = accessor::get_avl_key( root );
      if ( key < nodeKey )
      {
        // Go left
        cur_item->m_child = -1;
        next_item->m_node = &accessor::get_avl_node( root )->m_left;
      }
      else if ( key > nodeKey )
      {
        // Go right
        cur_item->m_child = 1;
        next_item->m_node = &accessor::get_avl_node( root )->m_right;
      }
      else
      {
        // Found the node to remove
        break;
      }

      cur_item = next_item;
    }

    // Verify we found the correct node
    data_type *root = *cur_item->m_node;
    tlAssert( key == accessor::get_avl_key( root ) );

    // Store node to be removed
    node_to_be_removed = cur_item->m_node;

    // Second phase: Remove the node and rebalance
    if ( accessor::get_avl_node( *node_to_be_removed )->m_right )
    {
      // Node has right child - replace with inorder successor
      cur_item->m_child = 1;
      ++cur_item;
      cur_item->m_node = &accessor::get_avl_node( *node_to_be_removed )->m_right;
      stack_item *right_item = cur_item;

      // Find leftmost child of right subtree
      while ( accessor::get_avl_node( *cur_item->m_node )->m_left )
      {
        tlAssert( cur_item + 1 - the_stack < 32 );
        stack_item *next_item = cur_item + 1;
        cur_item->m_child = -1;
        next_item->m_node = &accessor::get_avl_node( *cur_item->m_node )->m_left;
        cur_item = next_item;
      }

      // Replace node with successor
      data_type *replace_node = *cur_item->m_node;
      *cur_item->m_node = accessor::get_avl_node( replace_node )->m_right;
      accessor::get_avl_node( replace_node )->m_left = accessor::get_avl_node( *node_to_be_removed )->m_left;
      accessor::get_avl_node( replace_node )->m_right = accessor::get_avl_node( *node_to_be_removed )->m_right;
      accessor::get_avl_node( replace_node )->m_balance = accessor::get_avl_node( *node_to_be_removed )->m_balance;
      right_item->m_node = &accessor::get_avl_node( replace_node )->m_right;
      *node_to_be_removed = replace_node;
    }
    else
    {
      // No right child - replace with left child
      *node_to_be_removed = accessor::get_avl_node( *node_to_be_removed )->m_left;
    }

    // Third phase: Rebalance the tree
    do
    {
      if ( cur_item <= the_stack )
        break;

      --cur_item;
      data_type **root = cur_item->m_node;

      // Update balance factor
      accessor::get_avl_node( *root )->m_balance -= cur_item->m_child;

      // Left subtree heavy
      if ( accessor::get_avl_node( *root )->m_balance == -2 )
      {
        // Left-right case
        if ( accessor::get_avl_node( accessor::get_avl_node( *root )->m_left )->m_balance == 1 )
          rotate_left( accessor::get_avl_node( *root )->m_left );

        // Left-left case (or after left-right rotation)
        rotate_right( *root );
        tlAssert( accessor::get_avl_node( *root )->m_balance == 0 || accessor::get_avl_node( *root )->m_balance == 1 );
      }
      // Right subtree heavy
      else if ( accessor::get_avl_node( *root )->m_balance == 2 )
      {
        // Right-left case
        if ( accessor::get_avl_node( accessor::get_avl_node( *root )->m_right )->m_balance == -1 )
          rotate_right( accessor::get_avl_node( *root )->m_right );

        // Right-right case (or after right-left rotation)
        rotate_left( *root );
        tlAssert( accessor::get_avl_node( *root )->m_balance == 0 || accessor::get_avl_node( *root )->m_balance == -1 );
      }

      // Continue up the tree until we reach a node with non-zero balance
    } while ( accessor::get_avl_node( *cur_item->m_node )->m_balance != 0 );
  }
};

template <typename key_type, typename data_type, typename accessor>
inline data_type *avl_tree_find( data_type *root, const key_type *key )
{
  data_type *cur = root;
  while ( cur )
  {
    const key_type &node_key = accessor::get_avl_key( cur );
    if ( *key == node_key )
      return cur;
    if ( *key >= node_key )
      cur = accessor::get_avl_node( cur )->m_right;
    else
      cur = accessor::get_avl_node( cur )->m_left;
  }
  return NULL;
}