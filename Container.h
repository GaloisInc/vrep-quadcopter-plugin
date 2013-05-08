// -*- Mode: C++; indent-tabs-mode: nil; c-basic-offset: 2 -*-
//
// Container.h --- Generic container for V-REP scene objects.
//
// Copyright (C) 2013, Galois, Inc.
// All Rights Reserved.
//

#ifndef V_REP_EXT_QUADCOPTER_CONTAINER_H_INCLUDED
#define V_REP_EXT_QUADCOPTER_CONTAINER_H_INCLUDED

#include <stdio.h>

#include <map>
#include <memory>

#include "v_repLib.h"

// A generic container of scene objects that satisfy some predicate.
//
// The plug-in will:
//
// - Call "rebuild" when the scene changes to recreate the set of
//   objects as needed.  Currently, we destroy all objects and
//   recreate them when the scene changes for simplicity.
//
// - Use the "call" method to call methods on the items when the
//   simulation is started, stopped, or stepped.
//
// Requirements on class "Item":
//
// - It must have a static member function "bool query(int obj)" that
//   is a predicate called on each object handle in the scene to
//   determine if we should create an "Item" for that object.
//
// - It must have a constructor taking the object handle and
//   initializing itself as needed.
//
// - It must support calling any pointers to member functions passed
//   to "call".
template <class Item>
class GenericContainer
{
public:
  // A method that can be invoked for each object using "call".
  typedef void (Item::*ItemMethod)();

  // Remove all objects from the container.
  void clear()
  {
    m_items.clear();
  }

  // Return the number of objects in the container.
  size_t size() const
  {
    return m_items.size();
  }

  // Rebuild the container by querying the scene and creating an item
  // for each object that matches the item's predicate.
  void rebuild()
  {
    clear();

    int i = 0;

    for (;;) {
      int obj = simGetObjects(i++, sim_handle_all);
      if (obj == -1)
        break;

      if (Item::query(obj)) {
        auto item = std::make_shared<Item>(obj);
        m_items[obj] = item;
      }
    }
  }

  // Look up an item by ID, returning NULL if it doesn't exist.
  std::shared_ptr<Item> get(int id) const
  {
    auto i = m_items.find(id);

    if (i == m_items.end())
      return std::shared_ptr<Item>();
    else
      return *i;
  }

  // Call a member function of each item in the container.  I'm sure
  // we could use a variadic template here to allow passing arguments
  // to the function, but I don't think we're going to need it.
  void call(ItemMethod func)
  {
    for (auto e : m_items) {
      (e.second.get()->*func)();
    }
  }

private:
  // Map of scene objects stored by object ID.
  std::map<int, std::shared_ptr<Item>> m_items;
};

#endif   // !defined V_REP_EXT_QUADCOPTER_CONTAINER_H_INCLUDED
