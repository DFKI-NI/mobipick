/* This is a custom behavior tree implementation which can be paused after any task.
   * Tree execution is exited on pause and can be resumed by running the behavior tree again.
   * A task's behavior is intended to be implemented in its main() method.
   * By default, each node's preprocess() method outputs the name of the current task.
   * When using the default get_next_task() implementation to retrieve the next task's name,
   * an inner tree node returns itself if it has not been run at all. Otherwise,
   * it returns its first unfinished child node. This implementation aims at more meaningful
   * task descriptions by assuming inner nodes to describe coherent sub-behaviors.
   * Details of such a sub-behavior is revealed when it is visited and run.
   *
   * Note: Typically, a behavior tree is processed and left on each execution. It triggers
   * execution of tasks but doesn't wait until they are finished. Such a usage is in general
   * based on an environment state representation which was not available at creation time
   * of this behavior tree. The implementation at hand is chosen for simplicity and is
   * tailored closely to the specific pick and place scenario.
   */

#include <std_msgs/String.h>

class BehaviorTree;

class BehaviorTree
{
public:
  class Node
  {
  protected:
    bool visited = false;

  public:
    virtual void connect(BehaviorTree *root_ptr) {}
    virtual std::string get_task_name() = 0;
    virtual void preprocess() {}
    virtual bool main() = 0;
    virtual void postprocess() {}
    virtual Node *get_next_task() = 0;
    bool run()
    {
      visited = true;
      preprocess();
      bool result = main();
      postprocess();
      return result;
    }
  };

  class TaskNode : public Node
  {
  public:
    virtual void preprocess()
    {
      std::cout << "TASK " << get_task_name() << std::endl;
    }
    virtual Node *get_next_task()
    {
      return this;
    }
  };

  class InnerNode : public Node
  {
  protected:
    BehaviorTree *root_ptr;
    std::string base_name;
    std::vector<Node *> children;
    int child_index = 0;

  public:
    InnerNode(const std::string &name, std::initializer_list<Node *> &&new_children) : base_name(name)
    {
      add_children(new_children);
    }
    // Retroactively connect inner nodes to its behavior tree, so its pause flag can be accessed.
    // This is triggered by the behavior tree's constructor and not done in the node's constructor,
    // so the user can build the tree bottom-up without having to call the behavior tree constructor first.
    // It is a convenience implementation for demonstration and not recommended for general usage.
    // Referencing the outer class explicitely is necessary in C++.
    virtual void connect(BehaviorTree *root_ptr)
    {
      this->root_ptr = root_ptr;
      for (std::vector<Node *>::iterator it = children.begin(); it != children.end(); ++it)
      {
        (*it)->connect(root_ptr);
      }
    }
    virtual Node *get_next_task()
    {
      // If this inner node has not been run yet, return itself as a whole task.
      if (!visited)
      {
        return this;
      }

      // Otherwise return the first incomplete task of the children.
      for (int index = child_index; index < children.size(); ++index)
      {
        Node *next_task = children[index]->get_next_task();
        if (next_task != nullptr)
        {
          return next_task;
        }
      }

      return nullptr;
    }
    void add_child(Node *child) { children.push_back(child); }
    void add_children(std::initializer_list<Node *> &&new_children)
    {
      for (Node *child : new_children)
      {
        add_child(child);
      }
    }
    template <typename CONTAINER>
    void add_children(const CONTAINER &new_children)
    {
      for (Node *child : new_children)
      {
        add_child(child);
      }
    }
  };

  class Selector : public InnerNode
  {
  public:
    using InnerNode::InnerNode;
    virtual std::string get_task_name() { return "Selector " + base_name; }
    virtual bool main() override
    {
      // If this node has been completed before, return true.
      if (child_index < 0)
      {
        return true;
      }

      for (int index = child_index; index < children.size(); ++index)
      {
        // If a child returns true, remember this node as completed.
        if (children[index]->run())
        {
          child_index = -1;
          return true;
        }

        // Note: Do not increase child_index yet, if paused. Both failure and pause of a child return false,
        //   so child_index needs to be revisited on continuation in case it was not completed due to a pause.
        //   Ideally, every node would report both a result and a completion flag. This has been omitted
        //   in this implementation for simplicity.
        if (*root_ptr->pause_ptr)
        {
          return false;
        }

        child_index = index + 1;
      }
      return false;
    }
  };

  class Sequence : public InnerNode
  {
  public:
    using InnerNode::InnerNode;
    virtual std::string get_task_name() { return "Sequence " + base_name; }
    virtual bool main() override
    {
      for (int index = child_index; index < children.size(); ++index)
      {
        if (!children[index]->run())
        {
          return false;
        }

        child_index = index + 1;
        if (*root_ptr->pause_ptr)
        {
          return false;
        }
      }

      return true;
    }
  };

private:
  Node *root;

protected:
  bool *pause_ptr;

public:
  BehaviorTree(Node *main_node, bool *pause_ptr) : root(main_node), pause_ptr(pause_ptr)
  {
    main_node->connect(this);
  }
  std::string get_next_task_name()
  {
    Node *next_task = root->get_next_task();
    return (next_task != nullptr ? next_task->get_task_name() : "No more tasks in behavior tree!");
  }
  bool run() const { return root->run(); }
};
