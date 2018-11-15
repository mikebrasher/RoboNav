using System.Collections.Generic;

namespace RoboNav
{
    public class Tree<T> where T : class
    {
        private Tree<T> root = null;
        private Tree<T> parent = null;
        private T node = default(T);
        private List<Tree<T>> children = new List<Tree<T>>();
        bool leaf = false;

        public Tree(T node)
        {
            root = this;
            this.node = node;
        }

        public Tree(Tree<T> root, Tree<T> parent, T node)
        {
            this.root = root;
            this.parent = parent;
            this.node = node;
        }

        public Tree<T> Root { get { return root; } }
        public Tree<T> Parent { get { return parent; } }
        public T Node { get { return node; } }
        public List<Tree<T>> Children { get { return children; } }
        public bool IsLeaf { get { return leaf; } set { leaf = value; } }

        public bool IsDone
        {
            get
            {
                Tree<T> next = GetNext();
                return (next == null);
            }
        }

        public bool Contains(T t)
        {
            bool ret = t == node;

            if (!ret)
            {
                foreach (Tree<T> child in children)
                {
                    ret = child.Contains(t);
                    if (ret)
                    {
                        break;
                    }
                }
            }

            return ret;
        }

        public void AddChild(T child)
        {
            if (!root.Contains(child))
            {
                Tree<T> t = new Tree<T>(root, this, child);
                children.Add(t);
                leaf = false;
            }
        }

        public Tree<T> GetNext()
        {
            Tree<T> next = null;

            if (!leaf)
            {
                next = this;

                foreach (Tree<T> child in children)
                {
                    next = child.GetNext();

                    if (next != null)
                    {
                        break;
                    }
                }
            }

            return next;
        }
    }
}