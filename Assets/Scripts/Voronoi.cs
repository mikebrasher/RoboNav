using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

namespace RoboNav
{
    public interface IDisplayable
    {
        Transform Display();
    }

    public abstract class Site : IDisplayable
    {
        public Transform prefab;
        public Transform parent;

        public abstract Point Contact { get; }
        //public abstract double X { get; }
        //public abstract double Y { get; }
        //public abstract double Z { get; }

        public abstract Transform Display();

        public abstract double Distance(Point p);
        public double Intrusion(Node n)
        {
            return n.Clearance - Distance(n.Point);
        }

        public abstract bool Influences(Point p);

        public abstract List<Curve> GetBisectors(Site visitor);
        public abstract List<Curve> Visit(PointSite ps);
        public abstract List<Curve> Visit(LineSite ls);
    }

    public class PointSite : Site, ICartesian<PointSite>
    {
        private Point p_;

        public PointSite(PointSite template)
        {
            this.prefab = template.prefab;
            this.parent = template.parent;
            p_ = template.Contact.DeepCopy;
        }

        public PointSite(Point p, Transform prefab, Transform parent)
        {
            this.prefab = prefab;
            this.parent = parent;
            p_ = p.DeepCopy;
        }

        public PointSite(double x, double y, double z, Transform prefab, Transform parent)
        {
            this.prefab = prefab;
            this.parent = parent;

            p_ = new Point(x, y, z);
        }

        public override Point Contact
        {
            get
            {
                return p_;
            }
        }

        public double X { get { return p_.X; } }
        public double Y { get { return p_.Y; } }
        public double Z { get { return p_.Z; } }

        public override Transform Display()
        {
            Transform t = GameObject.Instantiate(prefab, parent);
            t.position = Contact.Position;
            return t;
        }

        public override double Distance(Point p)
        {
            Vector delta = p - Contact;
            return delta.Magnitude;
        }

        public override bool Influences(Point p)
        {
            return true;
        }

        public override List<Curve> GetBisectors(Site visitor)
        {
            return visitor.Visit(this);
        }

        public override List<Curve> Visit(PointSite ps)
        {
            List<Curve> bisectors = new List<Curve>();

            Point mid = 0.5 * (ps.Contact + p_);
            Vector normal = ps.Contact - p_;
            bisectors.Add(new Line(mid, normal));

            return bisectors;
        }

        public override List<Curve> Visit(LineSite ls)
        {
            return ls.Visit(this);
        }

        public PointSite DeepCopy
        {
            get
            {
                return new PointSite(this);
            }
        }

        //public int Key
        //{
        //    get
        //    {
        //        return Contact.Key;
        //    }
        //}
    }

    public class LineSite : Site
    {
        private Point tail = Point.Zero;
        private Point head = Point.Zero;
        private Vector unit = Vector.Zero;
        private Line line = null;

        public LineSite(Point tail, Point head, Transform prefab, Transform parent)
        {
            this.prefab = prefab;
            this.parent = parent;

            this.tail = tail;
            this.head = head;

            unit = (head - tail);
            unit.Normalize();

            line = new Line(tail, head, true);
        }

        public Point Tail { get { return tail; } }
        public Point Head { get { return head; } }
        public Vector Unit { get { return unit; } }
        public Line Line { get { return line; } }

        public double Length
        {
            get
            {
                return ((Vector)(head - tail)).Magnitude;
            }
        }

        public Point Center
        {
            get
            {
                return 0.5 * (tail + head);
            }
        }

        public override Point Contact
        {
            get
            {
                return tail;
            }
        }

        //public override double X { get { return Center.X; } }
        //public override double Y { get { return Center.Y; } }
        //public override double Z { get { return Center.Z; } }

        public bool IsParallel(LineSite ls)
        {
            return unit.IsParallel(ls.Unit);
        }

        public override Transform Display()
        {
            Transform t = GameObject.Instantiate(prefab, parent);
            t.position = Center.Position;
            t.localScale = new Vector3(t.localScale.x, (float)(0.5 * Length), t.localScale.z);
            t.localRotation = Quaternion.FromToRotation(
                new Vector3(0.0f, 1.0f, 0.0f),
                new Vector3((float)unit.X, (float)unit.Y, (float)unit.Z));
            return t;
        }

        public override double Distance(Point p)
        {
            return line.Distance(p);
        }

        public override bool Influences(Point p)
        {
            double t = line.Project(p);
            return line.Contains(t);
        }

        public override List<Curve> GetBisectors(Site visitor)
        {
            return visitor.Visit(this);
        }

        public override List<Curve> Visit(PointSite ps)
        {
            List<Curve> bisectors = new List<Curve>();

            double distance = this.Distance(ps.Contact);
            if (distance > Constants.tol)
            {
                bisectors.Add(new Parabola(ps.Contact, line));
            }
            else
            {
                bisectors.Add(new Line(ps.Contact, line.IHat));
            }

            return bisectors;
        }

        public override List<Curve> Visit(LineSite ls)
        {
            List<Curve> bisectors = new List<Curve>();

            Line line0 = Line;
            Line line1 = ls.Line;

            Vector n0 = line0.KHat;
            double c0 = line0.C;

            Vector n1 = line1.KHat;
            double c1 = line1.C;

            if ((n0.Magnitude2 > 0.0) && (n1.Magnitude2 > 0.0))
            {
                List<double> signs = new List<double> { -1.0, 1.0 };
                foreach (double sign in signs)
                {
                    Vector nb = n0 + sign * n1;
                    double db = c0 + sign * c1;

                    if (nb.Magnitude2 > Constants.tol)
                    {
                        Point b0 = nb * (db / nb.Magnitude2);
                        bisectors.Add(new Line(b0, nb));
                    }
                }
            }

            return bisectors;
        }

        //public int Key
        //{
        //    get
        //    {
        //        return (Head.Key + Tail.Key);
        //    }
        //}
    }

    public class Node : IDisplayable, ICartesian<Node>
    {
        private Transform prefab;
        private Transform parent;

        private Point p = Point.Zero;
        private List<Site> sites = new List<Site>();
        private List<Edge> edges = new List<Edge>();
        private bool connected = false;

        public Node(Node template)
        {
            this.prefab = template.prefab;
            this.parent = template.parent;

            this.p = template.Point.DeepCopy;

            sites = new List<Site>(template.Sites); // just shallow copy the sites
        }

        public Node(Point p, Transform prefab, Transform parent)
        {
            this.prefab = prefab;
            this.parent = parent;

            this.p = p;
        }

        public Point Point { get { return p; } }
        public double X { get { return p.X; } }
        public double Y { get { return p.Y; } }
        public double Z { get { return p.Z; } }

        public List<Site> Sites { get { return sites; } }
        public List<Edge> Edges { get { return edges; } }
        public bool Connected { get { return connected; } set { connected = value; } }

        public double Clearance
        {
            get
            {
                double clearance = double.MaxValue;

                foreach (Site s in sites)
                {
                    double d = s.Distance(p);
                    if (d < clearance)
                    {
                        clearance = d;
                    }
                }

                return clearance;
            }
        }

        public void Add(Edge e)
        {
            edges.Add(e);

            if (!sites.Contains(e.Left))
            {
                sites.Add(e.Left);
            }

            if (!sites.Contains(e.Right))
            {
                sites.Add(e.Right);
            }
        }

        public void Remove(Edge e)
        {
            edges.Remove(e);
            //connected = false;
        }

        public List<Site> SharedSites(Node n)
        {
            List<Site> shared = new List<Site>();

            foreach (Site s in sites)
            {
                if (n.Sites.Contains(s))
                {
                    shared.Add(s);
                }
            }

            return shared;
        }

        //public int Key
        //{
        //    get
        //    {
        //        return p.Key;
        //    }
        //}

        public Transform Display()
        {
            Transform t = GameObject.Instantiate(prefab, parent);
            Point offset = p + new Point(0.0, 0.0, 0.0);
            t.localPosition = offset.Position;
            return t;
        }

        public Node DeepCopy
        {
            get
            {
                return new Node(this);
            }
        }
    }

    public class NodeList : CartesianList<Node>
    {
        public new NodeList DeepCopy
        {
            get
            {
                NodeList copy = new NodeList();

                foreach (Node n in this)
                {
                    copy.AddNear(n.DeepCopy);
                }

                foreach (Edge e in this.Edges)
                {
                    Node tailCopy = copy.GetNear(e.Tail);
                    Node headCopy = copy.GetNear(e.Head);

                    new Edge(e.Left, e.Right, e.Curve, tailCopy, headCopy, e.Prefab, e.Parent);
                }

                return copy;
            }
        }

        public List<Edge> Edges
        {
            get
            {
                List<Edge> edges = new List<Edge>();

                foreach (Node n in this)
                {
                    foreach (Edge e in n.Edges)
                    {
                        if (!edges.Contains(e))
                        {
                            edges.Add(e);
                        }
                    }
                }

                return edges;
            }
        }
    }

    public class Edge : IDisplayable
    {
        private Transform prefab;
        private Transform parent;

        private Site left = null;
        private Site right = null;

        private Curve curve = null;
        private Node tail = null;
        private Node head = null;
        //private ColVector unit = null;

        public Edge(Site left, Site right, Curve curve, Node tail, Node head, Transform prefab, Transform parent)
        {
            this.prefab = prefab;
            this.parent = parent;

            this.left = left;
            this.right = right;

            this.curve = curve.Copy;

            this.tail = tail;
            this.head = head;

            tail.Add(this);
            head.Add(this);

            double t = this.curve.Project(tail.Point);
            double h = this.curve.Project(head.Point);
            if (t < h)
            {
                this.curve.Min = t;
                this.curve.Max = h;
            }
            else
            {
                this.curve.Min = h;
                this.curve.Max = t;
            }
        }

        public double Distance(Point p)
        {
            return curve.Distance(p);
        }

        public Transform Prefab { get { return prefab; } }
        public Transform Parent { get { return parent; } }

        public Site Left { get { return left; } }
        public Site Right { get { return right; } }
        public List<Site> Sites { get { return new List<Site> { left, right }; } }

        public Curve Curve { get { return curve; } }
        public Node Tail { get { return tail; } }
        public Node Head { get { return head; } }

        public Node OtherNode(Node n)
        {
            Node other = null;

            if (n == tail)
            {
                other = head;
            }
            else if (n == head)
            {
                other = tail;
            }

            return other;
        }

        public Vector Direction
        {
            get
            {
                return head.Point - tail.Point;
            }
        }

        public Vector Unit
        {
            get
            {
                Vector unit = Direction;
                unit.Normalize();
                return unit;
            }
        }

        public double Length
        {
            get
            {
                return Direction.Magnitude;
            }
        }

        public Site SharedSite(Edge e)
        {
            Site shared = null;

            if ((left == e.left) || (left == e.right))
            {
                shared = left;
            }
            else if ((right == e.left) || (right == e.right))
            {
                shared = right;
            }

            return shared;
        }

        public void RemoveFromNodes()
        {
            if (tail != null)
            {
                tail.Remove(this);
            }

            if (head != null)
            {
                head.Remove(this);
            }
        }

        public Transform Display()
        {
            Transform t = null;

            Line line = curve as Line;
            Parabola parabola = curve as Parabola;

            if (line != null)
            {
                DisplaySegment(tail.Point, head.Point);
            }
            else if (parabola != null)
            {
                int nn = 10;
                double span = parabola.Max - parabola.Min;
                double dt = span / nn;

                double tt = parabola.Min;
                for (int ii = 0; ii < nn; ++ii)
                {
                    Point p0 = parabola.GetPoint(tt);
                    Point p1 = parabola.GetPoint(tt + dt);

                    t = DisplaySegment(p0, p1);

                    tt += dt;
                }
            }

            return t;
        }

        private Transform DisplaySegment(Point p0, Point p1)
        {
            Point center = 0.5 * (p0 + p1) + new Point(0.0, 0.0, 0.0);
            Vector unit = p1 - p0;
            float length = 0.5f * (float)unit.Magnitude;
            unit.Normalize();

            Transform t = GameObject.Instantiate(prefab, parent);
            t.localPosition = center.Position;
            t.localScale = new Vector3(t.localScale.x, length, t.localScale.z);
            t.localRotation = Quaternion.FromToRotation(
                new Vector3(0.0f, 1.0f, 0.0f),
                new Vector3((float)unit.X, (float)unit.Y, (float)unit.Z));

            return t;
        }
    }

    public class Voronoi : MonoBehaviour
    {
        public Transform nodePrefab;
        public Transform edgePrefab;
        public Transform pointSitePrefab;
        public Transform lineSitePrefab;
        public Transform spheres;
        public Transform cylinders;

        private Polygon polygon;

        private CartesianList<PointSite> pointSites = new CartesianList<PointSite>();
        private List<LineSite> lineSites = new List<LineSite>();
        private List<Site> sites = new List<Site>();
        private NodeList nodes = new NodeList();

        private List<Site> displaySites = new List<Site>();
        
        public Polygon Polygon { get { return polygon; } set { polygon = value; } }
        public CartesianList<Node> Nodes { get { return nodes; } }
        public CartesianList<PointSite> PointSites { get { return pointSites; } }
        public List<LineSite> LineSites {  get { return lineSites; } }

        void Awake()
        {
        }

        public void GenerateSites()
        {
            foreach (PolygonEdge pe in polygon.Edges)
            {
                AddSite(pe.Tail, pe.Head);
            }
        }

        public void DisplayAllSites()
        {
            foreach (Site s in sites)
            {
                s.Display();
            }
        }

        private void AddSite(Point tail, Point head)
        {
            PointSite ps = new PointSite(tail, pointSitePrefab, spheres);
            if (!pointSites.ContainsNear(ps))
            {
                pointSites.Add(ps);
                sites.Add(ps);
            }

            LineSite ls = new LineSite(tail, head, lineSitePrefab, cylinders);
            if (!lineSites.Contains(ls))
            {
                lineSites.Add(ls);
                sites.Add(ls);
            }
        }

        private Node AddNodeNear(Point p)
        {
            Node node = new Node(p, nodePrefab, spheres);
            return nodes.AddNear(node);
        }

        public void ClearDiagram()
        {
            foreach (Transform t in spheres)
            {
                Destroy(t.gameObject);
            }

            foreach (Transform t in cylinders)
            {
                Destroy(t.gameObject);
            }
        }

        public void Display(NodeList displayNodes)
        {
            ClearDiagram();

            foreach (Site s in displaySites)
            {
                s.Display();
            }

            List<Edge> displayed = new List<Edge>();
            foreach (Node n in displayNodes)
            {
                n.Display();
                foreach (Edge e in n.Edges)
                {
                    if (!displayed.Contains(e))
                    {
                        e.Display();
                        displayed.Add(e);
                    }
                }
            }
        }

        public void Display()
        {
            Display(nodes);
        }

        public void InitializeNodes()
        {
            // lower left, lower right, upper right, upper left
            Point[] dummy = new Point[4];

            dummy[0] = new Point(double.MaxValue, 0.0, double.MaxValue);
            dummy[2] = new Point(double.MinValue, 0.0, double.MinValue);

            // find bounding box of point sites
            foreach (PointSite ps in pointSites)
            {
                if (ps.Contact.X < dummy[0].X)
                {
                    dummy[0].X = ps.Contact.X;
                }

                if (ps.Contact.X > dummy[2].X)
                {
                    dummy[2].X = ps.Contact.X;
                }

                if (ps.Contact.Z < dummy[0].Z)
                {
                    dummy[0].Z = ps.Contact.Z;
                }

                if (ps.Contact.Z > dummy[2].Z)
                {
                    dummy[2].Z = ps.Contact.Z;
                }
            }

            dummy[1] = new Point(dummy[2].X, 0.0, dummy[0].Z);
            dummy[3] = new Point(dummy[0].X, 0.0, dummy[2].Z);

            double scale = 2.0;
            Point c = 0.5 * (dummy[0] + dummy[2]);
            for (int idummy = 0; idummy < 4; ++idummy)
            {
                Vector delta = dummy[idummy] - c;
                dummy[idummy] = c + scale * delta;
            }

            // create dummy sites
            PointSite[] dummyPoint = new PointSite[4];
            for (int idummy = 0; idummy < 4; ++idummy)
            {
                PointSite ps = new PointSite(dummy[idummy], pointSitePrefab, spheres);
                dummyPoint[idummy] = ps;
                if (!sites.Contains(ps))
                {
                    sites.Add(ps);
                    displaySites.Add(ps);
                }
            }

            // place node at center
            Node center = AddNodeNear(0.5 * (dummy[0] + dummy[2]));
            center.Connected = true;

            // add four corner nodes
            for (int idummy = 0; idummy < 4; ++idummy)
            {
                int jdummy = (idummy + 1) % 4;

                Site left = dummyPoint[idummy];
                Site right = dummyPoint[jdummy];
                Point mid = 0.5 * (left.Contact + right.Contact);
                Vector delta = mid - c;

                Node n = AddNodeNear(c + scale * delta);
                n.Connected = true;

                List<Curve> bisectors = left.GetBisectors(right);
                if (bisectors.Count > 0)
                {
                    new Edge(left, right, bisectors[0], center, n, edgePrefab, cylinders);
                }
            }
        }

        public IEnumerator<string> AddPointSites()
        {
            foreach (PointSite ps in pointSites)
            {
                AddSite(ps);
                yield return string.Format("({0}, {1}, {2}", ps.X, ps.Y, ps.Z);
            }
        }

        public IEnumerator<string> AddLineSites()
        {
            foreach (LineSite ls in lineSites)
            {
                AddSite(ls);
                Point center = ls.Center;
                yield return string.Format("({0}, {1}, {2}", center.X, center.Y, center.Z);
            }
        }

        private void AddSite(Site s)
        {
            Node seed = DetermineSeed(s);
            if (seed != null)
            {
                DeleteNodes(s, seed);
                ConnectRegion(s);

                if (!displaySites.Contains(s))
                {
                    displaySites.Add(s);
                }
            }
        }

        private Node DetermineSeed(Site s)
        {
            Node seed = null;

            double max = double.MinValue;
            foreach (Node v in nodes)
            {
                double intrusion = s.Intrusion(v);
                if ((intrusion > 0.0) && (intrusion > max))
                {
                    max = intrusion;
                    seed = v;
                }
            }

            return seed;
        }

        private void DeleteNodes(Site site, Node seed)
        {
            Tree<Node> deleted = new Tree<Node>(seed);
            Tree<Node> next = deleted.GetNext();

            while (next != null)
            {
                Node node = next.Node;
                nodes.Remove(node);
                next.IsLeaf = true;
                List<Edge> edgeList = new List<Edge>(node.Edges); // save a copy to loop over so we can modify node.edges
                foreach (Edge e in edgeList)
                {
                    Node other = e.OtherNode(node);
                    if (other != null)
                    {
                        if (site.Distance(other.Point) < other.Clearance)
                        {
                            next.AddChild(other);
                        }
                        else
                        {
                            CartesianList<Point> intersections = new CartesianList<Point>();
                            foreach (Site side in e.Sites)
                            {
                                List<Curve> bisectors = side.GetBisectors(site);
                                foreach (Curve b in bisectors)
                                {
                                    CartesianList<Point> sideIntersection = e.Curve.Intersect(b);
                                    foreach (Point x in sideIntersection)
                                    {
                                        if (site.Influences(x) && side.Influences(x))
                                        {
                                            intersections.AddNear(x);
                                        }
                                    }
                                }
                            }

                            foreach (Point x in intersections)
                            {
                                Node newNode = AddNodeNear(x);

                                if (newNode == other)
                                {
                                    other.Connected = false;
                                }
                                else
                                {
                                    new Edge(e.Left, e.Right, e.Curve, other, newNode, edgePrefab, cylinders);
                                }
                            }
                        }

                        e.RemoveFromNodes();
                    }
                }

                next = deleted.GetNext();
            }
        }

        private void ConnectRegion(Site s)
        {
            List<Node> disconnected = new List<Node>();

            foreach (Node n in nodes)
            {
                if (!n.Connected)
                {
                    disconnected.Add(n);
                }
            }

            for (int ii = 0; ii < disconnected.Count; ++ii)
            {
                Node ni = disconnected[ii];

                for (int jj = ii+1; jj < disconnected.Count; ++jj)
                {
                    Node nj = disconnected[jj];

                    List<Site> shared = ni.SharedSites(nj);
                    shared.Remove(s);

                    foreach (Site common in shared)
                    {
                        ConnectEdge(s, common, ni, nj);
                    }
                }

                ni.Connected = true;
            }
        }

        private void ConnectEdge(Site left, Site right, Node tail, Node head)
        {
            List<Curve> bisectors = left.GetBisectors(right);
            foreach (Curve bisector in bisectors)
            {
                if (bisector.Contains(tail.Point) && bisector.Contains(head.Point))
                {
                    Node split = null;

                    bool crossing = false;
                    foreach (Node n in nodes)
                    {
                        crossing |= bisector.Between(n.Point, tail.Point, head.Point);
                    }

                    foreach (Site s in displaySites)
                    {
                        if (bisector.Contains(s.Contact))
                        {
                            if ((s == left) || (s == right))
                            {
                                // catch edges that line site end points
                                split = AddNodeNear(s.Contact);
                            }
                            else
                            {
                                crossing |= bisector.Between(s.Contact, tail.Point, head.Point);
                            }
                        }
                    }

                    // apex split parabolas
                    Parabola parabola = bisector as Parabola;
                    if (parabola != null)
                    {
                        double t = parabola.Project(tail.Point);
                        t *= parabola.Project(head.Point);

                        if (t < 0.0)
                        {
                            split = AddNodeNear(parabola.GetPoint(0.0));
                        }
                    }

                    if (!crossing)
                    {
                        if ((split != null) && (split != tail) && (split != head))
                        {
                            new Edge(left, right, bisector, tail, split, edgePrefab, cylinders);
                            new Edge(left, right, bisector, split, head, edgePrefab, cylinders);

                            split.Connected = true;
                        }
                        else
                        {
                            new Edge(left, right, bisector, tail, head, edgePrefab, cylinders);
                        }
                    }
                }
            }
        }

        public NodeList ExtractSkeleton()
        {
            NodeList skeleton = nodes.DeepCopy;
            List<Node> nodeList = new List<Node>(skeleton); // shallow copy
            foreach (Node n in nodeList)
            {
                if (polygon.SignedDistance(n.Point) < Constants.tol)
                {
                    skeleton.Remove(n);
                    List<Edge> edgeList = new List<Edge>(n.Edges);
                    foreach (Edge e in edgeList)
                    {
                        e.RemoveFromNodes();
                    }
                }
            }

            return skeleton;
        }

        void Start()
        {
        }

        void Update()
        {
        }
    }
}