using System.Collections.Generic;
using UnityEngine;

namespace RoboNav
{
    public class PolygonEdge
    {
        private Point tail;
        private Point head;
        private Line line;

        public PolygonEdge(Point tail, Point head)
        {
            this.tail = tail.DeepCopy;
            this.head = head.DeepCopy;

            line = new Line(tail, head, true);
        }

        public Point Tail { get { return tail; } }
        public Point Head { get { return head; } }

        // signed distance to the line defined by edge
        public double SignedDistance(Point p)
        {
            Vector tp = new Vector(tail, p);
            return tp.Dot(line.KHat);
        }

        // distance to nearest point on edge
        public double Distance(Point p)
        {
            return line.Distance(p);
        }

        public bool UpwardCrossing(Point p)
        {
            return (tail.Z <= p.Z) && (p.Z < head.Z);
        }

        public bool DownwardCrossing(Point p)
        {
            return (head.Z <= p.Z) && (p.Z < tail.Z);
        }
    }

    public class Polygon
    {
        private Transform[] maze = null;

        private CartesianList<Point> points = new CartesianList<Point>();
        private Dictionary<Point, List<WallIndex>> point2wall = new Dictionary<Point, List<WallIndex>>();
        private Point[,] wall2point = null;

        private CartesianList<Point> vertices = new CartesianList<Point>();
        private List<PolygonEdge> edges = new List<PolygonEdge>();

        private double xmin = double.MaxValue;
        private double xmax = double.MinValue;
        private double zmin = double.MaxValue;
        private double zmax = double.MinValue;

        private class WallIndex
        {
            private int wall;
            private int corner;

            public WallIndex(int wall, int corner)
            {
                this.wall = wall;
                this.corner = corner;
            }

            public int Wall { get { return wall; } set { wall = value; } }
            public int Corner { get { return corner; } set { corner = value; } }

            public WallIndex Rotate
            {
                get
                {
                    return new WallIndex(wall, (corner + 1) % 4);
                }
            }
        }

        private Point[] cubePoint = new Point[8]
        {
            new Point(-1.0, -1.0, -1.0),
            new Point( 1.0, -1.0, -1.0),
            new Point( 1.0, -1.0,  1.0),
            new Point(-1.0, -1.0,  1.0),
            new Point(-1.0,  1.0, -1.0),
            new Point( 1.0,  1.0, -1.0),
            new Point( 1.0,  1.0,  1.0),
            new Point(-1.0,  1.0,  1.0)
        };

        public Polygon(Transform[] maze)
        {
            this.maze = maze;
        }

        public void Generate()
        {
            wall2point = new Point[maze.Length, 4];

            for (int iwall = 0; iwall < maze.Length; ++iwall)
            {
                Transform wall = maze[iwall];

                for (int icorner = 0; icorner < 4; ++icorner)
                {
                    Point p = Corner(wall, icorner);

                    if (points.ContainsNear(p))
                    {
                        p = points.GetNear(p);
                    }
                    else
                    {
                        points.Add(p);

                        List<WallIndex> walls = new List<WallIndex>();
                        point2wall.Add(p, walls);
                    }

                    point2wall[p].Add(new WallIndex(iwall, icorner));
                    wall2point[iwall, icorner] = p;
                }
            }

            WallIndex currentWall = new WallIndex(0, 2);
            Point current = vertices.AddNear(LookupPoint(currentWall));
            Point first = current;

            Point prev = null;
            WallIndex peekWall = null;
            Point peek = null;

            for (int ii = 0; ii < 100; ++ii)
            {
                prev = current;

                currentWall = FindNextWall(currentWall).Rotate;

                current = LookupPoint(currentWall);
                if (current.IsCoincident(first))
                {
                    edges.Add(new PolygonEdge(prev, first));
                    break;
                }

                // combine parallel wall edges
                for (int jj = 0; jj < 5; ++jj)
                {
                    peekWall = FindNextWall(currentWall).Rotate;

                    peek = LookupPoint(peekWall);
                    Vector v0 = current - prev;
                    Vector v1 = peek - current;

                    if (v1.IsParallel(v0)) // (peek - current).IsParallel(current - prev))
                    {
                        currentWall = peekWall;
                        current = peek;
                    }
                    else
                    {
                        break;
                    }
                }

                current = vertices.AddNear(current);
                edges.Add(new PolygonEdge(prev, current));
            }

            DetermineBounds();
        }

        private void DetermineBounds()
        {
            foreach (Point p in vertices)
            {
                xmin = (xmin < p.X) ? xmin : p.X;
                xmax = (xmax > p.X) ? xmax : p.X;

                zmin = (zmin < p.Z) ? zmin : p.Z;
                zmax = (zmax > p.Z) ? zmax : p.Z;
            }
        }

        private bool IsInside(Point p)
        {
            return (xmin < p.X) && (p.X < xmax) && (zmin < p.Z) && (p.Z < zmax);
        }

        // simplified winding method for testing point in polygon
        // code adapted from http://geomalgorithms.com/a03-_inclusion.html
        // inside < 0; on edge == 0; outside > 0
        public double SignedDistance(Point p)
        {
            double min = double.MaxValue;

            bool inbound = IsInside(p);

            int wn = 0;
            foreach (PolygonEdge e in edges)
            {
                double dist = e.Distance(p);
                min = (min < dist) ? min : dist;

                if (inbound)
                {
                    double sd = e.SignedDistance(p);

                    if (e.UpwardCrossing(p))
                    {
                        if (sd > 0) // p left of edge
                        {
                            ++wn; // a valid upward intersection
                        }
                    }
                    else if (e.DownwardCrossing(p)) // a downward crossing
                    {
                        if (sd < 0) // p right of edge
                        {
                            --wn; // a valid downward intersection
                        }
                    }
                }
            }

            if (wn == 0)
            {
                // point is outside
                min = -min;
            }

            return min;
        }

        public int Count
        {
            get
            {
                return vertices.Count;
            }
        }

        public CartesianList<Point> Vertices { get { return vertices; } }
        public List<PolygonEdge> Edges { get { return edges; } }

        private Point Corner(Transform t, int index)
        {
            Vector3 center = t.position;
            Vector3 scale = t.localScale;

            Point p = cubePoint[index];

            return new Point(
                center.x + 0.5 * p.X * scale.x,
                center.y + 0.5 * p.Y * scale.y,
                center.z + 0.5 * p.Z * scale.z);
        }

        private Point LookupPoint(WallIndex index)
        {
            return wall2point[index.Wall, index.Corner];
        }

        // switch to touching wall if it exists
        // otherwise return current wall
        private WallIndex FindNextWall(WallIndex current)
        {
            WallIndex other = current;

            Point p = wall2point[current.Wall, current.Corner];

            List<WallIndex> pointWalls = point2wall[p];
            foreach (WallIndex index in pointWalls)
            {
                if (index.Wall != current.Wall)
                {
                    other = index;
                    break;
                }
            }

            return other;
        }
    }
}