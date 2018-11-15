using System.Collections.Generic;
using UnityEngine;

namespace RoboNav
{
    public interface ICartesian<T>
    {
        double X { get; }
        double Y { get; }
        double Z { get; }
        T DeepCopy { get; }
    }

    public class CartesianList<T> : List<T> where T : ICartesian<T>
    {
        public T GetNear(T check)
        {
            T ret = default(T);

            foreach (T current in this)
            {
                double dx = check.X - current.X;
                double dy = check.Y - current.Y;
                double dz = check.Z - current.Z;

                double mag = dx * dx + dy * dy + dz * dz;

                if (mag < Constants.tol)
                {
                    ret = current;
                    break;
                }
            }

            return ret;
        }

        public bool ContainsNear(T t)
        {
            T near = GetNear(t);
            return near != null;
        }

        public new void Add(T t)
        {
            if (ContainsNear(t))
            {
                throw new System.ArgumentException(string.Format("An item near ({0}, {1}, {2}) has already been added", t.X, t.Y, t.Z));
            }

            base.Add(t);
        }

        public T AddNear(T t)
        {
            T ret = t;

            T near = GetNear(t);
            if (near != null)
            {
                ret = near;
            }
            else
            {
                base.Add(t);
            }

            return ret;
        }

        public CartesianList<T> DeepCopy
        {
            get
            {
                CartesianList<T> copy = new CartesianList<T>();

                foreach (T t in this)
                {
                    copy.AddNear(t.DeepCopy);
                }

                return copy;
            }
        }
    }

    public class Point : ICartesian<Point>
    {
        private double x_ = 0.0;
        private double y_ = 0.0;
        private double z_ = 0.0;

        public Point(Point p)
        {
            x_ = p.X;
            y_ = p.Y;
            z_ = p.Z;
        }

        public Point(Vector v)
        {
            x_ = v.X;
            y_ = v.Y;
            z_ = v.Z;
        }

        public Point(double x, double y, double z)
        {
            x_ = x;
            y_ = y;
            z_ = z;
        }

        public Point DeepCopy
        {
            get
            {
                return new Point(x_, y_, z_);
            }
        }

        public double X { get { return x_; } set { x_ = value; } }
        public double Y { get { return y_; } set { y_ = value; } }
        public double Z { get { return z_; } set { z_ = value; } }
        public Vector3 Position { get { return new Vector3((float)x_, (float)y_, (float)z_); } }

        public bool IsCoincident(Point p)
        {
            return (
                (System.Math.Abs(p.X - x_) < Constants.tol) &&
                (System.Math.Abs(p.Y - y_) < Constants.tol) &&
                (System.Math.Abs(p.Z - z_) < Constants.tol));
        }

        public static Point operator -(Point p)
        {
            return new Point(-p.X, -p.Y, -p.Z);
        }

        public static Point operator *(double left, Point right)
        {
            return new Point(
                left * right.X,
                left * right.Y,
                left * right.Z);
        }

        public static Point operator *(Point left, double right)
        {
            return right * left;
        }

        public static Point operator +(Point left, Point right)
        {
            return new Point(
                left.X + right.X,
                left.Y + right.Y,
                left.Z + right.Z);
        }

        public static Point operator -(Point left, Point right)
        {
            return left + (-right);
        }

        public static Point operator +(Point left, Vector right)
        {
            return new Point(
                left.X + right.X,
                left.Y + right.Y,
                left.Z + right.Z);
        }

        public static Point operator +(Vector left, Point right)
        {
            return right + left;
        }

        public static Point operator -(Point left, Vector right)
        {
            return left + (-right);
        }

        public static Point operator -(Vector left, Point right)
        {
            return left + (-right);
        }

        public static implicit operator Point(Vector v)
        {
            return new Point(v);
        }

        public static explicit operator Point(Vector3 v)
        {
            return new Point(v.x, v.y, v.z);
        }

        public static explicit operator Vector3(Point p)
        {
            return new Vector3((float)p.X, (float)p.Y, (float)p.Z);
        }

        //public int Key
        //{
        //    get
        //    {
        //        return Hash3(x_, y_, z_);
        //    }
        //}

        public static Point Zero
        {
            get
            {
                return new Point(0.0, 0.0, 0.0);
            }
        }

        // hash function adapated from Optimized Spatial Hashing for Collision Detection of Deformable Objects by Teschner et al.
        // offset points by (20, 20, 20), discretize, and then wrap using mod
        // should be unique for points with coordinates in [-40, 40]
        public static int Hash3(double x, double y, double z)
        {
            double off = 40.0;
            double dx = 1.0e-4;
            int nn = (int)(1.0 + 2.0 * off / dx);
            int ix = (int)((x + off) / dx) % nn;
            int iy = (int)((y + off) / dx) % nn;
            int iz = (int)((z + off) / dx) % nn;

            int px = 73856093;
            int py = 19349663;
            int pz = 83492791;

            //int px = 100003;
            //int py = 131071;
            //int pz = 174763;

            return ((ix * px) ^ (iy * py) ^ (iz * pz));
        }
    }

    public class Vector
    {
        private double x_ = 0.0;
        private double y_ = 0.0;
        private double z_ = 0.0;

        public Vector(double x, double y, double z)
        {
            this.x_ = x;
            this.y_ = y;
            this.z_ = z;
        }

        public Vector(Vector v)
        {
            if (v != null)
            {
                x_ = v.X;
                y_ = v.Y;
                z_ = v.Z;
            }
        }

        public Vector(Point head)
        {
            x_ = head.X;
            y_ = head.Y;
            z_ = head.Z;
        }

        public Vector(Point tail, Point head)
        {
            x_ = head.X - tail.X;
            y_ = head.Y - tail.Y;
            z_ = head.Z - tail.Z;
        }

        public Vector Copy
        {
            get
            {
                return new Vector(x_, y_, z_);
            }
        }

        public double X { get { return x_; } }
        public double Y { get { return y_; } }
        public double Z { get { return z_; } }

        public static Vector operator +(Vector left, Vector right)
        {
            return new Vector(
                left.X + right.X,
                left.Y + right.Y,
                left.Z + right.Z
                );
        }

        public static Vector operator -(Vector left, Vector right)
        {
            return left + (-right);
        }

        public static Vector operator -(Vector v)
        {
            return new Vector(-v.X, -v.Y, -v.Z);
        }

        public static Vector operator *(Vector left, double right)
        {
            Vector ret = new Vector(left);
            ret.Scale(right);
            return ret;
        }

        public static Vector operator *(double left, Vector right)
        {
            return right * left;
        }

        public static Vector operator /(Vector left, double right)
        {
            return left * (1.0 / right);
        }

        public static implicit operator Vector(Point p)
        {
            return new Vector(p);
        }

        public double Magnitude
        {
            get
            {
                double ret = 0.0;

                double mag2 = Magnitude2;
                if (mag2 > 0.0)
                {
                    ret = System.Math.Sqrt(mag2);
                }

                return ret;
            }
        }

        public double Magnitude2
        {
            get
            {
                return Dot(this);
            }
        }

        public void Scale(double s)
        {
            x_ *= s;
            y_ *= s;
            z_ *= s;
        }

        public void Normalize()
        {
            double mag = Magnitude;
            if (mag > 0.0)
            {
                Scale(1.0 / mag);
            }
            else
            {
                Scale(0.0);
            }
        }

        public double Dot(Vector v)
        {
            return x_ * v.X + y_ * v.Y + z_ * v.Z;
        }

        public Vector Cross(Vector v)
        {
            // |  i   j   k  |   | t.y*v.z - t.z*v.y |
            // | t.x t.y t.z | = | t.z*v.x - t.x*v.z |
            // | v.x v.y v.z |   | t.x*v.y - t.y*v.x |

            return new Vector(
                y_ * v.Z - z_ * v.Y,
                z_ * v.X - x_ * v.Z,
                x_ * v.Y - y_ * v.X);
        }

        public bool IsParallel(Vector v)
        {
            return (System.Math.Abs(System.Math.Abs(Dot(v)) - Magnitude * v.Magnitude) < Constants.tol);
        }

        public Vector UnitNormal
        {
            get
            {
                Vector cv = new Vector(-Z, 0, X);
                cv.Normalize();
                return cv;
            }
        }

        public static Vector Zero
        {
            get
            {
                return new Vector(0.0, 0.0, 0.0);
            }
        }
    }

    public abstract class Curve
    {
        protected Point vertex = Point.Zero;
        protected Vector ihat = new Vector(1.0, 0.0, 0.0);
        protected Vector khat = new Vector(0.0, 0.0, 1.0);

        private double min = double.MinValue;
        private double max = double.MaxValue;

        public Point Vertex { get { return vertex; } }
        public Vector IHat { get { return ihat; } }
        public Vector KHat { get { return khat; } }

        public double Min { get { return min; } set { min = value; } }
        public double Max { get { return max; } set { max = value; } }

        public double Clamp(double t)
        {
            double clamp = t;

            clamp = (clamp > min) ? clamp : min;
            clamp = (clamp < max) ? clamp : max;

            return clamp;
        }

        public bool Contains(double t)
        {
            return ((min - Constants.tol <= t) && (t <= max + Constants.tol));
        }

        public bool Contains(Point p)
        {
            double distance = Distance(p);
            return (distance < Constants.tol);
        }

        public bool Between(Point p, Point limit1, Point limit2)
        {
            bool between = false;

            if (Contains(p))
            {
                double t = Project(p);
                double delta1 = t - Project(limit1);
                double delta2 = t - Project(limit2);

                between = delta1 * delta2 < -Constants.tol;
            }

            return between;
        }

        public double Distance(Point p)
        {
            double t = Clamp(Project(p));
            Point projection = GetPoint(t);

            return ((Vector)(p - projection)).Magnitude;
        }

        public double Project(Point p)
        {
            return ihat.Dot(p - vertex);
        }

        public abstract Curve Copy { get; }

        public abstract Point GetPoint(double t);

        public abstract CartesianList<Point> Intersect(Curve visitor);
        public abstract CartesianList<Point> Visit(Line line);
        public abstract CartesianList<Point> Visit(Parabola parbola);        
    }

    public class Line : Curve
    {
        public Line(Line template)
        {
            InitPointNormal(template.Vertex, template.KHat);
            //Min = template.Min;
            //Max = template.Max;
        }

        public Line(Point p, Vector normal)
        {
            InitPointNormal(p, normal);
        }

        //public Line(LineSite ls)
        //{
        //    InitTwoPoints(ls.Tail, ls.Head);
        //}

        public Line(Point p0, Point p1, bool setbound)
        {
            InitTwoPoints(p0, p1);

            if (setbound)
            {
                double t = Project(p0);
                double h = Project(p1);
                if (t < h)
                {
                    Min = t;
                    Max = h;
                }
                else
                {
                    Min = h;
                    Max = t;
                }
            }
        }

        private void InitPointNormal(Point p, Vector normal)
        {
            if (normal.Magnitude2 > 0.0)
            {
                khat = normal.Copy;
                khat.Normalize();
            }
            ihat = -khat.UnitNormal;

            vertex = khat.Dot(p) * khat; // origin projected on line
        }

        private void InitTwoPoints(Point p0, Point p1)
        {
            Vector normal = new Vector(0.0, 0.0, 1.0);

            Vector direction = p1 - p0;
            if (direction.Magnitude2 > 0.0)
            {
                normal = direction.UnitNormal;
            }

            InitPointNormal(p0, normal);
        }

        // a*x + b*z = c
        public double A { get { return khat.X; } }
        public double B { get { return khat.Z; } }
        public double C { get { return khat.Dot(vertex); } }

        public override Curve Copy { get { return new Line(this); } }

        public override Point GetPoint(double t)
        {
            return vertex + t * IHat;
        }

        public override CartesianList<Point> Intersect(Curve visitor)
        {
            return visitor.Visit(this);
        }

        public override CartesianList<Point> Visit(Line line)
        {
            CartesianList<Point> intersections = new CartesianList<Point>();

            //      this = 0       line = 1
            // x = v0 + t0 * i0 = v1 + t1 * i1
            // k0.dot(x) = k0.dot(v1) + t1 * k0.dot(i1) = k0.dot(v0)
            // k1.dot(x) = k1.dot(v0) + t0 * k1.dot(i0) = k1.dot(v1)
            // t0 = k1.dot(v1 - v0) / k1.dot(i0)
            // t1 = k0.dot(v0 - v1) / k0.dot(i1)
            
            double den0 = line.KHat.Dot(this.IHat);
            double den1 = this.KHat.Dot(line.IHat);
            if (System.Math.Abs(den0) > Constants.tol)
            {
                double t0 = line.KHat.Dot(line.Vertex - this.Vertex) / den0;
                double t1 = this.KHat.Dot(this.Vertex - line.Vertex) / den1;

                if (this.Contains(t0) && line.Contains(t1))
                {
                    intersections.Add(GetPoint(t0));
                }
            }

            return intersections;
        }

        public override CartesianList<Point> Visit(Parabola parabola)
        {
            CartesianList<Point> intersections = parabola.Visit(this);
            return intersections;
        }
    }

    public class Parabola : Curve
    {
        private Point focus = Point.Zero;
        private Line directrix = null;
        private double latusRectum = 0.0;

        public Parabola(Parabola template)
        {
            InitFocusDirectrix(template.Focus, template.Directrix);
        }

        public Parabola(Point focus, Line directrix)
        {
            InitFocusDirectrix(focus, directrix);
        }
        
        private void InitFocusDirectrix(Point focus, Line directrix)
        {
            this.focus = focus.DeepCopy;
            this.directrix = new Line(directrix);

            double t = directrix.Project(focus);
            Point projection = directrix.GetPoint(t);
            vertex = 0.5 * (focus + projection);

            khat = (focus - projection); // semi-latus rectum
            latusRectum = 2.0 * (khat).Magnitude;

            khat.Normalize();
            ihat = -khat.UnitNormal;
        }

        public override Curve Copy { get { return new Parabola(this); } }

        public Point Focus { get { return focus; } }
        public Line Directrix {  get { return directrix; } }

        public override Point GetPoint(double t)
        {
            double z = 0.0;
            if (latusRectum > 0.0)
            {
                z = t * t / latusRectum;
            }

            return vertex + t * ihat + z * khat;
        }

        public double LatusRectum { get { return latusRectum; } }

        public override CartesianList<Point> Intersect(Curve visitor)
        {
            return visitor.Visit(this);
        }

        public override CartesianList<Point> Visit(Line line)
        {
            CartesianList<Point> intersections = new CartesianList<Point>();
            List<double> solutions = new List<double>();

            // p = v + x * ihat + z * khat
            // n.dot(p) = n.dot(p0)
            // n.dot(v + x*ihat + z*khat) = n.dot(p0)
            // n.dot(ihat) * x + n.dot(khat) * z = n.dot(p0) - n.dot(v)
            // a*x + b*z = c

            Vector n = line.KHat;

            double a = n.Dot(ihat);
            double b = n.Dot(khat);
            double c = n.Dot(line.Vertex - vertex);

            // z = x^2 / r
            // a*x + b*x^2/r = c
            // t^2 + (a*r/b)*t - c*r/b = 0
            // t^2 + a*r/b*t + a^2*r^2/4*b^2 = a^2*r^2/4*b^2 + c*r/b
            // (t + a*r/2*b)^2 = (1/4*b^2)*(a^2*r^2 + 4*b*c*r)
            // t + a*r/2*b = +- sqrt(r*(a^2*r + 4*b*c)) / 2*b
            // t = (-a*r +- sqrt(r*(a^2*r + 4*b*c)) / 2*b

            double discriminant = latusRectum * (a * a * latusRectum + 4.0 * b * c);
            if (System.Math.Abs(b) > Constants.tol)
            {
                if (System.Math.Abs(discriminant) < Constants.tol)
                {
                    double t = -0.5 * a * latusRectum / b;
                    solutions.Add(t);
                }
                else if (discriminant > Constants.tol)
                {
                    double sqrt = System.Math.Sqrt(discriminant);
                    double t0 = -0.5 * (a * latusRectum - sqrt) / b;
                    double t1 = -0.5 * (a * latusRectum + sqrt) / b;

                    solutions.Add(t0);
                    solutions.Add(t1);
                }
                else
                {
                    // no solution
                }
            }
            else if (System.Math.Abs(a) > Constants.tol)
            {
                double t = c / a;
                solutions.Add(t);
            }

            foreach (double t in solutions)
            {
                if (this.Contains(t))
                {
                    Point p = GetPoint(t);
                    if (line.Contains(p))
                    {
                        intersections.Add(p);
                    }
                }
            }

            return intersections;
        }

        public override CartesianList<Point> Visit(Parabola parabola)
        {
            // points p on this parabola, p = t^2 / lr0
            // points q on parabola, q = s^2 / lr1
            // Debug.Log(string.Format("Oops! parabola parabola intersection not implemented"));
            CartesianList<Point> intersections = new CartesianList<Point>();
            return intersections;
        }
    }
}