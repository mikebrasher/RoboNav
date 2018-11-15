using System.Collections.Generic;
using UnityEngine;
using System.IO;

namespace RoboNav
{
    public enum Flag
    {
        Far,
        Considered,
        Accepted
    }

    public class FastMarchingMethod : MonoBehaviour
    {
        public Voronoi diagram;
        public const int size = 128;

        private MeshRenderer meshRenderer;
        private Texture2D blankTexture;
        private Texture2D viscosityTexture;
        private Texture2D arrivalTexture;
        private Texture2D pathTexture;

        private string viscosityResource;
        private string viscosityPNG;

        private double highViscosity = 10.0;
        private double lowViscosity = 1.0;

        private Color32 lowColor;
        private Color32 highColor;
        private Color32 maxColor;

        private double xoff = 0.0;
        private double zoff = 0.0;

        private double dh = 1.0;

        private double[,] viscosity = new double[size, size];
        private double[,] arrival = new double[size, size];

        private Point origin = Point.Zero;
        private Point destination = Point.Zero;
        private Pixel destPixel = null;
        private List<Point> path = new List<Point>();

        private Dictionary<int, Pixel> all = new Dictionary<int, Pixel>();
        private Dictionary<int, Pixel> far = new Dictionary<int, Pixel>();
        private Dictionary<int, Pixel> considered = new Dictionary<int, Pixel>();
        private Dictionary<int, Pixel> accepted = new Dictionary<int, Pixel>();

        private ColorMap colormap = new ColorMap();

        private class Pixel
        {
            FastMarchingMethod outer = null;

            private int row = 0;
            private int col = 0;
            private double viscosity = double.MaxValue;
            private double arrival = double.MaxValue;
            private Flag flag = Flag.Far;

            public Pixel(FastMarchingMethod outer, int row, int col, double viscosity)
            {
                this.outer = outer;
                this.row = row;
                this.col = col;
                this.viscosity = viscosity;
            }

            public int Row { get { return row; } }
            public int Col { get { return col; } }
            public int Key { get { return GetKey(row, col); } }

            public Point Position
            {
                get
                {
                    return outer.LookupPoint(row, col);
                }
            }

            public double Viscosity { get { return viscosity; } }
            public double Arrival { get { return arrival; } set { arrival = value; } }
            public Flag Flag { get { return flag; } }

            public Pixel North
            {
                get
                {
                    Pixel north = null;

                    int key = GetKey(row, col - 1);
                    if (outer.all.ContainsKey(key))
                    {
                        north = outer.all[key];
                    }

                    return north;
                }
            }

            public Pixel East
            {
                get
                {
                    Pixel east = null;

                    int key = GetKey(row - 1, col);
                    if (outer.all.ContainsKey(key))
                    {
                        east = outer.all[key];
                    }

                    return east;
                }
            }

            public Pixel South
            {
                get
                {
                    Pixel south = null;

                    int key = GetKey(row, col + 1);
                    if (outer.all.ContainsKey(key))
                    {
                        south = outer.all[key];
                    }

                    return south;
                }
            }

            public Pixel West
            {
                get
                {
                    Pixel west = null;

                    int key = GetKey(row + 1, col);
                    if (outer.all.ContainsKey(key))
                    {
                        west = outer.all[key];
                    }

                    return west;
                }
            }

            public List<Pixel> Neighbors
            {
                get
                {
                    List<Pixel> neighbors = new List<Pixel>();
                    
                    Pixel north = North;
                    if (north != null)
                    {
                        neighbors.Add(north);
                    }

                    Pixel east = East;
                    if (east != null)
                    {
                        neighbors.Add(east);
                    }

                    Pixel south = South;
                    if (south != null)
                    {
                        neighbors.Add(south);
                    }

                    Pixel west = West;
                    if (west != null)
                    {
                        neighbors.Add(west);
                    }

                    return neighbors;
                }
            }

            public double EikonalUpdate()
            {
                double update = double.MaxValue;

                Pixel north = North;
                Pixel east = East;
                Pixel south = South;
                Pixel west = West;

                double uh = double.MaxValue;
                if ((east != null) && (east.Arrival < uh))
                {
                    uh = east.Arrival;
                }

                if ((west != null) && (west.Arrival < uh))
                {
                    uh = west.Arrival;
                }

                double uv = double.MaxValue;
                if ((north != null) && (north.Arrival < uv))
                {
                    uv = north.Arrival;
                }

                if ((south != null) && (south.Arrival < uv))
                {
                    uv = south.Arrival;
                }

                if ((uh < double.MaxValue) || (uv < double.MaxValue))
                {
                    double hnu = outer.dh * viscosity;
                    if (System.Math.Abs(uh - uv) < hnu)
                    {
                        double sum = uh + uv;
                        double discrim = sum * sum - 2.0 * (uh * uh + uv * uv - hnu * hnu);
                        update = 0.5 * (sum + System.Math.Sqrt(discrim));
                    }
                    else
                    {
                        update = (uh < uv) ? uh : uv;
                        update += hnu;
                    }
                }

                return update;
            }

            public double GradX
            {
                get
                {
                    double gradx = 0.0;
                    
                    Pixel east = East;
                    Pixel west = West;

                    if ((east != null) && (east.Arrival < double.MaxValue))
                    {
                        double right = (east.arrival - arrival) / outer.dh;
                        gradx = right;
                    }

                    if ((west != null) && (west.Arrival < double.MaxValue))
                    {
                        double left = (arrival - west.Arrival) / outer.dh;
                        gradx = (gradx < left) ? gradx : left;
                    }

                    return gradx;
                }
            }

            public double GradZ
            {
                get
                {
                    double gradz = 0.0;

                    Pixel north = North;
                    Pixel south = South;

                    if ((north != null) && (north.Arrival < double.MaxValue))
                    {
                        double up = (north.arrival - arrival) / outer.dh;
                        gradz = up;
                    }

                    if ((south != null) && (south.Arrival < double.MaxValue))
                    {
                        double down = (arrival - south.Arrival) / outer.dh;
                        gradz = (gradz < down) ? gradz : down;
                    }

                    return gradz;
                }
            }

            public Vector Gradient
            {
                get
                {
                    return new Vector(GradX, 0.0, GradZ);
                }
            }

            public void SetConsidered()
            {
                int key = Key;

                if (outer.far.ContainsKey(key))
                {
                    outer.far.Remove(key);
                }
                else if (outer.accepted.ContainsKey(key))
                {
                    outer.accepted.Remove(key);
                }

                if (!outer.considered.ContainsKey(key))
                {
                    outer.considered.Add(key, this);
                }

                flag = Flag.Considered;
            }

            public void SetAccepted()
            {
                int key = Key;

                if (outer.far.ContainsKey(key))
                {
                    outer.far.Remove(key);
                }
                else if (outer.considered.ContainsKey(key))
                {
                    outer.considered.Remove(key);
                }

                if (!outer.accepted.ContainsKey(key))
                {
                    outer.accepted.Add(key, this);
                }

                flag = Flag.Accepted;
            }

            public static int GetKey(int ii, int jj)
            {
                int key = -1;

                if ((0 <= ii) && (ii < size) &&
                    (0 <= jj) && (jj < size))
                {
                    key = size * jj + ii;
                }

                return key;
            }
        }

        void Awake()
        {
            blankTexture = new Texture2D(size, size);

            Color32 fillColor = Color.gray;

            int npixel = blankTexture.width * blankTexture.height;
            Color32[] pixels = new Color32[npixel];
            for (int ipixel = 0; ipixel < npixel; ++ipixel)
            {
                pixels[ipixel] = fillColor;
            }

            blankTexture.SetPixels32(pixels);
            blankTexture.Apply(false);

            viscosityTexture = new Texture2D(size, size);
            arrivalTexture = new Texture2D(size, size);
            pathTexture = new Texture2D(size, size);

            colormap.Min = 0.0;
            colormap.Max = 2 * highViscosity;

            lowColor = colormap.Lookup(lowViscosity);
            highColor = colormap.Lookup(highViscosity);
            maxColor = colormap.Lookup(double.MaxValue);

            //for (int ii = 0; ii < texture.width; ++ii)
            //{
            //    for (int jj = 0; jj < texture.height; ++jj)
            //    {
            //        texture.SetPixel(ii, jj, Color.gray);
            //    }
            //}

            xoff = transform.position.x;
            zoff = transform.position.z;

            // plane template is 10x10 units
            double planeSize = 10 * transform.localScale.x;
            dh = planeSize / size;

            xoff = 0.5 * dh * (size - 1.0);
            zoff = 0.5 * dh * (size - 1.0);

            meshRenderer = GetComponent<MeshRenderer>();
            meshRenderer.material.SetTexture("_MainTex", blankTexture);

            viscosityResource = string.Format("viscosity{0}", size);
            viscosityPNG = string.Format("{0}/Resources/{1}.png", Application.dataPath, viscosityResource);
        }

        public Point Origin { get { return origin; } set { origin = value; } }
        public Point Destination { get { return destination; } set { destination = value; } }
        public List<Point> Path { get { return path; } }

        public Point LookupPoint(int ii, int jj)
        {
            double x = -ii * dh + xoff;
            double z = -jj * dh + zoff;

            return new Point(x, 0.0, z);
        }

        public int GetKey(Point p)
        {
            double row = (xoff - p.X) / dh;
            double col = (zoff - p.Z) / dh;
            return Pixel.GetKey((int)row, (int)col);
        }

        private bool ColorCompare(Color32 expect, Color32 actual)
        {
            double delr = expect.r - actual.r;
            double delg = expect.g - actual.g;
            double delb = expect.b - actual.b;
            double dela = expect.a - actual.a;

            double del2 = delr * delr + delg * delg + delb * delb + dela * dela;
            return (del2 < Constants.tol);
        }

        public void LoadViscosity()
        {
            viscosityTexture = Resources.Load(viscosityResource) as Texture2D;
            meshRenderer.material.SetTexture("_MainTex", viscosityTexture);

            Color32[] pixels = viscosityTexture.GetPixels32();
            for (int ii = 0; ii < size; ++ii)
            {
                for (int jj = 0; jj < size; ++jj)
                {
                    int key = Pixel.GetKey(ii, jj);
                    Color32 current = pixels[key];
                    if (ColorCompare(lowColor, current))
                    {
                        viscosity[ii, jj] = lowViscosity;
                    }
                    else if (ColorCompare(highColor, current))
                    {
                        viscosity[ii, jj] = highViscosity;
                    }
                    else
                    {
                        viscosity[ii, jj] = double.MaxValue;
                    }
                }
            }
        }

        public void CalculateViscosity()
        {
            NodeList skeleton = diagram.ExtractSkeleton();
            List<Edge> edges = skeleton.Edges;

            Color32[] pixels = new Color32[size * size];
            for (int ii = 0; ii < size; ++ii)
            {
                for (int jj = 0; jj < size; ++jj)
                {
                    int key = Pixel.GetKey(ii, jj);

                    Point p = LookupPoint(ii, jj);

                    viscosity[ii, jj] = highViscosity;
                    Color32 color = highColor;

                    if (diagram.Polygon.SignedDistance(p) < -Constants.tol)
                    {
                        // outside the polygon
                        viscosity[ii, jj] = double.MaxValue;
                        color = maxColor;
                    }
                    else
                    {
                        double min = double.MaxValue;
                        foreach (Edge e in edges)
                        {
                            double dist = e.Distance(p);
                            min = (dist < min) ? dist : min;
                        }

                        if (min < 0.25)
                        {
                            // close to the skeleton
                            viscosity[ii, jj] = lowViscosity;
                            color = lowColor;
                        }
                    }

                    pixels[key] = color;
                }
            }

            viscosityTexture.SetPixels32(pixels);
            viscosityTexture.Apply(false);
            meshRenderer.material.SetTexture("_MainTex", viscosityTexture);

            byte[] bytes = viscosityTexture.EncodeToPNG();
            File.WriteAllBytes(viscosityPNG, bytes);
        }

        public void InitializeArrival()
        {
            all.Clear();
            far.Clear();
            considered.Clear();
            accepted.Clear();

            // step 1
            for (int ii = 0; ii < size; ++ii)
            {
                for (int jj = 0; jj < size; ++jj)
                {
                    int key = Pixel.GetKey(ii, jj);

                    Pixel p = new Pixel(this, ii, jj, viscosity[ii, jj]);
                    all.Add(key, p);
                    far.Add(key, p);
                }
            }

            int destkey = GetKey(destination);
            destPixel = far[destkey];
            destPixel.Arrival = 0.0;
            destPixel.SetAccepted();

            // step 2
            foreach (Pixel n in destPixel.Neighbors)
            {
                double update = n.EikonalUpdate();
                if (update < n.Arrival)
                {
                    n.Arrival = update;
                    n.SetConsidered();
                }
            }
        }

        private void PerformStep()
        {
            // step 3
            double minArrival = double.MaxValue;
            Pixel min = null;
            foreach (Pixel c in considered.Values)
            {
                if (c.Arrival < minArrival)
                {
                    minArrival = c.Arrival;
                    min = c;
                }
            }

            min.SetAccepted();

            // step 4
            foreach (Pixel neighbor in min.Neighbors)
            {
                if (neighbor.Flag != Flag.Accepted)
                {
                    double update = neighbor.EikonalUpdate();
                    if (update < neighbor.Arrival)
                    {
                        neighbor.Arrival = update;
                        if (neighbor.Flag == Flag.Far)
                        {
                            neighbor.SetConsidered();
                        }
                    }
                }
            }
        }

        public IEnumerator<int> CalculateArrival(int nstep)
        {
            int istep = 0;
            int jstep = 0;

            while (considered.Count > 0)
            {
                PerformStep();

                ++istep;
                ++jstep;
                if (istep >= nstep)
                {
                    istep = 0;
                    yield return jstep;
                }
            }
        }

        public void DisplayArrival()
        {
            double maxArrival = double.MinValue;
            foreach (Pixel a in accepted.Values)
            {
                if (a.Arrival < 1.0e10) //double.MaxValue)
                {
                    maxArrival = (a.Arrival > maxArrival) ? a.Arrival : maxArrival;
                }
            }
            colormap.Min = 0.0;
            colormap.Max = 100.0; // maxArrival;

            Color32[] pixels = new Color32[size * size];
            for (int ii = 0; ii < size; ++ii)
            {
                for (int jj = 0; jj < size; ++jj)
                {
                    int key = Pixel.GetKey(ii, jj);
                    Pixel p = all[key];

                    double current = double.MaxValue;
                    if (p.Flag == Flag.Accepted)
                    {
                        current = p.Arrival;
                    }

                    arrival[ii, jj] = current;
                    pixels[key] = colormap.Lookup(current);
                }
            }

            arrivalTexture.SetPixels32(pixels);
            arrivalTexture.Apply(false);
            meshRenderer.material.SetTexture("_MainTex", arrivalTexture);
        }

        public Vector Gradient(Point p)
        {
            Vector gradient = Vector.Zero;

            double row = (xoff - p.X) / dh;
            double col = (zoff - p.Z) / dh;

            double fx = row - System.Math.Floor(row);
            double fz = col - System.Math.Floor(col);

            int key = Pixel.GetKey((int)row, (int)col);
            if (all.ContainsKey(key))
            {
                Pixel p00 = all[key];
                Pixel p10 = p00.East;
                Pixel p01 = p00.North;
                Pixel p11 = p01.East;

                Vector grad00 = p00.Gradient;
                Vector grad01 = p01.Gradient;
                Vector grad10 = p10.Gradient;
                Vector grad11 = p11.Gradient;

                Vector gradx0 = grad00 * (1.0 - fx) + grad10 * fx;
                Vector gradx1 = grad01 * (1.0 - fx) + grad11 * fx;

                gradient = gradx0 * (1.0 - fz) + gradx1 * fz;
                //gradient = p00.Gradient;

                gradient = -gradient;
                gradient.Normalize();

            }

            return gradient;
        }

        //public void UpdatePath()
        //{
        //    path.Clear();
        //    path.Add(origin);

        //    Point position = origin;

        //    double stepsize = 0.5;
        //    int maxstep = 1000;
        //    for (int istep = 0; istep < maxstep; ++istep)
        //    {
        //        Vector gradient = Gradient(position);
        //        position = position + gradient * stepsize;
        //        path.Add(position);

        //        Vector remaining = destination - position;
        //        if (remaining.Magnitude < stepsize)
        //        {
        //            path.Add(destination);
        //            break;
        //        }
        //    }
        //}

        public void UpdatePath()
        {
            path.Clear();
            path.Add(origin);

            Pixel position = null;

            int originKey = GetKey(origin);
            if (all.ContainsKey(originKey))
            {
                position = all[originKey];
                path.Add(position.Position);
            }

            int maxstep = 1000;
            for (int istep = 0; istep < maxstep; ++istep)
            {
                // find neighbor with miniumum arrival time
                foreach (Pixel neighbor in position.Neighbors)
                {
                    if (neighbor.Arrival < position.Arrival)
                    {
                        position = neighbor;
                    }
                }

                path.Add(position.Position);

                if (position == destPixel)
                {
                    break;
                }
            }
        }

        public void DisplayPath()
        {            
            Color32[] pixels = arrivalTexture.GetPixels32();
            foreach (Point p in path)
            {
                int key = GetKey(p);
                if ((0 <= key) && (key < pixels.Length))
                {
                    pixels[key] = Color.green;
                }
            }

            pathTexture.SetPixels32(pixels);
            pathTexture.Apply(false);
            meshRenderer.material.SetTexture("_MainTex", pathTexture);
        }

        void Start()
        {
        }

        void Update()
        {
        }
    }
}