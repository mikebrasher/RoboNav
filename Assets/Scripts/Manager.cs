using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

namespace RoboNav
{
    public class Manager : MonoBehaviour
    {
        public Voronoi voronoi;
        public FastMarchingMethod fastMarchingMethod;
        public Robot robot;
        public Transform destCylinder;
        
        public Transform[] maze;
        public Transform spheres;
        public Transform cylinders;

        public Button nextButton;
        public Text stepText;
        public Text calcText;
        
        bool stepMode = false;

        private Polygon polygon;
        
        private int step = 0;
        private int numStep = 0;
        private State current = null;
        private bool performUpdate = false;
        private float stepDuration = 0.3f;

        private int arrivalStep = 64;
        private Point origin = new Point(-9.0, 0.0, 9.0);
        private Point destination = new Point(9.0, 0.0, 9.0);

        private abstract class State
        {
            public abstract State Update();
        }

        private class AddPointSite : State
        {
            private Manager outer = null;
            private IEnumerator<string> generator = null;

            public AddPointSite(Manager outer, IEnumerator<string> generator)
            {
                this.outer = outer;
                this.generator = generator;
            }

            public override State Update()
            {
                State state = this;

                // incrementally add point sites, ps
                //   choose seed node v that maximizes rho_v - delta_v
                //   delete adjacent nodes if their clearance disk contains ps
                //   keep recursively deleting nodes
                //   compute new nodes along edges which have been cut
                bool keepSteppingPoints = generator.MoveNext();
                //Debug.Log(string.Format("key: {0}", addPointGenerator.Current));

                outer.voronoi.Display();

                if (keepSteppingPoints)
                {
                    outer.IncrementStep();
                }
                else
                {
                    state = new AddLineSite(outer, outer.voronoi.AddLineSites());
                }

                return state;
            }
        }

        private class AddLineSite : State
        {
            private Manager outer = null;
            private IEnumerator<string> generator = null;

            public AddLineSite(Manager outer, IEnumerator<string> generator)
            {
                this.outer = outer;
                this.generator = generator;
            }

            public override State Update()
            {
                State state = this;

                // incrementally add line sites
                bool keepSteppingLines = generator.MoveNext();
                outer.voronoi.Display();

                if (keepSteppingLines)
                {
                    outer.IncrementStep();
                }
                else
                {
                    state = new CalculateViscosity(outer);
                    //state = new Idle();
                }

                return state;
            }
        }

        private class CalculateViscosity : State
        {
            private Manager outer = null;

            public CalculateViscosity(Manager outer)
            {
                this.outer = outer;
            }

            public override State Update()
            {
                State state = this;

                NodeList skeleton = outer.voronoi.ExtractSkeleton();
                outer.voronoi.Display(skeleton);

                outer.fastMarchingMethod.CalculateViscosity();
                outer.IncrementStep();

                state = new InitializeArrival(outer, outer.destination);
                //state = new Idle();

                return state;
            }
        }

        private class InitializeArrival : State
        {
            private Manager outer = null;
            private Point destination = Point.Zero;

            public InitializeArrival(Manager outer, Point destination)
            {
                this.outer = outer;
                this.destination = destination;
            }

            public override State Update()
            {
                State state = this;

                outer.voronoi.ClearDiagram();

                outer.fastMarchingMethod.Destination = destination;
                outer.fastMarchingMethod.InitializeArrival();

                outer.fastMarchingMethod.DisplayArrival();
                outer.IncrementStep();

                state = new CalculateArrival(outer, outer.fastMarchingMethod.CalculateArrival(outer.arrivalStep));

                return state;
            }
        }

        private class CalculateArrival : State
        {
            private Manager outer = null;
            private IEnumerator<int> generator = null;

            public CalculateArrival(Manager outer, IEnumerator<int> generator)
            {
                this.outer = outer;
                this.generator = generator;
            }

            public override State Update()
            {
                State state = this;
                
                bool calculateArrival = generator.MoveNext();
                outer.fastMarchingMethod.DisplayArrival();

                if (calculateArrival)
                {
                    outer.IncrementStep();
                }
                else
                {
                    state = new CalculatePath(outer, outer.origin);
                    //state = new Idle();
                }

                return state;
            }
        }

        private class CalculatePath : State
        {
            private Manager outer = null;
            private Point origin = Point.Zero;

            public CalculatePath(Manager outer, Point origin)
            {
                this.outer = outer;
                this.origin = origin;
            }

            public override State Update()
            {
                State state = this;

                outer.fastMarchingMethod.Origin = origin;
                outer.fastMarchingMethod.UpdatePath();
                outer.fastMarchingMethod.DisplayPath();
                outer.IncrementStep();

                state = new Idle();

                return state;
            }
        }

        private class SkipToViscosity : State
        {
            private Manager outer = null;

            public SkipToViscosity(Manager outer)
            {
                this.outer = outer;
            }

            public override State Update()
            {
                State state = this;

                // add point sites
                IEnumerator<string> addPointGenerator = outer.voronoi.AddPointSites();
                while (addPointGenerator.MoveNext()) { }

                // add line sites
                IEnumerator<string> addLineGenerator = outer.voronoi.AddLineSites();
                while (addLineGenerator.MoveNext()) { }

                // display skeleton
                //NodeList skeleton = outer.voronoi.ExtractSkeleton();
                //outer.voronoi.Display(skeleton);

                // calculate viscosity
                outer.fastMarchingMethod.CalculateViscosity();

                outer.calcText.gameObject.SetActive(false);
                //state = new InitializeArrival(outer, outer.destination);
                state = new Idle();

                return state;
            }
        }

        private class LoadViscosity : State
        {
            private Manager outer = null;

            public LoadViscosity(Manager outer)
            {
                this.outer = outer;
            }

            public override State Update()
            {
                State state = this;

                // calculate viscosity
                outer.fastMarchingMethod.LoadViscosity();

                state = new Idle();

                return state;
            }
        }

        private class SkipToPath : State
        {
            private Manager outer = null;
            private Point origin = Point.Zero;
            private Point destination = Point.Zero;

            public SkipToPath(Manager outer, Point origin, Point destination)
            {
                this.outer = outer;
                this.origin = origin;
                this.destination = destination;
            }

            public override State Update()
            {
                State state = this;

                // initialize arrival
                outer.fastMarchingMethod.Destination = destination;
                outer.fastMarchingMethod.InitializeArrival();

                // calculate arrival
                IEnumerator<int> arrivalGenerator = outer.fastMarchingMethod.CalculateArrival(outer.arrivalStep);
                while (arrivalGenerator.MoveNext()) { }

                // calculate path
                outer.fastMarchingMethod.Origin = origin;
                outer.fastMarchingMethod.UpdatePath();

                outer.fastMarchingMethod.DisplayArrival();
                outer.fastMarchingMethod.DisplayPath();

                outer.robot.Path = outer.fastMarchingMethod.Path;

                state = new Idle();

                return state;
            }
        }

        private class Wait : State
        {
            private State previous = null;

            private float start = Time.time;
            private float duration = 0.0f;

            public Wait(State previous, float duration)
            {
                this.previous = previous;
                this.duration = duration;
            }

            public override State Update()
            {
                State state = this;

                float elapsed = Time.time - start;
                if (elapsed > duration)
                {
                    state = previous;
                }

                return state;
            }
        }

        private class Idle : State
        {
            public Idle()
            {
            }

            public override State Update()
            {
                State state = this;
                return state;
            }
        }

        public void IncrementStep()
        {
            ++step;
        }

        void Awake()
        {
            polygon = new Polygon(maze);
            polygon.Generate();

            voronoi.Polygon = polygon;
            voronoi.GenerateSites();
            //voronoi.DisplayAllSites();

            // initialize with bounding dummy points
            voronoi.InitializeNodes();

            if (stepMode)
            {
                nextButton.gameObject.SetActive(false);
                //performUpdate = true;

                stepText.gameObject.SetActive(true);
                calcText.gameObject.SetActive(false);
                robot.gameObject.SetActive(false);

                voronoi.Display();

                int narrival = FastMarchingMethod.size * FastMarchingMethod.size / arrivalStep;
                numStep = voronoi.PointSites.Count + voronoi.LineSites.Count + narrival + 2;

                current = new AddPointSite(this, voronoi.AddPointSites());
            }
            else
            {
                nextButton.gameObject.SetActive(false);
                stepText.gameObject.SetActive(false);
                calcText.gameObject.SetActive(true);

                performUpdate = true;
                //current = new SkipToViscosity(this);
                current = new LoadViscosity(this);
            }
        }

        void Start()
        {
            nextButton.onClick.AddListener(NextClick);
        }

        private void NextClick()
        {
            performUpdate = true;
        }

        private Vector3 ClipDestination(Vector3 world)
        {
            double x = world.x;
            if (x < -Constants.wall)
            {
                x = -Constants.wall;
            }
            else if (x > Constants.wall)
            {
                x = Constants.wall;
            }

            double z = world.z;
            if (z < -Constants.wall)
            {
                z = -Constants.wall;
            }
            else if (z > Constants.wall)
            {
                z = Constants.wall;
            }

            return new Vector3((float)x, 0.0f, (float)z);
        }

        void Update()
        {
            //if (Input.GetKey(KeyCode.Space))
            //{
            //    performUpdate = true;
            //}

            if (performUpdate)
            {
                bool needToWait = false;
                if (current == null)
                {
                    current = new Idle();
                }
                else
                {
                    needToWait = !(current is Wait);
                    current = current.Update();
                }

                if (stepMode)
                {
                    //performUpdate = false;
                    stepText.text = string.Format("Step: {0} / {1}", step, numStep);

                    if (needToWait)
                    {
                        current = new Wait(current, stepDuration);
                    }
                }
                else
                {
                    bool haveInput = false;
                    Vector3 screen = Vector3.zero;

                    if (Input.GetMouseButtonDown(0))
                    {
                        haveInput = true;
                        screen = Input.mousePosition;
                    }
                    else if (Input.touchCount > 0)
                    {
                        haveInput = true;
                        screen = Input.GetTouch(0).position;
                    }

                    if (haveInput)
                    {
                        Vector3 world = Camera.main.ScreenToWorldPoint(screen);
                        Vector3 dest = ClipDestination(world);

                        destCylinder.gameObject.SetActive(true);
                        destCylinder.position = dest;

                        destination = (Point)dest;
                        origin = (Point)robot.transform.position;

                        current = new SkipToPath(this, origin, destination);
                    }
                }
            }
        }
    }
}