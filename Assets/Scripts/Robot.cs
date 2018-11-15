using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace RoboNav
{
    public class Robot : MonoBehaviour
    {
        private List<Point> path = new List<Point>();
        private int step = 0;

        private Vector3 goal = Vector3.zero;

        private float speed = 10.0f;

        public List<Point> Path
        {
            get
            {
                return path;
            }

            set
            {
                path = value;
                step = 0;
                SetGoal();
            }
        }

        public Vector3 Goal { get { return goal; } }

        private void SetGoal()
        {
            if (step < path.Count)
            {
                goal = (Vector3)path[step];
                ++step;
            }
        }

        void Awake()
        {
            goal = transform.position;
        }

        void Start()
        {
        }

        void Update()
        {
            Vector3 delta = goal - transform.position;
            if (delta.magnitude < Constants.tol)
            {
                SetGoal();
            }

            float step = speed * Time.deltaTime;
            transform.position = Vector3.MoveTowards(transform.position, goal, step);
        }
    }
}