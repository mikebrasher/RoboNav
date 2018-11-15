using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace RoboNav
{
    public class Interval
    {
        private double lo = double.NegativeInfinity;
        private double hi = double.PositiveInfinity;

        private bool loClosed = false;
        private bool hiClosed = false;

        public Interval(double lo, bool loClosed, double hi, bool hiClosed)
        {
            this.lo = lo;
            this.hi = hi;

            this.loClosed = loClosed;
            this.hiClosed = hiClosed;
        }

        public double Lo { get { return lo; } set { lo = value; } }
        public double Hi { get { return hi; } set { hi = value; } }

        public bool Contains(double d)
        {
            return (
                (loClosed && (d == lo)) ||
                (hiClosed && (d == hi)) ||
                ((d > lo) && (d < hi)));
        }


    }

    //public class Set
    //{
    //    private List<Interval> intervals = new List<Interval>();

    //    public Set Copy
    //    {
    //        get
    //        {
    //            Set copy = new Set();
                
    //            foreach (Interval i in intervals)
    //            {
    //                copy.Add(i);
    //            }

    //            return copy;
    //        }
    //    }

    //    public void Add(Interval i)
    //    {
    //        foreach (Interval j in intervals)
    //        {
    //            if ((j.Hi >= i.Lo) && (i.Hi >= j.Lo))
    //            {

    //            }
    //        }

    //    }

    //    public static Set operator +(Set s, Interval i)
    //    {
    //        Set add = s.Copy;
    //    }
    //}

}
