using System;

namespace Extensions
{
    internal static class Extensions
    {
        /// <summary>
        /// Eta to a given true anomaly.
        /// </summary>
        /// <returns>
        /// Time (seconds)
        /// </returns>
        /// <param name='orbit'>
        /// Orbit.
        /// </param>
        /// <param name='Tanom'>
        /// True anomaly. (0-360)
        /// </param>
        public static double GetTimeToTrue(this Orbit orbit, double Tanom)
        {
            double t = GetTimeAtTrue(orbit, Tanom);
            //t= orbit.GetTimeToMean (orbit.GetMeanAtEccentric (orbit.GetEccentricAtTrue ((360 - orbit.argumentOfPeriapsis))));
            double p = orbit.period;
            double now = p - orbit.timeToPe;
            if (now < t)
                return t - now;
            else
                return orbit.timeToPe + t;
        }

        /// <summary>
        /// Gets the time for a given true anomaly.
        /// </summary>
        /// <returns>
        /// The time untill the target true anomaly in seconds.
        /// </returns>
        /// <param name='TAnom'>
        /// True anomaly. (0-360)
        /// </param>
        public static double GetTimeAtTrue(this Orbit orbit, double TAnom)
        {
            return orbit.GetTimeAtMean(orbit.GetMeanAtEccentric(orbit.GetEccentricAtTrue(TAnom)));
        }

        /// <summary>
        /// Finds the coordinates of a state vector.
        /// </summary>
        /// <returns>
        /// Double[] {lattitude, longitude}
        /// </returns>
        /// <param name='vinput'>
        /// State vector
        /// </param>
        public static double[] LatLonofVector(Vector3d vinput)
        {
            //get the geocentric latitude and longitude of a orbital element vector
            double rad = Math.PI/180;
            double c1 = vinput.x;
            double c2 = vinput.y;
            double c3 = vinput.z;
            double lon = 0;

            double lat = Math.Atan(c3/Math.Pow((c1*c1) + (c2*c2), .5))/rad;
            if (c1 < 0)
            {
                lon = (Math.Atan(c2/c1)/rad) + 90;
            }
            else
            {
                lon = (Math.Atan(c2/c1)/rad) + 270;
            }
            var coord = new[] {lat, lon};
            return coord;
        }

        /// <summary>
        /// Orbit foo, this finds the nodes of two orbits
        /// </summary>
        /// <returns>
        /// The true anomaly of the ascending node(descing node is 180degrees off)
        /// </returns>
        /// <param name='orbit'>
        /// Orbit.
        /// </param>
        /// <param name='tgtorbit'>
        /// Target Orbit
        /// </param>
        public static double FindAN(Orbit orbit, Orbit tgtorbit)
        {
            double rad = Math.PI/180;
            double Lan1 = orbit.LAN;
            double inc1 = orbit.inclination;
            double Lan2 = tgtorbit.LAN;
            double inc2 = tgtorbit.inclination;

            //see braeunig.us/space... cross product of two orbital planes gives the node location
            var a = new Vector3d(Math.Sin(inc1*rad)*Math.Cos(Lan1*rad), Math.Sin(inc1*rad)*Math.Sin(Lan1*rad),
                                 Math.Cos(inc1*rad));
            var b = new Vector3d(Math.Sin(inc2*rad)*Math.Cos(Lan2*rad), Math.Sin(inc2*rad)*Math.Sin(Lan2*rad),
                                 Math.Cos(inc2*rad));
            var c = new Vector3d(0, 0, 0);
            c = Vector3d.Cross(a, b);
            var coord = new double[] {0, 0};
            coord = LatLonofVector(c); //get the coordinates lat/lon of the ascending node
            double lat = coord[0];
            double lon = coord[1];

            //go look at the diagrams at braeunig.us/space
            double α = lon - Lan1; //its all greek to me
            if (α < 0) α += 360;
            double λ = Math.Atan(Math.Tan(α*rad)/Math.Cos(inc1*rad))/rad;
            double x = 180 + (λ - orbit.argumentOfPeriapsis);
            if (x > 360) return 360 - x;
            else return x;
        }

        /// <summary>
        /// Eta to this orbits ascending node relative to a second orbit.
        /// </summary>
        /// <returns>
        /// The time to ascending node.
        /// </returns>
        /// <param name='orbit'>
        /// Orbit.
        /// </param>
        /// <param name='tgtorbit'>
        /// Target orbit
        /// </param>
        public static double GetTimeToRelAN(this Orbit orbit, Orbit tgtorbit)
        {
            return orbit.GetTimeToTrue(FindAN(orbit, tgtorbit));
        }

        /// <summary>
        /// ETA to this orbits descending node relative to a second orbit
        /// </summary>
        /// <returns>
        /// The time to descending node
        /// </returns>
        /// <param name='orbit'>
        /// Orbit.
        /// </param>
        /// <param name='tgtorbit'>
        /// Target orbit
        /// </param>
        public static double GetTimeToRelDN(this Orbit orbit, Orbit tgtorbit)
        {
            double an = FindAN(orbit, tgtorbit);
            double dn = an - 180;
            if (dn < 0) dn += 360;
            return orbit.GetTimeToTrue(dn);
        }


        /// <summary>
        /// ETA to ascending node (Refference Plane).
        /// </summary>
        /// <returns>
        /// Remaining time in seconds.
        /// </returns>
        public static double GetTimeToRefAN(this Orbit orbit)
        {
            return orbit.GetTimeToTrue(360 - orbit.argumentOfPeriapsis);
            //return orbit.GetTimeToMean (orbit.GetMeanAtEccentric (orbit.GetEccentricAtTrue ((360 - orbit.argumentOfPeriapsis))));		
        }

        /// <summary>
        /// ETA to descending node(Refference Plane).
        /// </summary>
        /// <returns>
        /// Remaining time in seconds
        /// </returns>
        public static double GetTimeToRefDN(this Orbit orbit)
        {
            if (orbit.argumentOfPeriapsis <= 180)
            {
                return orbit.GetTimeToTrue(180 - orbit.argumentOfPeriapsis);
            }
            else
                return orbit.GetTimeToTrue(520 - orbit.argumentOfPeriapsis);
        }

        /// <summary>
        /// Gets the mean anomaly at a time.
        /// </summary>
        /// <returns>
        /// The mean at time.
        /// </returns>
        /// <param name='orbit'>
        /// Orbit.
        /// </param>
        /// <param name='t'>
        /// Time from periapsis
        /// </param>
        public static double GetMeanAnomalyAtTime(this Orbit orbit, double t)
        {
            double tau = 2*Math.PI;
            double M = 0;
            double p = orbit.period;
            M = (t*tau)/p;
            return M;
        }

        /// <summary>
        /// Takes a true anomaly(degrees from Periapsis), and computes Eccentric Anomaly at that target
        ///undefined if eccentricity > 1
        /// </summary>
        /// <returns>
        /// Returns Eccentric Anomaly
        /// </returns>
        /// <param name='orb'>Orbit</param>">
        /// <param name='TrueAnom'>
        /// True anomomaly (0-360)
        /// </param>
        /// 
        public static double GetEccentricAtTrue(this Orbit orb, double TrueAnom)
        {
            double e = orb.eccentricity;
            double E = 0f;
            bool g180 = false;
            if (TrueAnom > 180) g180 = true;
            if (TrueAnom == 180) return Math.PI;

            TrueAnom = TrueAnom*(Math.PI/180);
            double cose;
            double sine;
            double tane;
            cose = (e + Math.Cos(TrueAnom))/(1 + e*Math.Cos(TrueAnom));
            sine = Math.Sqrt(1 - (cose*cose));
            tane = sine/cose;
            E = Math.Atan(tane);
            if (E < 0.0)
            {
                E += 3/2*Math.PI;
            }
            if (g180)
            {
                E = (2*Math.PI - E);
            }
            return E;
        }


        /// <summary>
        /// You provide the eccentric anomaly, this method will bring the mean.
        /// </summary>
        /// <returns>
        /// Mean anomaly
        /// </returns>
        /// <param name='E'>
        /// orbit.eccentricAnomaly (0-2pi)
        /// </param>
        public static double GetMeanAtEccentric(this Orbit orb, double E)
        {
            // M=E-e*SIN(E)
            double e = orb.eccentricity;
            double M = 0;
            M = E - (e*Math.Sin(E));
            return M;
        }

        /// <summary>
        /// Gets the time at a given mean anomaly.
        /// </summary>
        /// <returns>
        /// The time from periapsis that the vessel will be at this mean anomaly
        /// </returns>
        /// <param name='orbit'>
        /// Orbit.
        /// </param>
        /// <param name='M'>
        /// Mean anomaly (0-2pi)
        /// </param>
        ///<remarks>Not compatible with eccentricity >=1 </remarks>
        public static double GetTimeAtMean(this Orbit orbit, double M)
        {
            double tau = 2*Math.PI;
            double p = orbit.period;
            return M*p/(tau);
        }

        /// <summary>
        /// Time untill vessel reaches specified mean.
        /// </summary>
        /// <returns>Time untill vessel reaches mean anomaly in seconds</returns>
        /// <param name='M'>
        /// Mean anomaly
        /// </param>
        ///<remarks>Not compatible with eccentricyt >=1 </remarks>
        public static double GetTimeToMean(this Orbit orbit, double M)
        {
            double t;
            double p = orbit.period;
            double timeFromPe;
            t = GetTimeAtMean(orbit, M);
            timeFromPe = p - orbit.timeToPe;
            if (timeFromPe < t) return timeFromPe + t;
            else return orbit.timeToPe + t;
        }

        /// <summary>
        /// Translates a true anomaly from this orbit to the argument's. Orbits should have the same reference body.
        /// </summary>
        /// <returns>
        /// True anomaly on the target orbit that matches the input
        /// </returns>
        /// <param name='tgt_Orbit'>
        /// Target Orbit
        /// </param>
        /// <param name='anomaly'>
        /// True Anomaly from this orbit.(0-360)
        /// </param>
        public static double TranslateAnomaly(this Orbit orbit, Orbit tgt_Orbit, double anomaly)
        {
            //a2= a1+lan1+ar1-(lan2+arg2)
            double a = anomaly + orbit.argumentOfPeriapsis + orbit.LAN - (tgt_Orbit.argumentOfPeriapsis + tgt_Orbit.LAN);
            if (a < 0) a += 360;
            return a;
        }

        public static double Syncorbits(this Orbit orbit, Orbit tgt_orb, double anomaly, int orbitnum)
        {
            //translate true anomaly
            double tgta = orbit.TranslateAnomaly(tgt_orb, anomaly);
            //if t>target period subtract 1 period...
            double p = tgt_orb.period;
            //double tgtc = FlightGlobals.Vessels[selVessel].orbit.GetTimeAtMean(FlightGlobals.Vessels[selVessel].orbit.meanAnomaly);
            //double t = this.vessel.orbit.GetTimeToTrue(anomaly);
            return tgt_orb.GetTimeToTrue(tgta) + (p*orbitnum);
        }

        public static double f(this Orbit orbit, Orbit tgt_orbit, double x)
        {
            double rad = Math.PI/180;
            double a1 = orbit.semiMajorAxis;
            double e1 = orbit.eccentricity;
            double w1 = orbit.argumentOfPeriapsis;
            double l1 = orbit.LAN;
            double a2 = tgt_orbit.semiMajorAxis;
            double e2 = tgt_orbit.eccentricity;
            double w2 = tgt_orbit.argumentOfPeriapsis;
            double l2 = tgt_orbit.LAN;

            return ((a2 - (e1*e1))/(e2*Math.Cos((x + w1 + l1 - (w2 + l2))*rad) + 1)) -
                   ((a1 - (e2*e2))/(e1*Math.Cos(x*rad) + 1));
        }

        public static double fprime(this Orbit orbit, Orbit tgt_orbit, double x)
        {
            double rad = Math.PI/180;
            double a1 = orbit.semiMajorAxis;
            double e1 = orbit.eccentricity;
            double w1 = orbit.argumentOfPeriapsis;
            double l1 = orbit.LAN;
            double a2 = tgt_orbit.semiMajorAxis;
            double e2 = tgt_orbit.eccentricity;
            double w2 = tgt_orbit.argumentOfPeriapsis;
            double l2 = tgt_orbit.LAN;

            return ((a2 - (e1*e1))/(e2*-Math.Sin(x))) - ((a1 - (e2*e2))/(e1*-Math.Sin(x*rad)));
        }
    }
}