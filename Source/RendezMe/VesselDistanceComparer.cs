using System.Collections.Generic;
using UnityEngine;

public class VesselDistanceComparer : IComparer<Vessel>
{
    public Vessel OriginVessel;

    public int Compare(Vessel a, Vessel b)
    {
        float aDist = Vector3.Distance(a.transform.position, OriginVessel.transform.position);
        float bDist = Vector3.Distance(b.transform.position, OriginVessel.transform.position);

        return aDist.CompareTo(bDist);
    }
    
}
