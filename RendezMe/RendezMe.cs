using System;
using Extensions;
using UnityEngine;

public class RendezMe : Part
{
    public enum Sinc
    {
        TGT_PER,
        TGT_APO,
        SHP_PER,
        SHP_APO,
        //INTERSECT1,
        //INTERSECT2,
    }

    private const int SincCount = 4;

    public enum UIMode
    {
        OFF,
        VESSELS,
        SELECTED,
        RENDEZVOUS,
        ALIGN,
        SYNC,
    }

    public enum Orient
    {
        OFF,
        RVEL,
        RVELNEG,
        TGT,
        TGTNEG,
        Normal,
        AntiNormal,
    }

    public string[] RendStrings = new[]
                                      {
                                          "RVel+", "Rvel-", "TGT+", "TGT-"
                                      };

    public string[] AlignStrings = new[]
                                       {
                                           "NML\n+", "NML\n-"
                                       };

    public bool Debug;

    public Orbit Target = new Orbit();
    public Orient TestTarget = Orient.OFF;
    public Orient PointAt = Orient.OFF;
    public Sinc SyncMode = Sinc.TGT_PER;

    #region UI State

    protected Rect windowPos;

    public int GridSelect;
    public bool ModeChanged;

    private Vector2 _scrollPosition = new Vector2(0, 0);
    private UIMode _mode = UIMode.OFF;

    private int _selstring;

    #endregion

    #region Sync State
    private double _min; // used in sync
    private double _ran = 180; // used in sync
    private readonly float[] _shTor = new float[4]; // used in sync
    private readonly float[] _tgTor = new float[4]; // used in sync
    private readonly string[] _syncString = new string[4];
    private int _closestApproachOrbit; // used in sync
    #endregion

    private double _timeCounter;

    public UIMode Mode
    {
        get { return _mode; }
        set
        {
            if (_mode == value) 
                return;

            _mode = value;
            ModeChanged = true;
        }
    }

    /// <summary>
    /// The selected vessel's location in FlightGlobals.Vessels[LIST].
    /// </summary>
    private int _selectedVesselIndex;

    /// <summary>
    /// Unique Instance ID of selected vessel
    /// </summary>
    private int _selVesselId;

    private float _relativeVelocityMagnitude;
    private float _relativeVelocity;
    //	private float prev_rvel;
    private float _relativeInclination;
    private Vector3 _vectorToTarget;
    private float _targetDistance;
    private float _prevTargetDistance;
    //private int thisVessel;
    private const int windowIDbase = 18420;
    // SAS
    private Vector3 _tgtFwd;
    private Vector3 _tgtUp;
    private bool _flyByWire;
    private Vector3 _deriv = Vector3.zero;
    private Vector3 _integral = Vector3.zero;
    private Vector3 _err = Vector3.zero;
    private Vector3 _prevErr = Vector3.zero;
    private Vector3 _act = Vector3.zero;
    private Vector3 _kIntegral = Vector3.zero;
    private Vector3 _kPrevErr = Vector3.zero;
    
    //  private double t_integral = 0;
    //  private double t_prev_err = 0;
    
    public float Kp = 20.0F;
    public float Ki;
    public float Kd = 40.0F;
    public float k_Kp = 13.0F;
    public float k_Ki;
    public float k_Kd;
    public float t_Kp = 1.0F;
    public float t_Ki;
    public float t_Kd;
    public float Damping;

    // Note that this function is increasing for x < 1 and decreasing for x > 1.


    // Derivative of f
    private static double fprime(double x)
    {
        return (1.0 - x)*Math.Exp(-x);
    }

    private void drive(FlightCtrlState s)
    {
        if (_flyByWire)
        {
            Quaternion tgt = Quaternion.LookRotation(_tgtFwd, _tgtUp);
            Quaternion delta =
                Quaternion.Inverse(Quaternion.Euler(90, 0, 0)*Quaternion.Inverse(vessel.transform.rotation)*tgt);

            _err =
                new Vector3((delta.eulerAngles.x > 180) ? (delta.eulerAngles.x - 360.0F) : delta.eulerAngles.x,
                            (delta.eulerAngles.y > 180) ? (delta.eulerAngles.y - 360.0F) : delta.eulerAngles.y,
                            (delta.eulerAngles.z > 180) ? (delta.eulerAngles.z - 360.0F) : delta.eulerAngles.z)/180.0F;
            _integral += _err*TimeWarp.fixedDeltaTime;
            _deriv = (_err - _prevErr)/TimeWarp.fixedDeltaTime;
            _act = Kp*_err + Ki*_integral + Kd*_deriv;
            _prevErr = _err;

            s.pitch = Mathf.Clamp(s.pitch + _act.x, -1.0F, 1.0F);
            s.yaw = Mathf.Clamp(s.yaw - _act.y, -1.0F, 1.0F);
            s.roll = Mathf.Clamp(s.roll + _act.z, -1.0F, 1.0F);
        }
    }

    protected override void onPartUpdate()
    {
        _timeCounter += TimeWarp.deltaTime;

        if (Mode == UIMode.SYNC)
        {
            switch (SyncMode)
            {
                case Sinc.SHP_APO:
                    _ran = 180;
                    break;
                case Sinc.SHP_PER:
                    _ran = 0;
                    break;
                case Sinc.TGT_APO:
                    _ran = FlightGlobals.Vessels[_selectedVesselIndex].orbit.TranslateAnomaly(vessel.orbit, 180);
                    break;
                case Sinc.TGT_PER:
                    _ran = FlightGlobals.Vessels[_selectedVesselIndex].orbit.TranslateAnomaly(vessel.orbit, 0);
                    break;
            }

            if (_timeCounter >= .5)
            {
                //update every .5 seconds
                print(vessel.orbit.TranslateAnomaly(FlightGlobals.Vessels[_selectedVesselIndex].orbit, _ran).ToString());
                for (int i = 0; i < 4; i++)
                {
                    _shTor[i] = (float) vessel.orbit.GetTimeToTrue(_ran) + (float) vessel.orbit.period*i;
                    _tgTor[i] = (float) vessel.orbit.Syncorbits(FlightGlobals.Vessels[_selectedVesselIndex].orbit, _ran, i);
                    if (i == 0)
                    {
                        _min = Math.Abs(_shTor[i] - _tgTor[i]);
                    }
                    if (_min > Math.Abs(_shTor[i] - _tgTor[i]))
                        _closestApproachOrbit = i;
                }

                for (int i = 0; i < 4; i++)
                {
                    _syncString[i] = i.ToString() + "			" + _shTor[i].ToString("f0") + "			" + _tgTor[i].ToString("f0");
                }
                _timeCounter = 0;
            }
        }

        UpdateVectors();

        if (ModeChanged)
        {
            windowPos.width = windowPos.height = 20;
            ModeChanged = false;
        }
    }

    protected override void onPartStart()
    {
        FlightInputHandler.OnFlyByWire += drive;
        if ((windowPos.x == 0) && (windowPos.y == 0))
        {
            windowPos = new Rect(Screen.width - 220, 10, 10, 10);
        }
    }

    private void WindowGUI(int windowID)
    {
        var sty = new GUIStyle(GUI.skin.button);
        sty.normal.textColor = sty.focused.textColor = Color.white;
        sty.hover.textColor = sty.active.textColor = Color.yellow;
        sty.onNormal.textColor = sty.onFocused.textColor = sty.onHover.textColor = sty.onActive.textColor = Color.green;
        sty.padding = new RectOffset(8, 8, 8, 8);
        if (Mode == UIMode.OFF)
        {
            GUILayout.BeginVertical();
            if (GUILayout.Button("OPEN", sty, GUILayout.ExpandWidth(true)))
            {
                Mode = UIMode.VESSELS;
            }
            GUILayout.EndVertical();
        }

        if (Mode == UIMode.VESSELS)
        {
            GUILayout.BeginVertical();
            if (GUILayout.Button("HIDE", sty, GUILayout.ExpandWidth(true)))
            {
                Mode = UIMode.OFF;
            }
            GUILayout.Box("Select Target");
            //TODO: ADD BODY SUPPORT
            //create a button for each vessel, and store the location of the selected vessel
            _scrollPosition = GUILayout.BeginScrollView(_scrollPosition, GUILayout.Width(200), GUILayout.Height(300));
            //GUILayout.BeginVertical();
            for (int i = 0; i < FlightGlobals.Vessels.Count; i++)
            {
                if (FlightGlobals.Vessels[i] != vessel &&
                    vessel.orbit.referenceBody == FlightGlobals.Vessels[i].orbit.referenceBody)
                {
                    if (GUILayout.Button((FlightGlobals.Vessels[i].vesselName), sty, GUILayout.ExpandWidth(true)))
                    {
                        Mode = UIMode.SELECTED;
                        _selVesselId = FlightGlobals.Vessels[i].GetInstanceID();
                        _selectedVesselIndex = i;
                    }
                }
            }

            GUILayout.EndScrollView();
            GUILayout.EndVertical();
        }

        if (Mode == UIMode.SELECTED)
        {
            if (!CheckVessel())
            {
                _flyByWire = false;
                Mode = UIMode.SELECTED;
            }
            GUILayout.BeginVertical();
            if (GUILayout.Button((FlightGlobals.Vessels[_selectedVesselIndex].vesselName), sty, GUILayout.ExpandWidth(true)))
            {
                _flyByWire = false;
                Mode = UIMode.VESSELS;
            }
            if (GUILayout.Button("Align Planes", sty, GUILayout.ExpandWidth(true)))
            {
                Mode = UIMode.ALIGN;
            }
            if (GUILayout.Button("Sync Orbits", sty, GUILayout.ExpandWidth(true)))
            {
                Mode = UIMode.SYNC;
            }
            if (GUILayout.Button("Rendezvous", sty, GUILayout.ExpandWidth(true)))
            {
                Mode = UIMode.RENDEZVOUS;
            }
            GUILayout.EndVertical();
        }

        if (Mode == UIMode.ALIGN)
        {
            if (!CheckVessel())
            {
                _flyByWire = false;
                Mode = UIMode.SELECTED;
            }
            GUILayout.BeginVertical();
            if (GUILayout.Button("Align Planes", sty, GUILayout.ExpandWidth(true)))
            {
                Mode = UIMode.SELECTED;
                _flyByWire = false;
            }

            GUILayout.Box("Time to AN : " +
                          vessel.orbit.GetTimeToRelAN(FlightGlobals.Vessels[_selectedVesselIndex].orbit).ToString("F2"));
            GUILayout.Box("Time to DN : " +
                          vessel.orbit.GetTimeToRelDN(FlightGlobals.Vessels[_selectedVesselIndex].orbit).ToString("F2"));
            GUILayout.Box("Relative Inclination :" + _relativeInclination.ToString("F2"));

            if (_flyByWire == false)
            {
                if (GUILayout.Button("Orbit Normal", sty, GUILayout.ExpandWidth(true)))
                {
                    _flyByWire = true;
                    PointAt = Orient.Normal;
                }

                if (GUILayout.Button("Anti Normal", sty, GUILayout.ExpandWidth(true)))
                {
                    _flyByWire = true;
                    PointAt = Orient.AntiNormal;
                }
            }

            if (_flyByWire)
            {
                if (GUILayout.Button("Disable " + PointAt.ToString(), sty, GUILayout.ExpandWidth(true)))
                {
                    FlightInputHandler.SetNeutralControls();
                    _flyByWire = false;
                    ModeChanged = true;
                }
            }

            GUILayout.EndVertical();
        }

        // TODO: RELATIVE INCLINATION
        // TIME TO NODES
        // BURN TIMER?
        // DELTA RINC
        // ORIENTATION FOR NEXT BURN (NORMAL/ANTINORMAL)
        // AUTOPILOT(NORMAL/ANTINORMAL)

        if (Mode == UIMode.SYNC)
        {
            if (!CheckVessel())
            {
                _flyByWire = false;
                Mode = UIMode.SELECTED;
            }
            GUILayout.BeginVertical();
            if (GUILayout.Button("Sync Orbits", sty, GUILayout.ExpandWidth(true)))
            {
                Mode = UIMode.SELECTED;
                _flyByWire = false;
            }
            GUILayout.EndVertical();
            GUILayout.BeginHorizontal();
            for (int i = 0; i < SincCount; i++)
            {
                if (i == (int) SyncMode)
                {
                    if (GUILayout.Button(" ", sty, GUILayout.ExpandWidth(true)))
                    {
                        if (i == SincCount - 1) SyncMode = 0;
                        else SyncMode = SyncMode + 1;
                    }
                    GUILayout.Box(SyncMode.ToString());
                }
            }
            GUILayout.EndHorizontal();
            GUILayout.BeginVertical();

            GUILayout.Box("Orbit		ShipToR		TgtToR ", sty, GUILayout.ExpandWidth(true));
            for (int i = 0; i < 4; i++)
                GUILayout.Box(_syncString[i]);
                
            GUILayout.Box("Closest Approach on Orbit " + _closestApproachOrbit.ToString());
            GUILayout.Box("DToR : " + _min.ToString("f1"));


            GUILayout.EndVertical();


            // TODO: 
            // SELECT RENDESVOUS POINT
            // SHIP APO
            // SHIP PER
            // TGT APO
            // TGT PER
            // INTERSECT 1
            // INTERSECT 2
            // MANUAL
            // DISPLAY ARRIVAL TIME (BY ORBIT)
            // DELTA ARRIVAL TIME
        }

        if (Mode == UIMode.RENDEZVOUS)
        {
            if (!CheckVessel())
            {
                _flyByWire = false;
                Mode = UIMode.SELECTED;
            }
            //the above check should prevent a crash when the vessel we are looking for is destroyed
            //learn how to use list.exists etc...

            GUILayout.BeginVertical();
            if (GUILayout.Button((FlightGlobals.Vessels[_selectedVesselIndex].vesselName), sty, GUILayout.ExpandWidth(true)))
            {
                _flyByWire = false;
                Mode = UIMode.SELECTED;
            }
            GUILayout.Box("Distance: " + _targetDistance.ToString("F1"));
            GUILayout.Box("Rel Inc : " + _relativeInclination.ToString("F3"));
            GUILayout.Box("Rel Vel : " + _relativeVelocity.ToString("F2"));

            if (_flyByWire == false)
            {
                if (GUILayout.Button(RendStrings[0], sty, GUILayout.ExpandWidth(true)))
                {
                    _flyByWire = true;
                    PointAt = Orient.RVEL;
                    ModeChanged = true;
                    _selstring = 0;
                }


                if (GUILayout.Button(RendStrings[1], sty, GUILayout.ExpandWidth(true)))
                {
                    _flyByWire = true;
                    PointAt = Orient.RVELNEG;
                    ModeChanged = true;
                    _selstring = 1;
                }


                if (GUILayout.Button(RendStrings[2], sty, GUILayout.ExpandWidth(true)))
                {
                    _flyByWire = true;
                    PointAt = Orient.TGT;
                    ModeChanged = true;
                    _selstring = 2;
                }


                if (GUILayout.Button(RendStrings[3], sty, GUILayout.ExpandWidth(true)))
                {
                    _flyByWire = true;
                    PointAt = Orient.TGTNEG;
                    ModeChanged = true;
                    _selstring = 3;
                }
            }

            if (_flyByWire)
            {
                if (GUILayout.Button("Disable " + RendStrings[_selstring], sty, GUILayout.ExpandWidth(true)))
                {
                    FlightInputHandler.SetNeutralControls();
                    _flyByWire = false;
                    ModeChanged = true;
                }
            }


            GUILayout.EndVertical();
        }

        //DragWindow makes the window draggable. The Rect specifies which part of the window it can by dragged by, and is 
        //clipped to the actual boundary of the window. You can also pass no argument at all and then the window can by
        //dragged by any part of it. Make sure the DragWindow command is AFTER all your other GUI input stuff, or else
        //it may "cover up" your controls and make them stop responding to the mouse.
        GUI.DragWindow(new Rect(0, 0, 10000, 20));
    }

    /// <summary>
    /// Checks  if the selected vessel is still where we expect it.
    /// </summary>
    /// <returns>
    /// The vessel.
    /// </returns>
    private bool CheckVessel()
    {
        //does Vessels[selVessel] contain a vessel?
        if (FlightGlobals.Vessels.Count - 1 < _selectedVesselIndex)
        {
            //no go back to vessel menu
            return false;
        }
        
        // Does the ID match the vessel selected?
        int id = FlightGlobals.Vessels[_selectedVesselIndex].GetInstanceID();
        if(id == _selVesselId)
            return true;

        // doesn't match, search vessels for matching id
        for (int i = 0; i < FlightGlobals.Vessels.Count; i++)
        {
            id = FlightGlobals.Vessels[i].GetInstanceID();
            if (id != _selVesselId) 
                continue;

            // found it!
            _selectedVesselIndex = i;
            return true;
        }

        // Couldn't find it.
        return false;
    }

    /// <summary>
    /// Updates the vectors.
    /// </summary>
    private void UpdateVectors()
    {
        //		Vector3d v3dRvel;
        Vector3 up = (vessel.findWorldCenterOfMass() - vessel.mainBody.position).normalized;
        Vector3 prograde = vessel.orbit.GetRelativeVel().normalized;

        Vector3 relativeVelocity = (Vector3) FlightGlobals.Vessels[_selectedVesselIndex].orbit.GetVel() - (Vector3) vessel.orbit.GetVel();

        _relativeVelocityMagnitude = (float) relativeVelocity.magnitude;
        _vectorToTarget = FlightGlobals.Vessels[_selectedVesselIndex].transform.position - vessel.transform.position;
        _targetDistance = Vector3.Distance(FlightGlobals.Vessels[_selectedVesselIndex].transform.position, vessel.transform.position);
        if (_targetDistance < _prevTargetDistance)
        {
            _relativeVelocity = -_relativeVelocityMagnitude;
        }
        _prevTargetDistance = _targetDistance;

        //Vector3d v3dRvelOff;
        //v3dRvelOff = v3dRvel.normalized - this.vessel.transform.forward.normalized;
        _relativeInclination = Mathf.Abs((float) FlightGlobals.Vessels[_selectedVesselIndex].orbit.inclination - (float) vessel.orbit.inclination);

        switch (PointAt)
        {
            case Orient.RVEL:
                _tgtFwd = relativeVelocity;
                _tgtUp = Vector3.Cross(_tgtFwd.normalized, vessel.orbit.vel.normalized);
                break;
            case Orient.RVELNEG:
                _tgtFwd = -relativeVelocity;
                _tgtUp = Vector3.Cross(_tgtFwd.normalized, vessel.orbit.vel.normalized);
                break;
            case Orient.TGT:
                _tgtFwd = _vectorToTarget;
                _tgtUp = Vector3.Cross(_tgtFwd.normalized, vessel.orbit.vel.normalized);
                break;
            case Orient.TGTNEG:
                _tgtFwd = -_vectorToTarget;
                _tgtUp = Vector3.Cross(_tgtFwd.normalized, vessel.orbit.vel.normalized);
                break;
            case Orient.Normal:
                _tgtFwd = Vector3.Cross(prograde, up);
                _tgtUp = up;
                break;
            case Orient.AntiNormal:
                _tgtFwd = -Vector3.Cross(prograde, up);
                _tgtUp = up;
                break;
        }
    }


    /// <summary>
    /// Draws the GUI.
    /// </summary>
    private void drawGUI()
    {
        //this.vessel.isActiveVessel
        if (vessel == FlightGlobals.ActiveVessel)
        {
            GUI.skin = HighLogic.Skin;
            windowPos = GUILayout.Window(windowIDbase, windowPos, WindowGUI, "RendezMe", GUILayout.MinWidth(100));
        }
    }

    protected override void onFlightStart()
    {
        RenderingManager.AddToPostDrawQueue(3, drawGUI);
    }

    protected override void onDisconnect()
    {
        _flyByWire = false;
    }

    protected override void onPartDestroy() //Called when the part is destroyed
    {
        _flyByWire = false;
        RenderingManager.RemoveFromPostDrawQueue(3, drawGUI);
    }
}