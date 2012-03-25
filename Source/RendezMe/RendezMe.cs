using System;
using System.Collections.Generic;
using Extensions;
using UnityEngine;

public class RendezMe : Part
{
    #region UI State

    protected Rect WindowPos;

    private Vector2 _scrollPosition = new Vector2(0, 0);

    public enum UIMode
    {
        OFF,
        VESSELS,
        SELECTED,
        RENDEZVOUS,
        ALIGN,
        SYNC,
    }
    
    private UIMode _mode = UIMode.OFF;
    private bool _modeChanged;

    public UIMode Mode
    {
        get { return _mode; }
        set
        {
            if (_mode == value)
                return;

            _mode = value;
            _modeChanged = true;
        }
    }

    private int _selectedFlyMode;

    private const int WindowIDBase = 18420;

    /// <summary>
    /// The selected vessel's location in FlightGlobals.Vessels[LIST].
    /// </summary>
    private int _selectedVesselIndex;

    /// <summary>
    /// Unique Instance ID of selected vessel
    /// </summary>
    private int _selectedVesselInstanceId;

    #endregion

    #region Sync State

    private const int NumberOfPredictedSyncPoints = 4;

    public enum SynchronizationType
    {
        TargetPeriapsis,
        TargetApoapsis,
        ShipPeriapsis,
        ShipApoapsis,
    }

    public SynchronizationType SyncMode = SynchronizationType.TargetPeriapsis;
    private double _minimumPredictedTimeFromTarget;
    private double _rendezvousAnomaly = 180;
    private readonly float[] _shipTimeToRendezvous = new float[4];
    private readonly float[] _targetTimeToRendezvous = new float[4];
    private readonly string[] _syncString = new string[4];
    private int _closestApproachOrbit;
    private double _rendezvousRecalculationTimer;
    
    #endregion

    #region Auto Align State

    private bool _autoAlign = false;
    private bool _autoAlignBurnTriggered = false;

    #endregion

    #region Rendezvous State

    private Vector3 _relativeVelocity;
    private float _relativeInclination;
    private Vector3 _vectorToTarget;
    private float _targetDistance;

    private bool _killRelativeVelocity = false;
    private Vector3 _localRelativeVelocity = Vector3.zero;

    private bool _homeOnRelativePosition = false;
    private Vector3 _localRelativePosition = Vector3.zero;

    #endregion

    #region FlyByWire PID Controller State
    public enum Orient
    {
        Off,
        RelativeVelocity,
        RelativeVelocityAway,
        Target,
        TargetAway,
        Normal,
        AntiNormal,
        MatchTarget,
        MatchTargetAway,
        Prograde,
        Retrograde
    }

    public string[] ControlModeCaptions = new[]
                                      {
                                          "RVel+", "Rvel-", "TGT+", "TGT-", "Match+", "Match-"
                                      };

    public string[] AlignmentCaptions = new[]
                                       {
                                           "NML\n+", "NML\n-"
                                       };

    public Orient PointAt = Orient.Off;
    private bool _flyByWire;
    private Vector3 _tgtFwd;
    private Vector3 _tgtUp;
    private Vector3 _deriv = Vector3.zero;
    private Vector3 _integral = Vector3.zero;
    private Vector3 _err = Vector3.zero;
    private Vector3 _prevErr = Vector3.zero;
    private Vector3 _act = Vector3.zero;
    
    public float Kp = 20.0F;
    public float Ki = 0.0F;
    public float Kd = 40.0F;

    #endregion

    #region User Interface

    private void WindowGUI(int windowID)
    {
        // Set up the UI style.
        var sty = new GUIStyle(GUI.skin.button);
        sty.normal.textColor = sty.focused.textColor = Color.white;
        sty.hover.textColor = sty.active.textColor = Color.yellow;
        sty.onNormal.textColor = sty.onFocused.textColor = sty.onHover.textColor = sty.onActive.textColor = Color.green;
        sty.padding = new RectOffset(8, 8, 8, 8);

        GUILayout.BeginVertical();

        if (Mode == UIMode.OFF)
            RenderOffUI(sty);

        if (Mode == UIMode.VESSELS)
            RenderVesselsUI(sty);

        if (Mode == UIMode.SELECTED)
            RenderSelectedUI(sty);

        if (Mode == UIMode.ALIGN)
            RenderAlignUI(sty);

        // TIME TO NODES
        // BURN TIMER?
        // DELTA RINC
        // ORIENTATION FOR NEXT BURN (NORMAL/ANTINORMAL)
        // AUTOPILOT(NORMAL/ANTINORMAL)

        if (Mode == UIMode.SYNC)
            RenderSyncUI(sty);

        if (Mode == UIMode.RENDEZVOUS)
            RenderRendezvousUI(sty);

        GUILayout.EndVertical();

        //DragWindow makes the window draggable. The Rect specifies which part of the window it can by dragged by, and is 
        //clipped to the actual boundary of the window. You can also pass no argument at all and then the window can by
        //dragged by any part of it. Make sure the DragWindow command is AFTER all your other GUI input stuff, or else
        //it may "cover up" your controls and make them stop responding to the mouse.
        GUI.DragWindow(new Rect(0, 0, 10000, 20));
    }

    private void RenderOffUI(GUIStyle sty)
    {
        if (GUILayout.Button("OPEN", sty, GUILayout.ExpandWidth(true)))
        {
            Mode = UIMode.VESSELS;
        }
    }

    private void RenderVesselsUI(GUIStyle sty)
    {
        if (GUILayout.Button("HIDE", sty, GUILayout.ExpandWidth(true)))
        {
            Mode = UIMode.OFF;
        }
        GUILayout.Box("Select Target");

        //TODO: ADD BODY SUPPORT
        //create a button for each vessel, and store the location of the selected vessel

        _scrollPosition = GUILayout.BeginScrollView(_scrollPosition, GUILayout.Width(300), GUILayout.Height(300));

        // Generate and sort an array of vessels by distance.
        List<Vessel> vesselList = new List<Vessel>(FlightGlobals.Vessels);
        var vdc = new VesselDistanceComparer();
        vdc.OriginVessel = vessel;

        vesselList.Sort(vdc);

        for (int i = 0; i < vesselList.Count; i++)
        {
            // Skip ourselves.
            if (vesselList[i] == vessel)
                continue;

            if (vesselList[i].LandedOrSplashed)
                continue;

            // Skip stuff around other worlds.
            if (vessel.orbit.referenceBody != vesselList[i].orbit.referenceBody)
                continue;

            // Calculate the distance.
            float d = Vector3.Distance(vesselList[i].transform.position, vessel.transform.position);

            if (GUILayout.Button((d / 1000).ToString("F1") + "km " + vesselList[i].vesselName, sty,
                                 GUILayout.ExpandWidth(true)))
            {
                Mode = UIMode.SELECTED;
                _selectedVesselInstanceId = vesselList[i].GetInstanceID();
                _selectedVesselIndex = FlightGlobals.Vessels.IndexOf(vesselList[i]);
            }
        }

        GUILayout.EndScrollView();
    }

    private void RenderSelectedUI(GUIStyle sty)
    {
        if (!CheckVessel())
        {
            _flyByWire = false;
            Mode = UIMode.SELECTED;
        }

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

    }

    private void RenderAlignUI(GUIStyle sty)
    {
        if (!CheckVessel())
        {
            _flyByWire = false;
            Mode = UIMode.SELECTED;
        }

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

        if(GUILayout.Button(_autoAlign ? "ALIGNING" : "Auto-Align", sty, GUILayout.ExpandWidth(true)))
        {
            _autoAlignBurnTriggered = false;
            _autoAlign = !_autoAlign;
        }

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
                _modeChanged = true;
            }
        }
    }

    private void RenderSyncUI(GUIStyle sty)
    {
        if (!CheckVessel())
        {
            _flyByWire = false;
            Mode = UIMode.SELECTED;
        }

        if (GUILayout.Button("Sync Orbits", sty, GUILayout.ExpandWidth(true)))
        {
            Mode = UIMode.SELECTED;
            _flyByWire = false;
        }
        GUILayout.EndVertical();
        GUILayout.BeginHorizontal();
        for (int i = 0; i < NumberOfPredictedSyncPoints; i++)
        {
            if (i != (int) SyncMode) 
                continue;

            if (GUILayout.Button(" ", sty, GUILayout.ExpandWidth(true)))
            {
                if (i == NumberOfPredictedSyncPoints - 1) SyncMode = 0;
                else SyncMode = SyncMode + 1;
            }
            GUILayout.Box(SyncMode.ToString());
        }
        GUILayout.EndHorizontal();
        GUILayout.BeginVertical();

        GUILayout.Box("Orbit		ShipToR		TgtToR ", sty, GUILayout.ExpandWidth(true));
        for (int i = 0; i < 4; i++)
            GUILayout.Box(_syncString[i]);

        GUILayout.Box("Closest Approach on Orbit " + _closestApproachOrbit.ToString());
        GUILayout.Box("Min Separation (sec) : " + _minimumPredictedTimeFromTarget.ToString("f1"));
    }

    private void RenderRendezvousUI(GUIStyle sty)
    {
        if (!CheckVessel())
        {
            _flyByWire = false;
            Mode = UIMode.SELECTED;
        }

        Vessel selectedVessel = FlightGlobals.Vessels[_selectedVesselIndex] as Vessel;

        //the above check should prevent a crash when the vessel we are looking for is destroyed
        //learn how to use list.exists etc...
        if (GUILayout.Button(selectedVessel.vesselName, sty, GUILayout.ExpandWidth(true)))
        {
            _flyByWire = false;
            Mode = UIMode.SELECTED;
        }

        GUILayout.Box("Distance: " + _targetDistance.ToString("F1"), GUILayout.Width(300));
        GUILayout.Box("Rel Inc : " + _relativeInclination.ToString("F3"));
        GUILayout.Box("Rel VelM: " + _relativeVelocity.magnitude.ToString("F2"));

        // Take the relative velocity and project into ship local space.
        _localRelativeVelocity = vessel.transform.worldToLocalMatrix.MultiplyVector(_relativeVelocity);
        _localRelativePosition = vessel.transform.worldToLocalMatrix.MultiplyPoint(selectedVessel.transform.position);

        if(GUILayout.Button(_killRelativeVelocity == false ? "Kill Rel Vel" : "FIRING", sty, GUILayout.ExpandWidth(true)))
            _killRelativeVelocity = !_killRelativeVelocity;

        if (GUILayout.Button(_homeOnRelativePosition == false ? "Home on Y+ 5m" : "HOMING", sty, GUILayout.ExpandWidth(true)))
            _homeOnRelativePosition = !_homeOnRelativePosition;

        GUILayout.Box("Rel Vel : " + _localRelativeVelocity.x.ToString("F2") + ", " + _localRelativeVelocity.y.ToString("F2") + ", " + _localRelativeVelocity.z.ToString("F2"));
        GUILayout.Box("Rel Pos : " + _localRelativePosition.x.ToString("F2") + ", " + _localRelativePosition.y.ToString("F2") + ", " + _localRelativePosition.z.ToString("F2"));

        if (_flyByWire == false)
        {
            GUILayout.BeginHorizontal();

            if (GUILayout.Button(ControlModeCaptions[0], sty, GUILayout.ExpandWidth(true)))
            {
                _flyByWire = true;
                PointAt = Orient.RelativeVelocity;
                _modeChanged = true;
                _selectedFlyMode = 0;
            }


            if (GUILayout.Button(ControlModeCaptions[1], sty, GUILayout.ExpandWidth(true)))
            {
                _flyByWire = true;
                PointAt = Orient.RelativeVelocityAway;
                _modeChanged = true;
                _selectedFlyMode = 1;
            }


            if (GUILayout.Button(ControlModeCaptions[2], sty, GUILayout.ExpandWidth(true)))
            {
                _flyByWire = true;
                PointAt = Orient.Target;
                _modeChanged = true;
                _selectedFlyMode = 2;
            }

            GUILayout.EndHorizontal();
            GUILayout.BeginHorizontal();

            if (GUILayout.Button(ControlModeCaptions[3], sty, GUILayout.ExpandWidth(true)))
            {
                _flyByWire = true;
                PointAt = Orient.TargetAway;
                _modeChanged = true;
                _selectedFlyMode = 3;
            }

            if (GUILayout.Button(ControlModeCaptions[4], sty, GUILayout.ExpandWidth(true)))
            {
                _flyByWire = true;
                PointAt = Orient.MatchTarget;
                _modeChanged = true;
                _selectedFlyMode = 4;
            }

            if (GUILayout.Button(ControlModeCaptions[5], sty, GUILayout.ExpandWidth(true)))
            {
                _flyByWire = true;
                PointAt = Orient.MatchTargetAway;
                _modeChanged = true;
                _selectedFlyMode = 5;
            }

            GUILayout.EndHorizontal();

        }

        if (_flyByWire)
        {
            if (GUILayout.Button("Disable " + ControlModeCaptions[_selectedFlyMode], sty, GUILayout.ExpandWidth(true)))
            {
                FlightInputHandler.SetNeutralControls();
                _flyByWire = false;
                _modeChanged = true;
            }
        }
    }

    /// <summary>
    /// Draws the GUI.
    /// </summary>
    private void DrawGUI()
    {
        if (vessel != FlightGlobals.ActiveVessel)
            return;

        GUI.skin = HighLogic.Skin;
        WindowPos = GUILayout.Window(WindowIDBase, WindowPos, WindowGUI, "RendezMe", GUILayout.MinWidth(200));
    }

    #endregion

    #region Control Logic

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
            return false;
        
        // Does the ID match the vessel selected?
        int id = FlightGlobals.Vessels[_selectedVesselIndex].GetInstanceID();
        if(id == _selectedVesselInstanceId)
            return true;

        // doesn't match, search vessels for matching id
        for (int i = 0; i < FlightGlobals.Vessels.Count; i++)
        {
            id = FlightGlobals.Vessels[i].GetInstanceID();
            if (id != _selectedVesselInstanceId) 
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
        Vector3 up = (vessel.findWorldCenterOfMass() - vessel.mainBody.position).normalized;
        Vector3 prograde = vessel.orbit.GetRelativeVel().normalized;

        Vessel selectedVessel = FlightGlobals.Vessels[_selectedVesselIndex] as Vessel;

        _relativeVelocity = selectedVessel.orbit.GetVel() - vessel.orbit.GetVel();
        _vectorToTarget = selectedVessel.transform.position - vessel.transform.position;
        _targetDistance = Vector3.Distance(selectedVessel.transform.position, vessel.transform.position);

        _relativeInclination = (float) selectedVessel.orbit.inclination - (float) vessel.orbit.inclination;

        switch (PointAt)
        {
            case Orient.RelativeVelocity:
                _tgtFwd = _relativeVelocity;
                _tgtUp = Vector3.Cross(_tgtFwd.normalized, vessel.orbit.vel.normalized);
                break;
            case Orient.RelativeVelocityAway:
                _tgtFwd = -_relativeVelocity;
                _tgtUp = Vector3.Cross(_tgtFwd.normalized, vessel.orbit.vel.normalized);
                break;
            case Orient.Target:
                _tgtFwd = _vectorToTarget;
                _tgtUp = Vector3.Cross(_tgtFwd.normalized, vessel.orbit.vel.normalized);
                break;
            case Orient.TargetAway:
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
            case Orient.MatchTarget:
                _tgtFwd = selectedVessel.transform.up;
                _tgtUp = selectedVessel.transform.right;
                break;
            case Orient.MatchTargetAway:
                _tgtFwd = -selectedVessel.transform.up;
                _tgtUp = selectedVessel.transform.right;
                break;
            case Orient.Prograde:
                _tgtFwd = vessel.rigidbody.velocity.normalized;
                _tgtUp = new Vector3(0, 0, 1);
                break;
            case Orient.Retrograde:
                _tgtFwd = -vessel.rigidbody.velocity.normalized;
                _tgtUp = new Vector3(0, 0, 1);
                break;
        }
    }

    private void DriveShip(FlightCtrlState controls)
    {
        if(!CheckVessel())
            return;

        Vessel selectedVessel = FlightGlobals.Vessels[_selectedVesselIndex] as Vessel;

        if(_autoAlign)
        {
            // Is it time to burn? Find soonest node.
            double timeToBurnAN = vessel.orbit.GetTimeToRelAN(selectedVessel.orbit);
            double timeToBurnDN = vessel.orbit.GetTimeToRelDN(selectedVessel.orbit);

            bool ascendingSoonest = timeToBurnAN < timeToBurnDN;
            double timeToBurnNode = ascendingSoonest ? timeToBurnAN : timeToBurnDN;

            // Figure out which way we want to burn to adjust our inclination.
            _flyByWire = true;
            if(!_autoAlignBurnTriggered)
            {
                if (_relativeInclination < 0.0)
                    PointAt = ascendingSoonest ? Orient.Normal : Orient.AntiNormal;
                else
                    PointAt = ascendingSoonest ? Orient.AntiNormal : Orient.Normal;                
            }

            // Do a burn just ahead of the ascending node - in the 5 seconds preceding.
            if ((timeToBurnNode < 10.0 || _autoAlignBurnTriggered) && _err.magnitude < 5.0 && Math.Abs(_relativeInclination) > 0.01)
            {
                _autoAlignBurnTriggered = true;
                if (Math.Abs(_relativeInclination) > 0.1)
                {
                    controls.mainThrottle = 1.0f;                    
                }
                else
                {
                    controls.mainThrottle = 0.25f;
                }
            }
            else
            {
                controls.mainThrottle = 0.0f;
            }

            if (Math.Abs(_relativeInclination) < 0.02)
            {
                _autoAlignBurnTriggered = false;
                _autoAlign = false;
            }
        }

        if(_killRelativeVelocity)
        {
            controls.X = Mathf.Clamp(-_localRelativeVelocity.x * 8.0f, -1.0f, 1.0f);
            controls.Y = Mathf.Clamp(-_localRelativeVelocity.z * 8.0f, -1.0f, 1.0f);
            controls.Z = Mathf.Clamp(-_localRelativeVelocity.y * 8.0f, -1.0f, 1.0f);

            if (_localRelativeVelocity.magnitude < 0.1)
                _killRelativeVelocity = false;
        } 
        else if (_homeOnRelativePosition)
        {
            Vector3 targetGoalPos = new Vector3(0.0f, 2.0f, 0.0f);
            targetGoalPos = selectedVessel.transform.localToWorldMatrix.MultiplyPoint(targetGoalPos);
            targetGoalPos = vessel.transform.worldToLocalMatrix.MultiplyPoint(targetGoalPos);

            Vector3 relPos = targetGoalPos;
            Vector4 goalVel = Vector3.zero;

            float velGoal = 0.1f;

            if (_targetDistance > 2.0f)
                velGoal = 0.3f;
            else if (_targetDistance > 10.0f)
                velGoal = 0.5f;
            else if (_targetDistance > 50.0f)
                velGoal = 1.0f;
            else if (_targetDistance > 150.0f)
                velGoal = 3.0f;

            if(Mathf.Abs(relPos.x) > 0.01f)
                goalVel.x = -Mathf.Sign(relPos.x) * velGoal;

            if (Mathf.Abs(relPos.y) > 0.01f)
                goalVel.y = -Mathf.Sign(relPos.y) * velGoal;

            if (Mathf.Abs(relPos.z) > 0.01f)
                goalVel.z = -Mathf.Sign(relPos.z) * velGoal;

            controls.X = Mathf.Clamp((goalVel.x - _localRelativeVelocity.x) * 8.0f, -1, 1);
            controls.Y = Mathf.Clamp((goalVel.z - _localRelativeVelocity.z) * 8.0f, -1, 1);
            controls.Z = Mathf.Clamp((goalVel.y - _localRelativeVelocity.y) * 8.0f, -1, 1);
        }

        if (!_flyByWire) 
            return;

        Quaternion tgt = Quaternion.LookRotation(_tgtFwd, _tgtUp);
        Quaternion delta =
            Quaternion.Inverse(Quaternion.Euler(90, 0, 0) * Quaternion.Inverse(vessel.transform.rotation) * tgt);

        _err =
            new Vector3((delta.eulerAngles.x > 180) ? (delta.eulerAngles.x - 360.0F) : delta.eulerAngles.x,
                        (delta.eulerAngles.y > 180) ? (delta.eulerAngles.y - 360.0F) : delta.eulerAngles.y,
                        (delta.eulerAngles.z > 180) ? (delta.eulerAngles.z - 360.0F) : delta.eulerAngles.z) / 180.0F;
        _integral += _err * TimeWarp.fixedDeltaTime;
        _deriv = (_err - _prevErr) / TimeWarp.fixedDeltaTime;
        _act = Kp * _err + Ki * _integral + Kd * _deriv;
        _prevErr = _err;

        controls.pitch = Mathf.Clamp(controls.pitch + _act.x, -1.0F, 1.0F);
        controls.yaw = Mathf.Clamp(controls.yaw - _act.y, -1.0F, 1.0F);
        controls.roll = Mathf.Clamp(controls.roll + _act.z, -1.0F, 1.0F);
    }

    #endregion

    #region Kerbel Interface

    protected override void onFlightStart()
    {
        RenderingManager.AddToPostDrawQueue(3, DrawGUI);
    }

    protected override void onDisconnect()
    {
        _flyByWire = false;
    }

    protected override void onPartDestroy() //Called when the part is destroyed
    {
        _flyByWire = false;
        RenderingManager.RemoveFromPostDrawQueue(3, DrawGUI);
    }

    protected override void onPartUpdate()
    {
        _rendezvousRecalculationTimer += TimeWarp.deltaTime;

        if (Mode == UIMode.SYNC)
            PerformSyncPartLogic();

        UpdateVectors();

        if (_modeChanged)
        {
            WindowPos.width = WindowPos.height = 20;
            _modeChanged = false;
        }
    }

    private void PerformSyncPartLogic()
    {
        // What anomaly are we trying to rendezvous at?
        switch (SyncMode)
        {
            case SynchronizationType.ShipApoapsis:
                _rendezvousAnomaly = 180;
                break;
            case SynchronizationType.ShipPeriapsis:
                _rendezvousAnomaly = 0;
                break;
            case SynchronizationType.TargetApoapsis:
                _rendezvousAnomaly = FlightGlobals.Vessels[_selectedVesselIndex].orbit.TranslateAnomaly(vessel.orbit, 180);
                break;
            case SynchronizationType.TargetPeriapsis:
                _rendezvousAnomaly = FlightGlobals.Vessels[_selectedVesselIndex].orbit.TranslateAnomaly(vessel.orbit, 0);
                break;
        }

        // Only recalculate if enough time has elapsed.
        if (_rendezvousRecalculationTimer < .1) 
            return;

        // Find the time away from the anomaly we'll be at rendezvous.
        for (int i = 0; i < 4; i++)
        {
            _shipTimeToRendezvous[i] = (float) vessel.orbit.GetTimeToTrue(_rendezvousAnomaly) + (float) vessel.orbit.period*i;
            _targetTimeToRendezvous[i] = (float) vessel.orbit.Syncorbits(FlightGlobals.Vessels[_selectedVesselIndex].orbit, _rendezvousAnomaly, i);

            if (i == 0)
                _minimumPredictedTimeFromTarget = Math.Abs(_shipTimeToRendezvous[i] - _targetTimeToRendezvous[i]);

            if (_minimumPredictedTimeFromTarget > Math.Abs(_shipTimeToRendezvous[i] - _targetTimeToRendezvous[i]))
                _closestApproachOrbit = i;
        }

        // Update the display.
        for (int i = 0; i < 4; i++)
        {
            _syncString[i] = i.ToString() + "			" + _shipTimeToRendezvous[i].ToString("f0") + "			" + _targetTimeToRendezvous[i].ToString("f0");
        }

        // Reset the timer.
        _rendezvousRecalculationTimer = 0;
    }

    protected override void onPartStart()
    {
        FlightInputHandler.OnFlyByWire += DriveShip;
        if ((WindowPos.x == 0) && (WindowPos.y == 0))
        {
            WindowPos = new Rect(Screen.width - 220, 10, 10, 10);
        }
    }

    #endregion
}