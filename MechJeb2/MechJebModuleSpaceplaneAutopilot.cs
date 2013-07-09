using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using UnityEngine;

namespace MuMech
{
    public class MechJebModuleSpaceplaneAutopilot : ComputerModule
    {
        public void Autoland(object controller)
        {
            mode = Mode.AUTOLAND;
            users.Add(controller);
            loweredGear = false;
            brakes = false;
        }

        public void HoldHeadingAndAltitude(object controller)
        {
            mode = Mode.HOLD;
            users.Add(controller);
            loweredGear = false;
            brakes = false;
        }

        bool autopilotOn = false;
        public void AutopilotOn()
        {
            core.attitude.users.Add(this);
            autopilotOn = true;
        }

        public void AutopilotOff()
        {
            core.attitude.users.Clear();
            autopilotOn = false;
        }

        public void Abort()
        {
            users.Clear();
            AutopilotOff();
            core.attitude.attitudeDeactivate();
            mode = Mode.OFF;
        }

        public static Runway[] runways = 
        {
            new Runway //The runway at KSC
            { 
                name = "KSC runway",
                start = new Runway.Endpoint { latitude = -0.040633, longitude = -74.6908, altitude = 67 }, 
                end = new Runway.Endpoint { latitude = -0.041774, longitude = -74.5241702, altitude = 67 } 
            },
            new Runway //The runway on the island off the KSC coast.
            { 
                name = "Island runway",
                start = new Runway.Endpoint { latitude = -1.547474, longitude = -71.9611702, altitude = 48 },
                end = new Runway.Endpoint { latitude = -1.530174, longitude = -71.8791702, altitude = 48 }
            }
        };

        public enum Mode { AUTOLAND, HOLD, OFF };
        public Mode mode = Mode.OFF;

        //autoland parameters
        public Runway runway = runways[0]; //the runway to land at
        public EditableDouble glideslope = 3;      //the FPA to approach at during autoland
        public EditableDouble touchdownPoint = 100; //how many meters down the runway to touch down

        //heading and altitude hold parameters
        public EditableDouble targetAltitude = 1000;
        public EditableDouble targetHeading = 90;

        bool loweredGear = false;
        bool brakes = false;

        public override void OnModuleDisabled()
        {
            core.attitude.attitudeDeactivate();
        }

        public override void Drive(FlightCtrlState s)
        {
            switch (mode)
            {
                case Mode.AUTOLAND:
                    DriveAutoland(s);
                    break;

                case Mode.HOLD:
                    DriveHeadingAndAltitudeHold(s);
                    break;
            }
        }

        public float takeoffPitch = 10F;
        public void DriveHeadingAndAltitudeHold(FlightCtrlState s)
        {
            if (!part.vessel.Landed)
            {
                if (!autopilotOn)
                    AutopilotOn();
                if (!loweredGear)
                {
                    if ((part.vessel.altitude - part.vessel.terrainAltitude) < 100.0)
                    {
                        vessel.ActionGroups.SetGroup(KSPActionGroup.Gear, true);
                        loweredGear = true;
                    }
                }
                else
                {
                    if ((part.vessel.altitude - part.vessel.terrainAltitude) > 100.0)
                    {
                        vessel.ActionGroups.SetGroup(KSPActionGroup.Gear, false);
                        loweredGear = false;
                    }
                }

                //takeoff or set for flight
                if (landed)
                {
                    //vessel.ActionGroups.SetGroup(KSPActionGroup.Gear, false);
                    loweredGear = true;
                    landed = false;
                }
            }
            else
            {
                if (!landed)
                {
                    vessel.ctrlState.mainThrottle = 0;
                    AutopilotOff();
                    landTime = vesselState.time;
                    landed = true;
                    loweredGear = true;
                }
                //apply breaks a little after touchdown
                if (!brakes && vesselState.time > landTime + 1.0)
                {
                    vessel.ActionGroups.SetGroup(KSPActionGroup.Brakes, true);
                    brakes = true;
                }
                //if engines are started, disengage brakes and prepare for takeoff
                if (brakes && vessel.ctrlState.mainThrottle > 0.01)
                {
                    vessel.ActionGroups.SetGroup(KSPActionGroup.Brakes, false);
                    brakes = false;
                }
                //pitch up for takeoff
                if (!brakes && vesselState.time > landTime + 1.0)
                {
                    vessel.ctrlState.pitch = (float)(takeoffPitch - vesselState.vesselPitch) / 15;
                }
                vessel.ctrlState.yaw = (float)MuUtils.ClampDegrees180(targetHeading - vesselState.vesselHeading) * 0.1F;
            }

            double targetClimbRate = (targetAltitude - vesselState.altitudeASL) / (30.0 * Math.Pow((CelestialBodyExtensions.RealMaxAtmosphereAltitude(mainBody) / (CelestialBodyExtensions.RealMaxAtmosphereAltitude(mainBody) - vesselState.altitudeASL)), 5));
            double targetFlightPathAngle = 180 / Math.PI * Math.Asin(Mathf.Clamp((float)(targetClimbRate / vesselState.speedSurface), (float)Math.Sin(-Math.PI / 7), (float)Math.Sin(Math.PI / 7)));

            double heading = targetHeading;
            //if (loweredGear == true) heading = vesselState.vesselHeading;

            AimVelocityVector(targetFlightPathAngle, targetHeading);
        }

        bool landed = false;
        double landTime = 0;
        public double aimAltitude = 0;
        public double distanceFrom = 0;
        public double runwayHeading = 0;
        public enum HeadingState { RIGHT, LEFT, OFF };
        public HeadingState autolandHeadingState = HeadingState.OFF;
        public void DriveAutoland(FlightCtrlState s)
        {
            if (!part.vessel.Landed)
            {
                if (landed) landed = false;

                Vector3d runwayStart = RunwayStart();

                if (!autopilotOn)
                    AutopilotOn();

                //prepare for landing
                if (!loweredGear && (vesselState.CoM - runwayStart).magnitude < 1000.0)
                {
                    vessel.ActionGroups.SetGroup(KSPActionGroup.Gear, true);
                    loweredGear = true;
                }

                Vector3d vectorToWaypoint = ILSAimDirection();
                double headingToWaypoint = vesselState.HeadingFromDirection(vectorToWaypoint);
                if (Math.Abs(MuUtils.ClampDegrees180(headingToWaypoint - vesselState.vesselHeading)) > 170)
                {
                    //make sure the heading doesn't keep switching from side to side
                    switch (autolandHeadingState)
                    {
                        case HeadingState.RIGHT:
                            headingToWaypoint = MuUtils.ClampDegrees360(vesselState.vesselHeading + 90);
                            break;
                        case HeadingState.LEFT:
                            headingToWaypoint = MuUtils.ClampDegrees360(vesselState.vesselHeading - 90);
                            break;
                        case HeadingState.OFF:
                            if (headingToWaypoint - vesselState.vesselHeading > 0)
                                autolandHeadingState = HeadingState.RIGHT;
                            else
                                autolandHeadingState = HeadingState.LEFT;
                            break;
                    }
                }
                else
                    autolandHeadingState = HeadingState.OFF;

                //stop any rolling and aim down runway before touching down
                if ((vesselState.CoM - runwayStart).magnitude < 500.0)
                {
                    Vector3d runwayDir = runway.End(vesselState.CoM) - runway.Start(vesselState.CoM);
                    runwayHeading = 180 / Math.PI * Math.Atan2(Vector3d.Dot(runwayDir, vesselState.east), Vector3d.Dot(runwayDir, vesselState.north));
                    headingToWaypoint = runwayHeading;
                }

                Vector3d vectorToRunway = runwayStart - vesselState.CoM;
                double verticalDistanceToRunway = Vector3d.Dot(vectorToRunway, vesselState.up);
                double horizontalDistanceToRunway = Math.Sqrt(vectorToRunway.sqrMagnitude - verticalDistanceToRunway * verticalDistanceToRunway);
                distanceFrom = horizontalDistanceToRunway;
                if (horizontalDistanceToRunway > 10000)
                {
                    aimAltitude = 1000;
                    double horizontalDifference = 9900;
                    if (horizontalDistanceToRunway > 40000) { aimAltitude = 5000; horizontalDifference = 39900; }
                    if (horizontalDistanceToRunway > 80000)
                    {
                        aimAltitude = vesselState.altitudeTrue;
                        if (aimAltitude < 5000) aimAltitude = 5000;
                        horizontalDifference = 79900;
                    }
                    double setupFPA = MuUtils.Clamp(180 / Math.PI * Math.Atan2(aimAltitude - vessel.altitude, horizontalDistanceToRunway - horizontalDifference), -20, 20);

                    AimVelocityVector(setupFPA, headingToWaypoint);
                }
                else
                {
                    double flightPathAngleToRunway = 180 / Math.PI * Math.Atan2(verticalDistanceToRunway, horizontalDistanceToRunway);
                    double desiredFPA = Mathf.Clamp((float)(flightPathAngleToRunway + 3 * (flightPathAngleToRunway + glideslope)), -20.0F, 5.0F);

                    aimAltitude = verticalDistanceToRunway;
                    AimVelocityVector(desiredFPA, headingToWaypoint);
                }
            }
            else
            {
                if (!landed)
                {
                    vessel.ctrlState.mainThrottle = 0;
                    landTime = vesselState.time;
                    AutopilotOff();
                    landed = true;
                }
                //apply breaks a little after touchdown
                if (!brakes && vesselState.time > landTime + 1.0)
                {
                    vessel.ActionGroups.SetGroup(KSPActionGroup.Brakes, true);
                    vessel.ctrlState.pitch = -1;
                    brakes = true;
                }
                //keep the plane aligned with the runway:
                Vector3d runwayDir = runway.End(vesselState.CoM) - runway.Start(vesselState.CoM);
                if (Vector3d.Dot(runwayDir, vesselState.forward) < 0) runwayDir *= -1;
                runwayHeading = 180 / Math.PI * Math.Atan2(Vector3d.Dot(runwayDir, vesselState.east), Vector3d.Dot(runwayDir, vesselState.north));
                vessel.ctrlState.pitch = (float)MuUtils.Clamp(-vesselState.vesselPitch * 0.1, -0.5, 0);
                vessel.ctrlState.yaw = (float)MuUtils.Clamp(MuUtils.ClampDegrees180(runwayHeading - vesselState.vesselHeading) * 0.1F, -0.1, 0.1);
                if (vesselState.vesselRoll < -1) vessel.ctrlState.roll = 1;
                if (vesselState.vesselRoll > 1) vessel.ctrlState.roll = -1;
            }
        }

        public double stableAoA = 0; //we average AoA over time to get an estimate of what pitch will produce what FPA
        public double pitchCorrection = 0; //we average (commanded pitch - actual pitch) over time in order to fix this offset in our commands
        public float maxYaw = 15.0F;
        public float maxRoll = 30.0F;
        public float maxAoA = 30.0F;
        public float minAoA = -10.0F;
        public float maxPitchCorrection = 5.0F;
        public double AoAtimeConstant = 2.0;
        public double pitchCorrectionTimeConstant = 15.0;
        public double velocityPitch = 0;
        public double percentAltitude = 0;

        void AimVelocityVector(double desiredFpa, double desiredHeading)
        {
            if (autopilotOn)
            {
                //horizontal control
                double velocityHeading = 180 / Math.PI * Math.Atan2(Vector3d.Dot(vesselState.velocityVesselSurface, vesselState.east),
                                                                    Vector3d.Dot(vesselState.velocityVesselSurface, vesselState.north));
                double headingTurn = Mathf.Clamp((float)MuUtils.ClampDegrees180(desiredHeading - velocityHeading), -maxYaw, maxYaw);
                double noseHeading = velocityHeading + headingTurn;
                double noseRoll = (maxRoll / maxYaw) * headingTurn;

                //vertical control
                percentAltitude = (vesselState.altitudeTrue / CelestialBodyExtensions.RealMaxAtmosphereAltitude(mainBody));
                velocityPitch = 180 / Math.PI * Math.Atan2(vesselState.speedVertical, vesselState.speedSurface);
                double nosePitch = MuUtils.Clamp(desiredFpa + stableAoA + pitchCorrection, velocityPitch + minAoA * (1 - percentAltitude), velocityPitch + maxAoA * (1 - percentAltitude));

                core.attitude.attitudeTo(noseHeading, nosePitch, noseRoll, this);

                double flightPathAngle = 180 / Math.PI * Math.Atan2(vesselState.speedVertical, vesselState.speedSurfaceHorizontal);
                double AoA = vesselState.vesselPitch - flightPathAngle;
                stableAoA = (AoAtimeConstant * stableAoA + vesselState.deltaT * AoA) / (AoAtimeConstant + vesselState.deltaT); //a sort of integral error

                pitchCorrection = (pitchCorrectionTimeConstant * pitchCorrection + vesselState.deltaT * (nosePitch - vesselState.vesselPitch)) / (pitchCorrectionTimeConstant + vesselState.deltaT);
                pitchCorrection = Mathf.Clamp((float)pitchCorrection, -maxPitchCorrection, maxPitchCorrection);
            }
        }

        Vector3d RunwayStart()
        {
            Vector3d runwayStart = runway.Start(vesselState.CoM);
            Vector3d runwayEnd = runway.End(vesselState.CoM);
            Vector3d runwayDir = (runwayEnd - runwayStart).normalized;
            runwayStart += touchdownPoint * runwayDir;
            return runwayStart;
        }

        public Vector3d ILSAimDirection()
        {
            //positions of the start and end of the runway
            Vector3d runwayStart = RunwayStart();
            Vector3d runwayEnd = runway.End(vesselState.CoM);

            //a coordinate system oriented to the runway
            Vector3d runwayUpUnit = runway.Up();
            Vector3d runwayHorizontalUnit = Vector3d.Exclude(runwayUpUnit, runwayStart - runwayEnd).normalized;
            Vector3d runwayLeftUnit = -Vector3d.Cross(runwayHorizontalUnit, runwayUpUnit).normalized;

            Vector3d vesselUnit = (vesselState.CoM - runwayStart).normalized;

            double leftSpeed = Vector3d.Dot(vesselState.velocityVesselSurface, runwayLeftUnit);
            double verticalSpeed = vesselState.speedVertical;
            double horizontalSpeed = vesselState.speedSurfaceHorizontal;
            double flightPathAngle = 180 / Math.PI * Math.Atan2(verticalSpeed, horizontalSpeed);

            double leftDisplacement = Vector3d.Dot(runwayLeftUnit, vesselState.CoM - runwayStart);

            Vector3d vectorToRunway = runwayStart - vesselState.CoM;
            double verticalDistanceToRunway = Vector3d.Dot(vectorToRunway, vesselState.up);
            double horizontalDistanceToRunway = Math.Sqrt(vectorToRunway.sqrMagnitude - verticalDistanceToRunway * verticalDistanceToRunway);
            double flightPathAngleToRunway = 180 / Math.PI * Math.Atan2(verticalDistanceToRunway, horizontalDistanceToRunway);

            Vector3d aimToward = runwayStart - 3 * leftDisplacement * runwayLeftUnit;
            Vector3d aimDir = aimToward - vesselState.CoM;

            return aimDir;
        }

        public MechJebModuleSpaceplaneAutopilot(MechJebCore core) : base(core) { }
    }

    public struct Runway
    {
        public struct Endpoint
        {
            public double latitude;
            public double longitude;
            public double altitude;

            public Vector3d Position()
            {
                //hardcoded to use Kerbin for the moment:
                return FlightGlobals.Bodies[1].GetWorldSurfacePosition(latitude, longitude, altitude);
            }
        }

        public string name;
        public Endpoint start;
        public Endpoint end;

        public Vector3d Start(Vector3d approachPosition)
        {
            Vector3d startPos = start.Position();
            Vector3d endPos = end.Position();
            if (Vector3d.Distance(startPos, approachPosition) < Vector3d.Distance(endPos, approachPosition)) return startPos;
            else return endPos;
        }

        public Vector3d End(Vector3d approachPosition)
        {
            Vector3d startPos = start.Position();
            Vector3d endPos = end.Position();
            if (Vector3d.Distance(startPos, approachPosition) < Vector3d.Distance(endPos, approachPosition)) return endPos;
            else return startPos;
        }

        public Vector3d Up()
        {
            return FlightGlobals.Bodies[1].GetSurfaceNVector(start.latitude, start.longitude);
        }
    }
}
