/**
 * Copyright (c) 2019-2021 LG Electronics, Inc.
 *
 * This software contains code licensed as described in LICENSE.
 *
 */

using SimpleJSON;
using Simulator.Api;
using Simulator.Bridge;
using Simulator.Bridge.Data;
using Simulator.Utilities;
using UnityEngine;
using Simulator.Sensors.UI;
using System.Collections.Generic;
using System.Collections;
using Simulator.Analysis;

namespace Simulator.Sensors
{
    [SensorType("Apollo Control", new[] { typeof(VehicleControlData) })]
    public class ApolloControlSensor : SensorBase, IVehicleInputs
    {
        VehicleControlData Data;
        IAgentController Controller;
        IVehicleDynamics Dynamics;

        double LastControlUpdate = 0f;
        float ActualLinVel = 0f;
        float ActualAngVel = 0f;

        public float SteerInput { get; private set; } = 0f;
        public float AccelInput { get; private set; } = 0f;
        public float BrakeInput { get; private set; } = 0f;

        [AnalysisMeasurement(MeasurementType.Input)]
        public float MaxSteer = 0f;

        [AnalysisMeasurement(MeasurementType.Input)]
        public float MaxAccel = 0f;

        [AnalysisMeasurement(MeasurementType.Input)]
        public float MaxBrake = 0f;

        [AnalysisMeasurement(MeasurementType.Misc)]
        public bool IsControlReceived = false;

        float ADAccelInput = 0f;
        float ADSteerInput = 0f;

        public AnimationCurve AccelerationInputCurve;
        public AnimationCurve BrakeInputCurve;

        private double LastTimeStamp = 0;  // from Apollo

        private VehicleControlData controlData;

        [SensorParameter]
        public float StuckTravelThreshold = 0.1f; // apollo autoware lgsvl sensor
        [SensorParameter]
        public float StuckTimeThreshold = 10.0f;
        private float ThrottleCommand = 0f;
        private float ThrottleCuttoff = 0.05f;
        private Vector3 StuckStartPosition;
        private float StuckTime;
        private bool EgoIsStuck = false;

        private void Awake()
        {
            LastControlUpdate = SimulatorManager.Instance.CurrentTime;
            Controller = GetComponentInParent<IAgentController>();
            Dynamics = GetComponentInParent<IVehicleDynamics>();
        }

        protected override void Initialize()
        {
            StuckStartPosition = transform.position;
        }

        protected override void Deinitialize()
        {
            
        }

        private void Update()
        {
            // stuck analysis
            ThrottleCommand = Dynamics.AccellInput;
            if (!EgoIsStuck && ThrottleCommand > ThrottleCuttoff && Vector3.Distance(transform.position, StuckStartPosition) < StuckTravelThreshold)
            {
                StuckTime += Time.fixedDeltaTime;
                if (StuckTime > StuckTimeThreshold)
                {
                    StuckEvent(Controller.GTID);
                    EgoIsStuck = true;
                }
            }
            else
            {
                StuckStartPosition = transform.position;
                StuckTime = 0f;
            }

            var projectedLinVec = Vector3.Project(Dynamics.Velocity, transform.forward);
            ActualLinVel = projectedLinVec.magnitude * (Vector3.Dot(Dynamics.Velocity, transform.forward) > 0 ? 1.0f : -1.0f);

            var projectedAngVec = Vector3.Project(Dynamics.AngularVelocity, transform.up);
            ActualAngVel = projectedAngVec.magnitude * (projectedAngVec.y > 0 ? -1.0f : 1.0f);

            // LastControlUpdate and Time.Time come from Unity.
            if (SimulatorManager.Instance.CurrentTime - LastControlUpdate >= 0.5)    // > 500ms
            {
                ADAccelInput = ADSteerInput = AccelInput = SteerInput = 0f;
            }
        }

        private void FixedUpdate()
        {
            if (SimulatorManager.Instance.CurrentTime - LastControlUpdate < 0.5f)
            {
                AccelInput = ADAccelInput;
                SteerInput = ADSteerInput;
                MaxSteer = Mathf.Max(MaxSteer, Mathf.Sign(SteerInput) * SteerInput);
                MaxAccel = Mathf.Max(MaxAccel, Mathf.Sign(AccelInput) * AccelInput);
                MaxBrake = Mathf.Max(MaxBrake, Mathf.Sign(BrakeInput) * BrakeInput);
            }
        }

        public override void OnBridgeSetup(BridgeInstance bridge)
        {
            var subscriber = BridgeMessageDispatcher.Instance.GetSynchronousSubscriber<VehicleControlData>(data =>
            {
                if (!IsControlReceived)
                {
                    IsControlReceived = true;

                    if (ApiManager.Instance != null)
                    {
                        var jsonData = new JSONObject();
                        ApiManager.Instance.AddCustom(transform.parent.gameObject, "checkControl", jsonData);
                    }
                }

                controlData = data;
                LastControlUpdate = SimulatorManager.Instance.CurrentTime;

                if (double.IsInfinity(data.Acceleration.GetValueOrDefault()) || double.IsInfinity(data.Braking.GetValueOrDefault()) ||
                    double.IsNaN(data.Acceleration.GetValueOrDefault()) || double.IsNaN(data.Braking.GetValueOrDefault()))
                {
                    return;
                }

                var timeStamp = data.TimeStampSec.GetValueOrDefault();
                var dt = (float)(timeStamp - LastTimeStamp);
                LastTimeStamp = timeStamp;

                Debug.Assert(data.Acceleration.GetValueOrDefault() >= 0 && data.Acceleration.GetValueOrDefault() <= 1);
                Debug.Assert(data.Braking.GetValueOrDefault() >= 0 && data.Braking.GetValueOrDefault() <= 1);
                var linearAccel = AccelerationInputCurve.Evaluate(data.Acceleration.GetValueOrDefault()) - BrakeInputCurve.Evaluate(data.Braking.GetValueOrDefault());

                var steeringTarget = -data.SteerTarget.GetValueOrDefault();
                var steeringAngle = Controller.SteerInput;
                var sgn = Mathf.Sign(steeringTarget - steeringAngle);
                var steeringRate = data.SteerRate.GetValueOrDefault() * sgn;

                steeringAngle += steeringRate * dt;
                if (sgn != steeringTarget - steeringAngle)  // prevent oversteering
                {
                    steeringAngle = steeringTarget;
                }

                ADSteerInput = steeringAngle;
                ADAccelInput = linearAccel;

                if (data.CurrentGear == GearPosition.Reverse)
                {
                    Dynamics.ShiftReverseAutoGearBox();
                }
                else if (data.CurrentGear == GearPosition.Drive)
                {
                    Dynamics.ShiftFirstGear();
                }
            });

            bridge.AddSubscriber(Topic, subscriber);
        }

        public override void OnVisualize(Visualizer visualizer)
        {
            Debug.Assert(visualizer != null);
            var graphData = new Dictionary<string, object>()
            {
                {"AD Accel Input", ADAccelInput},
                {"AD Steer Input", ADSteerInput},
                {"Last Control Update", LastControlUpdate},
                {"Actual Linear Velocity", ActualLinVel},
                {"Actual Angular Velocity", ActualAngVel},
                { "EgoIsStuck", EgoIsStuck },
            };

            if (controlData == null)
            {
                return;
            }
            graphData.Add("Acceleration", controlData.Acceleration.GetValueOrDefault());
            graphData.Add("Braking", controlData.Braking.GetValueOrDefault());
            graphData.Add("Time Stamp Sec", controlData.TimeStampSec.GetValueOrDefault());
            graphData.Add("Steer Rate", controlData.SteerRate.GetValueOrDefault());
            graphData.Add("Steer Target", controlData.SteerTarget.GetValueOrDefault());
            graphData.Add("Gear", controlData.CurrentGear);

            visualizer.UpdateGraphValues(graphData);
        }

        public override void OnVisualizeToggle(bool state)
        {
            //
        }

        private void StuckEvent(uint id)
        {
            Hashtable data = new Hashtable
            {
                { "Id", id },
                { "Type", "Stuck" },
                { "Time", SimulatorManager.Instance.GetSessionElapsedTimeSpan().ToString() },
                { "Status", AnalysisManager.AnalysisStatusType.Failed },
            };
            SimulatorManager.Instance.AnalysisManager.AddEvent(data);
        }
    }
}
