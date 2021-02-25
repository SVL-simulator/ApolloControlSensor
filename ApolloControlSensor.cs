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

namespace Simulator.Sensors
{
    [SensorType("Apollo Control", new[] { typeof(VehicleControlData) })]
    public class ApolloControlSensor : SensorBase, IVehicleInputs
    {
        VehicleControlData Data;
        VehicleController Controller;
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

        double LastTimeStamp = 0;  // from Apollo

        VehicleControlData controlData;

        private void Awake()
        {
            LastControlUpdate = SimulatorManager.Instance.CurrentTime;
            Controller = GetComponentInParent<VehicleController>();
            Dynamics = GetComponentInParent<IVehicleDynamics>();
        }

        private void Update()
        {
            var projectedLinVec = Vector3.Project(Dynamics.RB.velocity, transform.forward);
            ActualLinVel = projectedLinVec.magnitude * (Vector3.Dot(Dynamics.RB.velocity, transform.forward) > 0 ? 1.0f : -1.0f);

            var projectedAngVec = Vector3.Project(Dynamics.RB.angularVelocity, transform.up);
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
            bridge.AddSubscriber<VehicleControlData>(Topic, data =>
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
    }
}
