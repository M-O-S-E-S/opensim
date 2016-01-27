
// PhysX Plug-in
//
// Copyright 2015 University of Central Florida
//
//
// This plug-in uses NVIDIA's PhysX library to handle physics for OpenSim.
// Its goal is to provide all the functions that the Bullet plug-in does.
//
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


// NOTE: This file (PxActorVehicle.cs) is largely a PhysX implementation
//       of the BulletSim plug-in BSDynamics.cs file.  Appropriate credit
//       is acknowledged to that team.  Please see their code and their
//       license in OpenSim/Region/Physics/BulletSPlugin/BSDynamics.cs.


using log4net;

using System;
using System.Collections.Generic;
using System.Reflection;
using System.Runtime.InteropServices;
using OpenMetaverse;
using OpenSim.Framework;
using OpenSim.Region.Physics.Manager;

namespace OpenSim.Region.Physics.PhysXPlugin
{
    public sealed class PxActorVehicle : PxActor
    {   
        /// <summary>
        /// The logger for this plugin.
        /// </summary>
        internal static readonly ILog m_log = LogManager.GetLogger(
            System.Reflection.MethodBase.GetCurrentMethod().DeclaringType);

        /// <summary>
        /// Header used for logger to highlight logs made in this class.
        /// </summary>
        internal static readonly string LogHeader = "[PHYSX PXACTORVEHICLE]";

        /// <summary>
        /// A boolean denoting if this vehicle has pre/post step delegates
        /// registered within the physics scene. 
        /// </summary>
        private bool m_hasRegisteredForSceneEvents;

        /// <summary>
        /// The mass of the physical object fetched each time the
        /// refresh method is called, used in some calculations
        /// within this class.
        /// </summary>
        private float m_vehicleMass;

        /// <summary>
        /// The type of vehicle that this actor is, which determines the
        /// default/preset values for the vehicle.
        /// </summary>
        public Vehicle Type { get; set; }

        /// <summary>
        /// OpenSim vehicle flags to identify properties of the vehicles.
        /// </summary>
        private VehicleFlag m_flags = (VehicleFlag) 0;

        /// <summary>
        /// The blocking end point for this vehicle, to modify movement
        /// some.
        /// </summary>
        private Vector3 m_blockingEndPoint = Vector3.Zero;

        /// <summary>
        /// The roll reference frame for the vehicle, responsible for use
        /// in angular calculations.
        /// </summary>
        private Quaternion m_rollReferenceFrame = Quaternion.Identity;
        
        /// <summary>
        /// The reference frame for this vehice, responsible for use
        /// in angular calculations.
        /// </summary>
        private Quaternion m_referenceFrame = Quaternion.Identity;
        
        /// <summary>
        /// The buoyancy factor for this vehicle.
        /// </summary>
        private float m_vehicleBuoyancy = 0.0f;

        /// <summary>
        /// Determines if the current actor is active as a vehicle.
        /// </summary>
        public bool IsActive
        {
            get
            {
                // Returns whether this actor has a type of vehicle and is
                // currently a physical object
                return (Type != Vehicle.TYPE_NONE && PhysicsObject.IsPhysical);
            }
        }

        /// <summary>
        /// Determines if the current actor is a ground vehicle.
        /// </summary>
        public bool IsGroundVehicle
        {
            get
            {
                // The current ground vehicles are the sled and car so return
                // true if this actor controls either of those vehicles
                return (Type == Vehicle.TYPE_CAR || Type == Vehicle.TYPE_SLED);
            }
        }

        /// <summary>
        /// Getter and setter for the vehicle position of the vehicle
        /// utilizing the controlling prim.
        /// </summary>
        private Vector3 VehiclePosition
        {
            get
            {
                // If we know that the position has changed, go ahead
                // and update the current known position
                if ((m_knownHas & m_knownChangedPosition) == 0)
                {
                    m_knownPosition = PhysicsObject.Position;
                    m_knownHas |= m_knownChangedPosition;
                }

                return m_knownPosition;
            }
            set
            {
                // Update the known position, and update the flags
                // that indicate if we have updated the position
                m_knownPosition = value;
                m_knownChanged |= m_knownChangedPosition;
                m_knownHas |= m_knownChangedPosition;
            }
        }

        /// <summary>
        /// Quaternion representing the orientation of the vehicle actor.
        /// </summary>
        private Quaternion VehicleOrientation
        {
            get
            {
                // If we know that the orientation has changed, go ahead
                // and update the current known orientation
                if ((m_knownHas & m_knownChangedOrientation) == 0)
                {
                    m_knownOrientation = PhysicsObject.Orientation;
                    m_knownHas |= m_knownChangedOrientation;
                }

                return m_knownOrientation;
            }
            set
            {
                // Update the known orientation, and update the flags
                // that indicate if we have updated the orientation
                m_knownOrientation = value;
                m_knownChanged |= m_knownChangedOrientation;
                m_knownHas |= m_knownChangedOrientation;
            }
        }

        /// <summary>
        /// Vector representation of the velocity of the vehicle actor.
        /// </summary>
        private Vector3 VehicleVelocity
        {
            get
            {
                // If we know that the velocity has changed, go ahead
                // and update the current known velocity
                if ((m_knownHas & m_knownChangedVelocity) == 0)
                {
                    m_knownVelocity = PhysicsObject.Velocity;
                    m_knownHas |= m_knownChangedVelocity;
                }

                return m_knownVelocity;
            }
            set
            {
                // Update the known velocity, and update the flags
                // that indicate if we have updated the orientation
                m_knownVelocity = value;
                m_knownChanged |= m_knownChangedVelocity;
                m_knownHas |= m_knownChangedVelocity;
            }
        }

        /// <summary>
        /// Vector representation of the rotational velocity of the vehicle
        /// actor.
        /// </summary>
        private Vector3 VehicleRotationalVelocity
        {
            get
            {
                // If we know that the rotational velocity has changed, go ahead
                // and update the current known rotational velocity
                if ((m_knownHas & m_knownChangedRotationalVelocity) == 0)
                {
                    m_knownRotationalVelocity = PhysicsObject.RotationalVelocity;
                    m_knownHas |= m_knownChangedRotationalVelocity;
                }

                return m_knownRotationalVelocity;
            }
            set
            {
                // Update the known rotational velocity, and update the flags
                // that indicate we have updated the rotational velocity
                m_knownRotationalVelocity = value;
                m_knownChanged |= m_knownChangedRotationalVelocity;
                m_knownHas |= m_knownChangedRotationalVelocity;
            }
        }

        /// <summary>
        /// Vector representation of only the forward velocity of the vehicle
        /// actor.
        /// </summary>
        private Vector3 VehicleForwardVelocity
        {
            get
            {
                return VehicleVelocity * Quaternion.Inverse(
                    Quaternion.Normalize(VehicleFrameOrientation));
            }
        }

        /// <summary>
        /// Quaternion representation of the vehicle frame orentation.
        /// </summary>
        private Quaternion VehicleFrameOrientation
        {
            get
            {
                return VehicleOrientation * m_referenceFrame;
            }
        }

        /// <summary>
        /// Float representation for the forward speed of the vehicle actor.
        /// </summary>
        private float VehicleForwardSpeed
        {
            get
            {
                return VehicleForwardVelocity.X;
            }
        }

        /// <summary>
        /// Save the last linear velocity vector to compare to the
        /// new one as well as for other calculations on this vehicle.
        /// </summary>
        private Vector3 m_lastLinearVelocityVector = Vector3.Zero;

        /// <summary>
        /// Save the last position vector to compare the new one as well
        /// for other calculations on this vehicle.
        /// </summary>
        private Vector3 m_lastPositionVector = Vector3.Zero;

        /// <summary>
        /// Save the last queried position for getting the height of the
        /// terrain, so that we don't always have to requery.
        /// </summary>
        private Vector3 m_lastRememberedHeightPos = new Vector3(-1, -1, -1);

        /// <summary>
        /// Save the last queried position for getting the height of the
        /// water, so that we don't always have to requery.
        /// </summary>
        private Vector3 m_lastRememberedWaterPos = new Vector3(-1, -1, -1);

        /// <summary>
        /// Integer to be bit operated on to track change of the vehicle actor.
        /// </summary>
        private int m_knownChanged;

        /// <summary>
        /// Integer to be bit operated on to determine if there have been 
        /// any changed properties.
        /// </summary>
        private int m_knownHas;
        
        /// <summary>
        /// The last known terrain height that has been gotten from the scene.
        /// </summary>
        private float m_knownTerrainHeight;
        
        /// <summary>
        /// The last known water level that has been gotten from the scene. 
        /// </summary>
        private float m_knownWaterLevel;
        
        /// <summary>
        /// The last known position vector for the vehicle actor.
        /// </summary>
        private Vector3 m_knownPosition;

        /// <summary>
        /// The last known velocity vector for the vehicle actor.
        /// </summary>
        private Vector3 m_knownVelocity;

        /// <summary>
        /// The last known force vector that was applied to the vehicle actor.
        /// </summary>
        private Vector3 m_knownForce;

        /// <summary>
        /// The last known force vector that was applied as an impulse to the
        /// vehicle actor.
        /// </summary>
        private Vector3 m_knownForceImpulse;

        /// <summary>
        /// The last known orientation quaternion for the vehicle actor. 
        /// </summary>
        private Quaternion m_knownOrientation;

        /// <summary>
        /// The last known rotational velocity for the vehicle actor.
        /// </summary>
        private Vector3 m_knownRotationalVelocity;

        /// <summary>
        /// The last known rotational (angular) force applied to the vehicle
        /// actor. 
        /// </summary>
        private Vector3 m_knownRotationalForce;
        
        /// <summary>
        /// The last known rotational (angular) force applied to the vehicle
        /// actor as an impulse.
        /// </summary>
        private Vector3 m_knownRotationalImpulse;

        /// <summary>
        /// A bit shifted integer to help flag when the position has been
        /// changed or updated.
        /// </summary>
        private const int m_knownChangedPosition           = 1 << 0;

        /// <summary>
        /// A bit shifted integer to help flag when the velocity has been
        /// changed or updated.
        /// </summary>
        private const int m_knownChangedVelocity           = 1 << 1;

        /// <summary>
        /// A bit shifted integer to help flag when the force has been
        /// changed or updated.
        /// </summary>
        private const int m_knownChangedForce              = 1 << 2;

        /// <summary>
        /// A bit shifted integer to help flag when the force impulse has
        /// been changed or updated.
        /// </summary>
        private const int m_knownChangedForceImpulse       = 1 << 3;

        /// <summary>
        /// A bit shifted integer to help flag when the orientation has
        /// been changed or updated.
        /// </summary>
        private const int m_knownChangedOrientation        = 1 << 4;

        /// <summary>
        /// A bit shifted integer to help flag when the rotational velocity 
        /// has been changed or updated.
        /// </summary>
        private const int m_knownChangedRotationalVelocity = 1 << 5;

        /// <summary>
        /// A bit shifted integer to help flag when the rotational (angular)
        /// force applied has been changed or updated.
        /// </summary>
        private const int m_knownChangedRotationalForce    = 1 << 6;

        /// <summary>
        /// A bit shifted integer to help flag when the rotation (angular)
        /// impulse force applied has been changed or updated. 
        /// </summary>
        private const int m_knownChangedRotationalImpulse  = 1 << 7;

        /// <summary>
        /// A bit shifted integer to help flag when the terrain height has been
        /// changed or updated.
        /// </summary>
        private const int m_knownChangedTerrainHeight      = 1 << 8;

        /// <summary>
        /// A bit shifted integer to help flag when the water level has been
        /// changed or updated.
        /// </summary>
        private const int m_knownChangedWaterLevel         = 1 << 9;

        #region banking
        /// <summary>
        /// The banking effiency that will be passed to the motors.
        /// </summary>
        private float m_bankingEfficiency = 0.0f;

        /// <summary>
        /// A float to allow for turning in place rather than needing
        /// more forward motion to turn.
        /// </summary>
        private float m_bankingMix = 1.0f;
        
        /// <summary>
        /// The timescale for the banking motor.
        /// </summary>
        private float m_bankingTimescale = 0.0f;
        #endregion banking

        #region Angular Deflection
        /// <summary>
        /// A vector motor to drive the angular deflection of this vehicle.
        /// </summary>
        private PxVMotor m_angularDeflectionMotor = new PxVMotor();

        /// <summary>
        /// The angular deflection effiency to be used in the angular 
        /// deflection motor.
        /// </summary>
        private float m_angularDeflectionEfficiency = 0.0f;

        /// <summary>
        /// The angular deflection timescale to be used in the angular
        /// deflection motor.
        /// </summary>
        private float m_angularDeflectionTimescale = 0.0f;
        #endregion Angular Deflection

        #region Linear Deflection
        /// <summary>
        /// A vector motor to drive the linear deflection of this vehicle
        /// actor.
        /// </summary>
        private PxVMotor m_linearDeflectionMotor = new PxVMotor();

        /// <summary>
        /// The linear deflection efficiency to be used in the linear
        /// deflection motor.
        /// </summary>
        private float m_linearDeflectionEfficiency = 0.0f;

        /// <summary>
        /// The linear deflection timescale to be used in the linear
        /// deflection motor.
        /// </summary>
        private float m_linearDeflectionTimescale = 0.0f;
        #endregion Linear Deflection

        #region Hover
        /// <summary>
        /// A vector motor to drive the hovering of this vehicle actor.
        /// </summary>
        private PxVMotor m_hoverMotor = new PxVMotor();

        /// <summary>
        /// The current hover height of the vehicle actor, to be set as the
        /// current value of the hover motor.
        /// </summary>
        private float m_hoverHeight = 0.0f;

        /// <summary>
        /// The hover effiency to be used on the hover motor.
        /// </summary>
        private float m_hoverEfficiency = 0.0f;

        /// <summary>
        /// The hover timescale to be used on the hover motor.
        /// </summary>
        private float m_hoverTimescale = 0.0f;

        /// <summary>
        /// The hover target height that the hover motor will bring
        /// the height to.
        /// </summary>
        private float m_hoverTargetHeight = -1.0f;
        #endregion Hover

        #region Angular Motion
        /// <summary>
        /// A vector motor for managing rotation.
        /// </summary>
        private PxVMotor m_angularMotor = new PxVMotor();

        /// <summary>
        /// The direction that the angular motor will step the
        /// rotational velocity.
        /// </summary>
        private Vector3 m_angularMotorDirection = Vector3.Zero;
        
        /// <summary>
        /// The offset in the angular motor to account for a new center
        /// of mass.
        /// </summary>
        private Vector3 m_angularMotorOffset = Vector3.Zero;
        
        /// <summary>
        /// The angular motor timescale that helps determine the rate at which
        /// the angular motor will step the vector value.
        /// </summary>
        private float m_angularMotorTimescale = 1.0f;

        /// <summary>
        /// The angular motor decay timescale that helps determine how the
        /// angular motor will wayne off of stepping the vector value.
        /// </summary>
        private float m_angularMotorDecayTimescale = 1.0f;

        /// <summary>
        /// The angular friction vector that represents how much each axis
        /// is effected by the decay timescale at each step.
        /// </summary>
        private Vector3 m_angularFrictionTimescale = Vector3.Zero;
        #endregion Angular Motion

        #region Linear Motion
        /// <summary>
        /// A vector motor to help with linear motion on this physics vehicle
        /// actor.
        /// </summary>
        private PxVMotor m_linearMotor = new PxVMotor();
        
        /// <summary>
        /// Vector for the direction that the linear motor will be
        /// steppng.
        /// </summary>
        private Vector3 m_linearMotorDirection = Vector3.Zero;

        /// <summary>
        /// Linear motor offset to account for the addition
        /// of the rider.
        /// </summary>
        private Vector3 m_linearMotorOffset = Vector3.Zero;

        /// <summary>
        /// The rate at which each of the axis in the linear motion
        /// decay.
        /// </summary>
        private Vector3 m_linearFrictionTimescale = Vector3.Zero;
        
        /// <summary>
        /// The linear motor timescale that helps determine how quickly
        /// the motor steps the value to it's target value.
        /// </summary>
        private float m_linearMotorTimescale = 1.0f;
        
        /// <summary>
        /// The linear motor decay timescale that helps determine the rate
        /// that the amount changed by the motor is decreased.
        /// </summary>
        private float m_linearMotorDecayTimescale = 1.0f;
        #endregion Linear Motion

        #region Attractor Properties
        /// <summary>
        /// A vector motor to handle vertical attraction, which is leveling
        /// the orientation upward, which is more relevant for sea vehicles.
        /// </summary>
        private PxVMotor m_verticalAttractorMotor = new PxVMotor();

        /// <summary>
        /// The effiency at which the vertical attractor goes about leveling
        /// the orientation.
        /// </summary>
        private float m_verticalAttractorEfficiency = 1.0f;

        /// <summary>
        /// The maximum height of the vertical attractor.
        /// </summary>
        private float m_verticalAttractorCutoff = 500.0f;

        /// <summary>
        /// The vertical attractor timescale to determine how quickly the
        /// vertical attractor steps the orientation value.
        /// </summary>
        private float m_verticalAttractorTimescale = 510.0f;
        #endregion Attractor Properties

        /// <summary>
        /// A few constants to make the code easier to read later on.
        /// </summary>
        #pragma warning disable 414
        static readonly float TwoPI = ((float)Math.PI) * 2f; 
        static readonly float FourPI = ((float)Math.PI) * 4f; 
        static readonly float PIOverFour = ((float)Math.PI) / 4f;
        static readonly float PIOverTwo = ((float)Math.PI) / 2f;
        #pragma warning restore 414

        /// <summary>
        /// The constructor for an actor vehicle.
        /// </summary>
        /// <param name="physicsScene"> The physics scene that this actor 
        /// will act within. </param>
        /// <param name="physicsObject"> The physics object that this actor 
        /// will act upon. </param>
        /// <param name="actorName"> The physics actor name to identify 
        /// the intent. <param>
        public PxActorVehicle(PxScene physicsScene, PxPhysObject physObject,
            string actorName) : base (physicsScene, physObject, actorName)
        {
            // Set the type of this vehicle to none since the vehicle will be 
            // determined later
            Type = Vehicle.TYPE_NONE;
        }

        /// <summary>
        /// Applies the effects of the angular motor on this vehicle.
        /// </summary>
        /// <param name="pTimestep">The timestep of the physics scene</param>
        private void MoveAngular(float pTimestep)
        {
            Vector3 torqueFromOffset;

            // Compute and apply the angular calculations
            ComputeAngularTurning(pTimestep);
            ComputeAngularVerticalAttraction();
            ComputeAngularDeflection();
            ComputeAngularBanking();

            // If the vehicle rotational velocity is nearly zero, go ahead
            // and set the vehicle rotation velocity to actually zero
            if (VehicleRotationalVelocity.ApproxEquals(Vector3.Zero, 0.0001f))
            {
                VehicleRotationalVelocity = Vector3.Zero;
            }

            // If the linear motor offset is not equal to zero
            if (m_linearMotorOffset != Vector3.Zero)
            {
                torqueFromOffset = Vector3.Zero;

                // Set the torque from the offset x axis to zero
                // if it is not a number
                if (float.IsNaN(torqueFromOffset.X))
                {
                    torqueFromOffset.X = 0.0f;
                }

                // Set the torque from the offset y axis to zero
                // if it is not a number
                if (float.IsNaN(torqueFromOffset.Y))
                {
                    torqueFromOffset.Y = 0.0f;
                }

                // Set the torque from the offset z axis to zero
                // if it is not a number
                if (float.IsNaN(torqueFromOffset.Z))
                {
                    torqueFromOffset.Z = 0.0f;
                }

                // Apply angular force for the amount of the offset times
                // the mass of the vehicle
                VehicleAddAngularForce(torqueFromOffset * m_vehicleMass);
            }
        }

        /// <summary>
        /// Method to set any vehicle paramter with a new floating point
        /// value.
        /// </summary>
        /// <param name="pParam">The vehicle parameter to be updated </param>
        /// <param name="pValue">The new value for the parameter </param>
        public void ProcessFloatVehicleParam(Vehicle pParam, float pValue)
        {
            // A variable for temporarily storing the clamped float values
            float clampTemp;

            switch(pParam)
            {
                case Vehicle.ANGULAR_DEFLECTION_EFFICIENCY:
                    // Clamp the angular deflection efficiency in the range
                    // of 0.0f to 1.0f
                    m_angularDeflectionEfficiency = ClampInRange(0.0f, pValue,
                        1.0f);
                    break;
                case Vehicle.ANGULAR_DEFLECTION_TIMESCALE:
                    // Clamp the angular deflection timescale in the range
                    // of 0.25f to 120.0f
                    m_angularDeflectionTimescale = ClampInRange(0.25f, pValue,
                        120.0f);
                    break;

                case Vehicle.LINEAR_DEFLECTION_EFFICIENCY:
                    // Clamp the linear deflection efficiency in the range
                    // of 0.0f to 1.0f
                    m_linearDeflectionEfficiency = ClampInRange(0.0f, pValue,
                        1.0f);
                    break;
                case Vehicle.LINEAR_DEFLECTION_TIMESCALE:
                    // Clamp the linear deflection timescale in the range
                    // of 0.25f to 120.0f
                    m_angularDeflectionTimescale = ClampInRange(0.25f, pValue,
                        120.0f);
                    break;

                case Vehicle.ANGULAR_MOTOR_DECAY_TIMESCALE:
                    // Clamp the angular motor decay timescale in the range
                    // of 0.25f to 120.0f
                    m_angularMotorDecayTimescale = ClampInRange(0.25f, pValue,
                        120.0f);

                    // And then use the clampled value to update the
                    // angular motor decay timescale
                    m_angularMotor.TargetValueDecayTimeScale = 
                        m_angularMotorDecayTimescale;
                    break;
                case Vehicle.ANGULAR_MOTOR_TIMESCALE:
                    // Clamp the angular motor decay timescale in the range
                    // of 0.25f to 120.0f
                    m_angularMotorTimescale = ClampInRange(0.25f, pValue, 
                        120.0f);

                    // And then use the clamped value to update the angular
                    // motor timescale
                    m_angularMotor.TimeScale = m_angularMotorTimescale;
                    break;
                case Vehicle.ANGULAR_MOTOR_DIRECTION:
                    // Utilizing the temporary clamp float variable, go ahead
                    // and clamp the value inbetween -TwoPI and TwoPI
                    clampTemp = ClampInRange(-TwoPI, pValue, TwoPI);
                    
                    // Initialize/overwrite the current angular motor direction
                    // vector with a new vector using the clamped value
                    m_angularMotorDirection = new Vector3(clampTemp, clampTemp,
                        clampTemp);

                    // Reset/halt the angular motor by zero'ing it out
                    m_angularMotor.Zero();

                    // Set the target of the angular motor as the clamped 
                    // angular motor direction vector
                    m_angularMotor.SetTarget(m_angularMotorDirection);
                    break;

                case Vehicle.LINEAR_MOTOR_DECAY_TIMESCALE:
                    // Clamp the linear motor decay timescale in the range of
                    // 0.01f to 120.0f
                    m_linearMotorDecayTimescale = ClampInRange(0.01f, pValue,
                        120.0f);

                    // Set the target value decay timescale on the linear motor
                    // as the clamped linear motor decay timescale
                    m_linearMotor.TargetValueDecayTimeScale =
                        m_linearMotorDecayTimescale;
                    break;
                case Vehicle.LINEAR_MOTOR_TIMESCALE:
                    // Clamp the linear motor timescale in the range of
                    // 0.01f to 120.0f
                    m_linearMotorTimescale = ClampInRange(0.01f, pValue,
                        120.0f);

                    // Set the time scale on the linear motor as the clamped
                    // linear motor timescale
                    m_linearMotor.TimeScale = m_linearMotorTimescale;
                    break;
                case Vehicle.LINEAR_MOTOR_DIRECTION:
                    // Utilizing the temporary clamp variable, clamp the value
                    // in the negative and positive range of the max vehicle
                    // velocity value
                    clampTemp = ClampInRange(-PhysicsScene.UserConfig.VehicleMaxLinearVelocity,
                        pValue, PhysicsScene.UserConfig.VehicleMaxLinearVelocity);

                    // Update the linear motor direction vector value
                    // with the clamped values for each axis
                    m_linearMotorDirection = new Vector3(clampTemp, clampTemp,
                        clampTemp);

                    // Set the target vector value on the linear motor
                    // to the updated linear motor direction vector
                    m_linearMotor.SetTarget(m_linearMotorDirection);
                    break;
                case Vehicle.LINEAR_MOTOR_OFFSET:
                    // Utilizing the temporary clamp variable, clamp the value
                    // in the range of negative 1000.0f to positive 1000.0f
                    clampTemp = ClampInRange(-1000.0f, pValue, 1000.0f);

                    // Update the linear motor offset vector value with 
                    // a vector value of the clamped value 
                    m_linearMotorOffset = new Vector3(clampTemp, clampTemp,
                        clampTemp);
                    break;

                case Vehicle.HOVER_EFFICIENCY:
                    // Clamp the hover efficiency witihin the range of
                    // 0.01f and 1.0f
                    m_hoverEfficiency = ClampInRange(0.01f, pValue, 1.0f);
                    break;
                case Vehicle.HOVER_TIMESCALE:
                    // Clamp the hover timescale witihn the range of
                    // 0.01f and 120.0f
                    m_hoverTimescale = ClampInRange(0.01f, pValue, 120.0f);
                    break;
                case Vehicle.HOVER_HEIGHT:
                    // Clamp the hover height within the range of
                    // 0.0f to 1000000.0f
                    m_hoverHeight = ClampInRange(0.0f, pValue, 1000000.0f);
                    break;

                case Vehicle.VERTICAL_ATTRACTION_EFFICIENCY:
                    // Clamp the vertical attraction efficiency within the
                    // range of 0.1f to 1.0f
                    m_verticalAttractorEfficiency = ClampInRange(0.1f, pValue,
                        1.0f);

                    // Update the efficiency on the vertical attractor motor
                    // with the clamped vertical attractor efficiency value
                    m_verticalAttractorMotor.Efficiency =
                        m_verticalAttractorEfficiency;
                    break;
                case Vehicle.VERTICAL_ATTRACTION_TIMESCALE:
                    // Clamp the vertical attraction timescale within the
                    // range of 0.01f to 120.0f
                    m_verticalAttractorTimescale = ClampInRange(0.01f, pValue,
                        120.0f);

                    // Update the vertical attractor motor timescale with the
                    // clamped vertical attractor timescale value
                    m_verticalAttractorMotor.TimeScale =
                        m_verticalAttractorTimescale;  
                    break; 

                case Vehicle.BANKING_EFFICIENCY:
                    // Clamp the banking efficiency within the range of
                    // negative 1.0f to positive 1.0f
                    m_bankingEfficiency = ClampInRange(-1.0f, pValue,
                        1.0f);
                    break;
                case Vehicle.BANKING_MIX:
                    // Clamp the banking mix float value within the range
                    // of 0.01f to 1.0f
                    m_bankingMix = ClampInRange(0.01f, pValue, 1.0f);
                    break;
                case Vehicle.BANKING_TIMESCALE:
                    // Clamp the banking timescale witihin the range of
                    // 0.25f to 120.0f
                    m_bankingTimescale = ClampInRange(0.25f, pValue, 120.0f);
                    break;
            }
        }

        /// <summary>
        /// Method to store a vector parameter to this vehicle actor.
        /// </summary>
        /// <param name="pParam">The parameter that is being updated to the new
        /// value</param>
        /// <param name="pValue">The new value of the parameter being
        /// updated</param>
        public void ProcessVectorVehicleParam(Vehicle pParam, Vector3 pValue)
        {
           switch(pParam)
            {
                case Vehicle.ANGULAR_FRICTION_TIMESCALE:
                    // Clamp all three parts of the vector within the range
                    // of 0.25f and 120.0f
                    pValue.X = ClampInRange(0.25f, pValue.X, 120.0f);
                    pValue.Y = ClampInRange(0.25f, pValue.Y, 120.0f);
                    pValue.Z = ClampInRange(0.25f, pValue.Z, 120.0f);

                    // Update the angular friction timescale vector with
                    // the clamped values from above
                    m_angularFrictionTimescale = new Vector3(pValue.X,
                        pValue.Y, pValue.Z);
                    break;
                case Vehicle.ANGULAR_MOTOR_DIRECTION:
                    // Limit requested angular speed to 2 rps= 4 pi rads/sec
                    pValue.X = ClampInRange(-FourPI, pValue.X, FourPI);
                    pValue.Y = ClampInRange(-FourPI, pValue.Y, FourPI);
                    pValue.Z = ClampInRange(-FourPI, pValue.Z, FourPI);
                    
                    // Update the angular motor direction vector with the new
                    // vector
                    m_angularMotorDirection = new Vector3(pValue.X, pValue.Y,
                        pValue.Z);

                    // Zero out the angular motor before updating the target
                    m_angularMotor.Zero();

                    // Set the target of the angular motor to the updated,
                    // clamped vector value
                    m_angularMotor.SetTarget(m_angularMotorDirection);
                    break;

                case Vehicle.LINEAR_MOTOR_DIRECTION:
                    // Clamp all three parts of the vector within the range of
                    // the negative and positive max linear velocity of vehicles
                    pValue.X = ClampInRange(-PhysicsScene.UserConfig.VehicleMaxLinearVelocity, 
                        pValue.X, PhysicsScene.UserConfig.VehicleMaxLinearVelocity);
                    pValue.Y = ClampInRange(-PhysicsScene.UserConfig.VehicleMaxLinearVelocity, 
                        pValue.Y, PhysicsScene.UserConfig.VehicleMaxLinearVelocity);
                    pValue.Z = ClampInRange(-PhysicsScene.UserConfig.VehicleMaxLinearVelocity, 
                        pValue.Z, PhysicsScene.UserConfig.VehicleMaxLinearVelocity);

                    // Update the linear motor direction vector with the updated
                    // clamped values
                    m_linearMotorDirection = new Vector3(pValue.X, pValue.Y,
                        pValue.Z);

                    // Zero out the linear motor before updating the target
                    m_linearMotor.Zero();

                    // Set the target of the linear motor to the updated,
                    // clamped vector value
                    m_linearMotor.SetTarget(m_linearMotorDirection);
                    break;
                case Vehicle.LINEAR_FRICTION_TIMESCALE:
                    // Clamp all three parts of the vector within the range
                    // of 0.25f and 120.0f
                    pValue.X = ClampInRange(0.25f, pValue.X, 120.0f);
                    pValue.Y = ClampInRange(0.25f, pValue.Y, 120.0f);
                    pValue.Z = ClampInRange(0.25f, pValue.Z, 120.0f);

                    // Update the linear friction timescale vector with
                    // the clamped values from above
                    m_linearFrictionTimescale = new Vector3(pValue.X,
                        pValue.Y, pValue.Z);
                    break;
                case Vehicle.LINEAR_MOTOR_OFFSET:
                    // Clamp all three parts of the vector witihin the range
                    // of -1000.0f and 1000.0f
                    pValue.X = ClampInRange(-1000.0f, pValue.X, 1000.0f);
                    pValue.Y = ClampInRange(-1000.0f, pValue.Y, 1000.0f);
                    pValue.Z = ClampInRange(-1000.0f, pValue.Z, 1000.0f);

                    // Update the linear motor offset vector with the
                    // clamped values from above
                    m_linearMotorOffset = new Vector3(pValue.X, pValue.Y,
                        pValue.Z);
                    break;

                case Vehicle.BLOCK_EXIT:
                    // Clamp all the parts of the vector within the range of
                    // -10000.0f to 10000.0f 
                    pValue.X = ClampInRange(-10000.0f, pValue.X, 10000.0f);
                    pValue.Y = ClampInRange(-10000.0f, pValue.Y, 10000.0f);
                    pValue.Z = ClampInRange(-10000.0f, pValue.Z, 10000.0f);
                
                    // Update the blocking end point vector value
                    m_blockingEndPoint = new Vector3(pValue.X, pValue.Y,
                        pValue.Z);
                    break;
            }
        }

        /// <summary>
        /// Method to update this actors parameters for orientation.
        /// </summary>
        /// <param name="pParam">The parameter that is being updated</param>
        /// <param name="pValue">The new value of the parameter</param>
        public void ProcessRotationVehicleParam(Vehicle pParam,
            Quaternion pValue)
        {
            switch(pParam)
            {
                case Vehicle.REFERENCE_FRAME:
                    // Simply set the reference frame quaternion to the
                    // passed in quaternion value
                    m_referenceFrame = pValue;
                    break;
                case Vehicle.ROLL_FRAME:
                    // Simply set the roll reference frame quaternion to the
                    // passed in quaternion value
                    m_rollReferenceFrame = pValue;
                    break;
            }
        }


        /// <summary>
        /// Method to add and remove flags from the stored flags.
        /// </summary>
        /// <param name="pParam"> The flag to be updated </param>
        /// <param name="remove"> Determines if the flag should be removed 
        /// (true), or added (false) to the stored flags</param>
        public void ProcessVehicleFlags(int pParam, bool remove)
        {
            VehicleFlag param = (VehicleFlag) pParam;

            // If the parameter is equal to -1, we will reset all
            // the vehicle flags by simply setting them to zero
            if (pParam == -1)
            {
                m_flags = (VehicleFlag) 0;
            }
            else 
            {
                // If we are to remove the given parameter then go ahead
                // and remove it from the flags through bitwise operations
                // Otherwise, if we are to add the parameter to our flags
                // then go ahead and do so
                if (remove)
                {
                    m_flags &= ~param;
                }
                else
                {
                    m_flags |= param;
                }
            }
        }

        /// <summary>
        /// Updates this actor to a new vehicle type, and sets the internal
        /// values to match the default values of the vehicle.
        /// </summary>
        /// <param name="pType">The type of vehicle that this actor is going to
        /// be using</param>
        public void ProcessTypeChange(Vehicle pType)
        {
            // Set the type of this vehicle
            Type = pType;

            switch(pType)
            {
                case Vehicle.TYPE_NONE:
                    // Set up the attributes related to the linear motor
                    m_linearMotorDirection = Vector3.Zero;
                    m_linearMotorTimescale = 0.0f;
                    m_linearMotorDecayTimescale = 0.0f;
                    m_linearFrictionTimescale = new Vector3(0.0f, 0.0f, 0.0f);

                    // Set up the attributes related to the angular motor
                    m_angularMotorDirection = Vector3.Zero;
                    m_angularMotorDecayTimescale = 0.0f;
                    m_angularMotorTimescale = 0.0f;
                    m_angularFrictionTimescale = new Vector3(0.0f, 0.0f, 0.0f);

                    // Set up the attributes related to the hover motor
                    m_hoverHeight = 0.0f;
                    m_hoverEfficiency = 0.0f;
                    m_hoverTimescale = 0.0f;
                    m_vehicleBuoyancy = 0.0f;

                    // Set up attributes related to the linear deflection
                    m_linearDeflectionEfficiency = 1.0f;
                    m_linearDeflectionTimescale = 1.0f;

                    // Set up attributes related to the angular deflection
                    m_angularDeflectionEfficiency = 0.0f;
                    m_angularDeflectionTimescale = 1000.0f;

                    // Set up the attributes related to the vertical attractor
                    m_verticalAttractorEfficiency = 0.0f;
                    m_verticalAttractorTimescale = 0.0f;

                    // Set up the attributes related to banking
                    m_bankingEfficiency = 0.0f;
                    m_bankingTimescale = 1000.0f;
                    m_bankingMix = 1.0f;

                    // Set up the reference frame and base vehicle flags
                    m_referenceFrame = Quaternion.Identity;
                    m_flags = (VehicleFlag) 0;
                    break;

                case Vehicle.TYPE_SLED:
                    // Set up the attributes related to the linear motor
                    m_linearMotorDirection = Vector3.Zero;
                    m_linearMotorTimescale = 1000.0f;
                    m_linearMotorDecayTimescale = 120.0f;
                    m_linearFrictionTimescale = new Vector3(30.0f, 1.0f, 
                        1000.0f);

                    // Set up the attributes related to the angular motor
                    m_angularMotorDirection = Vector3.Zero;
                    m_angularMotorDecayTimescale = 1000.0f;
                    m_angularMotorTimescale = 120.0f;
                    m_angularFrictionTimescale = new Vector3(1000.0f, 1000.0f, 
                        1000.0f);

                    // Set up the attributes related to the hover motor
                    m_hoverHeight = 0.0f;
                    m_hoverEfficiency = 10.0f;
                    m_hoverTimescale = 10.0f;
                    m_vehicleBuoyancy = 0.0f;

                    // Set up attributes related to the linear deflection
                    m_linearDeflectionEfficiency = 1.0f;
                    m_linearDeflectionTimescale = 1.0f;

                    // Set up attributes related to the angular deflection
                    m_angularDeflectionEfficiency = 1.0f;
                    m_angularDeflectionTimescale = 1000.0f;

                    // Set up the attributes related to the vertical attractor
                    m_verticalAttractorEfficiency = 0.0f;
                    m_verticalAttractorTimescale = 0.0f;

                    // Set up the attributes related to banking
                    m_bankingEfficiency = 0.0f;
                    m_bankingTimescale = 10.0f;
                    m_bankingMix = 1.0f;

                    // Set up the reference frame and base vehicle flags
                    m_referenceFrame = Quaternion.Identity;
                    m_flags &= ~(VehicleFlag.HOVER_WATER_ONLY
                                | VehicleFlag.HOVER_TERRAIN_ONLY
                                | VehicleFlag.HOVER_GLOBAL_HEIGHT
                                | VehicleFlag.HOVER_UP_ONLY);
                    m_flags |= (VehicleFlag.NO_DEFLECTION_UP
                                | VehicleFlag.LIMIT_ROLL_ONLY
                                | VehicleFlag.LIMIT_MOTOR_UP);
                    break;

                case Vehicle.TYPE_CAR:
                    // Set up the attributes related to the linear motor
                    m_linearMotorDirection = Vector3.Zero;
                    m_linearMotorTimescale = 1.0f;
                    m_linearMotorDecayTimescale = 60.0f;
                    m_linearFrictionTimescale = new Vector3(100.0f, 2.0f,
                        1000.0f);

                    // Set up the attributes related to the angular motor
                    m_angularMotorDirection = Vector3.Zero;
                    m_angularMotorDecayTimescale = 1.0f;
                    m_angularMotorTimescale = 0.8f;
                    m_angularFrictionTimescale = new Vector3(1000.0f, 1000.0f, 
                        1000.0f);

                    // Set up the attributes related to the hover motor
                    m_hoverHeight = 0.0f;
                    m_hoverEfficiency = 0.0f;
                    m_hoverTimescale = 1000.0f;
                    m_vehicleBuoyancy = 0.0f;

                    // Set up attributes related to the linear deflection
                    m_linearDeflectionEfficiency = 1.0f;
                    m_linearDeflectionTimescale = 2.0f;

                    // Set up attributes related to the angular deflection
                    m_angularDeflectionEfficiency = 0.0f;
                    m_angularDeflectionTimescale = 10.0f;

                    // Set up the attributes related to the vertical attractor
                    m_verticalAttractorEfficiency = 1.0f;
                    m_verticalAttractorTimescale = 10.0f;

                    // Set up the attributes related to banking
                    m_bankingEfficiency = -0.2f;
                    m_bankingTimescale = 1.0f;
                    m_bankingMix = 1.0f;

                    // Set up the reference frame and base vehicle flags
                    m_referenceFrame = Quaternion.Identity;
                    m_flags &= ~(VehicleFlag.HOVER_WATER_ONLY
                                | VehicleFlag.HOVER_TERRAIN_ONLY
                                | VehicleFlag.HOVER_GLOBAL_HEIGHT);
                    m_flags |= (VehicleFlag.NO_DEFLECTION_UP
                                | VehicleFlag.LIMIT_ROLL_ONLY
                                | VehicleFlag.LIMIT_MOTOR_UP
                                | VehicleFlag.HOVER_UP_ONLY);
                    break;
                
                case Vehicle.TYPE_BOAT:
                    // Set up the attributes related to the linear motor
                    m_linearMotorDirection = Vector3.Zero;
                    m_linearMotorTimescale = 5.0f;
                    m_linearMotorDecayTimescale = 60.0f;
                    m_linearFrictionTimescale = new Vector3(10.0f, 3.0f, 2.0f);

                    // Set up the attributes related to the angular motor
                    m_angularMotorDirection = Vector3.Zero;
                    m_angularMotorDecayTimescale = 4.0f;
                    m_angularMotorTimescale = 4.0f;
                    m_angularFrictionTimescale = new Vector3(10.0f, 10.0f, 10.0f);

                    // Set up the attributes related to the hover motor
                    m_hoverHeight = 0.0f;
                    m_hoverEfficiency = 0.5f;
                    m_hoverTimescale = 2.0f;
                    m_vehicleBuoyancy = 1.0f;

                    // Set up attributes related to the linear deflection
                    m_linearDeflectionEfficiency = 0.5f;
                    m_linearDeflectionTimescale = 3.0f;

                    // Set up attributes related to the angular deflection
                    m_angularDeflectionEfficiency = 0.5f;
                    m_angularDeflectionTimescale = 5.0f;

                    // Set up the attributes related to the vertical attractor
                    m_verticalAttractorEfficiency = 0.5f;
                    m_verticalAttractorTimescale = 5.0f;

                    // Set up the attributes related to banking
                    m_bankingEfficiency = -0.3f;
                    m_bankingTimescale = 1.0f;
                    m_bankingMix = 0.8f;

                    // Set up the reference frame and base vehicle flags
                    m_referenceFrame = Quaternion.Identity;
                    m_flags &= ~(VehicleFlag.HOVER_TERRAIN_ONLY
                                    | VehicleFlag.HOVER_GLOBAL_HEIGHT
                                    | VehicleFlag.LIMIT_ROLL_ONLY
                                    | VehicleFlag.HOVER_UP_ONLY);
                    m_flags |= (VehicleFlag.NO_DEFLECTION_UP
                                    | VehicleFlag.LIMIT_MOTOR_UP
                                    | VehicleFlag.HOVER_WATER_ONLY);
                    break;

                case Vehicle.TYPE_AIRPLANE:
                    // Set up the attributes related to the linear motor
                    m_linearMotorDirection = Vector3.Zero;
                    m_linearMotorTimescale = 2.0f;
                    m_linearMotorDecayTimescale = 60.0f;
                    m_linearFrictionTimescale = new Vector3(200.0f, 10.0f,
                        5.0f);

                    // Set up the attributes related to the angular motor
                    m_angularMotorDirection = Vector3.Zero;
                    m_angularMotorDecayTimescale = 4.0f;
                    m_angularMotorTimescale = 4.0f;
                    m_angularFrictionTimescale = new Vector3(20.0f, 20.0f, 
                        20.0f);

                    // Set up the attributes related to the hover motor
                    m_hoverHeight = 0.0f;
                    m_hoverEfficiency = 0.5f;
                    m_hoverTimescale = 1000.0f;
                    m_vehicleBuoyancy = 0.0f;

                    // Set up attributes related to the linear deflection
                    m_linearDeflectionEfficiency = 0.5f;
                    m_linearDeflectionTimescale = 3.0f;

                    // Set up attributes related to the angular deflection
                    m_angularDeflectionEfficiency = 1.0f;
                    m_angularDeflectionTimescale = 2.0f;

                    // Set up the attributes related to the vertical attractor
                    m_verticalAttractorEfficiency = 0.9f;
                    m_verticalAttractorTimescale = 2.0f;

                    // Set up the attributes related to banking
                    m_bankingEfficiency = 1.0f;
                    m_bankingTimescale = 2.0f;
                    m_bankingMix = 0.7f;

                    // Set up the reference frame and base vehicle flags
                    m_referenceFrame = Quaternion.Identity;
                    m_flags &= ~(VehicleFlag.HOVER_WATER_ONLY
                                    | VehicleFlag.HOVER_TERRAIN_ONLY
                                    | VehicleFlag.HOVER_GLOBAL_HEIGHT
                                    | VehicleFlag.HOVER_UP_ONLY
                                    | VehicleFlag.NO_DEFLECTION_UP
                                    | VehicleFlag.LIMIT_MOTOR_UP);
                    m_flags |= (VehicleFlag.LIMIT_ROLL_ONLY);
                    break;

                case Vehicle.TYPE_BALLOON:
                    // Set up the attributes related to the linear motor
                    m_linearMotorDirection = Vector3.Zero;
                    m_linearMotorTimescale = 5.0f;
                    m_linearMotorDecayTimescale = 60.0f;
                    m_linearFrictionTimescale = new Vector3(5.0f, 5.0f, 5.0f);

                    // Set up the attributes related to the angular motor
                    m_angularMotorDirection = Vector3.Zero;
                    m_angularMotorDecayTimescale = 10.0f;
                    m_angularMotorTimescale = 6.0f;
                    m_angularFrictionTimescale = new Vector3(10.0f, 10.0f, 10.0f);

                    // Set up the attributes related to the hover motor
                    m_hoverHeight = 5.0f;
                    m_hoverEfficiency = 0.8f;
                    m_hoverTimescale = 10.0f;
                    m_vehicleBuoyancy = 1.0f;

                    // Set up attributes related to the linear deflection
                    m_linearDeflectionEfficiency = 0.0f;
                    m_linearDeflectionTimescale = 5.0f;

                    // Set up attributes related to the angular deflection
                    m_angularDeflectionEfficiency = 0.0f;
                    m_angularDeflectionTimescale = 5.0f;

                    // Set up the attributes related to the vertical attractor
                    m_verticalAttractorEfficiency = 1.0f;
                    m_verticalAttractorTimescale = 100.0f;

                    // Set up the attributes related to banking
                    m_bankingEfficiency = 0.0f;
                    m_bankingTimescale = 5.0f;
                    m_bankingMix = 0.7f;

                    // Set up the reference frame and base vehicle flags
                    m_referenceFrame = Quaternion.Identity;
                    m_flags &= ~(VehicleFlag.HOVER_WATER_ONLY
                                    | VehicleFlag.HOVER_TERRAIN_ONLY
                                    | VehicleFlag.HOVER_UP_ONLY
                                    | VehicleFlag.NO_DEFLECTION_UP
                                    | VehicleFlag.LIMIT_MOTOR_UP);
                    m_flags |= (VehicleFlag.LIMIT_ROLL_ONLY
                                    | VehicleFlag.HOVER_GLOBAL_HEIGHT);
                    break;
            }

            // Create a new linear motor for the new type of vehicle
            m_linearMotor = new PxVMotor(m_linearMotorTimescale,
                m_linearMotorDecayTimescale, 1.0f);

            // Create a new angular motor for the new type of vehicle
            m_angularMotor = new PxVMotor(m_angularMotorTimescale,
                m_angularMotorDecayTimescale, 1.0f);

            // As long as this is a non-none type vehicle, then go ahead
            // and register for scene events
            if (Type == Vehicle.TYPE_NONE)
            {
                UnregisterForSceneEvents();
            }
            else
            {
                RegisterForSceneEvents();
            }

            // Update any physical parameters based on this type
            Refresh();
        }

        /// <summary>
        /// Tell this actor that it needs to update the physical properties of
        /// the physical object.
        /// </summary>
        public override void Refresh()
        {
            SetPhysicalParameters();
        }

        /// <summary>
        /// Step the vehicle actor for the next 'pTimestep' seconds.
        /// </summary>
        /// <param name="pTimestep"> The timestep length</param>
        internal void Step(float pTimestep)
        {
            // If the actor is not active, simply return and do nothing
            if (!IsActive)
            {
                return;
            }

            // Reset the known vehicle properties to get the new ones
            ForgetKnownVehicleProperties();

            // Go ahead and process both the linear and angular motion
            MoveLinear(pTimestep);
            MoveAngular(pTimestep); 

            // Limit the rotation of the vehicle if the proper flags are
            // set
            LimitRotation(pTimestep);

            // Remember the position so next step we can limit absolute 
            // movement effects
            m_lastPositionVector = VehiclePosition;

            // If we forced the changing of some vehicle parameters, update
            // the values and push them to the physics engine
            PushKnownChanged();
        }

        /// <summary>
        /// Pulls the physical properties of the physical object so that this
        /// actor updates the object correctly.
        /// </summary>
        private void SetPhysicalParameters()
        {
            if (IsActive)
            {
                // Remember the mass so we don't have to fetch it every step
                m_vehicleMass = PhysicsObject.Mass;

                // Update the friction and restitution of the physics object
                PhysicsObject.Friction = PhysicsScene.UserConfig.VehicleFriction;
                PhysicsObject.Restitution = PhysicsScene.UserConfig.VehicleRestitution;
                    
                // Set the angular damping of the physics object as the
                // default configured value
                PhysicsScene.PhysX.SetAngularDamping(PhysicsObject.LocalID,
                    PhysicsScene.UserConfig.VehicleAngularDamping);

                // Set up a joint to help constrain the physics object be constrained
                // on the axes of the linear factor (applying only to linear motion),
                // as well as the angular factor (applying only to the angular motion)
                PhysicsObject.LockMotion(
                    PhysicsScene.UserConfig.VehicleLinearFactor,
                    PhysicsScene.UserConfig.VehicleAngularFactor);
            }
        }

        /// <summary>
        /// Add the pre step and post step action from the physics scene. 
        /// </summary>
        private void RegisterForSceneEvents()
        {
            // If we are currently not registered for scene events
            if (!m_hasRegisteredForSceneEvents)
            {
                // Add a timestep action that should take place before and after
                // the physics simulation step that aids in simulating vehicles
                PhysicsScene.BeforeStep += this.Step;
                PhysicsScene.AfterStep += this.PostStep;
                
                // Mark that we have registered for physics scene events
                m_hasRegisteredForSceneEvents = true;
            }
        }

        /// <summary>
        /// Post step action for the scene.
        /// </summary>
        /// <param name="pTimestep"> The timestep duration. </param>
        internal void PostStep(float pTimestep)
        {
            if (!IsActive)
            {
                return;
            }
        }

        /// <summary>
        /// Remove the pre step and post step action from the physics scene. 
        /// </summary>
        private void UnregisterForSceneEvents()
        {
            if (m_hasRegisteredForSceneEvents)
            {
                // Remove the timestep action that was taking place before and
                // after the physics simulation step that aided in simulating 
                // vehicles
                PhysicsScene.BeforeStep -= this.Step;
                PhysicsScene.AfterStep -= this.PostStep;
                
                // Mark that we have de-registered for physics scene events
                m_hasRegisteredForSceneEvents = false;
            }
        }

        /// <summary>
        /// Refreshes the physical parameters of the vehicle actor.
        /// </summary>
        public override void RemoveDependencies()
        {
            Refresh();
        }

        /// <summary>
        /// Dispose and get rid of this actor vehicle.
        /// </summary>
        public override void Dispose()
        {
            // Unregister from the physics scene events, set the type to none
            // enabled to false
            UnregisterForSceneEvents();
            Type = Vehicle.TYPE_NONE;
            Enabled = false;
        }

        /// <summary>
        /// Reset flags that mark if properties are known or not.
        /// </summary>
        public void ForgetKnownVehicleProperties()
        {
            // Reset the flags to represent that no known changes
            // or updates have occurred
            m_knownHas = 0;
            m_knownChanged = 0;
        }

        /// <summary>
        /// Push the known or updated properties to the physics scene.
        /// </summary>
        public void PushKnownChanged()
        {
            if (m_knownChanged != 0)
            {
                // If there has been a change made to the position, reflect it
                // onto the physics object
                if ((m_knownChanged & m_knownChangedPosition) != 0)
                {
                    PhysicsObject.Position = m_knownPosition;
                }

                // If there has been a change made to the orientation, reflect it
                // onto the physics object by updating its parameter
                if ((m_knownChanged & m_knownChangedOrientation) != 0)
                {
                    PhysicsObject.Orientation = m_knownOrientation;
                }

                // If there has been a change made to the velocity, reflect it
                // onto the physics object by updating its parameter
                if ((m_knownChanged & m_knownChangedVelocity) != 0)
                {
                    PhysicsObject.Velocity = m_knownVelocity;
                }

                // If there has been a change made to the force, reflect it
                // onto the physics object by applying it as a force
                if ((m_knownChanged & m_knownChangedForce) != 0)
                {
                    PhysicsObject.Force = m_knownForce;
                }

                // If there has been a change made to the force impulse,
                // then reflect it by adding the force impulse to the 
                // physics object
                if ((m_knownChanged & m_knownChangedForceImpulse) != 0)
                {
                    PhysicsScene.PhysX.AddForceImpulse(PhysicsObject.LocalID, m_knownForceImpulse);
                }

                // If there has been a change made to the rotational velocity,
                // then reflect it onto the physics object
                if ((m_knownChanged & m_knownChangedRotationalVelocity) != 0)
                {
                    PhysicsObject.RotationalVelocity = m_knownRotationalVelocity;
                }

                // If there has been a change made to the rotational impulse,
                // apply the torque impulse onto the physics object
                if ((m_knownChanged & m_knownChangedRotationalImpulse) != 0)
                {
                    PhysicsScene.PhysX.AddTorqueImpulse(PhysicsObject.LocalID, m_knownRotationalForce);
                }

                // If there has been a change made to the rotational force, reflect it
                // onto the physics object
                if ((m_knownChanged & m_knownChangedRotationalForce) != 0)
                {
                    PhysicsScene.PhysX.AddTorque(PhysicsObject.LocalID, m_knownRotationalForce);
                }
            }

            // Mark as no known changes
            m_knownChanged = 0;
        }

        /// <summary>
        /// Allows vehicles to be oriented upward constantly.
        /// </summary>
        public void ComputeAngularVerticalAttraction()
        {
            Vector3 vehicleUpAxis;
            Vector3 predictedUp; 
            Vector3 torqueVector; 
            Vector3 forwardAxis;
            Vector3 verticalContribution; 
            Vector3 currentEulerW;
            Vector3 differenceAxisW;
            Vector3 vertContributionW; 
            Vector3 origRotVelW;
            Vector3 unscaledContribVerticalErrorV;
            Quaternion justZOrientation;
            Quaternion correctionRotationW;
            float attractionSpeed;
            double differenceAngle;

            // If angular vertical attraction is enabled and the timescale
            // is less than the cutoff of the vertical attractor
            if (PhysicsScene.UserConfig.VehicleEnableAngularVerticalAttraction
                && m_verticalAttractorTimescale < m_verticalAttractorCutoff)
            {
                vehicleUpAxis = Vector3.UnitZ * VehicleFrameOrientation;

                // Dependent on the currently configured selected algorithim, we
                // will do one of a few algorithims
                switch(PhysicsScene.UserConfig.VehicleAngularVerticalAttractionAlgorithm)
                {
                    case 0:
                    {
                        // Flipping what was originally a timescale into a speed variable
                        // and then multiplying it by 2 since we are only computing half
                        // the diftance between the angles
                        attractionSpeed = (1 / m_verticalAttractorTimescale) * 2.0f;

                        // Make a prediction of where the up axis will be when this is applied
                        predictedUp = vehicleUpAxis * Quaternion.CreateFromAxisAngle(VehicleRotationalVelocity, 0f);

                        // This is only half the distance to the target so it
                        // will take 2 seconds to complete the turn
                        torqueVector = Vector3.Cross(predictedUp, Vector3.UnitZ);

                        // If the roll only flag is enabled, we only want the
                        // torque to roll forward, so we project the torque
                        // vector to the forward axis 
                        if ((m_flags & VehicleFlag.LIMIT_ROLL_ONLY) != 0)
                        {
                            forwardAxis = Vector3.UnitX * VehicleFrameOrientation;
                            torqueVector = ProjectVector(torqueVector, forwardAxis);
                        }

                        // Scale the vector by our timescale since it is an acceleration it is r/s^2
                        // or radians a timescale squared
                        verticalContribution = torqueVector * attractionSpeed * attractionSpeed;
    
                        // Add all the vertical contribution to the rotational velocity
                        // of the vehicle
                        VehicleRotationalVelocity += verticalContribution;
                        break;
                    }
                    case 1:
                    {
                        currentEulerW = Vector3.Zero;

                        // Extract the euler angles out of the vehicle frame orientation
                        VehicleFrameOrientation.GetEulerAngles(out currentEulerW.X, 
                            out currentEulerW.Y, out currentEulerW.Z);

                        // Get the z orientation as a quaternion from the axis angle
                        justZOrientation = Quaternion.CreateFromAxisAngle(
                            Vector3.UnitZ, currentEulerW.Z);

                        // Create the axis that is perpendicular to the up vector and the
                        // rotated up vector
                        differenceAxisW = Vector3.Cross(Vector3.UnitZ * 
                            justZOrientation, Vector3.UnitZ * VehicleFrameOrientation);

                        // Compute the angle between these two vectors, which will end up being
                        // the angle to rotate, and difference axisW is the plane to rate in
                        // to get the vehicle verticle
                        differenceAngle = Math.Acos((double)Vector3.Dot(
                            Vector3.UnitZ, Vector3.Normalize(Vector3.UnitZ
                            * VehicleFrameOrientation)));
                        
                        // Create the quaternion representing the correction angle
                        correctionRotationW = Quaternion.CreateFromAxisAngle(
                            differenceAxisW, (float)differenceAngle);

                        vertContributionW = Vector3.Zero;

                        // Get the euler angles of the correction rotation to the vertical
                        // contribution vector
                        correctionRotationW.GetEulerAngles(out vertContributionW.X, 
                            out vertContributionW.Y, out vertContributionW.Z);

                        // Invert the vector to be facing the opposite way
                        vertContributionW *= -1.0f;
                        vertContributionW /= m_verticalAttractorTimescale;

                        // Add the finalized vertical contribution vector
                        // to the vehicle rotational velocity
                        VehicleRotationalVelocity += vertContributionW;

                        break;
                    }
                    default:
                    {
                        break;
                    }
                }

            }
        }

        /// <summary>
        /// Step the linear motion of this vehicle for the given timestep.
        /// </summary>
        /// <param name="pTimestep"> The physics timestep </param>
        private void MoveLinear(float pTimestep)
        {
            float newVelocityLengthSq;

            ComputeLinearVelocity(pTimestep);

            ComputeLinearDeflection(pTimestep);

            ComputeLinearTerrainHeightCorrection(pTimestep);

            ComputeLinearHover(pTimestep);

            ComputeLinearBlockingEndPoint(pTimestep);

            ComputeLinearMotorUp(pTimestep);

            // If not changing some axis, reduce out velocity
            if ((m_flags & (VehicleFlag.NO_X | VehicleFlag.NO_Y | VehicleFlag.NO_Z)) != 0)
            {
                Vector3 vel = VehicleVelocity;
                // If the vehicle is flagged to have no x movement
                // then set the x axis on the velocity to zero
                if ((m_flags & (VehicleFlag.NO_X)) != 0)
                {
                    vel.X = 0;
                }

                // If the vehicle is flagged to have no y movement
                // then set the y axis on the velocity to zero
                if ((m_flags & (VehicleFlag.NO_Y)) != 0)
                {
                    vel.Y = 0;
                }

                // If the vehicle is flagged to have no z movement
                // then set the z axis on the velocity to zero
                if ((m_flags & (VehicleFlag.NO_Z)) != 0)
                {
                    vel.Z = 0;
                }

                VehicleVelocity = vel;
            }

            newVelocityLengthSq = VehicleVelocity.LengthSquared();

            // If the new velocity's squared length is greater than the max linear
            // velocity squared configuration value
            if (newVelocityLengthSq > PhysicsScene.UserConfig.VehicleMaxLinearVelocitySquared)
            {
                Vector3 orig = VehicleVelocity; // DEBUG

                // Divide the velocity by it's length and multiply it
                // by the max linear velocity
                VehicleVelocity /= VehicleVelocity.Length();
                VehicleVelocity *= PhysicsScene.UserConfig.VehicleMaxLinearVelocity;

                m_log.InfoFormat("{0}: ID = {1}, Original Velocity = {2}, " +
                    "Final = {3}", LogHeader, PhysicsObject.LocalID, 
                    orig, VehicleVelocity);
            }
            // Else if the the opposite is true, the squared length is too low
            else if (newVelocityLengthSq < PhysicsScene.UserConfig.VehicleMinLinearVelocitySquared)
            {
                // Otherwise, set the vehicle velocity equal to zero
                VehicleVelocity = Vector3.Zero;

                m_log.InfoFormat("{0}: ID = {1}, Velocity = 0.0f", 
                    PhysicsObject.LocalID, VehicleVelocity);
            }
        }

        /// <summary>
        /// Compute and add the linear velocity to the physics actor.
        /// </summary>
        /// <param name="pTimestep"> The physics timestep </param>
        public void ComputeLinearVelocity(float pTimestep)
        {
            Vector3 origVelW;
            Vector3 currentVelV;
            Vector3 linearMotorCorrectionV;
            Vector3 linearMotorCorrectionW;
            Vector3 frictionFactorV; 
            Vector3 linearMotorVelocityW;

            // Assign the original velocity and current velocity vectors
            // and then go ahead and step the linear motor to obtain a
            // correctional value
            origVelW = VehicleVelocity;
            currentVelV = VehicleForwardVelocity;
            linearMotorCorrectionV = m_linearMotor.Step(pTimestep, currentVelV);

            // Calculate and factor the friction factor into the linear motor
            // correction vector
            frictionFactorV = ComputeFrictionFactor(m_linearFrictionTimescale, 
                pTimestep);
            linearMotorCorrectionV -= (currentVelV * frictionFactorV);
            
            // Compute the overall velocity of the linear motor
            linearMotorVelocityW = linearMotorCorrectionV * 
                VehicleFrameOrientation;

            // If we are a ground vehicle, don't add any upward Z movement
            if ((m_flags & VehicleFlag.LIMIT_MOTOR_UP) != 0)
            {
                if (linearMotorVelocityW.Z > 0.0f)
                {
                    linearMotorVelocityW.Z = 0.0f;
                }
            }

            // Finally add the correction to the velocity to make it faster/slower
            VehicleVelocity += linearMotorVelocityW;
        }

        /// <summary>
        /// Compute and add the linear deflecion factor to the physics actor.
        /// </summary>
        public void ComputeLinearDeflection(float pTimestep)
        {
            Vector3 linearDeflectionV;
            Vector3 velocity;
            Vector3 linearDeflectionW;

            // Setup the linear deflection and velocity vectors
            linearDeflectionV = Vector3.Zero;
            velocity = VehicleForwardVelocity;

            if (PhysicsScene.UserConfig.VehicleEnableLinearDeflection)
            {
                // Velocity in Y and Z dimensions is movement to the side or turning
                // Compute deflection factor from the to the side and rotational velocity
                linearDeflectionV.Y = SortedClampInRange(0.0f, 
                    (velocity.Y * m_linearDeflectionEfficiency)
                    / m_linearDeflectionTimescale, velocity.Y);
                linearDeflectionV.Z = SortedClampInRange(0.0f, 
                    (velocity.Z * m_linearDeflectionEfficiency) 
                    / m_linearDeflectionTimescale, velocity.Z);


                // Velocity to the side and around is corrected and moved into 
                // the forward direction
                linearDeflectionV.X += Math.Abs(linearDeflectionV.Y);
                linearDeflectionV.X += Math.Abs(linearDeflectionV.Z);

                // Scale the deflection to the fractional simulation time
                linearDeflectionV *= pTimestep;

                // Subtract the sideways and rotational velocity deflection
                // factors while adding the correction forward
                linearDeflectionV *= new Vector3(1.0f, -1.0f, -1.0f);

                // Correction is vehicle relative, convert to world coordinates
                linearDeflectionW = linearDeflectionV * VehicleFrameOrientation;
            
                // Optionally, if not colliding, don't effect world downward 
                // velocity
                if (PhysicsScene.UserConfig.VehicleLinearDeflectionNotCollidingNoZ
                    && !PhysicsObject.IsColliding)
                {
                    linearDeflectionW.Z = 0.0f;
                }

                // Add the linear deflection value to the vehicle velocity
                VehicleVelocity += linearDeflectionW;
            }
        }

        /// <summary>
        /// Allow the vehicle to become unstuck if it is currently below the
        /// terrain.
        /// </summary>
        /// <param name="pTimestep"> The physics timestep </param>
        public void ComputeLinearTerrainHeightCorrection(float pTimestep)
        {
            // If below the terrain, move us above the ground a little
            if (VehiclePosition.Z < GetTerrainHeight(VehiclePosition))
            {
                // Force position because applying a force won't get the vehicle through
                // the terrain
                Vector3 newPosition = VehiclePosition;
                newPosition.Z = GetTerrainHeight(VehiclePosition) + 1.0f;

                // Update the vehicle position with the new position
                VehiclePosition = newPosition;
            }
        }

        /// <summary>
        /// Compute and add the linear hover to the vehicle.
        /// </summary>
        /// <param name="pTimestep"> The physics timestep. </param>
        public void ComputeLinearHover(float pTimestep)
        {
            Vector3 position;
            Vector3 velocity;
            float verticalError;
            float verticalCorrection;

            if ((m_flags & (VehicleFlag.HOVER_WATER_ONLY 
                | VehicleFlag.HOVER_TERRAIN_ONLY 
                | VehicleFlag.HOVER_GLOBAL_HEIGHT)) != 0 
                && (m_hoverHeight > 0.0f) && (m_hoverTimescale < 300.0f))            
            {
                // If the vehicle only hovers over water, set the target hover height
                // to the water level plus the hover height
                if ((m_flags & VehicleFlag.HOVER_WATER_ONLY) != 0)
                {
                    m_hoverTargetHeight = GetWaterLevel(VehiclePosition) + m_hoverHeight;
                }

                // If the vehicle hovers only over terrain, set the target hover height
                // to the terrain height plus the hover height
                if ((m_flags & VehicleFlag.HOVER_TERRAIN_ONLY) != 0)
                {
                    m_hoverTargetHeight = GetTerrainHeight(VehiclePosition) + m_hoverHeight;
                }

                // If the vehicle hovers anywhere, set the target hover height to
                // the hover height.
                if ((m_flags & VehicleFlag.HOVER_GLOBAL_HEIGHT) != 0)
                {
                    m_hoverTargetHeight = m_hoverHeight;
                }

                // If the vehicle hovers up only, keep its current height if it
                // is higher than the hover target height
                if ((m_flags & VehicleFlag.HOVER_UP_ONLY) != 0)
                {
                    if (VehiclePosition.Z > m_hoverTargetHeight)
                    {
                        m_hoverTargetHeight = VehiclePosition.Z;
                    }
                }

                // If the vehicle is hover locked, then go ahead and lock it at the 
                // hover target height
                if ((m_flags & VehicleFlag.LOCK_HOVER_HEIGHT) != 0)
                {
                    if (Math.Abs(VehiclePosition.Z - m_hoverTargetHeight) > 0.2f)
                    {
                        position = VehiclePosition;
                        position.Z = m_hoverTargetHeight;
                        VehiclePosition = position;
                    }
                }
                else 
                {
                    // Get the position of the vehicle
                    position = VehiclePosition;
                    
                    // Calculate the vertical error from the hover target
                    // height and the z axis
                    verticalError = m_hoverTargetHeight - position.Z;

                    // Calculate the vertical correction of the the hover at
                    // the current timescale
                    verticalCorrection = verticalError / m_hoverTimescale;
                    verticalCorrection *= m_hoverEfficiency;

                    // Add the vertical correction to the z axis
                    // and update the vehicle position
                    position.Z += verticalCorrection;
                    VehiclePosition = position;

                    // Reset the z axis velocity of the vehicle
                    velocity = VehicleVelocity;
                    velocity.Z = 0.0f;
                    VehicleVelocity = velocity;
                }
            }
        }

        /// <summary>
        /// Compute and add the linear up motor if it is appropriate for this
        /// physics vehicle.
        /// </summary>
        /// <param name="pTimestep">The physics timestep</param>
        public void ComputeLinearMotorUp(float pTimestep)
        {
            float upVelocity;

            // If the vehicle is flagged to limit the upward motor
            if ((m_flags & (VehicleFlag.LIMIT_MOTOR_UP)) != 0)
            {
                // If we are going up and not colliding, the vehicle is in the
                // air, fix that by pushing down
                if (!PhysicsObject.IsColliding && VehicleVelocity.Z > 0.1f)
                {
                    // Get rid of any of the velocity vector that is pushing up
                    upVelocity = VehicleVelocity.Z;
                    VehicleVelocity += new Vector3(0.0f, 0.0f, -upVelocity);
                }
            }
        }

        /// <summary>
        /// Compute the linear blocking end point of this vehicle
        /// actor, same as in bullet sim.
        /// </summary>
        /// <param name="pTimestep"> The timestep for the calculation </param>
        public bool ComputeLinearBlockingEndPoint(float pTimestep)
        {
            Vector3 position;
            Vector3 positionChange;
            bool changed = false;

            // Get the current position, and the difference in the
            // last position to this current position
            position = VehiclePosition;
            positionChange = position - m_lastPositionVector;

            // If the blocking endpoint vector is not all zero
            if (m_blockingEndPoint != Vector3.Zero)
            {
                // If the axes of our position is greater than
                // or equal to one less of the blocking endpoint's 
                // axes, subtract the difference in our last position
                // axes + 1f for each of the axes, as well as marked changed
                // as true
                if (position.X >= (m_blockingEndPoint.X - 1.0f))
                {
                    position.X -= positionChange.X + 1.0f;
                    changed = true;
                }
                if (position.Y >= (m_blockingEndPoint.Y - 1.0f))
                {
                    position.Y -= positionChange.Y + 1.0f;
                    changed = true;
                }
                if (position.Z >= (m_blockingEndPoint.Z - 1.0f))
                {
                    position.Z -= positionChange.Z + 1.0f;
                    changed = true;
                }

                // If the x or the y position is under zero
                // add the position change + 1f
                if (position.X <= 0)
                {
                    position.X += positionChange.X + 1.0f;
                    changed = true;
                }
                if (position.Y <= 0)
                {
                    position.Y += positionChange.Y + 1.0f;
                    changed = true;
                }

                // If we have changed the position, go ahead
                // and update the vehicle position
                if (changed)
                {
                    VehiclePosition = position;
                }
            }

            return changed;
        }

        /// <summary>
        /// Compute and handle the angular motion factoring in
        /// the friction factor.
        /// </summary>
        /// <param name="pTimestep">The physics timestep </param>
        public void ComputeAngularTurning(float pTimestep)
        {
            Vector3 originalVehicleRotationalVelocity;
            Vector3 currentAngularVelocity;
            Vector3 angularMotorContribution;
            Vector3 frictionFactor;

            // Set the original vehicle rotational velocity vector to the
            // vehicle rotational velocity vector
            originalVehicleRotationalVelocity = VehicleRotationalVelocity;

            // Calculate the current angular velocity from the rotational
            // velocity multiplied by the inverse of the orientation
            currentAngularVelocity = VehicleRotationalVelocity * 
                Quaternion.Inverse(VehicleFrameOrientation);

            // Get the current angular motor contribution from stepping the
            // angular motor
            angularMotorContribution = m_angularMotor.Step(pTimestep, 
                currentAngularVelocity);

            // Calculate the friction factor to begin reducing the vehicle
            // velocity
            frictionFactor = ComputeFrictionFactor(m_angularFrictionTimescale, 
                pTimestep);

            // Reduce the angular motor contribution by the current angular
            // velocity adjusted by the friction factor vector
            angularMotorContribution -= (currentAngularVelocity * frictionFactor);

            // Finally, add the updated angular motor contribution after we
            // have applied the friciton to lessen the vector
            VehicleRotationalVelocity += (angularMotorContribution * VehicleFrameOrientation);
        }

        /// <summary>
        /// Angular deflection is to correct the direction the vehicle is
        /// pointing to be the direction it should want to be pointing.
        /// </summary>
        public void ComputeAngularDeflection()
        {
            Vector3 deflectContributionV;
            Vector3 movingDirection; 
            Vector3 pointingDirection;
            Vector3 predictedPointingDirection;
            Vector3 deflectionError;
            Vector3 deflectContribution;

            if (PhysicsScene.UserConfig.VehicleEnableAngularDeflection 
                && m_angularDeflectionEfficiency != 0 
                && VehicleForwardSpeed > 0.2f)
            {
                deflectContributionV = Vector3.Zero;

                // Get the direction that the vehicle is moving
                movingDirection = VehicleVelocity;
                movingDirection.Normalize();

                // If the vehicle is going backward, it is still pointing forward
                movingDirection *= Math.Sign(VehicleForwardSpeed);

                // The direction the vehicle is pointing
                pointingDirection = Vector3.UnitX * VehicleFrameOrientation;

                // Predict where the Vehicle will be pointing after AngularVelocity change is applied. This will keep
                // from overshooting and allow this correction to merge with the Vertical Attraction peacefully
                predictedPointingDirection = pointingDirection * 
                    Quaternion.CreateFromAxisAngle(VehicleRotationalVelocity, 0.0f);
                predictedPointingDirection.Normalize();

                // The difference between what is and what should be
                deflectionError = Vector3.Cross(movingDirection, predictedPointingDirection);

                // If the deflection error is out of the range of -4Pi to 4Pi, set it to zero
                // on its respected axis
                if (Math.Abs(deflectionError.X) > PIOverFour) deflectionError.X = 0f;
                if (Math.Abs(deflectionError.Y) > PIOverFour) deflectionError.Y = 0f;
                if (Math.Abs(deflectionError.Z) > PIOverFour) deflectionError.Z = 0f;

                // Scale the correction by recovery timescale and efficiency
                deflectContribution = (-deflectionError) * ClampInRange(0.0f, 
                    m_angularDeflectionEfficiency/m_angularDeflectionTimescale, 1.0f);

                // Add the deflection vector to the angular motion of the vehicle
                VehicleRotationalVelocity += deflectContributionV;
            }

        }

        /// <summary>
        /// Calculate the angular change to rotate the vehicle around the Z axis
        /// when the vehicle is tipped around the X axis.
        /// </summary>
        public void ComputeAngularBanking()
        {
            Vector3 bankingContribution;
            Vector3 rollComponents;
            float yawAngle;
            float mixedYawAngle;

            // As long as angular banking is enabled, that the efficiency
            // is not zero, and the attractor timescale is less than its
            // cutoff
            if (PhysicsScene.UserConfig.VehicleEnableAngularBanking && m_bankingEfficiency != 0
                 && m_verticalAttractorTimescale < m_verticalAttractorCutoff)
            {
                bankingContribution = Vector3.Zero;

                // Rotate a UnitZ vector (pointing up) to how the vehicle is oriented
                // As the vehicle rolls to the right or left, the Y value will increase from
                // zero (straight up) to 1 or -1 (full tilt right  or left)
                rollComponents = Vector3.UnitZ * VehicleFrameOrientation;

                // Calculate the yaw value with the current roll
                yawAngle = m_angularMotorDirection.X * m_bankingEfficiency;

                // Calculate the error of the yaw angle
                mixedYawAngle = (yawAngle * (1.0f - m_bankingMix)) + ((yawAngle * m_bankingMix) * VehicleForwardSpeed);
                mixedYawAngle = ClampInRange(-FourPI, mixedYawAngle, FourPI);

                // Start to build up the banking contribution vector
                bankingContribution.Z = -mixedYawAngle;
                bankingContribution /= m_bankingTimescale * PhysicsScene.UserConfig.VehicleAngularBankingTimescaleFudge;

                // Add the banking contribution vector to the vehicles rotational velocity
                VehicleRotationalVelocity += bankingContribution;
            }
        }

        /// <summary>
        /// Given a friction vector (reduction in seconds) and a timestep, then
        /// return the computated result of the friction factor from 0f to 1f.
        /// </summary>
        /// <param name="friction"> The friction vector to calculate from </param>
        /// <param name="timestep"> The physics timestep at which this is being applied </param>
        private Vector3 ComputeFrictionFactor(Vector3 friction, float timestep)
        {
            Vector3 frictionFactor = Vector3.Zero;

            if (friction != PxMotor.InfiniteVector)
            {
                // Individual friction components can be 'infinite' so compute each separately
                frictionFactor.X = (friction.X == PxMotor.Infinite) ? 0f : (1f / friction.X);
                frictionFactor.Y = (friction.Y == PxMotor.Infinite) ? 0f : (1f / friction.Y);
                frictionFactor.Z = (friction.Z == PxMotor.Infinite) ? 0f : (1f / friction.Z);
                frictionFactor *= timestep;
            }

            return frictionFactor;
        }

        /// <summary>
        /// Limit the rotation of vehicle according to the
        /// roll reference frame.
        /// </summary>
        /// <param name="timestep">The physics timestep</param>
        public void LimitRotation(float timeStep)
        {
            Quaternion rotQ;
            Quaternion rotation;

            // Setup the quaternions to the current vehicle orientation
            rotQ = VehicleOrientation;
            rotation = rotQ;

            // If the roll reference frame is not an identity quaternion,
            // then constrain the X and Y 
            if (m_rollReferenceFrame != Quaternion.Identity)
            {
                if (rotQ.X >= m_rollReferenceFrame.X)
                {
                    rotation.X = rotQ.X - (m_rollReferenceFrame.X / 2);
                }
                if (rotQ.Y >= m_rollReferenceFrame.Y)
                {
                    rotation.Y = rotQ.Y - (m_rollReferenceFrame.Y / 2);
                }
                if (rotQ.X <= -m_rollReferenceFrame.X)
                {
                    rotation.X = rotQ.X + (m_rollReferenceFrame.X / 2);
                }
                if (rotQ.Y <= -m_rollReferenceFrame.Y)
                {
                    rotation.Y = rotQ.Y + (m_rollReferenceFrame.Y / 2);
                }
            }

            if ((m_flags & VehicleFlag.LOCK_ROTATION) != 0)
            {
                rotation.X = 0;
                rotation.Y = 0;
            }

            if (rotQ != rotation)
            {
                VehicleOrientation = rotation;
            }
        }

        /// <summary>
        /// Gets the scalar projection of the first vector onto
        /// the second.
        /// </summary>
        /// <param name="vector1"> The vector to be projected</param>
        /// <param name="vector2"> The vector to be projected to</param>
        /// <returns>The amount of the vector is on the same axis as the unit
        /// </returns>
        private Vector3 ProjectVector(Vector3 vector1, Vector3 vector2)
        {
            // Get the dot product of the two vectors, and then 
            // return it as a scalar of the second vector 
            float vectorDot = Vector3.Dot(vector1, vector2);
            return vector2 * vectorDot;
        }

        /// <summary>
        /// Clamp the middle value, val, within the high and low float values.
        /// </summary>
        /// <param name="low">Lower value to be clamped between</param>
        /// <param name="val">The value to be clamped between the others</param>
        /// <param name="high">Higher value to be clamped between</param>
        /// <returns>A float value clamped between the low and high ranges</returns>
        private float ClampInRange(float low, float val, float high)
        {
            return Math.Max(low, Math.Min(val, high));
        }

        /// <summary>
        /// Clamp the values within the range, after first sorting them.
        /// </summary>
        /// <param name="clampA"> The first float of the range</param>
        /// <param name="value"> The float value to be clamped</param>
        /// <param name="clampB"> The last float value of the range</param>
        /// <returns>A float value clamped between the clampA and clampB ranges</returns>
        private float SortedClampInRange(float clampA, float value, float clampB)
        {
            float temp;

            if (clampA > clampB)
            {
                temp = clampA;
                clampA = clampB;
                clampB = temp;
            }

            return ClampInRange(clampA, value, clampB);
        }

        /// <summary>
        /// Get the terrain height at the specified position within the scene.
        /// </summary>
        /// <param name="position"> The position being queried </param>
        /// <returns>The terrain height at the position</returns>
        private float GetTerrainHeight(Vector3 position)
        {
            // If we do not know the terrain height currently, obtain it
            // from the physics scene, and save its value
            if ((m_knownHas & m_knownChangedTerrainHeight) == 0 
                || position != m_lastRememberedHeightPos)
            {
                m_lastRememberedHeightPos = position;
                m_knownTerrainHeight = PhysicsScene.WaterHeight;
                m_knownHas |= m_knownChangedTerrainHeight;
            }

            // Return the known terrain height
            return m_knownTerrainHeight;
        }

        /// <summary>
        /// Get the water level at the specified position within the scene.
        /// </summary>
        /// <param name="position"> The position being queried </param>
        private float GetWaterLevel(Vector3 position)
        {
            // If we do not know the water level currently, obtain it 
            // from the physics scene, and save its value
            if ((m_knownHas & m_knownChangedWaterLevel) == 0
                || position != m_lastRememberedWaterPos)
            {
                m_lastRememberedWaterPos = position;
                m_knownWaterLevel = PhysicsScene.WaterHeight;
                m_knownHas |= m_knownChangedWaterLevel;
            }

            // Return the known water level
            return m_knownWaterLevel;
        }

        /// <summary>
        /// Add a force to the vehicle.
        /// </summary>
        /// <param name="force"> The force to be added </param>
        private void VehicleAddForce(Vector3 force)
        {
            // If we do not know the force change, then zero out the
            // known force
            if ((m_knownHas & m_knownChangedForce) == 0)
            {
                m_knownForce = Vector3.Zero;
                m_knownHas |= m_knownChangedForce;
            }

            // Add the force to the known forces
            m_knownForce += force;
            m_knownChanged |= m_knownChangedForce;
        }

        /// <summary>
        /// Add a force impulse to the vehicle.
        /// </summary>
        /// <param name="force"> The impulse force to be applied </param>
        private void VehicleAddForceImpulse(Vector3 impulse)
        {
            // If the force impulse has changed, first set it to zero
            // and thereafter add the force impulse 
            if ((m_knownHas & m_knownChangedForceImpulse) == 0)
            {
                m_knownForceImpulse = Vector3.Zero;
                m_knownHas |= m_knownChangedForceImpulse;
            }

            // Add the force impulse and mark it as changed
            m_knownForceImpulse += impulse;
            m_knownChanged |= m_knownChangedForceImpulse;
        }

        /// <summary>
        /// Add a rotational (angular) force to the vehicle.
        /// </summary>
        /// <param name="force"> The angular force to be applied </param>
        private void VehicleAddAngularForce(Vector3 force)
        {
            // If the rotational force has changed, first set it to zero
            // and thereafter add the angular force
            if ((m_knownHas & m_knownChangedRotationalForce) == 0)
            {
                m_knownRotationalForce = Vector3.Zero;
                m_knownHas |= m_knownChangedRotationalForce;
            }

            // Add the rotational force and mark it as changed
            m_knownRotationalForce += force;
            m_knownChanged |= m_knownChangedRotationalForce;
        }

        /// <summary>
        /// Add a rotational (angular) impulse to the vehicle.
        /// </summary>
        /// <param name="force"> The angular force to be applied </param>
        private void VehicleAddRotationalImpulse(Vector3 force)
        {
            // If the rotational impulse has changed, first set it to zero
            // and thereafter add the angular impulse
            if ((m_knownHas & m_knownChangedRotationalImpulse) == 0)
            {
                m_knownRotationalImpulse = Vector3.Zero;
                m_knownHas |= m_knownChangedRotationalImpulse;
            }

            // Add the angular impulse, and mark it as changed
            m_knownRotationalImpulse += force;
            m_knownChanged |= m_knownChangedRotationalImpulse;
        }

    }

}
