
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

using System;
using System.Collections.Generic;
using System.Text;

using OpenSim.Region.Physics.Manager;

using OpenMetaverse;
using OpenSim.Framework;


namespace OpenSim.Region.Physics.PhysXPlugin
{
    public class PxVMotor : PxMotor
    {
        /// <summary>
        /// The timescale value of this PxVMotor instance, which indicates
        /// how much time a step is.
        /// </summary>
        private float m_timeScale;

        /// <summary>
        /// The vector3 current value of this PxVMotor instance, which 
        /// indicates the current value of the vector3 we are 
        /// stepping/iterating.
        /// </summary>
        private Vector3 m_currentValue;

        /// <summary>
        /// The vector3 target value of this PxVMotor instance, which 
        /// indicates the upper limit of stepping/iterating the current 
        /// value to. 
        /// </summary>
        private Vector3 m_targetValue;

        /// <summary>
        /// The vector3 value that represents the difference in the target 
        /// value and the current value at the current timestep.
        /// </summary>
        private Vector3 m_lastError;

        /// <summary>
        /// The float value that represents the efficiency in achieving the
        /// target value from the current value.
        /// </summary>
        private float m_efficiency;

        /// <summary>
        /// Getter and setter for the timescale value of this PxVMotor.
        /// </summary>
        public float TimeScale
        {
            get
            {
                return m_timeScale;
            }

            set
            {
                m_timeScale = value;
            }
        }

        /// <summary>
        /// Getter and setter for the vector3 value that this PxMotor is
        /// currently at the current moment in time.
        /// </summary>
        public Vector3 CurrentValue 
        {
            get 
            {
                return m_currentValue;
            }

            protected set
            {
                m_currentValue = value;
            }
        }

        /// <summary>
        /// Getter and setter for the vector3 value that this PxVMotor is
        /// to step up and reach a step (timestep) at a time.
        /// </summary>
        public Vector3 TargetValue
        {
            get 
            { 
                return m_targetValue;
            }

            protected set
            {
                m_targetValue =  value;
            }
        }

        /// <summary>
        /// Getter and setter for the vector3 value that represents the
        /// difference in the target value and the current value.
        /// </summary>
        public Vector3 LastError
        {
            get
            {
                return m_lastError;
            }

            protected set
            {
                m_lastError = value;
            }
        }

        /// <summary>
        /// Getter and setter for the float value that represents the
        /// efficiency or step size of this PxMotor.
        /// </summary>
        public float Efficiency
        {
            get
            {
                return m_efficiency;
            }

            set
            {
                m_efficiency = value;
            }
        }

        /// <summary>
        /// The rate of decaying the timescale to slow down the stepping
        /// of the current value up to the target value.
        /// </summary>
        public virtual float TargetValueDecayTimeScale { get; set; }
        
        /// <summary>
        /// The threshold of the tolerance in calculating the error.
        /// </summary>
        public virtual float ErrorZeroThreshold { get; set; }

        /// <summary>
        /// Determines and returns if the last error calculated is
        /// zero, because of floats.
        /// </summary>
        public virtual bool ErrorIsZero()
        {
            // Return the result of checking if the last error was zero
            return ErrorIsZero(LastError);
        }

        /// <summary>
        /// Determines and returns if the float passed in as the assumed error
        /// is zero or not.
        /// </summary>
        public virtual bool ErrorIsZero(Vector3 err)
        {
            // Return the result of checking if the error is within our threshold
            return (err == Vector3.Zero || err.ApproxEquals(Vector3.Zero, ErrorZeroThreshold));
        }

        /// <summary>
        /// The default constructor for the vector motor.
        /// </summary>
        public PxVMotor()
        {
            // Set the timescale, decay, efficiency, and current values
            // as default
            TimeScale = TargetValueDecayTimeScale = PxMotor.Infinite;
            Efficiency = 1.0f;
            CurrentValue = TargetValue = Vector3.Zero;
            ErrorZeroThreshold = 0.001f;
        }

        /// <summary>
        /// The constructor for the PxVMotor to iterate a float to the
        /// proper target value.
        /// </summary>
        /// <param name="timeScale"> The timescale of the timestep </param>
        /// <param name="decayTimescale"> The rate at which the timestep slows </param>
        /// <param name="efficiency"> The efficiency at which it steps the 
        /// current value </param>
        public PxVMotor(float timeScale, float decayTimescale, 
            float efficiency) : base()
        {
            // Set the timescale, decay, and efficiency as given
            TimeScale = timeScale;
            TargetValueDecayTimeScale = decayTimescale;
            Efficiency = efficiency;

            // The defaults for the current and target value are zero
            // and we go ahead and set our zero threshold to 0.001f
            CurrentValue = TargetValue = Vector3.Zero;
            ErrorZeroThreshold = 0.001f;
        }

        /// <summary>
        /// Set the current value of this PxVMotor as passed in.
        /// </summary>
        /// <param name="current"> The updated current value </param>
        public void SetCurrent(Vector3 current)
        {
            // Set the CurrentValue to the given current value
            CurrentValue = current;
        }

        /// <summary>
        /// Set the target value of this PxVMotor as passed in.
        /// </summary>
        /// <param name="target"> The updated target value </param>
        public void SetTarget(Vector3 target)
        {
            // Set the TargetValue to the given target value
            TargetValue = target;
        }

        /// <summary>
        /// Reset and zero out both the target value and the current value
        /// of this PxVMotor.
        /// </summary>
        public override void Zero()
        {
            // Go ahead and set the target and current value to zero
            CurrentValue = TargetValue = Vector3.Zero;
        }


        /// <summary>
        /// Step the motor and return the resulting current value.
        /// </summary>
        /// <param name="timeStep"> The current timestep of the scene </param>
        public virtual Vector3 Step(float timeStep)
        {
            Vector3 errorValue, correctionValue; 
            float decayFactor;

            // If the motor is not enabled, we assume that we
            // have reached the target value, so simply just
            // return the target value that we have
            if (!Enabled)
            {
                return TargetValue;
            }

            // Calculate the difference in the current and target values
            errorValue = TargetValue - CurrentValue;

            correctionValue = Vector3.Zero;

            // If the calculated error value is not zero
            if (!ErrorIsZero(errorValue))
            {
                // Calculate the error correction value
                // and add it to the current value
                correctionValue = StepError(timeStep, errorValue);
                CurrentValue += correctionValue;

                // The desired value reduces to zero which also reduces the 
                // difference with current
                // If the decay timescale is not infinite, we decay 
                if (TargetValueDecayTimeScale != PxMotor.Infinite)
                {
                    decayFactor = (1.0f / TargetValueDecayTimeScale) * timeStep;
                    TargetValue *= (1f - decayFactor);
                }
            }
            else
            {
                // Difference between what we have and target is small, Motor 
                // is done
                if (TargetValue.ApproxEquals(Vector3.Zero, ErrorZeroThreshold))
                {
                    // The target can step down to nearly zero but not get 
                    // there If close to zero it is really zero
                    TargetValue = Vector3.Zero;
                }

                // If the motor is done, set the current value to the target value
                CurrentValue = TargetValue;
            }

            // Update the last error as the most recently calculated error
            LastError = errorValue;

            return correctionValue;
        }

        /// <summary>
        /// A method that will set the current value before doing the next
        /// physics step.
        /// <summary>
        /// <param name="timeStep">The current timestep of the scene</param>
        /// <param name="current">The new current value that will be used in
        /// this physics step</param>
        public virtual Vector3 Step(float timeStep, Vector3 current)
        {
            // Set the current value and then call the usual step method
            CurrentValue = current;
            return Step(timeStep);
        }

        /// <summary>
        /// Calculate and return the resulting correction in the error
        /// for the current timestep.
        /// </summary>
        /// <param name="timeStep"> The current timestep of the scene</param>
        /// <param name="error"> The difference in the current value and 
        /// target</param>
        public virtual Vector3 StepError(float timeStep, Vector3 error)
        {
            Vector3 returnCorrection, correctionAmount;

            // If this PxVMotor is not enabled, simply return 0.0f
            if (!Enabled) 
            {
                return Vector3.Zero;
            }

            // Initialize the return correction, and the correction amount
            // to a float value of zero, so that it is initialized
            returnCorrection = Vector3.Zero;
            correctionAmount = Vector3.Zero;

            // If the given error is not zero, we should calculate the
            // correction to be returned
            if (!ErrorIsZero(error))
            {
                // If the timescale is zero, or infinity, then the correction
                // amount is equal to the error * the timestep
                if (TimeScale == 0.0f || TimeScale == PxMotor.Infinite)
                {
                    correctionAmount = error * timeStep;
                }
                else
                {
                    // Otherwise, we can use the timescale so that it does not
                    // result in a divide by zero error
                    correctionAmount = error / TimeScale * timeStep;
                }

                // Set the returned correction to the calculated correction
                // amount
                returnCorrection = correctionAmount;
            }

            return returnCorrection;
        }

        /// <summary>
        /// Return a representation of the current values on the motor as
        /// a string.
        /// </summary>
        public override string ToString()
        {
            // Return a formatted string with the values existing on this motor
            return String.Format("<curr={0},targ={1},lastErr={2},decayTS={3}>",
                CurrentValue, TargetValue, LastError, 
                TargetValueDecayTimeScale);
        }

        /// <summary>
        /// Reset and zero out both the target value of this motor, as
        /// well as the current value of this motor.
        /// </summary>>
        public override void Reset()
        {
            Zero();
        }
    }
}
