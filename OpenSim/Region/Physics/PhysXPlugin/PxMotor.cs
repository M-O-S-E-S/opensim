
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
using OpenMetaverse;
using OpenSim.Framework;

namespace OpenSim.Region.Physics.PhysXPlugin
{
    public abstract class PxMotor
    {
        /// <summary>
        /// A float representation of an infinite value equivelent.
        /// </summary>
        public const float Infinite = Single.MaxValue;

        /// <summary>
        /// A vector representation of an infinite vector equivelent.
        /// </summary>
        public readonly static Vector3 InfiniteVector = new Vector3(
            PxMotor.Infinite, PxMotor.Infinite, PxMotor.Infinite);

        /// <summary>
        /// The boolean value representing if this PxMotor is enabled
        /// and acting within the scene.
        /// </summary>
        private bool m_enabled;

        /// <summary>
        /// Constructor for the PxMotor class.
        /// </summary>
        public PxMotor()
        {
            // By default the base PxMotor will be enabled
            Enabled = true;
        }
        
        /// <summary>
        /// Getter and setter for the boolean value representing if the
        /// PxMotor is enabled and acting within the scene.
        /// </summary>
        public virtual bool Enabled 
        {
            get 
            {
                return m_enabled;
            }

            private set
            {
                m_enabled = value;
            }
        }

        /// <summary>
        /// Reset and zeros out the current and target values of this motor.
        /// </summary>
        public abstract void Reset();

        /// <summary>
        /// Reset and zero out both the target value of this motor, as
        /// well as the current value of this motor.
        /// </summary>
        public abstract void Zero();
    
    }
}
