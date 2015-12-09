
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

using log4net;

namespace OpenSim.Region.Physics.PhysXPlugin
{
    
    /// <summary>
    /// Each physical object can have 'actors' who are pushing the object around.
    /// This can be used for hover, locking axis, making vehicles, etc.
    /// Each physical object can have multiple actors acting on it.
    ///
    /// An actor usually registers itself with physics scene events (pre-step action)
    /// and modifies the parameters on the host physical object.
    /// </summary>
    public abstract class PxActor
    {
        /// <summary>
        /// The physics scene that this physics actor will act within.
        /// </summary>
        private PxScene m_physicsScene;

        /// <summary>
        /// The physics object primitive that this physics actor will act upon. 
        /// </summary>
        private PxPhysObject m_physicsObject;

        /// <summary>
        /// The enabled property of this physics actor to determine
        /// if the actor should act upon the physical object.
        /// </summary>
        public virtual bool Enabled { get; set; }

        /// <summary>
        /// The actor name property of this physical actor to give it
        /// a name to be referenced by and to help in case of debugging.
        /// </summary>
        public string ActorName { get; private set; }

        /// <summary>
        /// The getter and setter for the physics scene property that this 
        /// physics actor will act within.
        /// </summary>
        public PxScene PhysicsScene
        {
            get 
            { 
                return m_physicsScene;
            }

            private set 
            {
                m_physicsScene = value;
            }
        }

        /// <summary>
        /// The getter and setter for the physics object that this
        /// physics actor will act upon.
        /// </summary>
        public PxPhysObject PhysicsObject
        {
            get 
            {
                return m_physicsObject;
            }

            private set 
            {
                m_physicsObject = value;
            }
        }

        /// <summary>
        /// The logger for this physics actor in case of debug.
        /// </summary>
        internal static readonly ILog m_log = LogManager.GetLogger(
            System.Reflection.MethodBase.GetCurrentMethod().DeclaringType);

        /// <summary>
        /// Constructor of the PxActor.
        /// </summary>
        /// <param name="physicsScene"> The physics scene that this actor 
        /// will act within. </param>
        /// <param name="physicsObject"> The physics object that this actor 
        /// will act upon. </param>
        /// <param name="actorName"> The physics actor name to identify 
        /// the intent. <param>
        public PxActor(PxScene physicsScene, PxPhysObject physicsObject, 
            string actorName)
        {
            PhysicsScene = physicsScene;
            PhysicsObject = physicsObject;
            ActorName = actorName;
            Enabled = true;
        }

        /// <summary>
        /// The getter for the isActive attribute detailing if it
        /// is actively updating or applying forces to the physics
        /// object that this physics actor acts upon.
        /// </summary>
        public virtual bool isActive
        {
            get 
            { 
                return Enabled; 
            }
        }

        /// <summary>
        /// Turn the physics actor on and off, only utilized by the 
        /// PxActorCollection to enable/disable all physics actors.
        /// </summary>
        /// <param name="setEnabled"> Boolean of what to set enabled to. </param>
        public void SetEnabled(bool setEnabled)
        {
            Enabled = setEnabled;
        }

        /// <summary>
        /// Deactivate if not already deactivated, and dispose of the allocated
        /// memory and resources used for the object instance.
        /// </summary>
        public abstract void Dispose();

        /// <summary>
        /// Refresh the physics actor and update if it should be enabled or
        /// disabled and act accordingly.
        /// </summary>
        public abstract void Refresh();

        /// <summary>
        /// The object's physical representation is being rebuilt so re-build 
        /// any physical dependencies (constraints, ...). Register a prestep
        /// action to restore physical requirements before the next simulation
        /// step.
        /// </summary>
        public abstract void RemoveDependencies();

    }
}