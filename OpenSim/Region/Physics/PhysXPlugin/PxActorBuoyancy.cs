
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
using System.Linq;
using System.Text;

using OpenSim.Region.Physics.Manager;

using OpenMetaverse;

namespace OpenSim.Region.Physics.PhysXPlugin
{
    public class PxActorBuoyancy : PxActor
    {
        /// <summary>
        /// The float motor helps to iterate a float to a specified value
        /// with each passing time-step.
        /// </summary>
        private PxFMotor m_buoyancyMotor;

        /// <summary>
        /// A boolean representation of if the physics actor should be active
        /// and acting upon the physics object.
        /// </summary>
        public override bool isActive
        {
            get 
            { 
                return PhysicsObject.Position.Z < PhysicsScene.WaterHeight; 
            }
        }

        /// <summary>
        /// Constructor for the buoyancy actor, includes the physics scene,
        /// the physics object, and the name of this physics actor.
        /// </summary>
        /// <param name="physicsScene"> The physics scene that this actor will
        /// act within. </param>
        /// <param name="physicsObject"> The physics object that this actor will
        /// act upon. </param>
        /// <param name="actorName"> The physics actor name to identify 
        /// the intent. <param>
        public PxActorBuoyancy(PxScene physicsScene, PxPhysObject physObject,
            string actorName) : base(physicsScene, physObject, actorName)
        {
            m_buoyancyMotor = null;
        }

        /// <summary>
        /// Deactivate if not already deactivated, and dispose of the allocated
        /// memory and resources used for the object instance.
        /// </summary>
        public override void Dispose()
        {
            Enabled = false;
            DeactivateBuoyancy();
        }

        /// <summary>
        /// Refresh the physics actor and update if it should be enabled or
        /// disabled based upon the water height in the scene and the physics
        /// object height. (z axis coordinate)
        /// </summary>
        public override void Refresh()
        {
            // Here we use the isActive property to represent the state of
            // if the requirements for this PxActor are met, in this case it
            // checks that the physics object is not flying, and below the
            // static water level of the physics scene
            if (!isActive)
            {
                Enabled = false;
            }
            else
            {
                Enabled = true;
            }

            // Furthermore, if in the end this physics actor is enabled, we
            // want to go ahead and activate buoyancy, or if this physics
            // actor is not enabled, we want to go ahead and deactivate
            // the buoyancy pre-step action
            if (Enabled)
            {
                ActivateBuoyancy();
            }
            else
            {
                DeactivateBuoyancy();
            }
        }

        /// <summary>
        /// The object's physical representation is being rebuilt so re-build 
        /// any physical dependencies (constraints, ...). Register a pre-step
        /// action to restore physical requirements before the next simulation
        /// step.
        /// </summary>
        public override void RemoveDependencies()
        {
            // Nothing to do for the buoyancy since it is all software
            // at pre-step action time.
        }

        /// <summary>
        /// Activate the buoyancy of the physics actor to begin applying force
        /// upon the physical object to simulate buoyancy within the physics
        /// scene.
        /// </summary>
        private void ActivateBuoyancy()
        {
            // If the buoyancy motor is null, initialize an instance
            // of the PxFMotor (Float motor)
            if (m_buoyancyMotor == null)
            {
                // Initialize the PxFMotor with the position of the 
                // physics object, and the infinite motor component
                // and a efficiency of 1.0f
                m_buoyancyMotor = new PxFMotor(
                    PhysicsObject.Position.Z, PxMotor.Infinite, 1.0f);

                // Set the target as the water height within the physics scene
                // and the current value as the height of the physics object
                // to that the motor knows the current value and the target
                // value that it should be iterating up to
                m_buoyancyMotor.SetTarget(PhysicsScene.WaterHeight);
                m_buoyancyMotor.SetCurrent(PhysicsObject.Position.Z);

                // Add the buoyancy function to the before step within the scene
                // to ensure it is called before every simulation step
                PhysicsScene.BeforeStep += Buoyancy;
            }
        }

        /// <summary>
        /// Deactivate the buoyancy actor for the physics object this
        /// actor acts upon.
        /// </summary>
        private void DeactivateBuoyancy()
        {
            // If the buoyancy motor instance is existent, it is active so
            // go ahead and remove the pre-step Buoyancy action
            // and set the buoyancy motor to null
            if (m_buoyancyMotor != null)
            {
                PhysicsScene.BeforeStep -= Buoyancy;
                m_buoyancyMotor = null;
            }
        }

        /// <summary>
        /// The function that will be utilized as a pre-step action/delegate
        /// to simulate buoyancy on the specified controlling primitive based
        /// on the water level height, and the height of the physics actor.
        /// </summary>
        /// <param name="timeStep"> The time-step within the physics scene. </param>
        private void Buoyancy(float timeStep)
        {
            // Set up variables for storing the different applicable forces
            // for calculating buoyancy
            float height; 
            float gravity; 
            float momentum; 
            float buoyancyForce;
            float volume;

            // If it is not enabled, simply return
            if (!Enabled)
            {
                return;
            }

            // Calculate the difference in the height of the water and the
            // current height (z coordinate of the position)
            height = PhysicsScene.WaterHeight - PhysicsObject.Position.Z;

            // Calculate the volume of the water that will be displaced by the
            // object, which is the volume of the object itself
            volume = PhysicsObject.Mass / PhysicsObject.Density;
            
            // Calculate the force of gravity acting on the object
            gravity = PhysicsObject.Mass * (-PhysicsScene.UserConfig.Gravity);

            // Check for whether our object should float
            if (volume * PhysicsScene.UserConfig.BuoyancyDensity < gravity)
            {
                // The object is not in the water so no buoyancy force should
                // be applied
                buoyancyForce = 0.0f;
            }
            else
            {
                // Get the momentum of the object (velocity * mass) in the z
                // axis
                momentum = (PhysicsObject.Velocity.Z * PhysicsObject.Mass);

                // Calculate the total applicable force for pushing the object
                // to the surface of the water
                buoyancyForce = (height * gravity) - ((1.0f + height) * 
                    momentum);

                // Apply the calculated force to the physics object that this 
                // acts upon
                PhysicsScene.PhysX.AddForce(PhysicsObject.LocalID, 
                    new Vector3(0f, 0f, buoyancyForce));
            }
        }
    }
}
