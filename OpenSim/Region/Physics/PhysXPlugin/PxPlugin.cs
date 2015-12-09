
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

using OpenSim.Framework;
using OpenSim.Region.Physics.Manager;
using OpenMetaverse;

namespace OpenSim.Region.Physics.PhysXPlugin
{
    /// <summary>
    /// Entry for PhysX to OpenSim.  This module interfaces to an unmanaged
    /// library which makes the actual calls into the PhysX physics engine.
    /// </summary>
    public class PxPlugin : IPhysicsPlugin
    {
        /// <summary>
        /// Interface to the PhysX implementation.
        /// </summary>
        private PxAPIManager m_physx;

        /// <summary>
        /// Main physics controller, using PhysX. Represents everything in
        /// the region.
        /// </summary>
        private PxScene m_scene;

        /// <summary>
        /// Constructor for the PhysX plugin.
        /// </summary>
        public PxPlugin()
        {
            // Create the class that will connect to the actual
            // PhysX implementation
            m_physx = new PxAPIManager();
        }

        /// <summary>
        /// Begin the initial startup for this plugin.
        /// </summary>
        /// <returns>Flag indicating whether the plugin startup was
        /// successful or not</returns>
        public bool Init()
        {
            bool success;

            // Initialize PhysX and return whether it was successful or not
            success = m_physx.Initialize();
            return success;
        }

        /// <summary>
        /// Return the physics scene.
        /// </summary>
        /// <param name="regionName">Name of the region the scene
        /// will represent</param>
        /// <returns>The physics scene</returns>
        public PhysicsScene GetScene(String regionName)
        {
            // Create a new scene if one doesn't already exist
            if (m_scene == null)
            {
                m_scene = new PxScene(GetName(), regionName, m_physx);
            }

            // Return the physics scene
            return m_scene;
        }

        /// <summary>
        /// Get the name of the physics engine used in this plugin.
        /// </summary>
        /// <returns>Name of physics engine</returns>
        public string GetName()
        {
            // Return the name of the physics engine used in this plugin
            return "PhysX";
        }

        /// <summary>
        /// Begin the shutdown for this plugin.
        /// </summary>
        public void Dispose()
        {
            // Begin the shutdown to PhysX
            m_physx.Uninitialize();
        }
    }
}
