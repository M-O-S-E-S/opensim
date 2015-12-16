
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
using System.Reflection;
using System.Text;

using OpenMetaverse;
using Nini.Config;

using OpenSim.Region.Physics.Manager;


namespace OpenSim.Region.Physics.PhysXPlugin
{
    /// <summary>
    /// The configuration values are publicly accessible, but their values are 
    /// determined by the configuration source passed into this object.
    /// </summary>
    public class PxConfiguration
    {
        #region Wrapper Configuration Properties

        /// <summary>
        /// Flag that determines if the GPU should be used for handling PhysX
        /// simulation. This will default to false and should be given in the
        /// configuration file /opensim/bin/config-include/PhysX.ini.
        /// </summary>
        public bool GPUEnabled { get; protected set; }

        /// <summary>
        /// The number of threads the CPU should use while running the PhysX
        /// simulation. This will default to 1 and should be given in the
        /// configuration file /opensim/bin/config-include/PhysX.ini.
        /// </summary>
        public int CPUMaxThreads { get; protected set; }

        /// <summary>
        /// Max number of updates allowed to process each frame.
        /// </summary>
        public int MaxUpdatesPerFrame { get; protected set; }

        /// <summary>
        /// Max number of collisions allowed to process for each frame.
        /// </summary>
        public int MaxCollisionsPerFrame { get; protected set; }

        #endregion

        #region Configuration Properties

        /// <summary>
        /// Indicates the default period of time simulated by each iteration of
        /// the integrated PhysX plugin.
        /// </summary>
        public float PhysicsTimeStep { get; protected set; }

        /// <summary>
        /// Indicates the default friction value used in the integrated PhysX
        /// plugin.
        /// </summary>
        public float DefaultFriction { get; protected set; }

        /// <summary>
        /// Indicates the default density used for objects in the integrated
        /// PhysX plugin.
        /// </summary>
        public float DefaultDensity { get; protected set; }

        /// <summary>
        /// Indicates the default restitution used for objects in the integrated
        /// PhysX plugin.
        /// </summary>
        public float DefaultRestitution { get; protected set; }

        /// <summary>
        /// Indicates the vertical force of the gravity being using in the
        /// integrated PhysX simulation.
        /// </summary>
        public float Gravity { get; protected set; }

        /// <summary>
        /// Indicates the height of the physical shape that is used for avatars
        /// in the integrated PhysX plugin.
        /// </summary>
        public float AvatarShapeHeight { get; protected set; }

        /// <summary>
        // Indicates the width of the physical shape that is used for avatars
        // in the integrated PhysX plugin.
        /// </summary>
        public float AvatarShapeWidth { get; protected set; }

        /// <summary>
        // Indicates the depth of the physical shape that is used for avatars
        // in the integrated PhysX plugin.
        /// </summary>
        public float AvatarShapeDepth { get; protected set; }

        /// <summary>
        // Indicates the static friction of the avatars in the integrated PhysX 
        /// plugin.
        /// </summary>
        public float AvatarStaticFriction { get; protected set; }

        /// <summary>
        // Indicates the kinetic friction of the avatars in the integrated PhysX 
        /// plugin.
        /// </summary>
        public float AvatarKineticFriction { get; protected set; }

        /// <summary>
        // Indicates the density of the avatars in the integrated PhysX plugin,
        // which is used to determine intertia and other physical attributes.
        /// </summary>
        public float AvatarDensity { get; protected set; }

        /// <summary>
        // Indicates the restitution of the avatars in the integrated PhysX
        // plugin, which is used to determine the speeds of objects after
        // collision (i.e.: bounciness).
        /// </summary>
        public float AvatarRestitution { get; protected set; }

        /// <summary>
        /// The factor that the avatar velocity will be multiplied by when
        /// running.
        /// </summary>
        public float RunFactor { get; protected set; }

        /// <summary>
        // Indicates the distance at which collision detection between objects
        // start. This helps with collision errors.
        /// </summary>
        public float CollisionMargin { get; protected set; }

        /// <summary>
        // Indicates the height of the ground plane in the integrated PhysX
        // plugin. No object will fall below this height.
        /// </summary>
        public float GroundPlaneHeight { get; protected set; }

        /// <summary>
        // Indicates the friction used for the terrain in the integrated PhysX
        // plugin.
        /// </summary>
        public float TerrainFriction { get; protected set; }

        /// <summary>
        // Indidicates the restitution used for the terrain in the integrated
        // PhysX plugin (i.e.: bounciness).
        /// </summary>
        public float TerrainRestitution { get; protected set; }

        /// <summary>
        /// Indicates the distance at which collision detection with the terrain
        /// starts. This leaves some room for collision errors.
        /// </summary>
        public float TerrainCollisionMargin { get; protected set; }

        /// <summary>
        /// Indicates the number of times an object may experience region
        /// boundary crossing failues before it is considered out of bounds.
        /// </summary>
        public int CrossingFailuresBeforeOutOfBounds { get; protected set; }

        /// <summary>
        /// This is the mass of the liquid used in the buoyancy calculations.
        /// This should be given in kg/m^3.
        /// </summary>
        public float BuoyancyDensity { get; protected set; }

        /// <summary>
        /// The scale factor to be used on height field height values. Height
        /// field values are stored as integers by PhysX, so a scale factor
        /// can be used to preserve some precision. A smaller value preserves
        /// more precision, but allows for a smaller range of height values.
        /// This value must be greater than 0. A value greater than 1 will
        /// result in loss of precision.
        /// </summary>
        public float HeightFieldScaleFactor { get; protected set; }

        #endregion


        /// <summary>
        /// Initializes the varaibles with default values while waiting on the
        /// configuration file that the user has made modifications to.
        /// </summary>
        public PxConfiguration()
        {
            // Initialize all the properties using default values
            // All default values are explained in the Initialize(...) method
            GPUEnabled = false;
            CPUMaxThreads = 1;
            MaxUpdatesPerFrame = 8192;
            MaxCollisionsPerFrame = 8192;
            PhysicsTimeStep = 0.89f;
            DefaultFriction = 0.2f;
            DefaultDensity = 1000.0006836f;
            DefaultRestitution = 0.0f;
            Gravity = -9.80665f;
            AvatarShapeHeight = 1.5f;
            AvatarShapeWidth = 0.6f;
            AvatarShapeDepth = 0.45f;
            AvatarStaticFriction = 0.95f;
            AvatarKineticFriction = 0.2f;
            AvatarDensity = 3500.0f;
            AvatarRestitution = 0.0f;
            RunFactor = 1.3f;
            CollisionMargin = 0.04f;
            GroundPlaneHeight = -10.0f;
            TerrainFriction = 0.2f;
            TerrainRestitution = 0.0f;
            TerrainCollisionMargin = 0.04f;
            CrossingFailuresBeforeOutOfBounds = 5;
            BuoyancyDensity = 1000.0f;
            HeightFieldScaleFactor = 0.01f;
        }


        /// <summary>
        /// Uses the given configuration file to apply the user values to
        /// variables that will be used throughout PhysX.
        /// </summary>
        /// <param name="config">Configuration file that will be used to get
        /// the user values for variables, this should be the file
        /// bin/config-include/PhysX.ini inside of the PhysX section</param>
        public void Initialize(IConfig config)
        {
            // Read in whether the GPU should be enabled for PhysX
            // calculations, the default will set the GPU to disabled
            GPUEnabled = config.GetBoolean("GPUEnabled", false);

            // Read in the max number of threads that PhysX is allowed to run
            // on the CPU, the default is set to 1 meaning that a single thread
            // will be created for all PhysX calculations not being ran on the
            // GPU
            CPUMaxThreads = config.GetInt("CPUMaxThreads", 1);

            // Read in the size of the updates array that will determine how
            // much information can be transferred from the wrapper during a
            // single frame, default is set to 8192
            MaxUpdatesPerFrame = config.GetInt("MaxUpdates", 8192);

            // Read in the size of the collisions array that will determine how
            // many collisions can be transferred from the wrapper during a
            // single frame, default is set to 8192
            MaxCollisionsPerFrame = config.GetInt("MaxCollisions", 8192);

            // Read in the default coefficient of friction used for objects
            // in the integrated PhysX simulation; default value is 0.2
            DefaultFriction = config.GetFloat("PrimFriction", 0.2f);

            // Read in the default density used for objects in the integrated
            // PhysX simulation 
            DefaultDensity = config.GetFloat("PrimDensity", 
                1000.0006836f);

            // Read in the default restitution for objects in the integrated
            // PhysX simulation; default value is 0, which means the object
            // will not bounce away after a collision
            DefaultRestitution = config.GetFloat("PrimRestitution", 0.0f);

            // Read in the default width for avatars in the integrated PhysX
            // plugin; default value is 0.6m
            AvatarShapeWidth = config.GetFloat("AvatarCapsuleWidth", 0.6f);

            // Read in the default depth for avatars in the integrated PhysX
            // plugin; default value is 0.45m
            AvatarShapeDepth = config.GetFloat("AvatarCapsuleDepth", 0.45f);

            // Read in the default coefficient of static friction used for 
            // avatars in the integrated PhysX plugin
            AvatarStaticFriction = config.GetFloat("AvatarStandingFriction", 
                0.95f);

            // Read in the default density used for avatars in the integrated
            // PhysX plugin
            AvatarDensity = config.GetFloat("AvatarDensity", 3500.0f);

            // Read in the run factor that should be used for avatars that are
            // currently running
            RunFactor = config.GetFloat("RunFactor", 1.3f);

            // Read in the buoyancy mass for buoyancy calculations, this value
            // is given in kg/m^3, the default is 1000 kg/m^3 which is the
            // density of water at 4 degrees celcius
            BuoyancyDensity = config.GetFloat("BuoyancyDensity", 1000.0f);

            /// Read in the scale factor to be used on height field height
            /// values. Height field values are stored as integers by PhysX,
            /// so a scale factor can be used to preserve some precision. A
            /// smaller value preserves more precision, but allows for a
            /// smaller range of height values.  This value must be greater
            /// than 0. A value greater than 1 will result in loss of precision.
            /// The default scale factor is 0.01
            HeightFieldScaleFactor = config.GetFloat("HeightFieldScaleFactor",
               0.01f);
        }
    }
}

