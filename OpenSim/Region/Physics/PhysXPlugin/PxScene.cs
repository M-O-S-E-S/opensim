
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
using System.Collections.Concurrent;
using System.Diagnostics;
using System.Linq;
using System.Reflection;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading;

using OpenSim.Framework;
using OpenSim.Framework.Monitoring;
using OpenSim.Region.CoreModules;
using OpenSim.Region.Framework;
using OpenSim.Region.Physics.Manager;
using Logging = OpenSim.Region.CoreModules.Framework.Statistics.Logging;
using Nini.Config;
using log4net;
using OpenMetaverse;

using System.Security;

namespace OpenSim.Region.Physics.PhysXPlugin
{
    /// <summary>
    /// A scene which represents the collection of physics objects found in
    /// a designated region of the OpenSim instance. The scene will simulate
    /// the behavior of the objects over time.
    /// </summary>
    public sealed class PxScene : PhysicsScene, IPhysicsParameters
    {
        #region Fields

        /// <summary>
        /// Keep track of all physical objects added to the scene.
        /// The unique ID, from OpenSim, is used as the key.
        /// </summary>
        public Dictionary<uint, PxPhysObject> m_physObjectsDict;

        /// <summary>
        /// Keep track of all the avatars so we can send them a collision event
        /// every tick to update the animations of OpenSim.
        /// </summary>
        public HashSet<PxPhysObject> m_avatarsSet;

        /// <summary>
        /// Keep track of all the objects with collisions so we can report
        /// the beginning and end of a collision.
        /// </summary>
        public HashSet<PxPhysObject> m_objectsWithCollisions =
            new HashSet<PxPhysObject>();

        /// <summary>
        /// Keep track of all the objects with NO collisions so we can report
        /// to the avatars their proper collisions.
        /// </summary>
        public HashSet<PxPhysObject> m_objectWithNoMoreCollisions =
            new HashSet<PxPhysObject>();

        /// <summary>
        /// Keeps track of the current objects that have been updated during
        /// this step of the simulation.
        /// <summary>
        public HashSet<PxPhysObject> m_updatedObjects = new 
            HashSet<PxPhysObject>();

        /// <summary>
        /// Keeps track of the objects that were updated during the last
        /// step of the simulation. Objects that occur during the current step 
        /// of the simulation are removed from this list to find any objects 
        /// that no longer are updating. In this way locating objects that have 
        /// ceased moving can be found and set their velocities to zero.
        /// </summary>
        public HashSet<PxPhysObject> m_lastUpdatedObjects = new 
            HashSet<PxPhysObject>();

        /// <summary>
        /// The logger for this plugin.
        /// </summary>
        internal static readonly ILog m_log = LogManager.GetLogger(
            System.Reflection.MethodBase.GetCurrentMethod().DeclaringType);

        /// <summary>
        /// Header used for logger to highlight logs made in this plugin.
        /// </summary>
        internal static readonly string LogHeader = "[PHYSX SCENE]";

        /// <summary>
        /// Indicates an invalid time step; used to denote events that have not
        /// occurred.
        /// </summary>
        public static readonly int InvalidTime = -1;

        /// <summary>
        /// Pinned memory that passes updated entity properties sent
        /// by PhysX.
        /// </summary>
        internal EntityProperties[] m_updateArray;

        /// <summary>
        /// Pinned memory that passes updated collision properties sent
        /// by PhysX.
        /// </summary>
        internal CollisionProperties[] m_collisionArray;

        /// <summary>
        /// States whether the PhysX engine is initialized and ready to do
        /// simulation steps.
        /// </summary>
        private bool m_isInitialized = false;

        /// <summary>
        /// The next available unique joint identifier.
        /// </summary>
        private uint m_nextJointID = 1;

        /// <summary>
        /// The water height declared statically throughout the scene.
        /// </summary>
        private float m_waterHeight = 21.0f;

        /// <summary>
        /// The next available unique shape identifier.
        /// </summary>
        private uint m_nextShapeID = 1;

        /// <summary>
        /// The ID used when creating the terrain shape from a height field.
        /// </summary>
        private const uint TERRAIN_ACTOR_ID = 0;

        /// <summary>
        /// The unique identifier used for the height field shape of the
        /// terrain. Will be generated at terrain creation time.
        /// </summary>
        private uint m_terrainShapeID;

        /// <summary>
        /// A library of various material archetypes and their physical
        /// properties.
        /// </summary>
        public PxMaterialLibrary MaterialLibrary { get; private set; }

        /// <summary>
        /// The size of the region.
        /// </summary>
        private Vector3 m_regionExtents;

        /// <summary>
        /// A value of time given by the Util class that allows for easier
        /// access when synchronizing collisions and updates for the prims and
        /// avatars.
        /// </summary>
        public int m_simulationTime;

        /// <summary>
        /// A dictionary containing tainted objects that need to be rebuilt
        /// before the next simulation step.
        /// </summary>
        private ConcurrentDictionary<uint, PxPhysObject> m_taintedObjects =
            new ConcurrentDictionary<uint, PxPhysObject>();

        #endregion Fields

        #region Delegates

        /// <summary>
        /// Delegate method to handle the updates to the different vehicle and
        /// prim actors before updating PhysX.
        /// </summary>
        public delegate void PreStepAction(float timeStep);

        /// <summary>
        /// List of PreStepActions to take before running the PhysX update.
        /// </summary>
        public event PreStepAction BeforeStep;

        /// <summary>
        /// Delegate method to handle the updates to the different vehicle and
        /// prim actors after updating PhysX.
        /// </summary>
        public delegate void PostStepAction(float timeStep);

        /// <summary>
        /// List of PostStepActions to take after running the PhysX update.
        /// </summary>
        public event PostStepAction AfterStep;

        #endregion

        #region Properties

        /// <summary>
        /// The name of the region that we're working for.
        /// </summary>
        public string RegionName { get; private set; }

        /// <summary>
        /// Interface to the PhysX implementation.
        /// </summary>
        public PxAPIManager PhysX { get; private set; }

        /// <summary>
        /// The mesher used to convert shape descriptions to meshes.
        /// </summary>
        public IMesher SceneMesher { get; private set; }

        /// <summary>
        /// The configuration that will be used in initializing this plugin.
        /// </summary>
        public PxConfiguration UserConfig { get; private set; }

        /// <summary>
        /// Identifier used to represent the terrain in the physics engine.
        /// </summary>
        public int TerrainID
        {
            get
            {
                // Return the unique identifier of the terrain
                return (int) TERRAIN_ACTOR_ID;
            }
        }

        /// <summary>
        /// Getter and setter for the water height within the scene
        /// used to help calculate and determine buoyancy.
        /// </summary>
        public float WaterHeight
        {
            get
            {
                return m_waterHeight;
            }

            protected set
            {
                m_waterHeight = value;
            }
        }

        #endregion Properties

        #region Construction and Initialization

        /// <summary>
        /// Constructor for the PhysX scene.
        /// </summary>
        /// <param name="engineType">Name of physics engine</param>
        /// <param name="name">Name of region</param>
        /// <param name="physX">PhysX class interface</param>
        public PxScene(string engineType, string name, PxAPIManager physX)
        {
            // The name of the region that we're working for is passed to us;
            // keep for identification
            RegionName = name;

            // Set the identifying variables in the PhysicsScene interface
            EngineType = engineType;
            Name = EngineType + "/" + RegionName;

            // Save reference to the PhysX interface for future calls to the
            // PhysX engine
            PhysX = physX;

            // Create the configuration with default values
            UserConfig = new PxConfiguration();
        }


        /// <summary>
        /// Method to set up the intial values of necessary variables.
        /// </summary>
        /// <param name="meshmerizer">Mesher used for creating meshes from
        /// shape descriptions</param>
        /// <param name="config">Configuration file that will load the initial
        /// values set up inside of TBD</param>
        public override void Initialise(
            IMesher meshmerizer, IConfigSource config)
        {
            Vector3 regionExtent;

            // Create region extents based on the old assigned values of
            // 256x256
            // NOTE: This is for older versions of OpenSim that doesn't send the
            // size of the region when creating it
            // NOTE: The third value is not used and is assigned arbitrarily to
            // match the value BulletSim used, it is here to match the override
            // method of Initialise inside of the PhysicsScene class
            regionExtent = new Vector3(Constants.RegionSize,
                Constants.RegionSize, Constants.RegionSize);

            // Call the actual initialization method with the new extents
            Initialise(meshmerizer, config, regionExtent);
        }


        /// <summary>
        /// Method to set up the initial values of necessary variables.
        /// <summary>
        /// <param name="meshmerizer">Mesher used for creating meshes from
        /// shape descriptions</param>
        /// <param name="config">Configuration file that will load the initial
        /// values set up inside of TBD</param>
        /// <param name="regionExtent">The size of the region which will either
        /// be the basic 256 by 256 or a value given by the scene class</param>
        public override void Initialise(IMesher meshmerizer,
            IConfigSource config, Vector3 regionExtent)
        {
            // Get the PhysX configuration section
            IConfig physicsConfig = config.Configs["PhysX"];

            // Create the configuration using the user given values inside
            // of the PhysX.ini file located in the bin/config-include
            UserConfig.Initialize(physicsConfig);

            // Create the PhysX scene that will be used to represent the
            // region; all physics objects will be depicted in this scene;
            PhysX.CreateScene(UserConfig.GPUEnabled, UserConfig.CPUMaxThreads);

            // Create the initial ground plane for the physics engine
            PhysX.CreateGroundPlane(0.0f, 0.0f, -500.0f);

            // The scene has now been initialized
            m_isInitialized = true;

            // Initialize the mesher
            SceneMesher = meshmerizer;

            // Store the size of the region for use with height field
            // generation
            m_regionExtents = regionExtent;

            // New dictionary to keep track of physical objects
            // added to the scene
            m_physObjectsDict = new Dictionary<uint, PxPhysObject>();

            // New dictionary to keep track of avatars that need to send a
            // collision update for OpenSim to update animations
            m_avatarsSet = new HashSet<PxPhysObject>();

            // Allocate memory for returning of the updates from the
            // physics engine and send data to the PhysX engine
            m_updateArray = new EntityProperties[
                UserConfig.MaxUpdatesPerFrame];
            PhysX.InitEntityUpdate(ref m_updateArray, 
                UserConfig.MaxUpdatesPerFrame);

            // Allocate memory for returning of the collisions
            // from the physics engine and send data to the PhysX engine
            m_collisionArray = 
                new CollisionProperties[UserConfig.MaxCollisionsPerFrame];
            PhysX.InitCollisionUpdate(
                ref m_collisionArray, UserConfig.MaxCollisionsPerFrame);

            // Create the material library that will track various material
            // archetypes
            MaterialLibrary = new PxMaterialLibrary(UserConfig);
        }

        #endregion Construction and Initialization

        #region Prim and Avatar addition and removal


        /// <summary>
        /// This function is not currently implemented please use the AddAvatar
        /// that include the localID as this is necessary to uniquely identify
        /// the physical object.
        /// </summary>
        /// <param name="avName"></param>
        /// <param name="position">The current position of the new user avatar
        /// inside of the scene</param>
        /// <param name="velocity">The current velocity of the new user avatar
        /// inside of the scene</param>
        /// <param name="size">The size of the new user avatar, that will be
        /// used to scale the physical object</param>
        /// <param name="isFlying">A flag that determines if the user's
        /// physical object avatar is currently flying</param>
        /// <returns>A null value as this function is not currently implemented
        /// </returns>
        public override PhysicsActor AddAvatar(string avName, Vector3 position,
            Vector3 velocity, Vector3 size, bool isFlying)
        {
            // This function is being ignored, because we need the unique ID to
            // identify the physics actor
            return null;
        }


        /// <summary>
        /// This function will add a user's avatar to the physics plugin.
        /// </summary>
        /// <param name="localID">The unique ID used by OpenSim to track the
        /// physical objects inside of the world</param>
        /// <param name="avName">Given name of the avatar</param>
        /// <param name="position">The current position of the new user avatar
        /// inside of the scene</param>
        /// <param name="velocity">The current velocity of the new user avatar
        /// inside of the scene</param>
        /// <param name="size">The size of the new user avatar, that will be
        /// used to scale the physical object</param>
        /// <param name="isFlying">A flag that determines if the user's
        /// physical object avatar is currently flying</param>
        /// <returns>The physical object that was added to the physics engine
        /// that will represent the user's avatar</returns>
        public override PhysicsActor AddAvatar(uint localID, string avName,
            Vector3 position, Vector3 velocity, Vector3 size, bool isFlying)
        {
            // Can't add avatar if the scene hasn't been initialized yet
            if (!m_isInitialized)
                return null;

            // Create a physical object to represent the user avatar
            PxPhysObject actor = new PxPhysObject(
                localID, avName, this, position, velocity, size, isFlying);

            // Add the new actor to the dictionary of all physical objects;
            // lock dictionary to ensure no changes are made during addition
            lock (m_physObjectsDict)
            {
                m_physObjectsDict.Add(localID, actor);
            }

            // Add the new avatar to the avatar dictionary to make sure that
            // OpenSim updates the animations for the avatars, specific to the
            // avatar sending the collision event
            lock (m_avatarsSet)
            {
                m_avatarsSet.Add(actor);
            }

            // Give OpenSim a handle to the physical object of the user's
            // physical object
            return actor;
        }


        /// <summary>
        /// Remove a user avatar from the physical objects inside of the
        /// physics engine.
        /// </summary>
        /// <param name="actor">The user's physical object that represents the
        /// user's avatar inside of the physics engine that is to be removed
        /// from the scene</param>
        public override void RemoveAvatar(PhysicsActor actor)
        {
            PxPhysObject pxActor;

            // Make sure the scene has been initialized before removing avatar
            if (!m_isInitialized)
            {
                return;
            }

            // Cast the abstract class to the specific PhysX class
            pxActor = actor as PxPhysObject;

            // Check that the actor was cast succesfully
            if (pxActor != null)
            {
                // Tell PhysX to remove this actor from the scene
                PhysX.RemoveActor(pxActor.LocalID);

                // Prevent updates being made to the object dictionary
                lock (m_physObjectsDict)
                {
                    // Remove object from the dictionary; if removal
                    // was unsuccessful, say so
                    if (m_physObjectsDict.Remove(pxActor.LocalID) == false)
                    {
                        m_log.WarnFormat("{0}: Attempted to remove avatar " +
                            "that is not in the physics scene", LogHeader);
                    }
                }

                // Prevent updates being made to the avatar set
                lock (m_avatarsSet)
                {
                    // Remove the avatar from the dictionary
                    if (m_avatarsSet.Remove(pxActor) == false)
                    {
                        // Warn the user that the avatar was unable to be
                        // removed from the avatar list
                        m_log.WarnFormat("{0}: Attempted to remove avatar " +
                            "from the avatar list and it didn't exist.",
                            LogHeader);
                    }
                    else
                    {
                        // Clean up the actor
                        pxActor.Destroy();
                    }
                }
            }
            else
            {
                // Log that the given actor could not be removed
                m_log.ErrorFormat("{0}: Requested to remove avatar that is " +
                    "not a Physics Object", LogHeader);
            }
        }


        /// <summary>
        /// Add a new prim object to the scene.
        /// </summary>
        /// <param name="primName">Given name of the prim object</param>
        /// <param name="pbs">Basic prim shape info used for
        /// determining the object's mass and geometry</param>
        /// <param name="position">Current position of the new prim inside
        /// the scene</param>
        /// <param name="size">Size of the new avatar, that will be used
        /// to scale the physical object</param>
        /// <param name="rotation">Current rotation of the new prim
        /// inside the scene</param>
        /// <param name="isPhysical">Whether the prim should react
        /// to forces and collisions</param>
        /// <param name="localid">The unique ID used to track the object
        /// inside of the world</param>
        public override PhysicsActor AddPrimShape(string primName,
            PrimitiveBaseShape pbs, Vector3 position, Vector3 size,
            Quaternion rotation, bool isPhysical, uint localid)
        {
            // Can't add prim if the scene hasn't been initialized yet
            if (!m_isInitialized)
            {
                m_log.ErrorFormat("{0}: Unable to create prim shape because " +
                    "PxScene has not been initialized.", LogHeader);

                return null;
            }

            // Create a physical object to represent the primitive object
            PxPhysObject prim = new PxPhysObject(localid, primName, this,
                position, size, rotation, pbs, isPhysical);

            // Lock dictionary to ensure no changes are made during addition
            lock (m_physObjectsDict)
            {
                // Add the new prim to the dictionary of all physical objects
                m_physObjectsDict.Add(localid, prim);
            }

            // Give OpenSim a handle to the physical object of the prim
            return prim;
        }


        /// <summary>
        /// Remove a pimitive, by value, from the scene.
        /// </summary>
        public override void RemovePrim(PhysicsActor prim)
        {
            PxPhysObject pxPrim;

            // Make sure the scene has been initialized before removing avatar
            if (!m_isInitialized)
            {
                return;
            }

            // Cast the abstract class to the specific PhysX class
            pxPrim = prim as PxPhysObject;

            // Check that the actor was cast succesfully
            if (pxPrim != null)
            {
                // Clean up the prim's associated PhysX objects
                pxPrim.Destroy();

                // Prevent updates being made to the object dictionary
                lock (m_physObjectsDict)
                {
                    // Remove object from the dictionary; if removal
                    // was unsuccessful, say so
                    if (m_physObjectsDict.Remove(pxPrim.LocalID) == false)
                    {
                        m_log.WarnFormat("{0}: Attempted to remove avatar " +
                            "that is not in the physics scene", LogHeader);
                    }
                }
            }
            else
            {
                // Log that the given actor could not be removed
                m_log.ErrorFormat("{0}: Requested to remove avatar that is " +
                    "not a Physics Object", LogHeader);
            }
        }


        public override void AddPhysicsActorTaint(PhysicsActor prim)
        {
        }


        /// <summary>
        /// Returns the next available unique joint identifier for this scene.
        /// </summary>
        public uint GetNewJointID()
        {
            uint newID;

            // Fetch the next available unique joint identifier and
            // update the next one
            newID = m_nextJointID;
            m_nextJointID++;

            // Return the new joint identifier
            return newID;
        }


        /// <summary>
        /// Returns the next available unique shape identifier for this scene.
        /// </summary>
        public uint GetNewShapeID()
        {
            uint newID;

            // Fetch the next available unique shape identifier and update
            // the next one
            newID = m_nextShapeID;
            m_nextShapeID++;

            // Return the new shape identifier
            return newID;
        }


        #endregion Prim and Avatar addition and removal

        #region Simulation

        /// <summary>
        /// After the simulation has occured, pass the updates to the OpenSim
        /// PxPhysObjects for this scene.
        /// </summary>
        public void SimulateUpdatedEntities(uint updatedEntityCount)
        {
            EntityProperties   entityProperties;
            PxPhysObject       physObject;

            // Lock the dictionary of physical objects to prevent
            // any changes while the objects are being updated
            lock (m_physObjectsDict)
            {
                // Go through the list of updated entities
                for (int i = 0; i < updatedEntityCount; i++)
                {
                    // Grab the set of properties for the current
                    // physical object; this data is updated and sent
                    // from the PhysX engine
                    entityProperties = m_updateArray[i];

                    // Check to make sure that we're keeping track
                    // of this object
                    if (m_physObjectsDict.TryGetValue(
                        entityProperties.ID, out physObject) == true)
                    {
                        // Update the physical properties, that were
                        // sent from PhysX, for OpenSim to match
                        physObject.UpdateProperties(entityProperties);

                        // Ask the scene to update the position for
                        // all physical objects that have been updated
                        physObject.RequestPhysicsterseUpdate();

                        // Add the physical object to the list of objects that
                        // were updated during this timestep
                        m_updatedObjects.Add(physObject);

                        // Attempt to remove the updated object from the
                        // previous list of updated objects
                        // NOTE: Remove will only return false if the object
                        // could not be found, no crash or error message will
                        // occur
                        m_lastUpdatedObjects.Remove(physObject);
                    }
                }

                // Go through all of the objects that were updated during the
                // last simulation step, but were not updated during this
                // simulation step
                foreach (PxPhysObject obj in m_lastUpdatedObjects)
                {
                    // Build an update for the object that sets the velocity of
                    // the object to 0
                    entityProperties.ID = obj.LocalID;
                    entityProperties.PositionX = obj.Position.X;
                    entityProperties.PositionY = obj.Position.Y;
                    entityProperties.PositionZ = obj.Position.Z;
                    entityProperties.RotationX = obj.Orientation.X;
                    entityProperties.RotationY = obj.Orientation.Y;
                    entityProperties.RotationZ = obj.Orientation.Z;
                    entityProperties.RotationW = obj.Orientation.W;
                    entityProperties.VelocityX = 0.0f;
                    entityProperties.VelocityY = 0.0f;
                    entityProperties.VelocityZ = 0.0f;
                    entityProperties.AngularVelocityX = 0.0f;
                    entityProperties.AngularVelocityY = 0.0f;
                    entityProperties.AngularVelocityZ = 0.0f;

                    // Update the object with the new velocity in order to
                    // prevent viewers from changing the position of the
                    // object
                    obj.UpdateProperties(entityProperties);

                    // Send the updated values to OpenSim which will use the
                    // SendScheduledUpdates call inside of SceneObjectPart to
                    // send the information to a viewer
                    obj.RequestPhysicsterseUpdate();
                }

                // Clear all the updates from the list now that they have been
                // updated with a zero velocity
                m_lastUpdatedObjects.Clear();

                // Loop through all of the objects updated during this time
                // step and add them to the list of previously updated list
                foreach (PxPhysObject obj in m_updatedObjects)
                {
                    m_lastUpdatedObjects.Add(obj);
                }

                // Remove all objects from the list now that they are stored in
                // the previous timestep list
                m_updatedObjects.Clear();
            }
        }


        /// <summary>
        /// After the simulation has occured, simulate and update entities
        /// in OpenSim by allowing OpenSim to handle the collisions.
        /// </summary>
        /// <param name="updatedCollisionCount">The number of collisions that
        /// have occured in the last simulation step.</param>
        public void SimulateUpdatedCollisions(uint updatedCollisionCount)
        {
            CollisionProperties collisionProperties;
            uint localID;
            uint collidingWith;
            float penetration;
            Vector3 collidePoint;
            Vector3 collideNormal;

            // Lock the collision array so that no collisions can be modified
            lock (m_collisionArray)
            {
                // Iterate through the list of update collisions
                for (int i = 0; i < updatedCollisionCount; i++)
                {
                    // Grab the set of properties for the current
                    // physical object; this data is sent to OpenSim
                    // for processing
                    collisionProperties = m_collisionArray[i];

                    // Form instances of the vectors and such to be
                    // passed to the "SendCollision" method
                    localID = collisionProperties.ActorId1;
                    collidingWith = collisionProperties.ActorId2;
                    collidePoint =
                        new Vector3(collisionProperties.PositionX,
                                    collisionProperties.PositionY,
                                    collisionProperties.PositionZ);
                    collideNormal =
                        new Vector3(collisionProperties.NormalX,
                                    collisionProperties.NormalY,
                                    collisionProperties.NormalZ);
                    penetration = collisionProperties.Penetration;

                    // Send the collision to OpenSim
                    SendCollision(localID, collidingWith, collidePoint,
                        collideNormal, penetration);
                    SendCollision(collidingWith, localID, collidePoint,
                        -collideNormal, penetration);
                }

                // Iterate through each of the objects that were sent collisions
                // and send each their own respective collisions, as well as
                // add them to the set signaling they have no more collisions
                foreach (PxPhysObject pxObj in m_objectsWithCollisions)
                {
                    if (!pxObj.SendCollisions())
                    {
                        m_objectWithNoMoreCollisions.Add(pxObj);
                    }
                }

                // Iterate through the avatars, and send them collisions
                // as long as they are not contained within the current
                // objects with collisions set
                foreach (PxPhysObject pxObj in m_avatarsSet)
                {
                    if (!m_objectsWithCollisions.Contains(pxObj))
                    {
                        pxObj.SendCollisions();
                    }
                }

                // Iterate through the objects with no more collisions,
                // and remove them from the objects with collisions list
                foreach (PxPhysObject pxObj in m_objectWithNoMoreCollisions)
                {
                    m_objectsWithCollisions.Remove(pxObj);
                }

                // Clear the objects with no more collisions list
                m_objectWithNoMoreCollisions.Clear();
            }
        }

        /// <summary>
        /// Adds a tainted object to the list of tainted objects. These objects
        /// will be re-built before the next simulation step.
        /// </summary>
        /// <param name="obj">The tainted physics object that needs to be
        /// re-built</param>
        public void AddTaintedObject(PxPhysObject obj)
        {
            // Attempt to add this object to the list of tainted objects in
            // a thread-safe manner. If the object already exists within
            // the list, this operation is ignored
            lock (m_taintedObjects)
            {
                m_taintedObjects.GetOrAdd(obj.LocalID, obj);
            }
        }

        /// <summary>
        /// Simulate the physics scene, and do all the related actions.
        /// </summary>
        /// <param name="timeStep">The timestep amount to be simulated</param>
        public override float Simulate(float timeStep)
        {
            Stopwatch physicsFrameTime;
            uint updatedEntityCount = 0;
            uint updateCollisionCount = 0;

            // Create a profile timer that will profile the different parts of
            // running PhysX
            #if DEBUG
                Stopwatch profileTimer = new Stopwatch();
            #endif

            // Start a stopwatch to get the total time it took PhysX to
            // complete an update of the physics
            physicsFrameTime = new Stopwatch();
            physicsFrameTime.Start();

            // Start the profiler to acquire the time it takes to update all
            // the avatar velocities
            #if DEBUG
                profileTimer.Start();
            #endif

            // Update all script movement and vehicle movement before starting
            // the PhysX update
            TriggerPreStepEvent(timeStep);

            // Re-build any tainted objects in a thread-safe manner
            lock (m_taintedObjects)
            {
                // Go through each of the tainted objects in the dictionary
                foreach (KeyValuePair<uint, PxPhysObject> currPair
                    in m_taintedObjects)
                {
                    // Rebuild the current object
                    currPair.Value.BuildPhysicalShape();
                }

                // Now that the objects have been re-built, clear the list
                // so that they do not get repeatedly re-built
                m_taintedObjects.Clear();
            }

            // Prevent the avatar set from being manipulated
            lock (m_avatarsSet)
            {
                // Update all avatars in the scene, during this frame
                foreach (PxPhysObject pxObj in m_avatarsSet)
                {
                    // Set the avatar's linear velocity, during each simulation
                    // frame, to ensure that it has a constant velocity
                    // while it is colliding with something (e.g. terrain)
                    if (pxObj.IsColliding)
                    {
                        // Use the target velocity which should
                        // indicate the desired velocity of the avatar
                        PhysX.SetLinearVelocity(
                            pxObj.LocalID, pxObj.TargetVelocity);
                    }
                }
            }

            // Report how long it took for the avatar velocities to be updated
            // and start the timer again for the time PhysX runs the simulation
            // call
            #if DEBUG
                profileTimer.Stop();
                m_log.DebugFormat("{0}: Time for avatar velocity to be" +
                    " updated = {1}MS", LogHeader, 
                    profileTimer.Elapsed.TotalMilliseconds);
                profileTimer.Restart();
            #endif

            // Tell PhysX to advance the simulation
            PhysX.RunSimulation(
               timeStep, out updatedEntityCount, out updateCollisionCount);

            // Report how long it took for the PhysX simulation call and start
            // the timer again for the time OpenSim took to process all the
            // collisions that occurred
            #if DEBUG
                profileTimer.Stop();
                m_log.DebugFormat("{0}: Time for PhysX Simulation call" +
                    " = {1}MS", LogHeader, 
                    profileTimer.Elapsed.TotalMilliseconds);
                profileTimer.Restart();
            #endif

            // Update the current simulation time; this is used to synchronize
            // the updates between collisions and physical objects
            m_simulationTime = Util.EnvironmentTickCount();

            // Update the physical object collisions and their physical
            // properties
            SimulateUpdatedCollisions(updateCollisionCount);

            // Report how long it took for OpenSim to process all the
            // collisions for this frame and start the timer again for the time
            // OpenSim took to process all the updates that occurred
            #if DEBUG
                profileTimer.Stop();
                m_log.DebugFormat("{0}: Time for OpenSim to process the " +
                    "frame collisions = {1}MS", LogHeader, 
                    profileTimer.Elapsed.TotalMilliseconds);
                profileTimer.Restart();
            #endif

            SimulateUpdatedEntities(updatedEntityCount);

            // Report how long it took for OpenSim to process all the updates
            // that occurred and turn off the profiling timer since that is the
            // last profiling value that is needed
            #if DEBUG
                profileTimer.Stop();
                m_log.DebugFormat("{0}: Time for OpenSim to update the " +
                    "entities = {1}MS", LogHeader, 
                    profileTimer.Elapsed.TotalMilliseconds);
            #endif

            // Stop the stopwatch now that PhysX has completed the update
            // of the physics scene
            physicsFrameTime.Stop();

            // Return the amount of time (in seconds) it took for the
            // PhysX engine to process the scene
            return (float)physicsFrameTime.Elapsed.TotalMilliseconds * 1000.0f;
        }


        /// <summary>
        /// Updates the latest physics scene.
        /// </summary>
        /// <note>Used when the physics engine is running on its own thread
        /// </note>
        public override void GetResults()
        {
        }

        #endregion Simulation

        #region Terrain

        /// <summary>
        /// Create the terrain inside of the physics engine using the height
        /// map provided by OpenSim.
        /// <summary>
        /// <param name="heightMap">The list of heights for the entire
        /// terrain</param>
        public override void SetTerrain(float[] heightMap)
        {
            // Fetch a new unique identifier for the height field shape
            m_terrainShapeID = GetNewShapeID();

            // Send the height map to the PhysX wrapper, so the wrapper can
            // generate the terrain
            PhysX.SetHeightField(TERRAIN_ACTOR_ID, m_terrainShapeID,
                (int)m_regionExtents.X, (int)m_regionExtents.Y, 1.0f, 1.0f,
                heightMap, UserConfig.HeightFieldScaleFactor);
        }


        /// <summary>
        /// Send and add the collision to the physics object, as well as the
        /// hashset of collided objects.
        /// </summary>
        /// <param name="localId">The local id of the collider object</param>
        /// <param name="collidingWith">The local id of object that is
        /// colliding with the collider</param>
        /// <param name="collidePoint">The vector of the point that has
        /// collided</param>
        /// <param name="collideNormal">The vector of the point that has
        /// collided relative to the collider</param>
        /// <param name="penetration">The amount of seperation/penetration
        /// between the two object</param>
        private void SendCollision(uint localId, uint collidingWith,
            Vector3 collidePoint, Vector3 collideNormal, float penetration)
        {
            PxPhysObject collider;
            PxPhysObject collidee;

            // This is a check to make sure that the terrain does not throw
            // an entity not found exception while trying to get it from
            // the dictionary (OpenSim id is nonexistant, while PhysX is)
            if (localId == TERRAIN_ACTOR_ID)
            {
                return;
            }

            // Ensure no changes are made to the dictionary while it
            // is being read from
            lock (m_physObjectsDict)
            {
                // Try to get the collider object out of the scene
                // Otherwise print the error/log and return
                if (!m_physObjectsDict.TryGetValue(localId, out collider))
                {
                    m_log.InfoFormat("{0}, Collider ID not found in " +
                        "object list, ID = {1}", LogHeader, localId);
                    return;
                }

                // Set the collidee to null as this will represent a collision
                // with the terrain
                collidee = null;

                // Try to get the collidee object out of the scene a value of
                // null means the collider is colliding with the terrain
                m_physObjectsDict.TryGetValue(collidingWith, out collidee);

                // If the collision was successful, from the PhysX object
                // add the object to the objects with collision hashset
                if (collider.Collide(collidingWith, collidee, collidePoint,
                    collideNormal, penetration))
                {
                    m_objectsWithCollisions.Add(collider);
                }
            }
        }

        public override void SetWaterLevel(float baseheight)
        {
        }

        public override void DeleteTerrain()
        {
        }

        #endregion Terrain

        /// <summary>
        /// Clean up all objects in the scene and the scene itself.
        /// </summary>
        public override void Dispose()
        {
            // Make sure that a physics step doesn't happen during
            // object deletion
            m_isInitialized = false;

            // Remove all physical objects that are being tracked
            m_physObjectsDict.Clear();

            // Delete the scene
            PhysX.DisposeScene();
        }


        public override Dictionary<uint, float> GetTopColliders()
        {
            List<PxPhysObject> colliderList;

            // Go through each of the objects in the scene and evaluate their
            // collision scores
            foreach (KeyValuePair<uint, PxPhysObject> currPair in
                     m_physObjectsDict)
            {
                // Evaluate the current object's collision score
                currPair.Value.EvaluateCollisionScore();
            }

            // Construct a list of objects with descending collision scores
            colliderList = new List<PxPhysObject>(m_physObjectsDict.Values);
            colliderList.OrderByDescending(obj => obj.CollisionScore);

            // Take the top 25 colliders and return them
            return colliderList.Take(25).ToDictionary(
               obj => obj.LocalID, obj => obj.CollisionScore);
        }

        /// <summary>
        /// This method is called internally before the PhysX update in order
        /// to process script movement and vehicle movement.
        /// </summary>
        /// <param name="timeStep">The amount of time that will elapse in the
        /// next PhysX update</param>
        private void TriggerPreStepEvent(float timeStep)
        {
            // Get the action that needs to be called before PhysX updates
            PreStepAction actions = BeforeStep;

            // Make sure there is a function to call before calling the
            // function
            if (actions != null)
            {
                // Go ahead and call the function
                actions(timeStep);
            }
        }

        /// <summary>
        /// This method is called internally after the PhysX update in order
        /// to process script movement and vehicle movement.
        /// </summary>
        /// <param name="timeStep">The amount of time that elapsed in the
        /// PhysX update</param>
        private void TriggerPostStepEvent(float timeStep)
        {
            // Get the action that needs to be called before PhysX updates
            PostStepAction actions = AfterStep;

            // Make sure there is a function to call before calling it
            if (actions != null)
            {
                // Go ahead and call the function
                actions(timeStep);
            }
        }

        public override bool IsThreaded
        {
            get { return false; }
        }

        #region IPhysicsParameters

        public bool SetPhysicsParameter(string param, string val, uint localID)
        {
            bool returnVal = false;

            return returnVal;
        }

        public bool GetPhysicsParameter(string param, out string value)
        {
            value = String.Empty;
            return false;
        }

        public PhysParameterEntry[] GetParameterList()
        {
            return null;
        }

        #endregion IPhysicsParameters
    }
}

