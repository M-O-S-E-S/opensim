
// TODO: Create banner

using System;
using System.Collections.Generic;
using System.Linq;
using System.Reflection;
using System.Text;
using System.Threading;
using System.Net.Sockets;
using System.Xml;
using OpenSim.Framework;
using OpenSim.Framework.Monitoring;
using OpenSim.Region.Framework;
using OpenSim.Region.CoreModules;
using Logging = OpenSim.Region.CoreModules.Framework.Statistics.Logging;
using OpenSim.Region.Physics.Manager;
using Nini.Config;
using log4net;
using OpenMetaverse;

namespace OpenSim.Region.Physics.RemotePhysicsPlugin
{
    // Enumeration for defining the various physical shapes used for actors
    // in the remote physics engine
    public enum RemotePhysicsShape : uint
    {
        SHAPE_UNKNOWN = 0,
        SHAPE_CAPSULE = 1,
        SHAPE_BOX = 2,
        SHAPE_MESH = 3,
    }

    public class RemotePhysicsScene : PhysicsScene, IPhysicsParameters
    {
        /// <summary>
        /// The logger to be used for this class.
        /// </summary>
        internal static readonly ILog m_log = LogManager.GetLogger(
            System.Reflection.MethodBase.GetCurrentMethod().DeclaringType);

        /// <summary>
        /// This is the tag that will be used to denote log messages from
        /// this class.
        /// </summary>
        internal static readonly string LogHeader = "[REMOTE PHYSICS SCENE]";

        /// <summary>
        /// The name of the region which will be represented by this
        /// physics scene.
        /// </summary>
        public string RegionName { get; protected set; }

        /// <summary>
        /// The configuration that will be used in initializing this plugin
        /// and the remote physics scene.
        /// </summary>
        public RemotePhysicsConfiguration RemoteConfiguration { get;
            protected set; }

        /// <summary>
        /// The messenger that will be used to communicate with the remote
        /// physics engine.
        /// </summary>
        public IRemotePhysicsMessenger RemoteMessenger { get; protected set; }

        /// <summary>
        /// The dictionary containing mapping all physical objects in this scene
        /// to their respective IDs.
        /// </summary>
        public Dictionary<uint, RemotePhysicsObject> PhysicsObjects;

        /// <summary>
        /// Keeps track of objects that have had collisions during the last
        /// physics step.
        /// </summary>
        public HashSet<RemotePhysicsObject> ObjectsWithCollisions =
            new HashSet<RemotePhysicsObject>();

        /// <summary>
        /// Keeps track of objects that have had no collisions during the last
        /// physics step
        /// </summary>
        public HashSet<RemotePhysicsObject> ObjectsWithNoMoreCollisions =
            new HashSet<RemotePhysicsObject>();

        /// <summary>
        /// Callback used to update an actor in the remote physics engine
        /// before a simulate call.
        /// </summary>
        public delegate void ActorTaintCallback();

        /// <summary>
        /// The list of delegates to be called in order to update tainted
        /// actors before a simulate call.
        /// </summary>
        protected List<ActorTaintCallback> m_taintCallbacks;

        /// <summary>
        /// The mutex object to ensure that the taint callback list is
        /// thread-safe.
        /// </summary>
        protected Object m_taintListLock;

        /// <summary>
        /// Keeps track of the avatars in the remote physics scene. These
        /// avatars have special operations that differentiates them
        /// from other objects.
        /// </summary>
        protected HashSet<RemotePhysicsAvatar> m_avatars =
            new HashSet<RemotePhysicsAvatar>();

        /// <summary>
        /// Mutex object used to make the avatar list thread-safe.
        /// </summary>
        protected Object m_avatarsLock = new Object();
        
        /// <summary>
        /// Keep track of the primitive objects in the scene.
        /// </summary>
        protected Dictionary<uint, RemotePhysicsPrimitive> m_primitives =
            new Dictionary<uint, RemotePhysicsPrimitive>();

        /// <summary>
        /// Mutex object used to make the primitive list thread-safe.
        /// </summary>
        protected Object m_primitivesLock = new Object();

        /// <summary>
        /// The dimensions of the region in Open Simulator's units.
        /// </summary>
        protected OpenMetaverse.Vector3 m_regionExtents;

        /// <summary>
        /// The ID used to represent the ground plane in the remote
        /// physics engine.
        /// </summary>
        protected const int m_groundPlaneID = 0;
        
        /// <summary>
        /// The ID used to represent the terrain in the remote physics engine.
        /// </summary>
        protected const int m_terrainID = 1;

        public uint TerrainID
        {
            get
            {
                // Return the unique identifier used to represent the terrain
                // in the remote
                // physics engine
                return m_terrainID;
            }
        }

        /// <summary>
        /// The ID used to represent the terrain geometry in the remote
        /// physics engine.
        /// </summary>
        protected const int m_terrainShapeID = 1;

        /// <summary>
        /// Indicates whether the terrain has been built in the remote physics
        /// engine.
        /// </summary>
        protected bool m_terrainBuilt = false;

        /// <summary>
        /// The mesher used to convert shape descriptions to meshes.
        /// </summary>
        public IMesher SceneMesher { get; protected set; }

        /// <summary>
        /// Tracks the last simulation timestamp that was simulated by the
        /// remote physics engine.
        /// </summary>
        internal float m_lastSimulatedTime = 0;

        /// <summary>
        /// Registration delegate for pre-simulation step events.
        /// </summary>
        /// <param name="timeStep">The amount of time by which the simulation
        /// is about to be advanced.</param>
        public delegate void PreStepAction(float timeStep);

        /// <summary>
        /// Registration delegate for post-simulation step events.
        /// </summary>
        /// <param name="timeStep">The amount of time by which the simulation
        /// has advanced.</param>
        public delegate void PostStepAction(float timeStep);

        /// <summary>
        /// Event that is fired just before simulation time is advanced.
        /// </summary>
        public event PreStepAction BeforeStep;

        /// <summary>
        /// Event that is fired just after simulation time has advanced.
        /// </summary>
        public event PostStepAction AfterStep;

        /// <summary>
        /// The last time simulated by the remote engine.
        /// </summary>
        public float SimulatedTime { get; protected set; }

        /// <summary>
        /// The length of time for which the plugin has been running.
        /// </summary>
        public float CurrentSimulationTime { get; protected set; }

        /// <summary>
        /// The current step of the simulation.
        /// </summary>
        public long CurrentSimulationStep = 0;

        /// <summary>
        /// Indicates an invalid time; used to denote events that have
        /// not occurred.
        /// </summary>
        public static readonly double InvalidTime = -1.0f;

        /// <summary>
        /// Indicates an invalid simulation step; used to denote that events
        /// have not occurred.
        /// </summary>
        public static readonly long InvalidStep = -1;

        /// <summary>
        /// Indicates whether this plugin has been initialized.
        /// </summary>
        protected bool m_initialized = false;

        /// <summary>
        /// Indicates whether this scene should use its own update thread.
        /// </summary>
        protected bool m_useThread = false;

        /// <summary>
        /// The next available unique shape identifier in the remote physics
        /// engine.
        /// </summary>
        public uint m_nextShapeId = 100;

        /// <summary>
        /// The next available unique actor identifier in the remote physics
        /// engine.
        /// </summary>
        public uint m_nextActorId = 100;

        /// <summary>
        /// The next avaiable unique joint identifier in the remote physics
        /// engine.
        /// </summary>
        public uint m_nextJointId = 100;

        /// <summary>
        /// The list of objects that have been updated during the current
        /// simulation step.
        /// </summary>
        protected List<RemotePhysicsObject> m_updatedObjects;

        /// <summary>
        /// The list of objects that were updated during the last simulation
        /// step.
        /// </summary>
        protected List<RemotePhysicsObject> m_lastUpdatedObjects;

        /// <summary>
        /// A library of various material archetypes and their physical
        /// properties.
        /// </summary>
        public RemotePhysicsMaterialLibrary
            MaterialLibrary { get; protected set; }

        /// <summary>
        /// Property that indicates the default gravity for the remote
        /// physics scene.
        /// </summary>
        public Vector3 DefaultGravity
        {
            get
            {
                return new Vector3(0.0f, 0.0f, RemoteConfiguration.Gravity);
            }
        }

        /// <summary>
        /// Property that maintains the water level across the entire scene
        /// </summary>
        public float SimpleWaterLevel { get; protected set; }

        /// <summary>
        /// The messaging system that allows this plugin to communicate with
        /// the remote physics server using the Archimedes Physics Protocol.
        /// </summary>
        protected RemotePhysicsAPPMessenger m_remoteMessenger = null;

        /// <summary>
        /// The packet manager that establishes and maintains the connection
        /// with the remote physics server.
        /// </summary>
        protected RemotePhysicsTCPPacketManager m_remotePacketManager = null;

        /// <summary>
        /// The packet manager that establishes and maintains an UDP connection
        /// with the remote physics server.
        /// </summary>
        protected RemotePhysicsUDPPacketManager m_remoteUdpPacketManager = null;

        /// <summary>
        /// Indicates the amount of time taken to compute and finalize the
        /// previous time advancement in seconds.
        /// </summary>
        protected float m_frameTime = 0.0f;

        /// <summary>
        /// Indicates the time at which the processing of the current
        /// simulation frame began.
        /// </summary>
        protected int m_frameTimeBegin = 0;

        /// <summary>
        /// Object used to enusre that the frame time values are thread-safe.
        /// </summary>
        protected Object m_frameTimeLock = new Object();

        #region Construction and Initialization

        /// <summary>
        /// Constructor.
        /// </summary>
        /// <param name="identifier">The name of this scene</param>
        public RemotePhysicsScene(string identifier)
        {
            // Indicate that the scene has yet to be initialized
            m_initialized = false;

            // Store the name of the region that will be represented by this
            // physics scene
            RegionName = identifier;

            // Create the configuration with the default values
            RemoteConfiguration = new RemotePhysicsConfiguration();

            // Create the list that will hold the delegates of tainted actors
            m_taintCallbacks = new List<ActorTaintCallback>();

            // Create the mutex object that will keep the taint callback list
            // thread-safe
            m_taintListLock = new Object();

            // Create the material library that will track various material
            // archetypes
            MaterialLibrary = new RemotePhysicsMaterialLibrary(
                RemoteConfiguration);
        }

        /// <summary>
        /// Initalizes the physics scene.
        /// NOTE: This is the old initialization method that assumes
        /// a legacy region size of 256 x 256.
        /// </summary>
        /// <param name="meshmerizer">The mesher to be used for creating
        /// meshes from shape descriptions</param>
        /// <param name="config">The configuration settings to be used to
        /// configure the scene</param>
        public override void Initialise(IMesher meshmerizer,
            IConfigSource config)
        {
            Vector3 regionExtent;

            // Create region extents based on the legacy region sizes
            regionExtent = new Vector3(Constants.RegionSize,
                Constants.RegionSize, Constants.RegionSize);

            // Call the actual intialization method with the new extents
            Initialise(meshmerizer, config, regionExtent);
        }

        // Initializes the physics scene
        /// <summary>
        /// Initializes the physics scene.
        /// </summary>
        /// <param name="meshmerizer">The mesher to be used for creating
        /// meshes from shape descriptions</param>
        /// <param name="config">The configuration settings to be used to
        /// configure the scene</param>
        /// <param name="regionExtent">The size of the scene in Open Simulator
        /// units</param>
        public override void Initialise(IMesher meshmerizer,
            IConfigSource config, Vector3 regionExtent)
        {
            OpenMetaverse.Vector3 gravityVector;

            // Create a configuration, which will be used for initializing the
            // scene, using the "RemotePhysics" section of the given
            // configuration file
            RemoteConfiguration.Initialize(config.Configs["RemotePhysics"]);

            // Initialize the dimensions of this scene
            m_regionExtents = regionExtent;
            
            // Initialize the mesher
            SceneMesher = meshmerizer;

            // Create the dictionary that will map physics objects to their
            // respective IDs
            PhysicsObjects = new Dictionary<uint, RemotePhysicsObject>();

            // Create the dictionaries that keep track of which objects have
            // collisions or don't have collisions
            ObjectsWithCollisions = new HashSet<RemotePhysicsObject>();
            ObjectsWithNoMoreCollisions = new HashSet<RemotePhysicsObject>();

            // The simulation time has not been updated yet
            m_lastSimulatedTime = 0.0f;

            // Create the packet manager that will maintain a connection
            // with the remote physics server
            m_remotePacketManager =
                new RemotePhysicsTCPPacketManager(RemoteConfiguration);

            // Create the packet manager that will be used for UDP
            // communications with the remote physics server
            m_remoteUdpPacketManager =
                new RemotePhysicsUDPPacketManager(RemoteConfiguration);

            // Create the messaging system that will allow this scene to
            // communicate with the remote physics server
            m_remoteMessenger = new RemotePhysicsAPPMessenger();
            m_remoteMessenger.Initialize(RemoteConfiguration,
                m_remotePacketManager, m_remoteUdpPacketManager);
            RemoteMessenger = m_remoteMessenger;

            // Initialize the lists that will track which objects were updated
            // duing this and the last step
            m_updatedObjects = new List<RemotePhysicsObject>();
            m_lastUpdatedObjects = new List<RemotePhysicsObject>();

            // Send the logon message
            m_remoteMessenger.Logon(RemoteConfiguration.SimulationID,
                RegionName);

            // Set the callbacks that listen for updates from the remote engine
            m_remoteMessenger.OnDynamicActorUpdateEvent +=
                new UpdateDynamicActorHandler(DynamicActorUpdated);
            m_remoteMessenger.OnActorsCollidedEvent +=
                new ActorsCollidedHandler(ActorsCollided);
            m_remoteMessenger.OnDynamicActorMassUpdateEvent +=
                new UpdateDynamicActorMassHandler(ActorMassUpdated);
            m_remoteMessenger.OnTimeAdvancedEvent +=
                new TimeAdvancedHandler(TimeAdvanced);

            // Send the message to the remote physics engine that creates
            // the remote scene and establishes the ground plane
            gravityVector = new OpenMetaverse.Vector3(0.0f, 0.0f,
                RemoteConfiguration.Gravity);
            m_remoteMessenger.InitializeWorld(gravityVector,
                RemoteConfiguration.DefaultFriction,
                RemoteConfiguration.DefaultFriction,
                RemoteConfiguration.CollisionMargin, 
                RemoteConfiguration.DefaultRestitution, m_groundPlaneID,
                RemoteConfiguration.GroundPlaneHeight,
                new OpenMetaverse.Vector3(0.0f, 0.0f, 1.0f));

            // Indicate that the scene is now initialized
            m_initialized = true;
        }

        #endregion

        /// <summary>
        /// Cleans up resources used by this scene.
        /// </summary>
        public override void Dispose()
        {
            // Indicate that the scene is no longer initialized, so that no
            // simulation happens while disposing is occurring
            m_initialized = false;

            // Check to see if a terrain has been set for this scene in the
            // remote physics engine
            if (m_terrainBuilt)
            {
                // Detach the old terrain shape and remove it, so that
                // only the new terrain will be attached to the terrain actor
                RemoteMessenger.DetachShape(m_terrainID, m_terrainShapeID);
                RemoteMessenger.RemoveShape(m_terrainShapeID);
            }

            // Log out of the remote physics engine
            RemoteMessenger.Logoff(RemoteConfiguration.SimulationID);

            // Clean up the remote messenger
            m_remoteMessenger.Dispose();
            m_remotePacketManager.Dispose();
        }

        #region Prim and Avatar addition and removal

        /// <summary>
        /// Add an avatar to the physics scene.
        /// </summary>
        /// <param name="avName">The name of the avatar</param>
        /// <param name="position">The initial position of the avatar</param>
        /// <param name="velocity">The initial velocity of the avatar</param>
        /// <param name="size">The size of the avatar</param>
        /// <param name="isFlying">Indicates whether the avatar is flying or
        /// not</param>
        /// <returns>The physics actor representing the avatar</returns>
        public override PhysicsActor AddAvatar(string avName, Vector3 position,
            Vector3 velocity, Vector3 size, bool isFlying)
        {
            Random idGenerator;
            uint randomID;

            // Since a local ID was not specified, keep trying to generate a
            // random one that is not already being used
            idGenerator = new Random();
            randomID = (uint)idGenerator.Next();
            while (PhysicsObjects.ContainsKey(randomID))
            {
                randomID = (uint)idGenerator.Next();
            }

            // Now that a unique key has been found, use it to add the avatar
            return AddAvatar(randomID, avName, position, velocity, size,
                isFlying);
        }

        /// <summary>
        /// Add an avatar to the physics scene.
        /// </summary>
        /// <param name="localID">The unique identifier of the avatar</param>
        /// <param name="avName">The name of the avatar</param>
        /// <param name="position">The initial position of the avatar</param>
        /// <param name="velocity">The initial velocity of the avatar</param>
        /// <param name="size">The size of the avatar</param>
        /// <param name="isFlying">Indicates whether the avatar is flying or
        /// not</param>
        /// <returns>The physics actor representing the avatar</returns>
        public override PhysicsActor AddAvatar(uint localID, string avName,
            Vector3 position, Vector3 velocity, Vector3 size, bool isFlying)
        {
            RemotePhysicsAvatar newAvatar;

            // Check to see if the scene has been initialized; if not, exit out
            // since the addition cannot be completed
            if (!m_initialized)
                return null;

            // Create a new object to represent this avatar
            newAvatar = new RemotePhysicsAvatar(localID, avName, this, position,
                velocity, size, isFlying, RemoteConfiguration);

            // Add the new avatar to the collection of physics objects in a
            // thread-safe manner
            lock (PhysicsObjects)
            {
                PhysicsObjects.Add(localID, newAvatar);
            }

            // Add the new avatar to teh collection of avatars in a thread-safe
            // manner
            lock (m_avatarsLock)
            {
                m_avatars.Add(newAvatar);
            }

            // Return the newly created actor
            return newAvatar;
        }

        /// <summary>
        /// Remove an actor from the physics scene.
        /// </summary>
        /// <param name="actor">The actor to be removed</param>
        public override void RemoveAvatar(PhysicsActor actor)
        {
            RemotePhysicsAvatar avatar;

            // Check to see if the scene has been initialized; if not, exit out
            // since the there are no avatars
            if (!m_initialized)
                return;

            // Attempt to convert the given actor into its corresponding form
            // in the plugin
            avatar = actor as RemotePhysicsAvatar;

            // Check to see if the given actor was an object of this plugin
            if (avatar != null)
            {
                // Check to see if this object is in this scene
                if (PhysicsObjects.ContainsKey(avatar.LocalID))
                {
                    // Remove this object from the scene's list in a thread-safe
                    // manner
                    lock (PhysicsObjects)
                    {
                        PhysicsObjects.Remove(avatar.LocalID);
                    }

                    // Remove this object from the scene's list of avatars in a
                    // thread-safe manner
                    lock (m_avatars)
                    {
                        m_avatars.Remove(avatar);
                    }

                    // Clean up the avatar
                    avatar.Destroy();
                }
                else
                {
                    // Inform the user that the scene doesn't contain the given
                    // actor
                    m_log.WarnFormat("{0}: Attempt to remove avatar that is " +
                        "not in physics scene.", LogHeader);
                }
            }
            else
            {
                // Inform the user that the given actor is not an avatar of
                // this plugin
                m_log.WarnFormat("{0}: Requested to remove avatar that is " +
                    "not a RemotePhysicsAvatar. ID={1}, type={2}",
                    LogHeader, actor.LocalID, actor.GetType().Name);
            }
        }

        /// <summary>
        /// Add a non-avatar shape to the physics scene.
        /// </summary>
        /// <param name="primName">The name of the primitive</param>
        /// <param name="pbs">The description of the shape being added</param>
        /// <param name="position">The initial position of the shape</param>
        /// <param name="size">The size of the shape</param>
        /// <param name="rotation">The initial orientation of the shape</param>
        /// <param name="isPhysical">Indicates whether the shape is static
        /// or dynamic (whether physical forces act upon the object)</param>
        /// <param name="localid">The unique identifier of the shape</param>
        /// <returns>The physics actor representing the shape</returns>
        public override PhysicsActor AddPrimShape(string primName,
            PrimitiveBaseShape pbs, Vector3 position, Vector3 size,
            Quaternion rotation, bool isPhysical, uint localid)
        {
            RemotePhysicsPrimitive newPrim;

            // Check to see if the scene has been initialized; if not,
            // exit out since the addition cannot be completed
            if (!m_initialized)
                return null;

            // Create the new primitive using the given parameters
            newPrim = new RemotePhysicsPrimitive(localid, primName, this,
                position, rotation, size, pbs, isPhysical);

            // Add the new avatar to the collection of physics objects in a
            // thread-safe manner
            lock (PhysicsObjects)
            {
                PhysicsObjects.Add(localid, newPrim);
            }

            // Add the primitive to the set of primitives in a thread-safe
            // manner
            lock (m_primitivesLock)
            {
                m_primitives.Add(localid, newPrim);
            }

            // Return the newly created primitive
            return newPrim;
        }

        /// <summary>
        /// Removes a shape from the physics scene.
        /// </summary>
        /// <param name="prim">The shape to be removed</param>
        public override void RemovePrim(PhysicsActor prim)
        {
            RemotePhysicsPrimitive removePrim;

            // Attempt to cast the given actor to a primitive
            removePrim = prim as RemotePhysicsPrimitive;

            // Check to see if the cast was successful
            if (removePrim == null)
            {
                // This means that the given actor is not a primitive, so exit
                return;
            }

            // Check to see if this scene contains a primitive by the given ID
            if (!PhysicsObjects.ContainsKey(removePrim.LocalID))
            {
                // This primitive is not of this scene, so exit
                return;
            }

            // Remove the actor from the list of primitives
            lock (m_primitivesLock)
            {
                m_primitives.Remove(prim.LocalID);
            }

            // Remove the actor from the list of objects
            PhysicsObjects.Remove(prim.LocalID);

            // Clean up the primitive
            removePrim.Destroy();
        }

        /// <summary>
        /// Returns the next available unique shape identifier for this scene.
        /// </summary>
        /// <returns>A new unique shape identifier</returns>
        public uint GetNewShapeID()
        {
            uint result;

            // Fetch the next available unique shape identifier
            result = m_nextShapeId;

            // Update the next shape identifier by incrementing the ID tracker
            m_nextShapeId++;

            // Return the result
            return result;
        }

        /// <summary>
        /// Returns the next available unique actor identifier for this scene.
        /// </summary>
        /// <returns>A new unique actor identifier</returns>
        public uint GetNewActorID()
        {
            // Generate the next actor ID and make sure that its is unique
            while (PhysicsObjects.ContainsKey(m_nextActorId))
            {
                m_nextActorId++;
            }

            // Return the result
            return m_nextActorId;
        }

        /// <summary>
        /// Returns the next available unique joint identifier for this scene.
        /// </summary>
        /// <returns>A new unique joint identifier</returns>
        public uint GetNewJointID()
        {
            uint result;

            // Fetch the next available unique joint identifier
            result = m_nextJointId;

            // Update the next joint identifier by incrementing the ID tracker
            m_nextJointId++;

            // Return the result
            return result;
        }

        /// <summary>
        /// Indicates that an actor has been updated by the engine.
        /// NOTE: This is an override method that is not needed.
        /// </summary>
        /// <param name="prim">The actor that was modified</param>
        public override void AddPhysicsActorTaint(PhysicsActor prim)
        {
            // Nothing to do here
        }

        #endregion

        #region Simulation

        /// <summary>
        /// Advances the time and state of the remote physics scene.
        /// </summary>
        /// <param name="timeStep">The amount of time by which to advance
        /// (in seconds)</param>
        /// <returns>The frame time needed to simulate the step</returns>
        public override float Simulate(float timeStep)
        {
            List<ActorTaintCallback> taintCallbacks;
            float frameTime;

            // Record the start time of this frame
            lock (m_frameTimeLock)
            {
                m_frameTimeBegin = Util.EnvironmentTickCount();
            }

            // Update any actors that have been tainted by firing any
            // callbacks they have scheduled
            lock (m_taintListLock)
            {
                taintCallbacks = new List<ActorTaintCallback>(m_taintCallbacks);
                m_taintCallbacks.Clear();
            }
            foreach (ActorTaintCallback currCallback in taintCallbacks)
            {
                currCallback();
            }

            // Update the orientation of each avatar
            lock (m_avatarsLock)
            {
                foreach (RemotePhysicsAvatar currAvatar in m_avatars)
                {
                    m_remoteMessenger.UpdateActorOrientation(currAvatar.LocalID,
                        currAvatar.Orientation);

                    // If the current avatar is on the ground, force the
                    // velocity as it decays due to friction
                    if (currAvatar.IsColliding)
                        m_remoteMessenger.UpdateActorVelocity(
                            currAvatar.LocalID, currAvatar.TargetVelocity);

                    // Send any collisions and physical property updates this
                    // avatar has to the simulator
                    currAvatar.SendCollisions();
                    currAvatar.RequestPhysicsterseUpdate();
                }
            }

            // Send a message to the remote physiscs simulator so that it will
            // simulate the timestep
            RemoteMessenger.AdvanceTime(timeStep);

            // Go through each of the primitives and send their updates to
            // the simulator
            lock (m_primitivesLock)
            {
                foreach(KeyValuePair<uint, RemotePhysicsPrimitive> prim in
                        m_primitives)
                {
                    // Send any collisions and physical property updates this
                    // primitive has to the simulator
                    prim.Value.SendCollisions();
                    //if (prim.Value.m_linkParent == null)
                    prim.Value.RequestPhysicsterseUpdate();
                }
            }

            // Update the current time & simulation step
            CurrentSimulationTime = Util.EnvironmentTickCount();
            CurrentSimulationStep++;

            // Fetch the time it took to process the frame (in a thread-safe
            // manner) and return it
            lock (m_frameTimeLock)
            {
               frameTime = m_frameTime;
            }
            return frameTime;
        }

        #endregion

        /// <summary>
        /// Retrieve the results of a frame from the remote engine.
        /// NOTE: This method is not needed, since callbacks are used to post
        /// updates from the remote physics engine.
        /// </summary>
        public override void GetResults()
        {
        }

        /// <summary>
        /// Adds a callback to update a tainted actor in the remote physics
        /// engine.
        /// </summary>
        /// <param name="newCallback">The delegate to be called before time is
        /// advanced</param>
        public void AddActorTaintCallback(ActorTaintCallback newCallback)
        {
            // Add the new callback in a thread-safe manner
            lock (m_taintListLock)
            {
                m_taintCallbacks.Add(newCallback);
            }
        }

        #region Terrain

        /// <summary>
        /// Set the terrain for the physics scene.
        /// </summary>
        /// <param name="heightMap">The height values for the height map.
        /// Should contain enough elements to fill the region size</param>
        public override void SetTerrain(float[] heightMap)
        {
            OpenMetaverse.Quaternion hfOrienation;

            if (m_terrainBuilt)
            {
                // Detach old terrain shape and remove it, so that
                // only the new terrain will be attached to the terrain actor
                RemoteMessenger.DetachShape(m_terrainID, m_terrainShapeID);
                RemoteMessenger.RemoveShape(m_terrainShapeID);
            }
            else
            {
                // Send a message that creates an actor for the height map
                RemoteMessenger.CreateStaticActor(m_terrainID,
                    OpenMetaverse.Vector3.Zero,
                    OpenMetaverse.Quaternion.Identity, false);
            }

            // Create the shape for the height map
            hfOrienation = OpenMetaverse.Quaternion.CreateFromEulers(
                (float)Math.PI / 2.0f, (float)Math.PI / 2.0f, 0.0f);
            RemoteMessenger.AddHeightField(m_terrainShapeID,
                (uint)m_regionExtents.X, (uint)m_regionExtents.Y, 1.0f, 1.0f,
                heightMap);

            // Attach the height map shape to the actor
            RemoteMessenger.AttachShape(m_terrainID, m_terrainShapeID,
                RemoteConfiguration.DefaultDensity,
                RemoteConfiguration.DefaultFriction,
                RemoteConfiguration.DefaultFriction,
                RemoteConfiguration.DefaultRestitution, hfOrienation,
                OpenMetaverse.Vector3.Zero);

            // Indicate that the terrain has been built in the remote
            // physics engine
            m_terrainBuilt = true;
        }

        /// <summary>
        /// Sets the water level for the physics scene.
        /// </summary>
        /// <param name="baseHeight">The height of the water level</param>
        public override void SetWaterLevel(float baseHeight)
        {
            // Update the water level for this physics scene
            SimpleWaterLevel = baseHeight;
        }

        /// <summary>
        /// Removes the terrain height map from the physics scene.
        /// </summary>
        public override void DeleteTerrain()
        {
            // Remove the actor and the shape from the remote physics engine
            RemoteMessenger.RemoveActor(m_terrainID);
            RemoteMessenger.RemoveShape(m_terrainShapeID);
        }

        #endregion

        /// <summary>
        /// Returns the actors with the highest collision scores.
        /// </summary>
        /// <returns></returns>
        public override Dictionary<uint, float> GetTopColliders()
        {
            List <RemotePhysicsObject> colliderList;

            // Go through each of the objects in the scene and evaluate their
            // collision scores
            foreach (KeyValuePair<uint, RemotePhysicsObject> currPair in
                PhysicsObjects)
            {
                // Evaluate the current object's collisions score
                currPair.Value.EvaluateCollisionScore();
            }

            // Construct a list of objects with descending collision scores
            colliderList = new List<RemotePhysicsObject>(
                PhysicsObjects.Values);
            colliderList.OrderByDescending(obj => obj.CollisionScore);

            // Take the top 25 colliders and return them
            return colliderList.Take(25).ToDictionary(
                obj => obj.LocalID, obj => obj.CollisionScore);
        }

        /// <summary>
        /// Indicates whether this scene should use its own internal thread
        /// for updates.
        /// </summary>
        public override bool IsThreaded
        {
            get { return m_useThread; }
        }

        /// <summary>
        /// Return the list of parameters supported by this physics scene.
        /// </summary>
        /// <returns>The list of supported parameters</returns>
        public PhysParameterEntry[] GetParameterList()
        {
            // TODO: Implement this
            return new PhysParameterEntry[1];
        }

        /// <summary>
        /// Updates a physics parameters.
        /// </summary>
        /// <param name="param">The name of the parameter</param>
        /// <param name="val">The new value of the parameter</param>
        /// <param name="localID">The object(s) to which the parameter should
        /// be applied</param>
        /// <returns>Whether the parameter was successfully set</returns>
        public bool SetPhysicsParameter(string param, string val, uint localID)
        {
            // TODO: Implement this
            return false;
        }

        /// <summary>
        /// Get the value of a physics parameter.
        /// </summary>
        /// <param name="param">The name of the parameter</param>
        /// <param name="value">The value of the parameter</param>
        /// <returns>Whether the parameter was successfully retrieved</returns>
        public bool GetPhysicsParameter(string param, out string value)
        {
            // TODO: Implement this
            value = "";
            return false;
        }

        /// <summary>
        /// Callback used by the remote physics engine to indicate that a
        /// dynamic actor has had an update to its physical properties.
        /// </summary>
        /// <param name="actorID">The unique identifier of the actor</param>
        /// <param name="position">The position of the actor</param>
        /// <param name="orientation">The orientation of the actor</param>
        /// <param name="linearVelocity">The linear velocity of the
        /// actor</param>
        /// <param name="angularVelocity">The angular velocity of the
        /// actor</param>
        protected void DynamicActorUpdated(uint actorID,
            OpenMetaverse.Vector3 position,
            OpenMetaverse.Quaternion orientation,
            OpenMetaverse.Vector3 linearVelocity,
            OpenMetaverse.Vector3 angularVelocity)
        {
            HashSet<RemotePhysicsAvatar> avatars;
            HashSet<RemotePhysicsPrimitive> prims;
            RemotePhysicsPrimitive prim;
            bool actorFound;

            // Look through the list of avatars to see if there is an avatar
            // with a matching ID
            actorFound = false;
            lock (m_avatarsLock)
            {
                avatars = new HashSet<RemotePhysicsAvatar>(m_avatars);
            }
            foreach (RemotePhysicsAvatar avatar in avatars)
            {
                if (avatar.LocalID == actorID)
                {
                    // Update the desired properties of this avatar
                    avatar.UpdatePosition(position);
                    avatar.UpdateOrientation(orientation);
                    avatar.UpdateVelocity(linearVelocity);
                    avatar.UpdateRotationalVelocity(angularVelocity);

                    // Indicate that the actor in question was found
                    actorFound = true;
                }
            }

            // If dynamic actor was an avatar, exit now
            if (actorFound)
                return;

            // Look through the list of primitives to see if there is one with
            // a matching ID
            lock (m_primitivesLock)
            {
                m_primitives.TryGetValue(actorID, out prim);
            }

            // Check to see if the desired primitive was found
            if (prim != null)
            {
               // Update the position and orientation of the 
               prim.UpdatePosition(position);
               prim.UpdateOrientation(orientation);
               prim.UpdateVelocity(linearVelocity);
               prim.UpdateRotationalVelocity(angularVelocity);

               // Indicate that this object has been updated during the
               // current step
               m_updatedObjects.Add(prim);
               m_lastUpdatedObjects.Remove(prim);
            }
        }

        /// <summary>
        /// Callback used by the remote physics engine to indicate an update
        /// to an actor's mass.
        /// </summary>
        /// <param name="actorID">The unique ID of the actor whose mass is
        /// being updated</param>
        /// <param name="mass">The new mass of the actor</param>
        protected void ActorMassUpdated(uint actorID, float mass)
        {
            HashSet<RemotePhysicsPrimitive> prims;
            RemotePhysicsPrimitive prim;

            // Look through the list of primitives to see if there is one with
            // a matching ID
            lock (m_primitivesLock)
            {
                m_primitives.TryGetValue(actorID, out prim);
            }

            // If a primitive was found, update its mass
            if (prim != null)
            {
               prim.UpdateMass(mass);
            }
        }

        /// <summary>
        /// Callback used by the remote physics engine to indicate that
        /// it has completed a time step.
        /// </summary>
        protected void TimeAdvanced()
        {
            RemotePhysicsPrimitive prim;

            // Update the time it took to complete the time step
            lock (m_frameTimeLock)
            {
                // Calculate the time it took to complete the time step in
                // seconds
                m_frameTime =
                    ((float) Util.EnvironmentTickCountSubtract(
                        m_frameTimeBegin)) / 1000.0f;
            }

            // Go through each of the objects that were not updated during this
            // step, but were updated during the previous step
            foreach (RemotePhysicsObject obj in m_lastUpdatedObjects)
            {
               // Zero out the velocities of the object to prevent extraneous
               // dead reckoning
               prim = obj as RemotePhysicsPrimitive;
               prim.UpdateVelocity(OpenMetaverse.Vector3.Zero);
               prim.UpdateRotationalVelocity(OpenMetaverse.Vector3.Zero);

               // Send an update of the physical properties of the primitive
               // to the simulator
               prim.RequestPhysicsterseUpdate();
            }

            // Clear the list of objects that were updated during the previous
            // step
            m_lastUpdatedObjects.Clear();

            // Add all the objects that were updated during the current step
            // into the list of objects that were updated during the previous
            // step, because the current step is now complete
            foreach (RemotePhysicsObject obj in m_updatedObjects)
            {
                m_lastUpdatedObjects.Add(obj);
            }
        }

        /// <summary>
        /// Callback used by the remote physics engine to indicate that two
        /// actors have collided.
        /// </summary>
        /// <param name="collidedActor">The unique identifier of the actor
        /// with which a collision has occurred</param>
        /// <param name="collidingActor">The unique identifier of the actor
        /// that has collided</param>
        /// <param name="contactPoint">The point at which contact was made in
        /// world space</param>
        /// <param name="contactNormal">The normal of the surfaces at the
        /// contact point</param>
        /// <param name="separation">The distance between the actors.
        /// Negative value means penetration</param>
        protected void ActorsCollided(uint collidedActor, uint collidingActor,
            OpenMetaverse.Vector3 contactPoint,
            OpenMetaverse.Vector3 contactNormal, float separation)
        {
            RemotePhysicsObject actor1;
            RemotePhysicsObject actor2;

            // Check to see if the first actor is the ground
            if (collidedActor <= TerrainID)
            {
                // Attempt to fetch the second actor
                PhysicsObjects.TryGetValue(collidingActor, out actor2);

                // Check to see if the actor was found
                if (actor2 != null)
                {
                    // Add the collision to the actor
                    // Note: Since this is a ground collision, no physics
                    // object needs to be sent
                    actor2.Collide(collidedActor, null, contactPoint,
                        contactNormal, separation);
                }
            }
            else if (collidingActor <= TerrainID)
            {
                // Attempt to fetch the first actor
                PhysicsObjects.TryGetValue(collidedActor, out actor1);

                // Check to see if the actor was found
                if (actor1 != null)
                {
                    // Add the collision to the actor
                    // Note: Since this is a ground collision, no physics
                    // object needs to be sent
                    actor1.Collide(collidingActor, null, contactPoint,
                        contactNormal, separation);
                }
            }
            else
            {
                // This is a collision between two objects, so attempt to
                // fetch both of them
                PhysicsObjects.TryGetValue(collidedActor, out actor1);
                PhysicsObjects.TryGetValue(collidingActor, out actor2);

                // Check to see if both of the actors were found
                if (actor1 != null && actor2 != null)
                {
                    // Call the collision with the both actors
                    actor1.Collide(collidingActor, actor2, contactPoint,
                        contactNormal, separation);
                    actor2.Collide(collidedActor, actor1, contactPoint,
                        contactNormal, separation);
                }
            }
        }
    }
}
