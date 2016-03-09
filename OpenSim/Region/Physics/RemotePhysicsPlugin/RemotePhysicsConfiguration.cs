
// TODO: Create a banner

using System;
using System.Collections.Generic;
using System.Reflection;
using System.Text;

using OpenMetaverse;
using Nini.Config;

using OpenSim.Region.Physics.Manager;


namespace OpenSim.Region.Physics.RemotePhysicsPlugin
{
    public class RemotePhysicsConfiguration
    {
        // The following section defines the various properties for
        // configuration values held within this object
        // NOTE: The configuration values are publicly accessible, but their
        // values are determined by the configuration source passed into this
        // object

        #region Configuration Properties

        // Indicates the communications address of the remote physics engine
        public String RemoteAddress { get; protected set; }

        // Indicates the communications port of the remote physics engine
        public int RemotePort { get; protected set; }

        // The ID of this simulation in the remote physics engine
        public uint SimulationID { get; protected set; }

        // Indicates the default period of time simulated by each iteration of
        //  the remote physics engine
        public float PhysicsTimeStep { get; protected set; }

        // Indicates the default friction value used in the remote physics
        // engine
        public float DefaultFriction { get; protected set; }

        // Indicates the default density used for objects in the remote
        // physics engine
        public float DefaultDensity { get; protected set; }

        // Indicates the default restitution used for objects in the remote
        // physics engine
        public float DefaultRestitution { get; protected set; }

        // Indicates the vertical force of the gravity being using in the
        // remote physics simulation
        public float Gravity { get; protected set; }

        // Indicates the default physical shape that should be used for avatars
        public RemotePhysicsShape AvatarShape { get; protected set; }

        // Indicates the height of the physical shape that is used for avatars
        // in the remote physics engine
        public float AvatarShapeHeight { get; protected set; }

        // Indicates the width of the physical shape that is used for avatars
        // in the remote physics engine
        public float AvatarShapeWidth { get; protected set; }

        // Indicates the depth of the physical shape that is used for avatars
        // in the remote physics engine
        public float AvatarShapeDepth { get; protected set; }

        // Indicates the static friction of the avatars in the remote
        // physics engine
        public float AvatarStaticFriction { get; protected set; }

        // Indicates the kinetic friction of the avatars in the remote physics
        // engine
        public float AvatarKineticFriction { get; protected set; }

        // Indicates the density of the avatars in the remote physics engine,
        // which is used to determine intertia and other physical attributes
        public float AvatarDensity { get; protected set; }

        // Indicates the restitution of the avatars in the remote physics
        // engine, which is used to determine the speeds of objects after
        // collision (i.e.: bounciness)
        public float AvatarRestitution { get; protected set; }

        // Indicates the velocity multiplier used for avatars, when they are
        // running
        public float AvatarRunFactor { get; protected set; }

        // Indicates the distance at which collision detection between objects
        // start; this helps with collision errors
        public float CollisionMargin { get; protected set; }

        // Indicates the height of the ground plane in the remote physics
        // engine; no object will fall below this height
        public float GroundPlaneHeight { get; protected set; }

        // Indicates the friction used for the terrain in the remote physics
        // engine
        public float TerrainFriction { get; protected set; }

        // Indidicates the restitution used for the terrain in the remote
        // physics engine (i.e.: bounciness)
        public float TerrainRestitution { get; protected set; }

        // Indicates the distance at which collision detection with the terrain
        // starts; this leaves some room for collision errors
        public float TerrainCollisionMargin { get; protected set; }

        // Indicates whether the packet manager used to communicate with the
        // remote physics engine
        // should use its own internal thread for updates
        public bool PacketManagerInternalThread { get; protected set; }

        /// <summary>
        /// Indicates whether the remote messenger used to send messages to
        /// the remote physics engine should use its own internal thread.
        /// </summary>
        public bool MessengerInternalThread { get; protected set; }

        /// <summary>
        /// Indicates the number of times an object may experience region
        /// boundary crossing failures before it is considered out of bounds.
        /// </summary>
        public int CrossingFailuresBeforeOutOfBounds { get; protected set; }

        /// <summary>
        /// Indicates whether collisions not involving avatars will get reported
        /// to the simulator.
        /// </summary>
        public bool ReportNonAvatarCollisions { get; protected set; }

        #endregion

        public RemotePhysicsConfiguration()
        {
            // Initialize all the properties using default values
            // All default values are explained in the Initialize(...) method
            RemoteAddress = "127.0.0.1";
            RemotePort = 30000;
            SimulationID = 0;
            PhysicsTimeStep = 0.89f;
            DefaultFriction = 0.2f;
            DefaultDensity = 7700.0f;
            DefaultRestitution = 0.0f;
            Gravity = -9.80665f;
            AvatarShape = RemotePhysicsShape.SHAPE_CAPSULE;
            AvatarShapeHeight = 1.5f;
            AvatarShapeWidth = 0.6f;
            AvatarShapeDepth = 0.45f;
            AvatarStaticFriction = 0.8f;
            AvatarKineticFriction = 0.6f;
            AvatarDensity = 1062.0f;
            AvatarRestitution = 0.0f;
            AvatarRunFactor = 1.5f;
            CollisionMargin = 0.04f;
            GroundPlaneHeight = -10.0f;
            TerrainFriction = 0.2f;
            TerrainRestitution = 0.0f;
            TerrainCollisionMargin = 0.04f;
            PacketManagerInternalThread = true;
            MessengerInternalThread = true;
            CrossingFailuresBeforeOutOfBounds = 5;
            ReportNonAvatarCollisions = true;
        }

        public void Initialize(IConfig config)
        {
            // Read in the address that is to be used to connect to the remote
            // physics engine; the default value is local host
            RemoteAddress = config.GetString("RemoteAddress", "127.0.0.1");

            // Read in the port number that is to be used to connect to the
            // remote physics engine; the default value is 30000
            RemotePort = config.GetInt("RemotePort", 30000);

            // Read in the ID that will be used to identify the simlation in
            // the remote physics engine
            SimulationID = (uint) config.GetInt("SimulationID", 0);

            // Read in the setting for the period of time that is simulated
            // by each step of the remote physics engine; the default value
            // is an 11th of a second
            PhysicsTimeStep = config.GetFloat("PhysicsTimeStep", 0.89f);

            // Read in the default coefficient of friction used for objects
            // in the remote physics simulation; default value is 0.2
            DefaultFriction = config.GetFloat("DefaultFriction", 0.2f);

            // Read in the default density used for objects in the remote
            // physics simulation; default value is 7700 kg/m^3 which is an
            // approximate density of 1 cubic meter of aluminium
            DefaultDensity = config.GetFloat("DefaultDensity", 7700.0f);

            // Read in the default restitution for objects in the remote
            // physics simulation; default value is 0, which means the object
            // will not bounce away after a collision
            DefaultRestitution = config.GetFloat("DefaultRestitution", 0.0f);

            // Read in the vertical component of the gravity used in the
            // remote physics engine; the default value is -9.80665 m/(s^2),
            // which is is an approximation of earth's gravity
            Gravity = config.GetFloat("Gravity", -9.80665f);

            // Read in the default physical shape used for avatars in the
            // remote physics engine; default value is capsule
            AvatarShape = (RemotePhysicsShape) config.GetInt("AvatarShape", 1);

            // Read in the default height for avatars in the remote physics
            // engine; default value is 1.5m
            AvatarShapeHeight = config.GetFloat("AvatarShapeHeight", 1.5f);

            // Read in the default width for avatars in the remote physics
            // engine; default value is 0.6m
            AvatarShapeWidth = config.GetFloat("AvatarShapeWidth", 0.6f);

            // Read in the default depth for avatars in the remote physics
            // engine; default value is 0.45m
            AvatarShapeDepth = config.GetFloat("AvatarShapeDepth", 0.45f);

            // Read in the default coefficient of static friction used for
            // avatars in the remote physics engine; the default value is 0.8
            AvatarStaticFriction = config.GetFloat("AvatarStaticFriction",
                0.8f);

            // Read in the default coefficient of kinetic friction used for
            // avatars in the remote physics engine; the default value is 0.6
            AvatarKineticFriction = config.GetFloat("AvatarKineticFriction",
                0.6f);

            // Read in the default density used for avatars in the remote
            // physics engine; the default value is 1062 kg/(m^3), which is
            // the density of an average human
            AvatarDensity = config.GetFloat("AvatarDensity", 1062.0f);

            // Read in the restitution used for avatars in the remote physics
            // engine; the default value is 0 (avtar's don't bounce)
            AvatarRestitution = config.GetFloat("AvatarRestitution", 0.0f);

            // Read in the velocity multiplier used for avatars, when they are
            // running; the efault value is 1.5 times the avatars normal
            // velocity
            AvatarRunFactor = config.GetFloat("AvatarRunFactor", 1.5f);

            // Read in the distance at which collisions between objects are 
            // detected in the remote physics engine; the default value is 0.04m
            CollisionMargin = config.GetFloat("CollisionMargin", 0.04f);

            // Read in the altitude of the ground plane, below which no object
            // will fall in the physics simulation; the default value is -500m
            GroundPlaneHeight = config.GetFloat("GroundPlaneHeight", -10.0f);

            // Read in the coefficient of friction used for interactions with 
            // the terrain in the remote physics engine; the default 
            // value is 0.3
            TerrainFriction = config.GetFloat("TerrainFriction", 0.3f);

            // Read in the restitution used for interactions with the
            // terrain in the remote physics engine; the default value is 0
            // (the terrain is not firm)
            TerrainRestitution = config.GetFloat("TerrainRestitution", 0.0f);

            // Read in the distance at which collisions between objects and the
            // terrain are detected in the remote physics engine; the default
            // value is 0.04m
            TerrainCollisionMargin = config.GetFloat("TerrainCollisionMargin",
                0.04f);

            // Read in whether the packet manager used for communicating with
            // the remote physics engine should use its own internal thread
            // for updates; the default value is false
            PacketManagerInternalThread = config.GetBoolean(
                "PacketManagerInternalThread", true);

            // Read in whether the remote messenger used to send messages to
            // the remote physics engine should use its own internal thread
            // for updates; the default value is true
            MessengerInternalThread = config.GetBoolean(
                "MessengerInternalThread", true);

            // Read in how many times an object may experience region boundary
            // crossing failures before it is considered out of bounds
            CrossingFailuresBeforeOutOfBounds = config.GetInt(
                "CrossingFailuresBeforeOutOfBounds", 5);

            // Read in whether collisions not involving avatars should be  
            // reported; the default value is true
            ReportNonAvatarCollisions = config.GetBoolean(
                "ReportNonAvatarCollisions", true);
        }
    }
}

