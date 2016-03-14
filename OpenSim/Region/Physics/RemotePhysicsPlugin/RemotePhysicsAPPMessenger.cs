
using System;
using System.IO;
using System.Net;
using System.Threading;
using System.Runtime.InteropServices;
using OpenSim.Framework.Monitoring;
using log4net;

namespace OpenSim.Region.Physics.RemotePhysicsPlugin
{
    public class RemotePhysicsAPPMessenger : IDisposable,
        IRemotePhysicsMessenger
    {
        #region Base Structures

        /// <summary>
        /// Structure representing the header for all ARCHIMEDES Physics
        /// Protocol messages (APP).
        /// </summary>
        protected struct APPHeader
        {
            public ushort version;
            public ushort msgType;
            public uint msgIndex;
            public uint length;
            public float timestamp;
            public uint reserved1;
            public uint reserved2;
        }

        /// <summary>
        /// The size of the APPHeader structure in bytes.
        /// </summary>
        protected static readonly int m_APPHeaderSize = sizeof(ushort) * 2 +
            sizeof(uint) * 4 + sizeof(float);


        /// <summary>
        /// The offset (in bytes) at which the packet length field is located.
        /// </summary>
        protected static readonly int m_APPPacketLengthOffset =
            sizeof(ushort) * 2 + sizeof(uint);
 
        /// <summary>
        /// APP structure representing an unique actor within a simulation.
        /// </summary>
        protected struct APPActorID
        {
            public uint simID;
            public uint actorID;
        }

        /// <summary>
        /// The size of the APPActorID strcuture in bytes.
        /// </summary>
        protected static readonly int m_APPActorIDSize = sizeof(uint) * 2;
 
        /// <summary>
        /// APP structure representing a 3-dimensional vector.
        /// </summary>
        protected struct APPVector
        {
            public float x;
            public float y;
            public float z;
        }

        /// <summary>
        /// The size of the APPVector structure in bytes.
        /// </summary>
        protected static readonly int m_APPVectorSize = sizeof(float) * 3;
 
        /// <summary>
        /// APP structure representing a quaternion (usually used to denote
        /// orientation).
        /// </summary>
        protected struct APPQuat
        {
            public float x;
            public float y;
            public float z;
            public float w;
        }

        /// <summary>
        /// The size of the APPQuat structure in bytes.
        /// </summary>
        protected static readonly int m_APPQuatSize = sizeof(float) * 4;
 
        /// <summary>
        /// APP structure containing a 3-tuple of vertex indices that is usually
        /// used to represent a triangle.
        /// </summary>
        protected struct APPIndexVector
        {
            public uint p1;
            public uint p2;
            public uint p3;
        }

        /// <summary>
        /// The size of the APPIndexVector structure in bytes.
        /// </summary>
        protected static readonly int m_APPIndexVectorSize = sizeof(uint) * 3;
 
        /// <summary>
        /// APP structure representing an unique shape within a simulation.
        /// </summary>
        protected struct APPShapeID
        {
            public uint simID;
            public uint shapeID;
        }

        /// <summary>
        /// The size of the APPShapeID structure in bytes.
        /// </summary>
        protected static readonly int m_APPShapeIDSize = sizeof(uint) * 2;

        /// <summary>
        /// APP Structure representing an unique joint within a simulation.
        /// </summary>
        protected struct APPJointID
        {
            public uint simID;
            public uint jointID;
        }

        /// <summary>
        /// The size of the APPJointID structure in bytes.
        /// </summary>
        protected static readonly int m_APPJointIDSize = sizeof(uint) * 2;
 
        /// <summary>
        /// APP structure used to describe physical properties.
        /// </summary>
        protected struct APPMaterial
        {
            public float density;
            public float coeffStaticFriction;
            public float coeffKineticFriction;
            public float coeffRestitution;
        }

        /// <summary>
        /// The size of the APPMaterial structure in bytes.
        /// </summary>
        protected static readonly int m_APPMaterialSize = sizeof(float) * 4;

        #endregion

        #region Actor Messages

        /// <summary>
        /// APP structure used to create a static actor (entity that
        /// doesn't move).
        /// </summary>
        protected struct APPCreateStaticActor
        {
            public APPHeader header;
            public APPActorID actor;
            public APPVector position;
            public APPQuat orientation;
            public uint flags;
        }

        /// <summary>
        /// The size of the APPCreateStaticActor structure in bytes.
        /// </summary>
        protected static readonly int m_APPCreateStaticActorSize =
            m_APPHeaderSize + m_APPActorIDSize + m_APPVectorSize +
            m_APPQuatSize + sizeof(uint);
 
        /// <summary>
        /// An instance of the create static actor message that will be used for
        /// sending out the corresponding APP message. This instance reduces
        /// the amount of memory allocation required for this frequent message.
        /// </summary>
        protected APPCreateStaticActor m_createStaticActor;

        /// <summary>
        /// The lock object that will ensure that the create static actor
        /// structure instance is thread-safe.
        /// </summary>
        protected object m_createStaticActorLock = new Object();

        /// <summary>
        /// APP structure used to create a dynamic actor (entity that moves).
        /// </summary>
        protected struct APPCreateDynamicActor
        {
            public APPHeader header;
            public APPActorID actor;
            public APPVector position;
            public APPQuat orientation;
            public float gravityModifier;
            public APPVector linearVelocity;
            public APPVector angularVelocity;
            public uint flags;
        }

        /// <summary>
        /// The size of the APPCreateDynamicActor structure in bytes.
        /// </summary>
        protected static readonly int m_APPCreateDynamicActorSize =
            m_APPHeaderSize + m_APPActorIDSize + m_APPVectorSize * 3 +
            m_APPQuatSize + sizeof(float) + sizeof(uint);
 
        /// <summary>
        /// An instance of the create dynamic actor message that will be
        /// used for sending out the corresponding APP message. This instance
        /// reduces the amount of memory allocation required for this
        /// frequent message.
        /// </summary>
        protected APPCreateDynamicActor m_createDynamicActor;

        /// <summary>
        /// The lock object that will ensure that the create dynamic actor
        /// structure instance is thread-safe.
        /// </summary>
        protected object m_createDynamicActorLock = new Object();

        /// <summary>
        /// APP structure used to describe a static actor (entity that
        /// doesn't move).
        /// </summary>
        protected struct APPSetStaticActor
        {
            public APPHeader header;
            public APPActorID actor;
            public APPVector position;
            public APPQuat orientation;
        }

        /// <summary>
        /// The size of the APPSetStaticActor structure in bytes.
        /// </summary>
        protected static readonly int m_APPSetStaticActorSize =
            m_APPHeaderSize + m_APPActorIDSize + m_APPVectorSize +
            m_APPQuatSize;

        /// <summary>
        /// An instance of the set static actor message that will be used for
        /// sending out the corresponding APP message. This instance reduces
        /// the amount of memory allocation required for this frequent message.
        /// </summary>
        protected APPSetStaticActor m_setStaticActor;

        /// <summary>
        /// The lock object that will ensure that the set static actor structure
        /// instance is thread-safe.
        /// </summary>
        protected object m_setStaticActorLock = new Object();
 
        /// <summary>
        /// APP structure used to describe a dynamic actor (entity that moves).
        /// </summary>
        protected struct APPSetDynamicActor
        {
            public APPHeader header;
            public APPActorID actor;
            public APPVector position;
            public APPQuat orientation;
            public float gravityModifier;
            public APPVector linearVelocity;
            public APPVector angularVelocity;
        }

        /// <summary>
        /// The size of the APPSetDynamicActor structure in bytes.
        /// </summary>
        protected static readonly int m_APPSetDynamicActorSize =
            m_APPHeaderSize + m_APPActorIDSize + m_APPVectorSize * 3 +
            m_APPQuatSize + sizeof(float);

        /// <summary>
        /// An instance of the set dynamic actor message that will be used for
        /// sending out the corresponding APP message. This instance reduces
        /// the amount of memory allocation required for this frequent message.
        /// </summary>
        protected APPSetDynamicActor m_setDynamicActor;

        /// <summary>
        /// The lock object that will ensure that the set static actor structure
        /// instance is thread-safe.
        /// </summary>
        protected object m_setDynamicActorLock = new Object();

        /// <summary>
        /// APP structure used to update an actor's position.
        /// </summary>
        protected struct APPUpdateActorPosition
        {
            public APPHeader header;
            public APPActorID actor;
            public APPVector position;
        }

        /// <summary>
        /// The size of the APPUpdateActorPosition structure in bytes.
        /// </summary>
        protected static readonly int m_APPUpdateActorPositionSize =
            m_APPHeaderSize + m_APPActorIDSize + m_APPVectorSize;

        /// <summary>
        /// APP structure used to update an actor's orientation.
        /// </summary>
        protected struct APPUpdateActorOrientation
        {
            public APPHeader header;
            public APPActorID actor;
            public APPQuat orientation;
        }

        /// <summary>
        /// The size of the APPUpdateActorOrientation structure in bytes.
        /// </summary>
        protected static readonly int m_APPUpdateActorOrientationSize =
            m_APPHeaderSize + m_APPActorIDSize + m_APPQuatSize;

        /// <summary>
        /// APP structure used to update a dynamic actor's gravity modifier.
        /// </summary>
        protected struct APPUpdateDynamicActorGravityModifier
        {
            public APPHeader header;
            public APPActorID actor;
            public float gravityModifier;
        }

        /// <summary>
        /// The size of the APPUpdateDynamicActorGravityModifier structure
        /// in bytes.
        /// </summary>
        protected static readonly int
            m_APPUpdateDynamicActorGravityModifierSize = m_APPHeaderSize +
            m_APPActorIDSize + sizeof(float);

        /// <summary>
        /// APP structure used to update a dynamic actor's linear velocity.
        /// </summary>
        protected struct APPUpdateDynamicActorLinearVelocity
        {
            public APPHeader header;
            public APPActorID actor;
            public APPVector linearVelocity;
        }

        /// <summary>
        /// The size of the APPUpdateDynamicActorLinearVelocity structure
        /// in bytes.
        /// </summary>
        protected static readonly int m_APPUpdateDynamicActorLinearVelocitySize
            = m_APPHeaderSize + m_APPActorIDSize + m_APPVectorSize;

        /// <summary>
        /// APP structure used to update a dynamic actor's angular velocity.
        /// </summary>
        protected struct APPUpdateDynamicActorAngularVelocity
        {
            public APPHeader header;
            public APPActorID actor;
            public APPVector angularVelocity;
        }

        /// <summary>
        /// The size of the APPUpdateDynamicActorAngularVelocity structure
        /// in bytes.
        /// </summary>
        protected static readonly int
            m_APPUpdateDynamicActorAngularVelocitySize = m_APPHeaderSize +
            m_APPActorIDSize + m_APPVectorSize;

        /// <summary>
        /// APP structure used to update a dynamic actor's mass.
        /// </summary>
        protected struct APPUpdateDynamicActorMass
        {
            public APPHeader header;
            public APPActorID actor;
            public float mass;
        }

        /// <summary>
        /// The size of the APPUpdateDynamicActorMass structure in bytes.
        /// </summary>
        protected static readonly int m_APPUpdateDynamicActorMassSize =
            m_APPHeaderSize + m_APPActorIDSize + sizeof(float);

        /// <summary>
        /// APP structure used to fetch an actor's mass.
        /// </summary>
        protected struct APPGetDynamicActorMass
        {
            public APPHeader header;
            public APPActorID actor;
        }

        /// <summary>
        /// The size of the APPGetDynamicActorMass structure in bytes.
        /// </summary>
        protected static readonly int m_APPGetDynamicActorMassSize =
            m_APPHeaderSize + m_APPActorIDSize;
 
        /// <summary>
        /// APP structure used to remove an actor from the remote
        /// physics engine.
        /// </summary>
        protected struct APPRemoveActor
        {
            public APPHeader header;
            public APPActorID actor;
        }

        /// <summary>
        /// The size of the APPRemoveActor structure in bytes.
        /// </summary>
        protected static readonly int m_APPRemoveActorSize = m_APPHeaderSize +
            m_APPActorIDSize;

        /// <summary>
        /// APP structure used to describe a collision between actors.
        /// </summary>
        protected struct APPActorsCollided
        {
            public APPHeader header;
            public APPActorID collidingActor;
            public APPActorID collidedActor;
            public APPVector contactPoint;
            public APPVector contactNormal;
            public float separation;
        }

        /// <summary>
        /// The size of the APPActorsCollided structure in bytes.
        /// </summary>
        protected static readonly int m_APPActorsCollidedSize =
            m_APPHeaderSize + m_APPActorIDSize * 2 + m_APPVectorSize * 2 +
            sizeof(float);

        #endregion

        #region Simulation Messages

        /// <summary>
        /// APP structure used to log into the remote physics engine.
        /// </summary>
        protected struct APPLogon
        {
            public APPHeader header;
            public uint simID;
            public string simName;
        }

        /// <summary>
        /// The maximum length of the simulation name in the APPLogon
        /// structure in characters.
        /// </summary>
        protected static readonly int m_APPSimNameSize = 48;

        /// <summary>
        /// The size of the APPLogon structure in bytes.
        /// </summary>
        protected static readonly int m_APPLogonSize = m_APPHeaderSize +
            sizeof(uint) + m_APPSimNameSize;

        /// <summary>
        /// APP structure used to indicate that a logon attempt has been
        /// processed by the remote physics engine.
        /// </summary>
        protected struct APPLogonReady
        {
            public APPHeader header;
            public uint simID;
        }

        /// <summary>
        /// The size of the APPLogonReady structure in bytes.
        /// </summary>
        protected static readonly int m_APPLogonReadySize = m_APPHeaderSize +
            sizeof(uint);

        /// <summary>
        /// APP structure used to log off from the remote physics engine.
        /// </summary>
        protected struct APPLogoff
        {
            public APPHeader header;
            public uint simID;
        }

        /// <summary>
        /// The size of the APPLogoff structure in bytes.
        /// <summary>
        protected static readonly int m_APPLogoffSize = m_APPHeaderSize +
            sizeof(uint);

        /// <summary>
        /// APP structure used to advance the simulation time in the
        /// remote physics engine.
        /// </summary>
        protected struct APPAdvanceTime
        {
            public APPHeader header;
            public uint simID;
            public float time;
        }

        /// <summary>
        /// The size of the APPAdvanceTime structure in bytes.
        /// </summary>
        protected static readonly int m_APPAdvanceTimeSize =
            m_APPHeaderSize + sizeof(uint) + sizeof(float);

        protected struct APPTimeAdvanced
        {
            public APPHeader header;
            public uint simID;
        }

        protected static readonly int m_APPTimeAdvancedSize =
            m_APPHeaderSize + sizeof(uint);

        /// <summary>
        /// APP structure that describes an error that occurred in the
        /// remote physics engine.
        /// </summary>
        protected struct APPError
        {
            public APPHeader header;
            public uint msgIndex;
            public char[] reason;
        }

        /// <summary>
        /// The maximum size of the reason field in the APPError structure.
        /// </summary>
        protected static readonly int m_APPReasonSize = 256;

        /// <summary>
        /// The size of the APPError structure in bytes.
        /// </summary>
        protected static readonly int m_APPErrorSize = m_APPHeaderSize +
            sizeof(uint) + sizeof(char) * m_APPReasonSize;
 
        /// <summary>
        /// APP structure used to describe the base parameters of the world
        /// in the remote physics engine.
        /// </summary>
        protected struct APPSetWorld
        {
            public APPHeader header;
            public APPActorID world;
            public APPVector gravity;
            public float coeffStaticFriction;
            public float coeffKineticFriction;
            public float coeffRestitution;
            public float collisionMargin;
            public float groundPlaneHeight;
            public APPVector groundPlaneNormal;
        }

        /// <summary>
        /// The size of the APPSetWorld structure int bytes.
        /// </summary>
        protected static readonly int m_APPSetWorldSize =
            m_APPHeaderSize + m_APPActorIDSize + m_APPVectorSize * 2 +
            sizeof(float) * 5;

        #endregion

        #region Joint Messages

        /// <summary>
        /// APP structure used to add a joint between two actors in the
        /// remote physics engine.
        /// </summary>
        protected struct APPAddJoint
        {
            public APPHeader header;
            public APPJointID joint;
            public APPActorID actor1;
            public APPQuat orientation1;
            public APPVector translation1;
            public APPActorID actor2;
            public APPQuat orientation2;
            public APPVector translation2;
            public APPVector linearLowerLimits;
            public APPVector linearUpperLimits;
            public APPVector angularLowerLimits;
            public APPVector angularUpperLimits;
        }

        /// <summary>
        /// The size of the APPAddJoint structure in bytes.
        /// </summary>
        protected static readonly int m_APPAddJointSize =
            m_APPHeaderSize + m_APPJointIDSize + m_APPActorIDSize * 2 +
            m_APPQuatSize * 2 + m_APPVectorSize * 6;

        /// <summary>
        /// APP structure used to remove a joint from the remote physics engine.
        /// </summary>
        protected struct APPRemoveJoint
        {
            public APPHeader header;
            public APPJointID joint;
        }

        /// <summary>
        /// The size of the APPRemoveJoint structure in bytes.
        /// </summary>
        protected static readonly int m_APPRemoveJointSize =
            m_APPHeaderSize + m_APPJointIDSize;

        #endregion

        #region Shape Messages

        /// <summary>
        /// APP structure used to add a sphere shape to the remote
        /// physics engine.
        /// </summary>
        protected struct APPAddSphere
        {
            public APPHeader header;
            public APPShapeID shape;
            public APPVector origin;
            public float radius;
        }

        /// <summary>
        /// The size of the APPAddSphere structure in bytes.
        /// </summary>
        protected static readonly int m_APPAddSphereSize =
            m_APPHeaderSize + m_APPShapeIDSize + m_APPVectorSize +
            sizeof(float);
 
        /// <summary>
        /// APP structure used to add a plane shape to the remote
        /// physics engine.
        /// </summary>
        protected struct APPAddPlane
        {
            public APPHeader header;
            public APPShapeID shape;
            public APPVector planeNormal;
            public float planeConstant;
        }

        /// <summary>
        /// The size of the APPAddPlane structure in bytes.
        /// </summary>
        protected static readonly int m_APPAddPlaneSize =
            m_APPHeaderSize + m_APPShapeIDSize + m_APPVectorSize +
            sizeof(float);
 
        /// <summary>
        /// APP strcuture used to add a capsule shape to the remote
        /// physics engine.
        /// </summary>
        protected struct APPAddCapsule
        {
            public APPHeader header;
            public APPShapeID shape;
            public float radius;
            public float height;
        }

        /// <summary>
        /// The size of the APPAddCapsule structure in bytes.
        /// </summary>
        protected static readonly int m_APPAddCapsuleSize = m_APPHeaderSize +
            m_APPShapeIDSize + sizeof(float) * 2;
 
        /// <summary>
        /// APP structure used to add a box shape to the remote physics engine.
        /// </summary>
        protected struct APPAddBox
        {
            public APPHeader header;
            public APPShapeID shape;
            public float length;
            public float width;
            public float height;
        }

        /// <summary>
        /// The size of the APPAddBox structure in bytes.
        /// </summary>
        protected static readonly int m_APPAddBoxSize = m_APPHeaderSize +
            m_APPShapeIDSize + sizeof(float) * 3;
 
        /// <summary>
        /// APP structure used to add a convex mesh shape to the remote
        /// physics engine.
        /// </summary>
        protected struct APPAddConvexMesh
        {
            public APPHeader header;
            public APPShapeID shape;
            public uint numPoints;
            public APPVector[] points;
        }
 
        /// <summary>
        /// APP structure used to add a triangle mesh shape to the remote
        /// physics engine.
        /// </summary>
        protected struct APPAddTriangleMesh
        {
            public APPHeader header;
            public APPShapeID shape;
            public uint numPoints;
            public uint numTriangles;
            public APPIndexVector[] triangles;
            public APPVector[] points;
        }

        /// <summary>
        /// APP structure used to add a heightfield to the remote physics
        /// engine.
        /// </summary>
        protected struct APPAddHeightField
        {
            public APPHeader header;
            public APPShapeID shape;
            public uint numRows;
            public uint numColumns;
            public float rowSpacing;
            public float columnSpacing;
            public float[] posts;
        }

        /// <summary>
        /// APP strucutre used to remove a shape from the remote physics engine.
        /// </summary>
        protected struct APPRemoveShape
        {
            public APPHeader header;
            public APPShapeID shape;
        }

        /// <summary>
        /// The size of the APPRemoveShape structure in bytes.
        /// </summary>
        protected static readonly int m_APPRemoveShapeSize = m_APPHeaderSize +
            m_APPShapeIDSize;

        /// <summary>
        /// APP structure used to attach a shape from an actor in the
        /// remote physics engine.
        /// </summary>
        protected struct APPAttachShape
        {
            public APPHeader header;
            public APPActorID actor;
            public APPShapeID shape;
            public APPMaterial material;
            public APPQuat orientation;
            public APPVector translation;
        }

        /// <summary>
        /// The size of the APPAttachShape structure in bytes.
        /// </summary>
        protected static readonly int m_APPAttachShapeSize =
            m_APPHeaderSize + m_APPActorIDSize + m_APPShapeIDSize +
            m_APPMaterialSize + m_APPVectorSize + m_APPQuatSize;

        /// <summary>
        /// APP structure used to update the physical properties of a shape
        /// attached to an actor in the remote physics engine.
        /// </summary>
        protected struct APPUpdateShapeMaterial
        {
            public APPHeader header;
            public APPActorID actor;
            public APPShapeID shape;
            public APPMaterial material;
        }

        /// <summary>
        /// The size of the APPUpdateShapeMaterial structure in bytes.
        /// </summary>
        protected static readonly int m_APPUpdateShapeMaterialSize =
            m_APPHeaderSize + m_APPActorIDSize + m_APPShapeIDSize +
            m_APPMaterialSize;

        /// <summary>
        /// APP structure used to detach a shape from an actor in the
        /// remote physics engine.
        /// </summary>
        protected struct APPDetachShape
        {
            public APPHeader header;
            public APPActorID actor;
            public APPShapeID shape;
        }

        /// <summary>
        /// The size of the APPDetachShape structure in bytes.
        /// </summary>
        protected static readonly int m_APPDetachShapeSize =
            m_APPHeaderSize + m_APPActorIDSize + m_APPShapeIDSize;

        /// <summary>
        /// APP structure used to apply a force to an actor in the remote
        /// physics engine.
        /// </summary>
        protected struct APPApplyForce
        {
            public APPHeader header;
            public APPActorID actor;
            public APPVector force;
        }

        /// <summary>
        /// The size of the APPApplyForce structure in bytes.
        /// </summary>
        protected static readonly int m_APPApplyForceSize =
            m_APPHeaderSize + m_APPActorIDSize + m_APPVectorSize;

        /// <summary>
        /// APP structure used to apply torque to an actor in the remote
        /// physics engine.
        /// </summary>
        protected struct APPApplyTorque
        {
            public APPHeader header;
            public APPActorID actor;
            public APPVector torque;
        }

        /// <summary>
        /// The size of the APPApplyTorque structure in bytes.
        /// </summary>
        protected static readonly int m_APPApplyTorqueSize =
            m_APPHeaderSize + m_APPActorIDSize + m_APPVectorSize;

        #endregion

        /// <summary>
        /// The logger to be used for this class.
        /// </summary>
        internal static readonly ILog m_log = LogManager.GetLogger(
            System.Reflection.MethodBase.GetCurrentMethod().DeclaringType);

        /// <summary>
        /// The tag used to denote log messages from this class.
        /// </summary>
        internal static readonly string LogHeader = "[REMOTE APP MESSENGER]";

        /// <summary>
        /// Indicates the ARCHIMEDES Physics Protocol version used by
        /// this messenger.
        /// </summary>
        protected readonly short m_protocolVersion = 1;

        /// <summary>
        /// Enumeration that denotes the type of an ARCHIMEDES Physics
        /// Protocol message.
        /// </summary>
        protected enum MessageType : ushort
        {
            Unknown = 0,
            Error = 1,
            Logon = 11,
            LogonReady = 12,
            Logoff = 13,
            AdvanceTime = 14,
            TimeAdvanced = 15,
            SetWorld = 101,
            CreateStaticActor = 102,
            CreateDynamicActor = 103,
            SetStaticActor = 104,
            SetDynamicActor = 105,
            UpdateActorPosition = 106,
            UpdateActorOrientation = 107,
            DynamicActorUpdateGravityModifier = 108,
            DynamicActorUpdateLinearVelocity = 109,
            DynamicActorUpdateAngularVelocity = 110,
            DynamicActorUpdateMass = 111,
            GetDynamicActorMass = 112,
            RemoveActor = 113,
            AddJoint = 201,
            RemoveJoint = 202,
            AddSphere = 301,
            AddPlane = 302,
            AddCapsule = 303,
            AddBox = 304,
            AddConvexMesh = 305,
            AddTriangleMesh = 306,
            AddHeightField = 307,
            RemoveShape = 308,
            AttachShape = 309,
            UpdateShapeMaterial = 310,
            DetachShape = 311,
            ActorsCollided = 401,
            ApplyForce = 402,
            ApplyTorque = 403
        }

        /// <summary>
        /// The packet manager used to send and receive messages to and from
        /// the remote physics engine.
        /// </summary>
        protected IRemotePhysicsPacketManager m_packetManager = null;

        /// <summary>
        /// The packet manager used to send and receive message to and from
        /// the remote physics engine over UDP.
        /// </summary>
        protected IRemotePhysicsPacketManager m_udpPacketManager = null;

        /// <summary>
        /// Denotes the next free message index to be used. This is used to
        /// uniquely identify messages sent from this messenger, as well as
        /// denote their order.
        /// </summary>
        protected short m_currentMessageIndex;

        /// <summary>
        /// Indicates that this messenger has been initialized, and is ready
        /// to process messages.
        /// </summary>
        protected bool m_isInitialized = false;

        /// <summary>
        /// Indicates whether this messenger should use its own internal
        /// thread for updates.
        /// </summary>
        protected bool m_useInternalThread = true;

        /// <summary>
        /// The thread that may be used for internal updates of this messenger.
        /// </summary>
        protected Thread m_updateThread;

        /// <summary>
        /// Indicates whether the internal update thread should (if one is
        /// being used).
        /// </summary>
        protected bool m_stopUpdates = false;

        /// <summary>
        /// The ID used to represent this simulation in the remote
        /// physics engine.
        /// </summary>
        protected uint m_simulationID;

        /// <summary>
        /// The event that will be used to implement the RemoteEngineError
        /// event of the messenger interface.
        /// </summary>
        public event ErrorCallbackHandler OnErrorEvent;

        /// <summary>
        /// The event that will be used to implement the LogonReady event of
        /// the messenger interface.
        /// </summary>
        public event LogonReadyHandler OnLogonReadyEvent;

        /// <summary>
        /// The event that will be used to implement the DynamicActorUpdated
        /// event of the messenger interface.
        /// </summary>
        public event UpdateDynamicActorHandler OnDynamicActorUpdateEvent;

        /// <summary>
        /// The event that will be used to implement the StaticActorUpdated
        /// event of the messenger interface.
        /// </summary>
        public event UpdateStaticActorHandler OnStaticActorUpdateEvent;

        /// <summary>
        /// The event that will be used to implement the ActorsCollided event
        /// of the messenger interface.
        /// </summary>
        public event ActorsCollidedHandler OnActorsCollidedEvent;

        /// <summary>
        /// The event that will be used to implement the TimeAdvanced event
        /// of the messenger interface.
        /// </summary>
        public event TimeAdvancedHandler OnTimeAdvancedEvent;

        /// <summary>
        /// The event that will be used to implement the
        /// DynamicActorMassUpdated event of the message interface.
        /// </summary>
        public event UpdateDynamicActorMassHandler
            OnDynamicActorMassUpdateEvent;

        /// <summary>
        /// The object that acts as a mutex for thread-safe access of the
        /// events in this messenger.
        /// </summary>
        protected object m_eventLock = new Object();

        /// <summary>
        /// The maximum number of packets that should be processed in one
        /// update loop.
        /// </summary>
        protected static readonly int m_maxPackets = 5000;

        /// <summary>
        /// Indicates whether the packet manager is using its own thread for
        /// updates.
        /// </summary>
        protected bool m_packetManagerInternalThread = false;

        /// <summary>
        /// Buffer used to convert deserialize floating point values from the
        /// remote physics engine.
        /// </summary>
        protected byte[] m_floatArray = new byte[sizeof(float)];

        /// <summary>
        /// Closes threads opened by the messenger.
        /// </summary>
        public void Dispose()
        {
            // Stop the update thread for the messenger
            if (m_useInternalThread && m_updateThread != null)
            {
                // Wait half a second, so that any remaining messages get
                // processed.
                m_updateThread.Join(500);
            }
        }

        /// <summary>
        /// Initializes this messenger and readies it to send and receive
        /// messages from the remote physics engine.
        /// </summary>
        /// <param name="config">The configuration parameters for this
        /// messenger</param>
        /// <param name="packetManager">The packet manager that is used to send
        /// and receive messages to and from the remote physics engine</param>
        /// <param name="udpPacketManager">The packet manager that is used to
        /// send and receive messages to and from the remote physics engine
        /// over UDP</param>
        public void Initialize(RemotePhysicsConfiguration config,
            IRemotePhysicsPacketManager packetManager,
            IRemotePhysicsPacketManager udpPacketManager)
        {
            // Initialize the packet managers that will allow this messenger
            // to communicate with the remote manager
            m_packetManager = packetManager;
            m_udpPacketManager = udpPacketManager;

            // Reset the indexing that should be used for messages sent
            // from this messenger
            m_currentMessageIndex = 0;

            // Check to see if this messenger should use its own internal
            // thread for updates
            m_useInternalThread = config.MessengerInternalThread;

            // Check to see if the give packet manager is valid
            if (m_packetManager != null)
            {
                // Set the packet header size and length offset
                m_packetManager.InitializePacketParameters(
                    (uint)m_APPHeaderSize, (uint)m_APPPacketLengthOffset);

                // Check to see whether the packet manager will be using its
                // own internal thread for updates
                m_packetManagerInternalThread =
                    config.PacketManagerInternalThread;
            }

            // Initialize the static actor message structure that will be used
            // to reduce allocations when sending out the message
            m_createStaticActor = new APPCreateStaticActor();
            m_createStaticActor.header = new APPHeader();
            m_createStaticActor.actor = new APPActorID();
            m_createStaticActor.position = new APPVector();
            m_createStaticActor.orientation = new APPQuat();
            m_createDynamicActor = new APPCreateDynamicActor();
            m_createDynamicActor.header = new APPHeader();
            m_createDynamicActor.position = new APPVector();
            m_createDynamicActor.orientation = new APPQuat();
            m_createDynamicActor.linearVelocity = new APPVector();
            m_createDynamicActor.angularVelocity = new APPVector();
            m_setStaticActor = new APPSetStaticActor();
            m_setStaticActor.header = new APPHeader();
            m_setStaticActor.actor = new APPActorID();
            m_setStaticActor.position = new APPVector();
            m_setStaticActor.orientation = new APPQuat();
            m_setDynamicActor = new APPSetDynamicActor();
            m_setDynamicActor.header = new APPHeader();
            m_setDynamicActor.position = new APPVector();
            m_setDynamicActor.orientation = new APPQuat();
            m_setDynamicActor.linearVelocity = new APPVector();
            m_setDynamicActor.angularVelocity = new APPVector();

            // If the messenger is supposed to manage its own update thread,
            // create the thread and ensure it runs
            if (m_useInternalThread)
            {
                m_stopUpdates = false;
                m_updateThread = WorkManager.StartThread(
                    new ThreadStart(RunUpdate), "RemotePhysics",
                    ThreadPriority.Normal, true, true);
            }

            // Initialization is now complete
            m_isInitialized = true;
        }

        /// <summary>
        /// Sends a logon message to the remote physics engine.
        /// </summary>
        /// <param name="simID">The unique ID that will be used to identify
        /// this simulation in the remote physics engine</param>
        /// <param name="simulationName">The name of this simulation in the
        /// remote physics engine</param>
        public void Logon(uint simID, string simulationName)
        {
            APPLogon logonMsg;
            byte[] logonArray;
            byte[] tempArray;
            int offset;
            int copyLength;

            // Check to see if the messenger has been initialized; if it has
            // not, exit out
            if (!m_isInitialized)
                return;

            // Record the ID for later use
            m_simulationID = simID;

            // Initialize the message and its header
            logonMsg = new APPLogon();
            logonMsg.header = new APPHeader();

            // Initialize the header with the message type
            logonMsg.header.msgType = (ushort)IPAddress.HostToNetworkOrder(
                (short)MessageType.Logon);

            // Initialize the rest of the header
            InitializeAPPHeader(ref logonMsg.header, (uint)m_APPLogonSize);

            // Set the body of the message using the given parameters
            logonMsg.simID = (uint)IPAddress.HostToNetworkOrder((int)simID);

            // Make sure to only get the first 48 charcters of the name as 48
            // is the maximum number of characters for the name in APP
            copyLength = Math.Min(m_APPSimNameSize, simulationName.Length);
            logonMsg.simName = simulationName.Substring(0, copyLength);

            // Convert the message into a byte array, so that it can be sent
            // to the remote physics engine
            // Start by allocating the byte array
            logonArray = new byte[m_APPLogonSize];

            // Convert the header into its byte representation
            ConvertHeaderToBytes(logonMsg.header, ref logonArray, 0);

            // Convert the simulation ID
            offset = m_APPHeaderSize;
            tempArray = BitConverter.GetBytes(logonMsg.simID);
            Buffer.BlockCopy(tempArray, 0, logonArray, offset, sizeof(uint));
            offset += sizeof(uint);

            // Convert the simulation name
            Buffer.BlockCopy(logonMsg.simName.ToCharArray(), 0, logonArray,
                offset, tempArray.Length);

            // Now that the byte array has been constructed, send it to
            // the remote physics engine
            m_packetManager.SendPacket(logonArray);
            m_udpPacketManager.SendPacket(logonArray);

            // Increment the message index, now that the message has been sent
            m_currentMessageIndex++;
        }

        /// <summary>
        /// Sends a logoff message to the remote physics engine.
        /// </summary>
        /// <param name="simID">The unique ID of the simulation</param>
        public void Logoff(uint simID)
        {
            APPLogoff logoffMsg;
            byte[] logoffArray;
            byte[] tempArray;

            // Check to see if the messenger has been initialized; if it has
            // not, exit out
            if (!m_isInitialized)
                return;

            // Initialize the message and its header
            logoffMsg = new APPLogoff();
            logoffMsg.header = new APPHeader();

            // Initialize the header with the message type
            logoffMsg.header.msgType = (ushort) IPAddress.HostToNetworkOrder(
                (short) MessageType.Logoff);

            // Initialize the rest of the header
            InitializeAPPHeader(ref logoffMsg.header, (uint) m_APPLogoffSize);

            // Set the simulation ID of the message using the given parameter
            logoffMsg.simID = (uint) IPAddress.HostToNetworkOrder((int) simID);

            // Convert the message into a byte array, so that it can be sent
            // to the remote physics engine
            // Start by allocating the byte array
            logoffArray = new byte[m_APPLogoffSize];

            // Convert the header into its byte representation
            ConvertHeaderToBytes(logoffMsg.header, ref logoffArray, 0);

            // Convert the simulation ID
            tempArray = BitConverter.GetBytes(logoffMsg.simID);
            Buffer.BlockCopy(tempArray, 0, logoffArray, m_APPHeaderSize,
                sizeof(uint));

            // Now that the byte array has been constructed, send it to the
            // remote physics engine
            m_packetManager.SendPacket(logoffArray);

            // Increment the message index, now that the message has been sent
            m_currentMessageIndex++;
        }

        /// <summary>
        /// Sets up the basic parameters of the world in the remote
        /// physics engine.
        /// </summary>
        /// <param name="gravity">A vector denoting the gravity of the world
        /// in meters/(second^2)</param>
        /// <param name="staticFriction">The coefficient of static friction of
        /// the ground plane</param>
        /// <param name="kineticFriction">The coefficient of kinetic friction
        /// of the ground plane</param>
        /// <param name="collisionMargin">The distance at which actors are
        /// considered colliding (collision tolerance) in meters</param>
        /// <param name="groundPlaneID">The unique identifier for the ground
        /// plane</param>
        /// <param name="groundPlaneHeight">The height ground plane in
        /// meters</param>
        /// <param name="groundPlaneNormal">The normal of the
        /// ground plane</param>
        public void InitializeWorld(OpenMetaverse.Vector3 gravity, 
            float staticFriction, float kineticFriction, float restitution,   
            float collisionMargin, uint groundPlaneID, float groundPlaneHeight,
            OpenMetaverse.Vector3 groundPlaneNormal)
        {
            APPSetWorld setWorldMsg;
            byte[] setWorldArray;
            int offset;

            // Check to see if the messenger has been initialized; if it has
            // not, exit out
            if (!m_isInitialized)
                return;

            // Initialize the message and its header
            setWorldMsg = new APPSetWorld();
            setWorldMsg.header = new APPHeader();

            // Initialize the header with the message type
            setWorldMsg.header.msgType = (ushort)IPAddress.HostToNetworkOrder(
                (short)MessageType.SetWorld);

            // Initialize the rest of the header
            InitializeAPPHeader(ref setWorldMsg.header,
                (uint)m_APPSetWorldSize);

            // Set the actual body of message using the given parameters
            setWorldMsg.world = new APPActorID();
            setWorldMsg.world.actorID = (uint)IPAddress.HostToNetworkOrder(
                (int)groundPlaneID);
            setWorldMsg.world.simID = (uint)IPAddress.HostToNetworkOrder(
                (int)m_simulationID);
            setWorldMsg.gravity = new APPVector();
            setWorldMsg.gravity.x = gravity.X;
            setWorldMsg.gravity.y = gravity.Y;
            setWorldMsg.gravity.z = gravity.Z;
            setWorldMsg.coeffStaticFriction = staticFriction;
            setWorldMsg.coeffKineticFriction = kineticFriction;
            setWorldMsg.coeffRestitution = restitution;
            setWorldMsg.collisionMargin = collisionMargin;
            setWorldMsg.groundPlaneHeight = groundPlaneHeight;
            setWorldMsg.groundPlaneNormal.x = groundPlaneNormal.X;
            setWorldMsg.groundPlaneNormal.y = groundPlaneNormal.Y;
            setWorldMsg.groundPlaneNormal.z = groundPlaneNormal.Z;

            // Convert the message into a byte array, so that it can be sent
            // to the remote physics engine
            // Start by allocating the byte array
            setWorldArray = new byte[m_APPSetWorldSize];

            // Convert the header to its byte array representation
            ConvertHeaderToBytes(setWorldMsg.header, ref setWorldArray, 0);

            // Convert the simulation ID
            offset = m_APPHeaderSize;
            offset += ConvertAPPActorIDToBytes(setWorldMsg.world,
                ref setWorldArray, offset);

            // Convert the gravity
            offset += ConvertAPPVectorToBytes(setWorldMsg.gravity,
                ref setWorldArray, offset);

            // Convert the coefficients of friction and restitution
            FloatToNetworkOrder(setWorldMsg.coeffStaticFriction,
                ref setWorldArray, offset);
            offset += sizeof(float);
            FloatToNetworkOrder(setWorldMsg.coeffKineticFriction,
                ref setWorldArray, offset);
            offset += sizeof(float);
            FloatToNetworkOrder(setWorldMsg.coeffRestitution,
                ref setWorldArray, offset);
            offset += sizeof(float);

            // Convert the collision margin
            FloatToNetworkOrder(setWorldMsg.collisionMargin,
                ref setWorldArray, offset);
            offset += sizeof(float);

            // Convert the ground plane parameters
            FloatToNetworkOrder(setWorldMsg.groundPlaneHeight,
                ref setWorldArray, offset);
            offset += sizeof(float);
            ConvertAPPVectorToBytes(setWorldMsg.groundPlaneNormal,
                ref setWorldArray, offset);

            // Now that the byte array has been constructed, send it to
            // the remote engine
            m_packetManager.SendPacket(setWorldArray);

            // Increment the message index now that a message has been sent
            m_currentMessageIndex++;
        }

        /// <summary>
        /// Creates a static actor in the remote physics engine.
        /// </summary>
        /// <param name="actorID">The unique ID of the actor</param>
        /// <param name="position">The position of the actor</param>
        /// <param name="orientation">The orientation of the actor</param>
        /// <param name="reportCollisions">Indicates whether collisions
        /// involving the actor should be reported</param>
        public void CreateStaticActor(uint actorID,
            OpenMetaverse.Vector3 position,
            OpenMetaverse.Quaternion orientation, bool reportCollisions)
        {
            byte[] staticActorArray;
            int offset;
            ushort tempValue;
            byte[] tempArray;

            // Check to see if the messenger has been initialized; if it has
            // not, exit out
            if (!m_isInitialized)
                return;

            // Ensure that access to the set static actor structure is
            // thread-safe
            lock (m_createStaticActorLock)
            {
                // Initialize the header with the message type
                tempValue = (ushort)MessageType.CreateStaticActor;
                m_createStaticActor.header.msgType = (ushort)IPAddress.
                    HostToNetworkOrder((short)tempValue);
 
                // Initialize the rest of the header
                InitializeAPPHeader(ref m_createStaticActor.header,
                    (uint)m_APPCreateStaticActorSize);
 
                // Set the body of the message using the given parameters
                m_createStaticActor.actor.actorID =
                    (uint)IPAddress.HostToNetworkOrder((int)actorID);
                m_createStaticActor.actor.simID =
                    (uint)IPAddress.HostToNetworkOrder((int)m_simulationID);
                m_createStaticActor.position.x = position.X;
                m_createStaticActor.position.y = position.Y;
                m_createStaticActor.position.z = position.Z;
                m_createStaticActor.orientation.x = orientation.X;
                m_createStaticActor.orientation.y = orientation.Y;
                m_createStaticActor.orientation.z = orientation.Z;
                m_createStaticActor.orientation.w = orientation.W;

                // Initialize the actor flags field to its default value
                m_createStaticActor.flags = 0;
 
                // Add the report collisions flag to the flags field
                // The report collisions flag is the least significant bit
                // of the flags field
                if (reportCollisions)
                    m_createStaticActor.flags |= 1 << 0;

                // Convert the flags to network byte order, so that it
                // can be properly deserialized by the remote physics engine
                m_createStaticActor.flags =
                    (uint)IPAddress.HostToNetworkOrder(
                    m_createStaticActor.flags);

                // Convert the message to a byte array, so that it can be sent
                // to the remote physics engine
                // Start by allocating the byte array
                staticActorArray = new byte[m_APPCreateStaticActorSize];
 
                // Convert the header to its byte array representation
                ConvertHeaderToBytes(m_createStaticActor.header,
                   ref staticActorArray, 0);
 
                // Convert the actor ID
                offset = m_APPHeaderSize;
                offset += ConvertAPPActorIDToBytes(m_createStaticActor.actor,
                    ref staticActorArray, offset);
 
                // Convert the position and orientation
                offset += ConvertAPPVectorToBytes(m_createStaticActor.position,
                    ref staticActorArray, offset);
                offset += ConvertAPPQuaternionToBytes(
                    m_createStaticActor.orientation, ref staticActorArray,
                    offset);

                // Convert the report collisions flag
                tempArray = BitConverter.GetBytes(m_createStaticActor.flags);
                Buffer.BlockCopy(tempArray, 0, staticActorArray, offset,
                    sizeof(uint));
            }

            // Now that the byte array has been constructed, send it to the
            // remote physics engine
            m_packetManager.SendPacket(staticActorArray);

            // Increment the message index now that a message has been sent
            m_currentMessageIndex++;
        }

        /// <summary>
        /// Creates a dynamic actor in the remote physics engine.
        /// </summary>
        /// <param name="actorID">The unique ID of the actor</param>
        /// <param name="position">The position of the actor</param>
        /// <param name="orientation">The orientation of the actor</param>
        /// <param name="gravityModifier">The multiplier for the gravity
        /// acting on this object</param>
        /// <param name="linearVelocity">The linear velocity of the actor
        /// (in meters/second)</param>
        /// <param name="angularVelocity">The angular velocity of the actor
        /// (in meters/second)</param>
        /// <param name="reportCollisions">Indicates whether collisions
        /// involving the actor should be reported</param>
        public void CreateDynamicActor(uint actorID,
            OpenMetaverse.Vector3 position,
            OpenMetaverse.Quaternion orientation, float gravityModifier,
            OpenMetaverse.Vector3 linearVelocity,
            OpenMetaverse.Vector3 angularVelocity, bool reportCollisions)
        {
            byte[] dynamicActorArray;
            ushort tempValue;
            int offset;
            byte[] tempArray;

            // Check to see if the messenger has been initialized; if it has
            // not, exit out
            if (!m_isInitialized)
                return;

            // Ensure that access to the set dynamic actor structure is
            // thread-safe
            lock (m_createDynamicActorLock)
            {
                // Initialize the header with the message type
                tempValue = (ushort)MessageType.CreateDynamicActor;
                m_createDynamicActor.header.msgType =
                    (ushort)IPAddress.HostToNetworkOrder((short)tempValue);
 
                // Initialize the rest of the header
                InitializeAPPHeader(ref m_createDynamicActor.header,
                    (uint)m_APPCreateDynamicActorSize);
 
                // Set the body of the message using the given parameters
                m_createDynamicActor.actor.actorID =
                    (uint)IPAddress.HostToNetworkOrder((int)actorID);
                m_createDynamicActor.actor.simID =
                    (uint)IPAddress.HostToNetworkOrder((int)m_simulationID);
                m_createDynamicActor.position.x = position.X;
                m_createDynamicActor.position.y = position.Y;
                m_createDynamicActor.position.z = position.Z;
                m_createDynamicActor.orientation.x = orientation.X;
                m_createDynamicActor.orientation.y = orientation.Y;
                m_createDynamicActor.orientation.z = orientation.Z;
                m_createDynamicActor.orientation.w = orientation.W;
                m_createDynamicActor.gravityModifier = gravityModifier;
                m_createDynamicActor.linearVelocity.x = linearVelocity.X;
                m_createDynamicActor.linearVelocity.y = linearVelocity.Y;
                m_createDynamicActor.linearVelocity.z = linearVelocity.Z;
                m_createDynamicActor.angularVelocity.x = angularVelocity.X;
                m_createDynamicActor.angularVelocity.y = angularVelocity.Y;
                m_createDynamicActor.angularVelocity.z = angularVelocity.Z;

                // Initialize the actor flags field to its default value
                m_createDynamicActor.flags = 0;

                // Add the report collisions flag to the flags field
                // The report collisions flag is the least significant bit
                // of the flags field
                if (reportCollisions)
                    m_createDynamicActor.flags = 1 << 0;

                // Convert the flags to network byte order, so that it
                // can be properly deserialized by the remote physics engine
                m_createDynamicActor.flags =
                    (uint)IPAddress.HostToNetworkOrder(
                    (int)m_createDynamicActor.flags);
 
                // Convert the message into a byte array, so that it can
                // be sent to the remote physics engine
                // Start by allocating the array
                dynamicActorArray = new byte[m_APPCreateDynamicActorSize];
 
                // Convert the header to its byte array representation
                ConvertHeaderToBytes(m_createDynamicActor.header,
                    ref dynamicActorArray, 0);
 
                // Convert the actor ID
                offset = m_APPHeaderSize;
                offset += ConvertAPPActorIDToBytes(m_createDynamicActor.actor,
                    ref dynamicActorArray, offset);
 
                // Convert the position
                offset += ConvertAPPVectorToBytes(m_createDynamicActor.position,
                    ref dynamicActorArray, offset);
 
                // Convert the orientation
                offset += ConvertAPPQuaternionToBytes(
                    m_createDynamicActor.orientation,
                    ref dynamicActorArray, offset);
 
                // Convert the gravity modifier
                FloatToNetworkOrder(m_createDynamicActor.gravityModifier,
                    ref dynamicActorArray, offset);
                offset += sizeof(float);
 
                // Convert the linear and angular velocities
                offset += ConvertAPPVectorToBytes(
                    m_createDynamicActor.linearVelocity,
                    ref dynamicActorArray, offset);
                offset += ConvertAPPVectorToBytes(
                    m_createDynamicActor.angularVelocity,
                    ref dynamicActorArray, offset);

                // Convert the report collisions flag
                tempArray = BitConverter.GetBytes(m_createDynamicActor.flags);
                Buffer.BlockCopy(tempArray, 0, dynamicActorArray, offset,
                    sizeof(uint));
            }

            // Now that the byte array has been constructed, send it to the
            // remote physics engine
            m_packetManager.SendPacket(dynamicActorArray);

            // Increment the message index now that a message has been sent
            m_currentMessageIndex++;
        }


        /// <summary>
        /// Updates the state of a static actor in the remote physics engine.
        /// </summary>
        /// <param name="actorID">The unique ID of the actor</param>
        /// <param name="position">The new position of the actor</param>
        /// <param name="orientation">The new orientation of the actor</param>
        public void SetStaticActor(uint actorID, OpenMetaverse.Vector3 position,
            OpenMetaverse.Quaternion orientation)
        {
            byte[] staticActorArray;
            int offset;
            ushort tempValue;

            // Check to see if the messenger has been initialized; if it has
            // not, exit out
            if (!m_isInitialized)
                return;

            // Ensure that access to the set static actor structure is
            // thread-safe
            lock (m_setStaticActorLock)
            {
                // Initialize the header with the message type
                tempValue = (ushort)MessageType.SetStaticActor;
                m_setStaticActor.header.msgType = (ushort)IPAddress.
                    HostToNetworkOrder((short)tempValue);
 
                // Initialize the rest of the header
                InitializeAPPHeader(ref m_setStaticActor.header,
                    (uint)m_APPSetStaticActorSize);
 
                // Set the body of the message using the given parameters
                m_setStaticActor.actor.actorID =
                    (uint)IPAddress.HostToNetworkOrder((int)actorID);
                m_setStaticActor.actor.simID =
                    (uint)IPAddress.HostToNetworkOrder((int)m_simulationID);
                m_setStaticActor.position.x = position.X;
                m_setStaticActor.position.y = position.Y;
                m_setStaticActor.position.z = position.Z;
                m_setStaticActor.orientation.x = orientation.X;
                m_setStaticActor.orientation.y = orientation.Y;
                m_setStaticActor.orientation.z = orientation.Z;
                m_setStaticActor.orientation.w = orientation.W;
 
                // Convert the message to a byte array, so that it can be sent
                // to the remote physics engine
                // Start by allocating the byte array
                staticActorArray = new byte[m_APPSetStaticActorSize];
 
                // Convert the header to its byte array representation
                ConvertHeaderToBytes(m_setStaticActor.header,
                   ref staticActorArray, 0);
 
                // Convert the actor ID
                offset = m_APPHeaderSize;
                offset += ConvertAPPActorIDToBytes(m_setStaticActor.actor,
                    ref staticActorArray, offset);
 
                // Convert the position and orientation
                offset += ConvertAPPVectorToBytes(m_setStaticActor.position,
                    ref staticActorArray, offset);
                offset += ConvertAPPQuaternionToBytes(
                    m_setStaticActor.orientation, ref staticActorArray, offset);
            }

            // Now that the byte array has been constructed, send it to the
            // remote physics engine
            m_packetManager.SendPacket(staticActorArray);

            // Increment the message index now that a message has been sent
            m_currentMessageIndex++;
        }

        /// <summary>
        /// Updates the state of a dynamic actor in the remote physics engine.
        /// </summary>
        /// <param name="actorID">The unique ID of the actor</param>
        /// <param name="position">The new position of the actor</param>
        /// <param name="orientation">The new orientation of the actor</param>
        /// <param name="gravityModifier">The multiplier for the gravity
        /// acting on this object</param>
        /// <param name="linearVelocity">The new linear velocity of the actor
        /// (in meters/second)</param>
        /// <param name="angularVelocity">The new angular velocity of the actor
        /// (in meters/second)</param>
        public void SetDynamicActor(uint actorID,
            OpenMetaverse.Vector3 position,
            OpenMetaverse.Quaternion orientation, float gravityModifier,
            OpenMetaverse.Vector3 linearVelocity,
            OpenMetaverse.Vector3 angularVelocity)
        {
            APPSetDynamicActor dynamicActorMsg;
            byte[] dynamicActorArray;
            ushort tempValue;
            int offset;

            // Check to see if the messenger has been initialized; if it has
            // not, exit out
            if (!m_isInitialized)
                return;

            // Ensure that access to the set dynamic actor structure is
            // thread-safe
            lock (m_setDynamicActorLock)
            {
                // Initialize the header with the message type
                tempValue = (ushort)MessageType.SetDynamicActor;
                m_setDynamicActor.header.msgType =
                    (ushort)IPAddress.HostToNetworkOrder((short)tempValue);
 
                // Initialize the rest of the header
                InitializeAPPHeader(ref m_setDynamicActor.header,
                    (uint)m_APPSetDynamicActorSize);
 
                // Set the body of the message using the given parameters
                m_setDynamicActor.actor.actorID =
                    (uint)IPAddress.HostToNetworkOrder((int)actorID);
                m_setDynamicActor.actor.simID =
                    (uint)IPAddress.HostToNetworkOrder((int)m_simulationID);
                m_setDynamicActor.position.x = position.X;
                m_setDynamicActor.position.y = position.Y;
                m_setDynamicActor.position.z = position.Z;
                m_setDynamicActor.orientation.x = orientation.X;
                m_setDynamicActor.orientation.y = orientation.Y;
                m_setDynamicActor.orientation.z = orientation.Z;
                m_setDynamicActor.orientation.w = orientation.W;
                m_setDynamicActor.gravityModifier = gravityModifier;
                m_setDynamicActor.linearVelocity.x = linearVelocity.X;
                m_setDynamicActor.linearVelocity.y = linearVelocity.Y;
                m_setDynamicActor.linearVelocity.z = linearVelocity.Z;
                m_setDynamicActor.angularVelocity.x = angularVelocity.X;
                m_setDynamicActor.angularVelocity.y = angularVelocity.Y;
                m_setDynamicActor.angularVelocity.z = angularVelocity.Z;
 
                // Convert the message into a byte array, so that it can
                // be sent to the remote physics engine
                // Start by allocating the array
                dynamicActorArray = new byte[m_APPSetDynamicActorSize];
 
                // Convert the header to its byte array representation
                ConvertHeaderToBytes(m_setDynamicActor.header,
                    ref dynamicActorArray, 0);
 
                // Convert the actor ID
                offset = m_APPHeaderSize;
                offset += ConvertAPPActorIDToBytes(m_setDynamicActor.actor,
                    ref dynamicActorArray, offset);
 
                // Convert the position
                offset += ConvertAPPVectorToBytes(m_setDynamicActor.position,
                    ref dynamicActorArray, offset);
 
                // Convert the orientation
                offset += ConvertAPPQuaternionToBytes(
                    m_setDynamicActor.orientation,
                    ref dynamicActorArray, offset);
 
                // Convert the gravity modifier
                FloatToNetworkOrder(m_setDynamicActor.gravityModifier,
                    ref dynamicActorArray, offset);
                offset += sizeof(float);
 
                // Convert the linear and angular velocities
                offset += ConvertAPPVectorToBytes(
                    m_setDynamicActor.linearVelocity,
                    ref dynamicActorArray, offset);
                offset += ConvertAPPVectorToBytes(
                    m_setDynamicActor.angularVelocity,
                    ref dynamicActorArray, offset);
            }

            // Now that the byte array has been constructed, send it to the
            // remote physics engine
            m_packetManager.SendPacket(dynamicActorArray);

            // Increment the message index now that a message has been sent
            m_currentMessageIndex++;
        }

        /// <summary>
        /// Updates the position of an actor in the remote physics engine.
        /// </summary>
        /// <param name="actorID">The unique identifier of the actor</param>
        /// <param name="gravityModifier">The new position of the actor</param>
        public void UpdateActorPosition(uint actorID,
            OpenMetaverse.Vector3 position)
        {
            APPUpdateActorPosition updatePositionMsg;
            byte[] updatePositionArray;
            ushort tempValue;
            int offset;

            // Check to see if the messenger has been initialized; if it has
            // not, exit out
            if (!m_isInitialized)
                return;

            // Initialize the message and its header
            updatePositionMsg = new APPUpdateActorPosition();
            updatePositionMsg.header = new APPHeader();

            // Initialize the header with the message type
            tempValue = (ushort)MessageType.UpdateActorPosition;
            updatePositionMsg.header.msgType =
                (ushort)IPAddress.HostToNetworkOrder((short)tempValue);

            // Initialize the rest of the header
            InitializeAPPHeader(ref updatePositionMsg.header,
                (uint)m_APPUpdateActorPositionSize);

            // Set the body of the message using the given parameters
            updatePositionMsg.actor = new APPActorID();
            updatePositionMsg.actor.actorID =
                (uint)IPAddress.HostToNetworkOrder((int)actorID);
            updatePositionMsg.actor.simID =
                (uint)IPAddress.HostToNetworkOrder((int)m_simulationID);
            updatePositionMsg.position.x = position.X;
            updatePositionMsg.position.y = position.Y;
            updatePositionMsg.position.z = position.Z;

            // Convert the message to a byte array, so that it can be sent to
            // the remote physics engine
            // Start by allocating the byte array
            updatePositionArray = new byte[m_APPUpdateActorPositionSize];

            // Convert the header to its byte array representation
            ConvertHeaderToBytes(updatePositionMsg.header,
                ref updatePositionArray, 0);

            // Convert the actor ID
            offset = m_APPHeaderSize;
            offset += ConvertAPPActorIDToBytes(updatePositionMsg.actor,
                ref updatePositionArray, offset);

            // Convert the position
            offset += ConvertAPPVectorToBytes(updatePositionMsg.position,
                ref updatePositionArray, offset);

            // Now that the byte array has been constructed, send it to the
            // remote physics engine
            m_packetManager.SendPacket(updatePositionArray);

            // Increment the message index now that a msesage has been sent
            m_currentMessageIndex++;
        }

        /// <summary>
        /// Updates the orientation of an actor in the remote physics engine.
        /// </summary>
        /// <param name="actorID">The unique identifier of the actor</param>
        /// <param name="orientation">The new orientation of the actor</param>
        public void UpdateActorOrientation(uint actorID,
            OpenMetaverse.Quaternion orientation)
        {
            APPUpdateActorOrientation updateOrientationMsg;
            byte[] updateOrientationArray;
            ushort tempValue;
            int offset;

            // Check to see if the messenger has been initialized; if it
            // has not, exit out
            if (!m_isInitialized)
                return;

            // Initialize the message and its header
            updateOrientationMsg = new APPUpdateActorOrientation();
            updateOrientationMsg.header = new APPHeader();

            // Initialize the header with the message type
            tempValue = (ushort)MessageType.UpdateActorOrientation;
            updateOrientationMsg.header.msgType =
                (ushort)IPAddress.HostToNetworkOrder((short)tempValue);

            // Initialize the rest of the header
            InitializeAPPHeader(ref updateOrientationMsg.header,
                (uint)m_APPUpdateActorOrientationSize);

            // Set the body of the message using the given parameters
            updateOrientationMsg.actor = new APPActorID();
            updateOrientationMsg.actor.actorID =
                (uint)IPAddress.HostToNetworkOrder((int)actorID);
            updateOrientationMsg.actor.simID =
                (uint)IPAddress.HostToNetworkOrder((int)m_simulationID);
            updateOrientationMsg.orientation.x = orientation.X;
            updateOrientationMsg.orientation.y = orientation.Y;
            updateOrientationMsg.orientation.z = orientation.Z;
            updateOrientationMsg.orientation.w = orientation.W;

            // Convert the message to a byte array, so that it can be sent to
            // the remote physics engine
            // Start by allocating the byte array
            updateOrientationArray = new byte[m_APPUpdateActorOrientationSize];

            // Convert the header to its byte array representation
            ConvertHeaderToBytes(updateOrientationMsg.header,
                ref updateOrientationArray, 0);

            // Convert the actor ID
            offset = m_APPHeaderSize;
            offset += ConvertAPPActorIDToBytes(updateOrientationMsg.actor,
                ref updateOrientationArray, offset);

            // Convert the orientation
            offset += ConvertAPPQuaternionToBytes(
                updateOrientationMsg.orientation, ref updateOrientationArray,
                offset);

            // Now that the byte array has been constructed, send it to the
            // remote physics engine
            m_packetManager.SendPacket(updateOrientationArray);

            // Increment the message index now that a msesage has been sent
            m_currentMessageIndex++;
        }

        /// <summary>
        /// Updates the gravity modifier of a dynamic actor in the remote
        /// physics engine.
        /// </summary>
        /// <param name="actorID">The unique identifier of the actor</param>
        /// <param name="gravityModifier">The new gravity modifier</param>
        public void UpdateActorGravityModifier(uint actorID,
            float gravityModifier)
        {
            APPUpdateDynamicActorGravityModifier updateGravityMsg;
            byte[] updateGravityArray;
            ushort tempValue;
            int offset;

            // Check to see if the messenger has been initialized; if it has
            // not, exit out
            if (!m_isInitialized)
                return;

            // Initialize the message and its header
            updateGravityMsg = new APPUpdateDynamicActorGravityModifier();
            updateGravityMsg.header = new APPHeader();

            // Initialize the header with the message type
            tempValue = (ushort)MessageType.DynamicActorUpdateGravityModifier;
            updateGravityMsg.header.msgType =
                (ushort)IPAddress.HostToNetworkOrder((short)tempValue);

            // Initialize the rest of the header
            InitializeAPPHeader(ref updateGravityMsg.header,
                (uint)m_APPUpdateDynamicActorGravityModifierSize);

            // Set the body of the message using the given parameters
            updateGravityMsg.actor = new APPActorID();
            updateGravityMsg.actor.actorID =
                (uint)IPAddress.HostToNetworkOrder((int)actorID);
            updateGravityMsg.actor.simID = (uint)IPAddress.HostToNetworkOrder(
                (int)m_simulationID);
            updateGravityMsg.gravityModifier = gravityModifier;

            // Convert the message to a byte array, so that it can be sent to
            // the remote physics engine
            // Start by allocating the byte array
            updateGravityArray = new byte[
                m_APPUpdateDynamicActorGravityModifierSize];

            // Convert the header to its byte array representation
            ConvertHeaderToBytes(updateGravityMsg.header,
                ref updateGravityArray, 0);

            // Convert the actor ID
            offset = m_APPHeaderSize;
            offset += ConvertAPPActorIDToBytes(updateGravityMsg.actor,
                ref updateGravityArray, offset);

            // Convert the gravity modifier
            FloatToNetworkOrder(updateGravityMsg.gravityModifier,
                ref updateGravityArray, offset);
            offset += sizeof(float);

            // Now that the byte array has been constructed, send it to the
            // remote physics engine
            m_packetManager.SendPacket(updateGravityArray);

            // Increment the message index now that a msesage has been sent
            m_currentMessageIndex++;
        }

        /// <summary>
        /// Updates the linear velocity of an actor in the remote
        /// physics engine.
        /// </summary>
        /// <param name="actorID">The unique identifier of the actor</param>
        /// <param name="velocity">The new linear velocity of the actor</param>
        public void UpdateActorVelocity(uint actorID,
            OpenMetaverse.Vector3 velocity)
        {
            APPUpdateDynamicActorLinearVelocity updateVelocityMsg;
            byte[] updateVelocityArray;
            ushort tempValue;
            int offset;

            // Check to see if the messenger has been initialized; if it has
            // not, exit out
            if (!m_isInitialized)
                return;

            // Initialize the message and its header
            updateVelocityMsg = new APPUpdateDynamicActorLinearVelocity();
            updateVelocityMsg.header = new APPHeader();

            // Initialize the header with the message type
            tempValue = (ushort)MessageType.DynamicActorUpdateLinearVelocity;
            updateVelocityMsg.header.msgType =
                (ushort)IPAddress.HostToNetworkOrder((short)tempValue);

            // Initialize the rest of the header
            InitializeAPPHeader(ref updateVelocityMsg.header,
                (uint)m_APPUpdateDynamicActorLinearVelocitySize);

            // Set the body of the message using the given parameters
            updateVelocityMsg.actor = new APPActorID();
            updateVelocityMsg.actor.actorID =
                (uint)IPAddress.HostToNetworkOrder((int)actorID);
            updateVelocityMsg.actor.simID =
                (uint)IPAddress.HostToNetworkOrder((int)m_simulationID);
            updateVelocityMsg.linearVelocity.x = velocity.X;
            updateVelocityMsg.linearVelocity.y = velocity.Y;
            updateVelocityMsg.linearVelocity.z = velocity.Z;

            // Convert the message to a byte array, so that it can be sent
            // to the remote physics engine
            // Start by allocating the byte array
            updateVelocityArray = new byte[
                m_APPUpdateDynamicActorLinearVelocitySize];

            // Convert the header to its byte array representation
            ConvertHeaderToBytes(updateVelocityMsg.header,
                ref updateVelocityArray, 0);

            // Convert the actor ID
            offset = m_APPHeaderSize;
            offset += ConvertAPPActorIDToBytes(updateVelocityMsg.actor,
                ref updateVelocityArray, offset);

            // Convert the linear velocity
            offset += ConvertAPPVectorToBytes(updateVelocityMsg.linearVelocity,
                ref updateVelocityArray, offset);

            // Now that the byte array has been constructed, send it to the
            // remote physics engine
            m_packetManager.SendPacket(updateVelocityArray);

            // Increment the message index now that a msesage has been sent
            m_currentMessageIndex++;
        }

        /// <summary>
        /// Updates the angular velocity of an actor in the remote
        /// physics engine.
        /// </summary>
        /// <param name="actorID">The unique identifier of the actor</param>
        /// <param name="velocity">The new angular velocity of the actor</param>
        public void UpdateActorAngularVelocity(uint actorID,
            OpenMetaverse.Vector3 velocity)
        {
            APPUpdateDynamicActorAngularVelocity updateVelocityMsg;
            byte[] updateVelocityArray;
            ushort tempValue;
            int offset;

            // Check to see if the messenger has been initialized; if it has
            // not, exit out
            if (!m_isInitialized)
                return;

            // Initialize the message and its header
            updateVelocityMsg = new APPUpdateDynamicActorAngularVelocity();
            updateVelocityMsg.header = new APPHeader();

            // Initialize the header with the message type
            tempValue = (ushort)MessageType.DynamicActorUpdateAngularVelocity;
            updateVelocityMsg.header.msgType =
                (ushort)IPAddress.HostToNetworkOrder((short)tempValue);

            // Initialize the rest of the header
            InitializeAPPHeader(ref updateVelocityMsg.header,
                (uint)m_APPUpdateDynamicActorAngularVelocitySize);

            // Set the body of the message using the given parameters
            updateVelocityMsg.actor = new APPActorID();
            updateVelocityMsg.actor.actorID =
                (uint)IPAddress.HostToNetworkOrder((int)actorID);
            updateVelocityMsg.actor.simID =
                (uint)IPAddress.HostToNetworkOrder((int)m_simulationID);
            updateVelocityMsg.angularVelocity.x = velocity.X;
            updateVelocityMsg.angularVelocity.y = velocity.Y;
            updateVelocityMsg.angularVelocity.z = velocity.Z;

            // Convert the message to a byte array, so that it can be sent
            // to the remote physics engine
            // Start by allocating the byte array
            updateVelocityArray = new byte[
                m_APPUpdateDynamicActorAngularVelocitySize];

            // Convert the header to its byte array representation
            ConvertHeaderToBytes(updateVelocityMsg.header,
                ref updateVelocityArray, 0);

            // Convert the actor ID
            offset = m_APPHeaderSize;
            offset += ConvertAPPActorIDToBytes(updateVelocityMsg.actor,
                ref updateVelocityArray, offset);

            // Convert the angular velocity
            offset += ConvertAPPVectorToBytes(updateVelocityMsg.angularVelocity,
                ref updateVelocityArray, offset);

            // Now that the byte array has been constructed, send it to the
            // remote physics engine
            m_packetManager.SendPacket(updateVelocityArray);

            // Increment the message index now that a msesage has been sent
            m_currentMessageIndex++;
        }

        /// <summary>
        /// Requests the mass of an actor from the remote physics engine.
        /// </summary>
        /// <param name="actorID">The unique identifier of the actor</param>
        public void GetActorMass(uint actorID)
        {
            APPGetDynamicActorMass getMassMsg;
            byte[] getMassArray;
            ushort tempValue;
            int offset;

            // Check to see if the messenger has been initialized; if it has
            // not, exit out
            if (!m_isInitialized)
                return;

            // Initialize the message and its header
            getMassMsg = new APPGetDynamicActorMass();
            getMassMsg.header = new APPHeader();

            // Initialize the header with the message type
            tempValue = (ushort) MessageType.GetDynamicActorMass;
            getMassMsg.header.msgType = (ushort)
                IPAddress.HostToNetworkOrder((short) tempValue);

            // Initialize the rest of the header
            InitializeAPPHeader(ref getMassMsg.header, 
                (uint) m_APPGetDynamicActorMassSize);

            // Set the body of the message using the given parameters
            getMassMsg.actor = new APPActorID();
            getMassMsg.actor.actorID = (uint)
                IPAddress.HostToNetworkOrder((int) actorID);
            getMassMsg.actor.simID = (uint)
                IPAddress.HostToNetworkOrder((int) m_simulationID);

            // Convert the message to a byte array, so that it can be sent to
            // the remote physics engine
            // Start by allocating the byte array
            getMassArray = new byte[m_APPGetDynamicActorMassSize];

            // Convert the header to its byte array representation
            ConvertHeaderToBytes(getMassMsg.header, ref getMassArray, 0);

            // Convert the actor ID
            offset = m_APPHeaderSize;
            ConvertAPPActorIDToBytes(getMassMsg.actor, ref getMassArray,
                offset);

            // Now that the byte array has been constructed, send it to the
            // remote physics engine
            m_packetManager.SendPacket(getMassArray);

            // Increment the message index now that a message has been sent
            m_currentMessageIndex++;
        }

        /// <summary>
        /// Removes the actor from the remote physics engine.
        /// </summary>
        /// <param name="actorID">The unique ID of the actor</param>
        public void RemoveActor(uint actorID)
        {
            APPRemoveActor removeActorMsg;
            byte[] removeActorArray;
            int offset;

            // Check to see if the messenger has been initialized; if it has
            // not, exit out
            if (!m_isInitialized)
                return;

            // Initialize the message and its header
            removeActorMsg = new APPRemoveActor();
            removeActorMsg.header = new APPHeader();

            // Initialize the header with the message type
            removeActorMsg.header.msgType =
                (ushort)IPAddress.HostToNetworkOrder(
                    (short)MessageType.RemoveActor);

            // Initialize the rest of the header
            InitializeAPPHeader(ref removeActorMsg.header,
                (uint)m_APPRemoveActorSize);

            // Set the body of the message using the given parameters
            removeActorMsg.actor = new APPActorID();
            removeActorMsg.actor.actorID = (uint)IPAddress.HostToNetworkOrder(
                (int)actorID);
            removeActorMsg.actor.simID = (uint)IPAddress.HostToNetworkOrder(
                (int)m_simulationID);

            // Convert the message to a byte array, so that it can be sent
            // to the remote physics engine
            // Start by allocating the byte array
            removeActorArray = new byte[m_APPRemoveActorSize];

            // Convert the header to its byte array representation
            ConvertHeaderToBytes(removeActorMsg.header,
                ref removeActorArray, 0);

            // Convert the actor ID
            offset = m_APPHeaderSize;
            offset += ConvertAPPActorIDToBytes(removeActorMsg.actor,
                ref removeActorArray, offset);

            // Now that the byte array has been constructed, send it to the
            // remote physics engine
            m_packetManager.SendPacket(removeActorArray);

            // Increment the message index now that a msesage has been sent
            m_currentMessageIndex++;
        }

        /// <summary>
        /// Adds a joint between two actors to the remote physics engine.
        /// </summary>
        /// <param name="jointID">The unique identifier to be used for the
        /// new joint</param>
        /// <param name="actor1ID">The unique identifier of the first actor
        /// to which the joint is being attached</param>
        /// <param name="actor1Translation">The position relative to the
        /// actor at which the joint is being attached to the
        /// first actor</param>
        /// <param name="actor1Orientation">The orientation relative to the
        /// actor at which the joint is being attached to the
        /// second actor</param>
        /// <param name="actor2ID">The unique identifier of the second actor
        /// to which the joint is being attached</param>
        /// <param name="actor2Translation">The position relative to the
        /// actor at which the joint is being attached to the
        /// second actor</param>
        /// <param name="actor2Orientation">The orientation relative to
        /// the actor at which the joint is being attached to the
        /// second actor</param>
        /// <param name="linearLowerLimits">The lower linear limits of the
        /// joint for the 3 translational axes</param>
        /// <param name="linearUpperLimits">The upper linear limits of the
        /// joint for the 3 translational axes</param>
        /// <param name="angularLowerLimits">The lower angular limits of the
        /// joint for the 3 rotational axes</param>
        /// <param name="angularUpperLimits">The upper angular limits of the
        /// joint for the 3 rotational axes</param>
        /// <remarks>For both the linear and angular limits:
        /// If lowerLimit is equal to the upperLimit, axis is locked.
        /// If lowerLimit is less than the upperLimit, axis is limited to
        /// be between the two limits. If lowerLimit is greater than upperLimit,
        /// axis is free.</remarks>
        public void AddJoint(uint jointID, uint actor1ID, 
            OpenMetaverse.Vector3 actor1Translation,
            OpenMetaverse.Quaternion actor1Orientation,
            uint actor2ID, OpenMetaverse.Vector3 actor2Translation,
            OpenMetaverse.Quaternion actor2Orientation,
            OpenMetaverse.Vector3 linearLowerLimits,
            OpenMetaverse.Vector3 linearUpperLimits,
            OpenMetaverse.Vector3 angularLowerLimits,
            OpenMetaverse.Vector3 angularUpperLimits)
        {
            APPAddJoint addJointMsg;
            byte[] addJointArray;
            int offset;

            // Check to see if the messenger has been initialized; if it has
            // not, exit out
            if (!m_isInitialized)
                return;

            // Initialize the message and its header
            addJointMsg = new APPAddJoint();
            addJointMsg.header = new APPHeader();
            
            // Initialize the header with the message type
            addJointMsg.header.msgType = (ushort)IPAddress.HostToNetworkOrder(
                (short)MessageType.AddJoint);

            // Initialize the rest of the header
            InitializeAPPHeader(ref addJointMsg.header,
                (uint)m_APPAddJointSize);

            // Set the body of the message using the given parameters
            addJointMsg.joint = new APPJointID();
            addJointMsg.joint.simID = (uint)IPAddress.HostToNetworkOrder(
                (int)m_simulationID);
            addJointMsg.joint.jointID = (uint)IPAddress.HostToNetworkOrder(
                (int)jointID);
            addJointMsg.actor1 = new APPActorID();
            addJointMsg.actor1.simID = (uint)IPAddress.HostToNetworkOrder(
                (int)m_simulationID);
            addJointMsg.actor1.actorID = (uint)IPAddress.HostToNetworkOrder(
                (int)actor1ID);
            addJointMsg.orientation1 = new APPQuat();
            addJointMsg.orientation1.x = actor1Orientation.X;
            addJointMsg.orientation1.y = actor1Orientation.Y;
            addJointMsg.orientation1.z = actor1Orientation.Z;
            addJointMsg.orientation1.w = actor1Orientation.W;
            addJointMsg.translation1 = new APPVector();
            addJointMsg.translation1.x = actor1Translation.X;
            addJointMsg.translation1.y = actor1Translation.Y;
            addJointMsg.translation1.z = actor1Translation.Z;
            addJointMsg.actor2 = new APPActorID();
            addJointMsg.actor2.simID = (uint)IPAddress.HostToNetworkOrder(
                (int)m_simulationID);
            addJointMsg.actor2.actorID = (uint)IPAddress.HostToNetworkOrder(
                (int)actor2ID);
            addJointMsg.orientation2 = new APPQuat();
            addJointMsg.orientation2.x = actor2Orientation.X;
            addJointMsg.orientation2.y = actor2Orientation.Y;
            addJointMsg.orientation2.z = actor2Orientation.Z;
            addJointMsg.orientation2.w = actor2Orientation.W;
            addJointMsg.translation2 = new APPVector();
            addJointMsg.translation2.x = actor2Translation.X;
            addJointMsg.translation2.y = actor2Translation.Y;
            addJointMsg.translation2.z = actor2Translation.Z;
            addJointMsg.linearLowerLimits = new APPVector();
            addJointMsg.linearLowerLimits.x = linearLowerLimits.X;
            addJointMsg.linearLowerLimits.y = linearLowerLimits.Y;
            addJointMsg.linearLowerLimits.z = linearLowerLimits.Z;
            addJointMsg.linearUpperLimits = new APPVector();
            addJointMsg.linearUpperLimits.x = linearUpperLimits.X;
            addJointMsg.linearUpperLimits.y = linearUpperLimits.Y;
            addJointMsg.linearUpperLimits.z = linearUpperLimits.Z;
            addJointMsg.angularLowerLimits = new APPVector();
            addJointMsg.angularLowerLimits.x = angularLowerLimits.X;
            addJointMsg.angularLowerLimits.y = angularLowerLimits.Y;
            addJointMsg.angularLowerLimits.z = angularLowerLimits.Z;
            addJointMsg.angularUpperLimits = new APPVector();
            addJointMsg.angularUpperLimits.x = angularUpperLimits.X;
            addJointMsg.angularUpperLimits.y = angularUpperLimits.Y;
            addJointMsg.angularUpperLimits.z = angularUpperLimits.Z;

            // Convert the message into a byte array, so that it can be
            // sent to the remote physics engine
            // Start by allocating the array
            addJointArray = new byte[m_APPAddJointSize];

            // Convert the header into its byte array representation
            ConvertHeaderToBytes(addJointMsg.header, ref addJointArray, 0);

            // Convert the joint ID 
            offset = m_APPHeaderSize;
            offset += ConvertAPPJointIDToBytes(addJointMsg.joint,
                ref addJointArray, offset);

            // Convert the first actor ID
            offset += ConvertAPPActorIDToBytes(addJointMsg.actor1,
                ref addJointArray, offset);

            // Convert the first actor's orientation and translation values
            offset += ConvertAPPQuaternionToBytes(addJointMsg.orientation1,
                ref addJointArray, offset);
            offset += ConvertAPPVectorToBytes(addJointMsg.translation1,
                ref addJointArray, offset);

            // Convert the second actor ID
            offset += ConvertAPPActorIDToBytes(addJointMsg.actor2,
                ref addJointArray, offset);

            // Convert the second actor's orientation and translation values
            offset += ConvertAPPQuaternionToBytes(addJointMsg.orientation2,
                ref addJointArray, offset);
            offset += ConvertAPPVectorToBytes(addJointMsg.translation2,
                ref addJointArray, offset);

            // Convert the linear limits
            offset += ConvertAPPVectorToBytes(addJointMsg.linearLowerLimits,
                ref addJointArray, offset);
            offset += ConvertAPPVectorToBytes(addJointMsg.linearUpperLimits,
                ref addJointArray, offset);

            // Convert the angular limits
            offset += ConvertAPPVectorToBytes(addJointMsg.angularLowerLimits,
                ref addJointArray, offset);
            offset += ConvertAPPVectorToBytes(addJointMsg.angularUpperLimits,
                ref addJointArray, offset);

            // Now that the byte array has been constructed, send it to the
            // remote physics engine
            m_packetManager.SendPacket(addJointArray);

            // Increment the message index now that a msesage has been sent
            m_currentMessageIndex++;
        }

        /// <summary>
        /// Removes a joint from the remote physics engine.
        /// </summary>
        /// <param name="jointID">The unique identifier of the joint to be
        /// removed</param>
        public void RemoveJoint(uint jointID)
        {
            APPRemoveJoint removeJointMsg;
            byte[] removeJointArray;
            int offset;

            // Check to see if the messenger has been initialized; if it has
            // not, exit out
            if (!m_isInitialized)
                return;

            // Initialize the message and its header
            removeJointMsg = new APPRemoveJoint();
            removeJointMsg.header = new APPHeader();

            // Initialize the header with the message type
            removeJointMsg.header.msgType =
                (ushort)IPAddress.HostToNetworkOrder(
                    (short)MessageType.RemoveJoint);

            // Initialize the rest of the header
            InitializeAPPHeader(ref removeJointMsg.header,
                (uint)m_APPRemoveJointSize);

            // Initialize the joint ID
            removeJointMsg.joint = new APPJointID();
            removeJointMsg.joint.simID =
                (uint)IPAddress.HostToNetworkOrder((int)m_simulationID);
            removeJointMsg.joint.jointID =
                (uint)IPAddress.HostToNetworkOrder((int)jointID);

            // Convert the message into a byte array, so that it can be sent
            // to the remote physics engine
            // Start by allocating the array
            removeJointArray = new byte[m_APPRemoveJointSize];

            // Convert the header to its byte array representation
            ConvertHeaderToBytes(removeJointMsg.header,
                ref removeJointArray, 0);

            // Convert the joint ID
            offset = m_APPHeaderSize;
            ConvertAPPJointIDToBytes(removeJointMsg.joint,
                ref removeJointArray, offset);

            // Now that the byte array has been constructed, send it to the
            // remote physics engine
            m_packetManager.SendPacket(removeJointArray);

            // Increment the message index now that a message has been sent
            m_currentMessageIndex++;
        }

        /// <summary>
        /// Adds a sphere shape to the remote physics engine
        /// </summary>
        /// <param name="shapeID">The unique ID of the shape</param>
        /// <param name="origin">The origin of the sphere relative to any
        /// attached actor</param>
        /// <param name="radius">The radius of the sphere</param>
        public void AddSphere(uint shapeID, OpenMetaverse.Vector3 origin,
            float radius)
        {
            APPAddSphere addSphereMsg;
            byte[] addSphereArray;
            int offset;

            // Check to see if the messenger has been initialized; if it has
            // not, exit out
            if (!m_isInitialized)
                return;

            // Initialize the message and its header
            addSphereMsg = new APPAddSphere();
            addSphereMsg.header = new APPHeader();

            // Initialize the header with the message type
            addSphereMsg.header.msgType = (ushort)IPAddress.HostToNetworkOrder(
                (short)MessageType.AddSphere);

            // Initialize the rest of the header
            InitializeAPPHeader(ref addSphereMsg.header,
                (uint)m_APPAddSphereSize);

            // Set the body of the message using the given parameters
            addSphereMsg.shape = new APPShapeID();
            addSphereMsg.shape.shapeID = (uint)IPAddress.HostToNetworkOrder(
                (int)shapeID);
            addSphereMsg.shape.simID = (uint)IPAddress.HostToNetworkOrder(
                (int)m_simulationID);
            addSphereMsg.origin = new APPVector();
            addSphereMsg.origin.x = origin.X;
            addSphereMsg.origin.y = origin.Y;
            addSphereMsg.origin.z = origin.Z;
            addSphereMsg.radius = radius;

            // Convert the message into a byte array, so that it can be sent
            // to the remote physics engine
            // Start by allocating the array
            addSphereArray = new byte[m_APPAddSphereSize];

            // Convert the header to its byte array representation
            ConvertHeaderToBytes(addSphereMsg.header, ref addSphereArray, 0);

            // Convert the shape ID
            offset = m_APPHeaderSize;
            offset += ConvertAPPShapeIDToBytes(addSphereMsg.shape,
                ref addSphereArray, offset);

            // Convert the origin vector
            offset += ConvertAPPVectorToBytes(addSphereMsg.origin,
                ref addSphereArray, offset);

            // Convert the radius
            FloatToNetworkOrder(addSphereMsg.radius, ref addSphereArray,
                offset);

            // Now that the byte array has been constructed, send it to the
            // remote physics engine
            m_packetManager.SendPacket(addSphereArray);

            // Increment the message index now that a message has been sent
            m_currentMessageIndex++;
        }

        /// <summary>
        /// Adds a plane to the remote physics engine.
        /// </summary>
        /// <param name="shapeID">The unique ID of the shape</param>
        /// <param name="planeNormal">The normal of the plane</param>
        /// <param name="planeConstant">The distance from the origin of the
        /// attached actor (in meters)</param>
        public void AddPlane(uint shapeID, OpenMetaverse.Vector3 planeNormal,
            float planeConstant)
        {
            APPAddPlane addPlaneMsg;
            byte[] addPlaneArray;
            int offset;

            // Check to see if the messenger has been initialized; if it has
            // not, exit out
            if (!m_isInitialized)
                return;

            // Initialize the message and its header
            addPlaneMsg = new APPAddPlane();
            addPlaneMsg.header = new APPHeader();

            // Initialize the header with the message type
            addPlaneMsg.header.msgType = (ushort)IPAddress.HostToNetworkOrder(
                (short)MessageType.AddPlane);

            // Initialize the rest of the header
            InitializeAPPHeader(ref addPlaneMsg.header,
                (uint)m_APPAddPlaneSize);

            // Set the body of the message using the given parameters
            addPlaneMsg.shape = new APPShapeID();
            addPlaneMsg.shape.shapeID = (uint)IPAddress.HostToNetworkOrder(
                (int)shapeID);
            addPlaneMsg.shape.simID = (uint)IPAddress.HostToNetworkOrder(
                (int)m_simulationID);
            addPlaneMsg.planeNormal = new APPVector();
            addPlaneMsg.planeNormal.x = planeNormal.X;
            addPlaneMsg.planeNormal.y = planeNormal.Y;
            addPlaneMsg.planeNormal.z = planeNormal.Z;
            addPlaneMsg.planeConstant = planeConstant;

            // Convert the message to a byte array, so that it can be sent to
            // the remote physics engine
            // Start by allocating the byte array
            addPlaneArray = new byte[m_APPAddPlaneSize];

            // Convert the header into its byte array representation
            ConvertHeaderToBytes(addPlaneMsg.header, ref addPlaneArray, 0);

            // Convert the shape ID
            offset = m_APPHeaderSize;
            offset += ConvertAPPShapeIDToBytes(addPlaneMsg.shape,
                ref addPlaneArray, offset);

            // Convert the plane constant
            FloatToNetworkOrder(addPlaneMsg.planeConstant,
                ref addPlaneArray, offset);

            // Now that the byte array has been constructed, send it to the
            // remote physics engine
            m_packetManager.SendPacket(addPlaneArray);

            // Increment the message index now that a message has been sent
            m_currentMessageIndex++;
        }

        /// <summary>
        /// Adds a capsule shape to the remote physics engine.
        /// </summary>
        /// <param name="shapeID">The unique ID of the shape</param>
        /// <param name="radius">The radius of the capsule (in meters)</param>
        /// <param name="height">The height of the capsule (in meters)</param>
        public void AddCapsule(uint shapeID, float radius, float height)
        {
            APPAddCapsule addCapsuleMsg;
            byte[] addCapsuleArray;
            int offset;

            // Check to see if the messenger has been initialized; if it has
            // not, exit out
            if (!m_isInitialized)
                return;

            // Initialize the message and its header
            addCapsuleMsg = new APPAddCapsule();
            addCapsuleMsg.header = new APPHeader();

            // Initialize the header with the message type
            addCapsuleMsg.header.msgType =
                (ushort)IPAddress.HostToNetworkOrder(
                    (short)MessageType.AddCapsule);

            // Initialize the rest of the header
            InitializeAPPHeader(ref addCapsuleMsg.header,
                (uint)m_APPAddCapsuleSize);

            // Set the body of the message using the given parameters
            addCapsuleMsg.shape = new APPShapeID();
            addCapsuleMsg.shape.shapeID = (uint)IPAddress.HostToNetworkOrder(
                (int)shapeID);
            addCapsuleMsg.shape.simID = (uint)IPAddress.HostToNetworkOrder(
                (int)m_simulationID);
            addCapsuleMsg.radius = radius;
            addCapsuleMsg.height = height;

            // Convert the message into a byte array, so that it can be sent
            // to the remote physics engine
            // Start by allocating the byte array
            addCapsuleArray = new byte[m_APPAddCapsuleSize];

            // Convert the header into its byte array representation
            ConvertHeaderToBytes(addCapsuleMsg.header, ref addCapsuleArray, 0);

            // Convert the shape ID
            offset = m_APPHeaderSize;
            offset += ConvertAPPShapeIDToBytes(addCapsuleMsg.shape,
                ref addCapsuleArray, offset);

            // Convert the radius and height
            FloatToNetworkOrder(addCapsuleMsg.radius, ref addCapsuleArray,
                offset);
            offset += sizeof(float);
            FloatToNetworkOrder(addCapsuleMsg.height, ref addCapsuleArray,
                offset);

            // Now that the byte array has been constructed, send it to the
            // remote physics engine
            m_packetManager.SendPacket(addCapsuleArray);

            // Increment the message index now that a message has been sent
            m_currentMessageIndex++;
        }

        /// <summary>
        /// Adds a box shape to the remote physics engine.
        /// </summary>
        /// <param name="shapeID">The unique ID of the shape</param>
        /// <param name="length">The length of the box in meters</param>
        /// <param name="width">The width of the box in meters</param>
        /// <param name="height">The height of the box in meters</param>
        public void AddBox(uint shapeID, float length, float width,
            float height)
        {
            APPAddBox addBoxMsg;
            byte[] addBoxArray;
            int offset;

            // Check to see if the messenger has been initialized; if it has
            // not, exit out
            if (!m_isInitialized)
                return;

            // Initialize the message and its header
            addBoxMsg = new APPAddBox();
            addBoxMsg.header = new APPHeader();

            // Initialize the header with the message type
            addBoxMsg.header.msgType = (ushort)IPAddress.HostToNetworkOrder(
                (short)MessageType.AddBox);

            // Initialize the rest of the header
            InitializeAPPHeader(ref addBoxMsg.header, (uint)m_APPAddBoxSize);

            // Set the body of the message using the given parameters
            addBoxMsg.shape = new APPShapeID();
            addBoxMsg.shape.shapeID = (uint)IPAddress.HostToNetworkOrder(
                (int)shapeID);
            addBoxMsg.shape.simID = (uint)IPAddress.HostToNetworkOrder(
                (int)m_simulationID);
            addBoxMsg.length = length;
            addBoxMsg.width = width;
            addBoxMsg.height = height;

            // Convert the message into a byte array, so that it can be sent
            // to the remote physics engine
            // Start by allocating the byte array
            addBoxArray = new byte[m_APPAddBoxSize];

            // Convert the header into its byte array representation
            ConvertHeaderToBytes(addBoxMsg.header, ref addBoxArray, 0);

            // Convert the shape ID
            offset = m_APPHeaderSize;
            offset += ConvertAPPShapeIDToBytes(addBoxMsg.shape,
                ref addBoxArray, offset);

            // Convert the the dimension fields
            FloatToNetworkOrder(addBoxMsg.length, ref addBoxArray, offset);
            offset += sizeof(float);
            FloatToNetworkOrder(addBoxMsg.width, ref addBoxArray, offset);
            offset += sizeof(float);
            FloatToNetworkOrder(addBoxMsg.height, ref addBoxArray, offset);

            // Now that the byte array has been constructed, send it to the
            // remote physics engine
            m_packetManager.SendPacket(addBoxArray);

            // Increment the message index now that a message has been sent
            m_currentMessageIndex++;
        }

        /// <summary>
        /// Adds a convex mesh to the remote physics engine.
        /// </summary>
        /// <param name="shapeID">The unique ID of the shape</param>
        /// <param name="points">The points that make up the convex mesh</param>
        public void AddConvexMesh(uint shapeID, OpenMetaverse.Vector3[] points)
        {
            APPAddConvexMesh addMeshMsg;
            uint msgLength;
            byte[] addMeshArray;
            byte[] tempArray;
            int offset;

            // Check to see if the messenger has been initialized; if it has
            // not, exit out
            if (!m_isInitialized)
                return;

            // Initialize the message and its header
            addMeshMsg = new APPAddConvexMesh();
            addMeshMsg.header = new APPHeader();

            // Initialize the header with the message type
            addMeshMsg.header.msgType = (ushort)IPAddress.HostToNetworkOrder(
                (short)MessageType.AddConvexMesh);

            // Calculate the size of the message parts and sum them up;
            // start by calculating the sizes of the header and non-variable
            // fields in the body
            msgLength = (uint)(m_APPHeaderSize + m_APPShapeIDSize +
                sizeof(uint));

            // Finally add up the sizes of the points themselves; each point
            // is made up of three single-precision floats
            msgLength += (uint)(points.Length * 3 * sizeof(float));

            // Initialize the rest of the header
            InitializeAPPHeader(ref addMeshMsg.header, msgLength);

            // Set the body of the message using the given parameters
            addMeshMsg.shape = new APPShapeID();
            addMeshMsg.shape.shapeID = (uint)IPAddress.HostToNetworkOrder(
                (int)shapeID);
            addMeshMsg.shape.simID = (uint)IPAddress.HostToNetworkOrder(
                (int)m_simulationID);
            addMeshMsg.numPoints = (uint)IPAddress.HostToNetworkOrder(
                points.Length);

            // Initialize the array in the message structure that will
            // hold the points
            addMeshMsg.points = new APPVector[points.Length];

            // Add the points into the array that was created above
            for (int i = 0; i < points.Length; i++)
            {
                addMeshMsg.points[i] = new APPVector();
                addMeshMsg.points[i].x = points[i].X;
                addMeshMsg.points[i].y = points[i].Y;
                addMeshMsg.points[i].z = points[i].Z;
            }
            
            // Convert the message into a byte array, so that it can be sent
            // to the remote physics engine
            // Start by allocating the byte array
            addMeshArray = new byte[msgLength];

            // Convert the header into its byte array representation
            ConvertHeaderToBytes(addMeshMsg.header, ref addMeshArray, 0);

            // Convert the shape ID
            offset = m_APPHeaderSize;
            offset += ConvertAPPShapeIDToBytes(addMeshMsg.shape,
                ref addMeshArray, offset);

            // Convert the number of points field
            tempArray = BitConverter.GetBytes(addMeshMsg.numPoints);
            Buffer.BlockCopy(tempArray, 0, addMeshArray, offset, sizeof(uint));
            offset += sizeof(uint);

            // Convert the points
            for (int i = 0; i < addMeshMsg.points.Length; i++)
            {
                offset += ConvertAPPVectorToBytes(addMeshMsg.points[i],
                   ref addMeshArray, offset);
            }

            // Now that the byte array has been constructed, send it to the
            // remote physics engine
            m_packetManager.SendPacket(addMeshArray);

            // Increment the message index now that a message has been sent
            m_currentMessageIndex++;
        }

        /// <summary>
        /// Adds a triangle mesh to the remote physics engine.
        /// </summary>
        /// <param name="shapeID">The unique ID of the shape</param>
        /// <param name="points">The points that are used to construct the
        /// triangle mesh</param>
        /// <param name="triangles">The vertex indices of the triangles in the
        /// mesh (all indices must be in the domain of
        /// the "points" array)</param>
        public void AddTriangleMesh(uint shapeID,
            OpenMetaverse.Vector3[] points, int[] triangles)
        {
            APPAddTriangleMesh addMeshMsg;
            uint msgLength;
            byte[] addMeshArray;
            int i;
            byte[] tempArray;
            int offset;
            int numTriangles;

            // Check to see if the messenger has been initialized; if it has
            // not, exit out
            if (!m_isInitialized)
                return;

            // Initialize the message and its header
            addMeshMsg = new APPAddTriangleMesh();
            addMeshMsg.header = new APPHeader();

            // Initialize the header with the message type
            addMeshMsg.header.msgType = (ushort)IPAddress.HostToNetworkOrder(
                (short)MessageType.AddTriangleMesh);

            // Calculate the size of the message parts and sum them up;
            // start by calculating the sizes of the header and non-variable
            // fields in the body
            msgLength = (uint)(m_APPHeaderSize + m_APPShapeIDSize +
                sizeof(uint) * 2);

            // Finally, add in the size of the points and the triangle indices;
            // each point is a vector of 3 floats
            msgLength += (uint)(points.Length * 3 * sizeof(float));
            msgLength += (uint)(triangles.Length * sizeof(uint));

            // Initialize the rest of the header
            InitializeAPPHeader(ref addMeshMsg.header, msgLength);

            // Set the body of the message using the given parameters
            addMeshMsg.shape = new APPShapeID();
            addMeshMsg.shape.shapeID = (uint)IPAddress.HostToNetworkOrder(
                (int)shapeID);
            addMeshMsg.shape.simID = (uint)IPAddress.HostToNetworkOrder(
                (int)m_simulationID);
            addMeshMsg.numPoints = (uint)IPAddress.HostToNetworkOrder(
                points.Length);
            numTriangles = triangles.Length / 3;
            addMeshMsg.numTriangles = (uint)IPAddress.HostToNetworkOrder(
                numTriangles);

            // Initialize the array that will hold the points of the
            // triangle mesh
            addMeshMsg.points = new APPVector[points.Length];
            for (i = 0; i < points.Length; i++)
            {
                addMeshMsg.points[i] = new APPVector();
                addMeshMsg.points[i].x = points[i].X;
                addMeshMsg.points[i].y = points[i].Y;
                addMeshMsg.points[i].z = points[i].Z;
            }

            // Initialize the array that will hold the triangle indices of
            // the triangle mesh
            addMeshMsg.triangles = new APPIndexVector[numTriangles];

            // Take each 3-tuple of indices and form a triangle out of it
            for (i = 0; i < numTriangles; i++)
            {
                addMeshMsg.triangles[i] = new APPIndexVector();
                addMeshMsg.triangles[i].p1 =
                    (uint)IPAddress.HostToNetworkOrder(triangles[i * 3]);
                addMeshMsg.triangles[i].p2 =
                    (uint)IPAddress.HostToNetworkOrder(triangles[i * 3 + 1]);
                addMeshMsg.triangles[i].p3 =
                    (uint)IPAddress.HostToNetworkOrder(triangles[i * 3 + 2]);
            }

            // Convert the message into a byte array, so that it can be sent
            // to the remote physics engine
            // Start by allocating the byte array
            addMeshArray = new byte[msgLength];

            // Convert the header into its byte array representation
            ConvertHeaderToBytes(addMeshMsg.header, ref addMeshArray, 0);

            // Convert the shape ID
            offset = m_APPHeaderSize;
            offset += ConvertAPPShapeIDToBytes(addMeshMsg.shape,
                ref addMeshArray, offset);

            // Convert the number of points field
            tempArray = BitConverter.GetBytes(addMeshMsg.numPoints);
            Buffer.BlockCopy(tempArray, 0, addMeshArray, offset, sizeof(uint));
            offset += sizeof(uint);

            // Convert the number of triangles field
            tempArray = BitConverter.GetBytes(addMeshMsg.numTriangles);
            Buffer.BlockCopy(tempArray, 0, addMeshArray, offset, sizeof(uint));
            offset += sizeof(uint);

            // Convert the points
            for (i = 0; i < addMeshMsg.points.Length; i++)
            {
                offset += ConvertAPPVectorToBytes(addMeshMsg.points[i],
                    ref addMeshArray, offset);
            }

            // Convert the index vectors
            for (i = 0; i < addMeshMsg.triangles.Length; i++)
            {
                offset += ConvertAPPIndexVectorToBytes(addMeshMsg.triangles[i],
                    ref addMeshArray, offset);
            }

            // Now that the byte array has been constructed, send it to the
            // remote physics engine
            m_packetManager.SendPacket(addMeshArray);

            // Increment the message index now that a message has been sent
            m_currentMessageIndex++;
        }

        /// <summary>
        /// Adds a height field to the remote physics engine.
        /// </summary>
        /// <param name="shapeID">The unique ID of the shape</param>
        /// <param name="numRows">The number of rows in the height field</param>
        /// <param name="numColumns">The number of columns in the
        /// height field</param>
        /// <param name="rowSpacing">The distance between the rows in the
        /// height field (in meters)</param>
        /// <param name="columnSpacing">The distance between columns in the
        /// height field (in meters)</param>
        /// <param name="posts">The height field values</param>
        public void AddHeightField(uint shapeID, uint numRows, uint numColumns,
            float rowSpacing, float columnSpacing, float[] posts)
        {
            APPAddHeightField addFieldMsg;
            uint msgLength;
            byte[] addFieldArray;
            byte[] tempArray;
            int offset;

            // Check to see if the messenger has been initialized; if it has
            // not, exit out
            if (!m_isInitialized)
                return;

            // Initialize the message and the header
            addFieldMsg = new APPAddHeightField();
            addFieldMsg.header = new APPHeader();

            // Initialize the header with the message type
            addFieldMsg.header.msgType = (ushort)IPAddress.HostToNetworkOrder(
                (short)MessageType.AddHeightField);

            // Calculate the size of the message parts and sum them up;
            // start with the header and the non-variable fields in the body
            msgLength = (uint)m_APPHeaderSize;
            msgLength = (uint)(m_APPHeaderSize + m_APPShapeIDSize +
                sizeof(uint) * 2 + sizeof(float) * 2);

            // Finally, add in the size for the posts of the height field
            msgLength += (uint)(numRows * numColumns * sizeof(float));

            // Initialize the rest of the header
            InitializeAPPHeader(ref addFieldMsg.header, msgLength);

            // Set the body of the message using the given parameters
            addFieldMsg.shape = new APPShapeID();
            addFieldMsg.shape.shapeID = (uint)IPAddress.HostToNetworkOrder(
                (int)shapeID);
            addFieldMsg.shape.simID = (uint)IPAddress.HostToNetworkOrder(
                (int)m_simulationID);
            addFieldMsg.numRows = (uint)IPAddress.HostToNetworkOrder(
                (int)numRows);
            addFieldMsg.numColumns = (uint)IPAddress.HostToNetworkOrder(
                (int)numColumns);
            addFieldMsg.rowSpacing = rowSpacing;
            addFieldMsg.columnSpacing = columnSpacing;

            // Initialize the array that will hold the posts of the height field
            addFieldMsg.posts = new float[posts.Length];
            for (int i = 0; i < posts.Length; i++)
            {
                addFieldMsg.posts[i] = posts[i];
            }

            // Convert the message to a byte array, so that it can be sent
            // to the remote physics engine
            // Start by allocating the byte array
            addFieldArray = new byte[msgLength];

            // Convert the header to its byte array representation
            ConvertHeaderToBytes(addFieldMsg.header, ref addFieldArray, 0);

            // Convert the shape ID
            offset = m_APPHeaderSize;
            offset += ConvertAPPShapeIDToBytes(addFieldMsg.shape,
                ref addFieldArray, offset);

            // Convert the number of rows and columns
            tempArray = BitConverter.GetBytes(addFieldMsg.numRows);
            Buffer.BlockCopy(tempArray, 0, addFieldArray, offset, sizeof(uint));
            offset += sizeof(uint);
            tempArray = BitConverter.GetBytes(addFieldMsg.numColumns);
            Buffer.BlockCopy(tempArray, 0, addFieldArray, offset, sizeof(uint));
            offset += sizeof(uint);

            // Convert the row and column spacing
            FloatToNetworkOrder(addFieldMsg.rowSpacing, ref addFieldArray,
                offset);
            offset += sizeof(float);
            FloatToNetworkOrder(addFieldMsg.columnSpacing, ref addFieldArray,
                offset);
            offset += sizeof(float);

            // Convert each of the height posts
            for (int i = 0; i < addFieldMsg.posts.Length; i++)
            {
                FloatToNetworkOrder(addFieldMsg.posts[i], ref addFieldArray,
                    offset);
                offset += sizeof(float);
            }

            // Now that the byte array has been constructed, send it to the
            // remote physics engine
            m_packetManager.SendPacket(addFieldArray);

            // Increment the message index now that a message has been sent
            m_currentMessageIndex++;
        }

        /// <summary>
        /// Removes a shape from the remote physics engine.
        /// </summary>
        /// <param name="shapeID">The unique ID of a shape in the remote
        /// physics engine</param>
        public void RemoveShape(uint shapeID)
        {
            APPRemoveShape removeShapeMsg;
            byte[] removeShapeArray;
            int offset;

            // Check to see if the messenger has been initialized; if it has
            // not, exit out
            if (!m_isInitialized)
                return;

            // Initialize the message and its header
            removeShapeMsg = new APPRemoveShape();
            removeShapeMsg.header = new APPHeader();

            // Initialize the header with the message type
            removeShapeMsg.header.msgType =
                (ushort)IPAddress.HostToNetworkOrder(
                    (short)MessageType.RemoveShape);

            // Initialize the rest of the header
            InitializeAPPHeader(ref removeShapeMsg.header,
                (uint)m_APPRemoveShapeSize);

            // Set the body of the message using the given parameters
            removeShapeMsg.shape = new APPShapeID();
            removeShapeMsg.shape.shapeID = (uint)IPAddress.HostToNetworkOrder(
                (int)shapeID);
            removeShapeMsg.shape.simID = (uint)IPAddress.HostToNetworkOrder(
                (int)m_simulationID);

            // Convert the message into a byte array, so that it can be sent
            // to the remote physics engine
            // Start by allocating the array
            removeShapeArray = new byte[m_APPRemoveShapeSize];

            // Convert the header into its byte array representation
            ConvertHeaderToBytes(removeShapeMsg.header,
                ref removeShapeArray, 0);

            // Convert the shape ID
            offset = m_APPHeaderSize;
            offset += ConvertAPPShapeIDToBytes(removeShapeMsg.shape,
                ref removeShapeArray, offset);

            // Now that the byte array has been constructed, send it to the
            // remote physics engine
            m_packetManager.SendPacket(removeShapeArray);

            // Increment the message index now that a message has been sent
            m_currentMessageIndex++;
        }

        /// <summary>
        /// Attaches an instance of a shape to an actor in the remote
        /// physics engine. The shape must be added to the engine before an
        /// instance of it can be attached.
        /// </summary>
        /// <param name="actorID">The unique ID of the actor to which the
        /// shape will be attached</param>
        /// <param name="shapeID">The unique ID of the shape to be
        /// attached</param>
        /// <param name="density">The density of the shape instance being
        /// attached (in grams/(meters^3)</param>
        /// <param name="staticFriction">The coefficient of static friction
        /// of the shape instance</param>
        /// <param name="kineticFriction">The coefficient of kinetic
        /// friction of the shape instance</param>
        /// <param name="restitution">The coefficient of restitution of the
        /// shape instance (bounciness)</param>
        /// <param name="orientation">The orientation of the shape relative
        /// to the actor</param>
        /// <param name="translation">The displacement from the origin of
        /// the actor</param>
        public void AttachShape(uint actorID, uint shapeID, float density,
            float staticFriction, float kineticFriction, float restitution, 
            OpenMetaverse.Quaternion orientation,
            OpenMetaverse.Vector3 translation)
        {
            APPAttachShape attachShapeMsg;
            byte[] attachShapeArray;
            int offset;

            // Check to see if the messenger has been initialized; if it
            // has not, exit out
            if (!m_isInitialized)
                return;

            // Initialize the message and its header
            attachShapeMsg = new APPAttachShape();
            attachShapeMsg.header = new APPHeader();

            // Initialize the header with the message type
            attachShapeMsg.header.msgType =
                (ushort)IPAddress.HostToNetworkOrder(
                    (short)MessageType.AttachShape);

            // Initialize the rest of the header
            InitializeAPPHeader(ref attachShapeMsg.header,
                (uint)m_APPAttachShapeSize);

            // Set the body of the message using the given parameters
            attachShapeMsg.actor = new APPActorID();
            attachShapeMsg.actor.actorID =
                (uint)IPAddress.HostToNetworkOrder((int)actorID);
            attachShapeMsg.actor.simID =
                (uint)IPAddress.HostToNetworkOrder((int)m_simulationID);
            attachShapeMsg.shape = new APPShapeID();
            attachShapeMsg.shape.shapeID =
                (uint)IPAddress.HostToNetworkOrder((int)shapeID);
            attachShapeMsg.shape.simID =
                (uint)IPAddress.HostToNetworkOrder(m_simulationID);
            attachShapeMsg.material = new APPMaterial();
            attachShapeMsg.material.density = density;
            attachShapeMsg.material.coeffStaticFriction = staticFriction;
            attachShapeMsg.material.coeffKineticFriction = kineticFriction;
            attachShapeMsg.material.coeffRestitution = restitution;
            attachShapeMsg.orientation = new APPQuat();
            attachShapeMsg.orientation.x = orientation.X;
            attachShapeMsg.orientation.y = orientation.Y;
            attachShapeMsg.orientation.z = orientation.Z;
            attachShapeMsg.orientation.w = orientation.W;
            attachShapeMsg.translation = new APPVector();
            attachShapeMsg.translation.x = translation.X;
            attachShapeMsg.translation.y = translation.Y;
            attachShapeMsg.translation.z = translation.Z;

            // Convert the message structure into a byte array, so that it
            // can be sent to the remote physics engine
            // Start by allocating the byte array
            attachShapeArray = new byte[m_APPAttachShapeSize];

            // Convert the header to its byte representation
            ConvertHeaderToBytes(attachShapeMsg.header,
                ref attachShapeArray, 0);

            // Convert the rest of the fields, starting with the actor ID
            offset = m_APPHeaderSize;
            offset += ConvertAPPActorIDToBytes(attachShapeMsg.actor,
                ref attachShapeArray, offset);

            // Convert the shape ID
            offset += ConvertAPPShapeIDToBytes(attachShapeMsg.shape,
                ref attachShapeArray, offset);

            // Convert the material
            offset += ConvertAPPMaterialToBytes(attachShapeMsg.material,
                ref attachShapeArray, offset);

            // Convert the orientation
            offset += ConvertAPPQuaternionToBytes(attachShapeMsg.orientation,
                ref attachShapeArray, offset);

            // Convert the position
            offset += ConvertAPPVectorToBytes(attachShapeMsg.translation,
                ref attachShapeArray, offset);

            // Now that the byte array has been constructed, send it to the
            // remote physics engine
            m_packetManager.SendPacket(attachShapeArray);

            // Increment the message index now that a message has been sent
            m_currentMessageIndex++;
        }

        /// <summary>
        /// Updates the physical properties of a shape attached to an actor
        /// in the remote physics engine.
        /// </summary>
        /// <param name="actorID">The unique identifier of the actor to which
        /// the desired shape is attached</param>
        /// <param name="shapeID">The unique identifier of the shape whose
        /// material properties are to be modified</param>
        /// <param name="staticFriction">The new coefficient of static friction
        /// for the shape</param>
        /// <param name="kineticFriction">The new coefficient of kinetic
        /// friction for the shape</param>
        /// <param name="restitution">The new coefficient of restitution for
        /// the shape</param>
        /// <param name="density">The new density of the shape</param>
        public void UpdateShapeMaterial(uint actorID, uint shapeID,
            float staticFriction, float kineticFriction, float restitution,
            float density)
        {
            APPUpdateShapeMaterial matMsg;
            byte[] matArray;
            int offset;

            // Check to see if the messenger has been initialized; if it has
            // not, exit out
            if (!m_isInitialized)
                return;

            // Initialize the message and its header
            matMsg = new APPUpdateShapeMaterial();
            matMsg.header = new APPHeader();

            // Initialize the header with the message type
            matMsg.header.msgType =
                (ushort) IPAddress.HostToNetworkOrder(
                    (short) MessageType.UpdateShapeMaterial);

            // Initialize the rest of the header
            InitializeAPPHeader(ref matMsg.header,
                (uint) m_APPUpdateShapeMaterialSize);

            // Set the body of the mesage using hte given parameters
            matMsg.actor = new APPActorID();
            matMsg.actor.actorID =
                (uint) IPAddress.HostToNetworkOrder((int) actorID);
            matMsg.actor.simID =
                (uint) IPAddress.HostToNetworkOrder((int) m_simulationID);
            matMsg.shape = new APPShapeID();
            matMsg.shape.shapeID =
                (uint) IPAddress.HostToNetworkOrder((int) shapeID);
            matMsg.shape.simID =
                (uint) IPAddress.HostToNetworkOrder((int) m_simulationID);
            matMsg.material = new APPMaterial();
            matMsg.material.density = density;
            matMsg.material.coeffStaticFriction = staticFriction;
            matMsg.material.coeffKineticFriction = kineticFriction;
            matMsg.material.coeffRestitution = restitution;

            // Convert the message structure into a byte array, so that it can
            // be sent over to the remote physics engine
            // Start by allocating the byte array
            matArray = new byte[m_APPUpdateShapeMaterialSize];

            // Convert the header to its byte array representation
            ConvertHeaderToBytes(matMsg.header, ref matArray, 0);

            // Convert the rest of the fields, starting with the actor ID
            offset = m_APPHeaderSize;
            offset += ConvertAPPActorIDToBytes(matMsg.actor, ref matArray,
                offset);

            // Convert the shape ID
            offset += ConvertAPPShapeIDToBytes(matMsg.shape, ref matArray,
                offset);

            // Convert the material
            offset += ConvertAPPMaterialToBytes(matMsg.material,
                ref matArray, offset);

            // Now that the byte array has been constructed, send it to the
            // remote physics engine
            m_packetManager.SendPacket(matArray);

            // Increment the message index now that a message has been sent
            m_currentMessageIndex++;
        }
        

        /// <summary>
        /// Detaches a shape instance from an actor in the remote
        /// physics engine.
        /// </summary>
        /// <param name="actorID">The unique ID from which the shape
        /// instance will be detached</param>
        /// <param name="shapeID">The unique ID of the shape to be
        /// detached</param>
        public void DetachShape(uint actorID, uint shapeID)
        {
            APPDetachShape detachShapeMsg;
            byte[] detachShapeArray;
            int offset;

            // Check to see if the messenger has been initialized; if it has
            // not, exit out
            if (!m_isInitialized)
                return;

            // Initialize the message and its header
            detachShapeMsg = new APPDetachShape();
            detachShapeMsg.header = new APPHeader();

            // Initialize the header with the message type
            detachShapeMsg.header.msgType =
                (ushort)IPAddress.HostToNetworkOrder(
                    (short)MessageType.DetachShape);

            // Initialize the rest of the header
            InitializeAPPHeader(ref detachShapeMsg.header,
                (uint)m_APPDetachShapeSize);

            // Set the body of the message using the given parameters
            detachShapeMsg.actor = new APPActorID();
            detachShapeMsg.actor.actorID =
                (uint)IPAddress.HostToNetworkOrder((int)actorID);
            detachShapeMsg.actor.simID =
                (uint)IPAddress.HostToNetworkOrder((int)m_simulationID);
            detachShapeMsg.shape = new APPShapeID();
            detachShapeMsg.shape.shapeID =
                (uint)IPAddress.HostToNetworkOrder((int)shapeID);
            detachShapeMsg.shape.simID =
                (uint)IPAddress.HostToNetworkOrder(m_simulationID);

            // Convert the message structure into a byte array, so that it
            // can be sent to the remote physics engine
            // Start by allocating the byte array
            detachShapeArray = new byte[m_APPDetachShapeSize];

            // Convert the header to its byte representation
            ConvertHeaderToBytes(detachShapeMsg.header,
                ref detachShapeArray, 0);

            // Convert the rest of the fields, starting with the actorID
            offset = m_APPHeaderSize;
            offset += ConvertAPPActorIDToBytes(detachShapeMsg.actor,
                ref detachShapeArray, offset);

            // Convert the shape ID
            offset += ConvertAPPShapeIDToBytes(detachShapeMsg.shape,
                ref detachShapeArray, offset);

            // Now that the packet has been constructed, send it to the
            // remote physics engine
            m_packetManager.SendPacket(detachShapeArray);

            // Increment the message index now that a message has been sent
            m_currentMessageIndex++;
        }

        /// <summary>
        /// Advances the simulation time of the remote physics scene.
        /// </summary>
        /// <param name="time">The time to advance (in seconds)</param>
        public void AdvanceTime(float time)
        {
            APPAdvanceTime advTimeMsg;
            byte[] advTimeArray;
            byte[] tempArray;
            int offset;

            // Check to see if the messenger has been initialized; if it has
            // not, exit out
            if (!m_isInitialized)
                return;

            // Initialize the message and its header
            advTimeMsg = new APPAdvanceTime();
            advTimeMsg.header = new APPHeader();

            // Initialize the header with the message type
            advTimeMsg.header.msgType =
                (ushort)IPAddress.HostToNetworkOrder(
                    (short)MessageType.AdvanceTime);

            // Initialize the rest of the header
            InitializeAPPHeader(ref advTimeMsg.header,
                (uint)m_APPAdvanceTimeSize);

            // Initialize the simulation ID
            advTimeMsg.simID = (uint)IPAddress.HostToNetworkOrder(
                (int)m_simulationID);

            // Set the body of the message using the given parameters
            advTimeMsg.time = time;

            // Convert the message structure into a byte array, so that it
            // can be sent to the remote physics engine
            // Start by allocating the byte array
            advTimeArray = new byte[m_APPAdvanceTimeSize];

            // Convert the header to its byte representation
            ConvertHeaderToBytes(advTimeMsg.header, ref advTimeArray, 0);

            // Convert the rest of the fields, starting with the simID field
            offset = m_APPHeaderSize;
            tempArray = BitConverter.GetBytes(advTimeMsg.simID);
            Buffer.BlockCopy(tempArray, 0, advTimeArray, offset, sizeof(uint));
            offset += sizeof(uint);

            // Convert the time field
            FloatToNetworkOrder(advTimeMsg.time, ref advTimeArray, offset);

            // Send the packet to the remote physics engine
            m_packetManager.SendPacket(advTimeArray);

            // Increment the message index now that a message has been sent
            m_currentMessageIndex++;
        }

        /// <summary>
        /// Applies a force to an actor in the remote physics engine.
        /// </summary>
        /// <param name="actorID">The unique identifier of the actor to which
        /// the force is being applied</param>
        /// <param name="force">The force that is being applied</param>
        public void ApplyForce(uint actorID, OpenMetaverse.Vector3 force)
        {
            APPApplyForce applyForceMsg;
            byte[] applyForceArray;
            int offset;

            // Check to see if the messenger has been initialized; if it has
            // not, exit out
            if (!m_isInitialized)
                return;

            // Initialize the message and its header
            applyForceMsg = new APPApplyForce();
            applyForceMsg.header = new APPHeader();

            // Initialize the header with the message type
            applyForceMsg.header.msgType =
                (ushort)IPAddress.HostToNetworkOrder(
                    (short)MessageType.ApplyForce);

            // Initialize the rest of the header
            InitializeAPPHeader(ref applyForceMsg.header,
                (uint)m_APPApplyForceSize);

            // Set the body of the message using given parameters
            applyForceMsg.actor = new APPActorID();
            applyForceMsg.actor.actorID =
                (uint)IPAddress.HostToNetworkOrder((int)actorID);
            applyForceMsg.actor.simID =
                (uint)IPAddress.HostToNetworkOrder((int)m_simulationID);
            applyForceMsg.force = new APPVector();
            applyForceMsg.force.x = force.X;
            applyForceMsg.force.y = force.Y;
            applyForceMsg.force.z = force.Z;

            // Convert the message structure into a byte array, so that it
            // can be sent to the remote physics engine
            // Start by allocating the byte array
            applyForceArray = new byte[m_APPApplyForceSize];

            // Convert the header to its byte representation
            ConvertHeaderToBytes(applyForceMsg.header, ref applyForceArray, 0);

            // Convert the rest of the fields, starting with the actorID;
            offset = m_APPHeaderSize;
            offset += ConvertAPPActorIDToBytes(applyForceMsg.actor,
                ref applyForceArray, offset);
            ConvertAPPVectorToBytes(applyForceMsg.force, ref applyForceArray,
                offset);

            // Send the packet to the remote physics engine
            m_packetManager.SendPacket(applyForceArray);

            // Increment the message index now that a message has been sent
            m_currentMessageIndex++;
        }

        /// <summary>
        /// Applies a torque to an actor in the remote physics engine.
        /// </summary>
        /// <param name="actorID">The unique identifier of the actor to which
        /// the torque is being applied</param>
        public void ApplyTorque(uint actorID, OpenMetaverse.Vector3 torque)
        {
            APPApplyTorque applyTorqueMsg;
            byte[] applyTorqueArray;
            int offset;

            // Check to see if the messenger has been initialized; if it has
            // not, exit out
            if (!m_isInitialized)
                return;

            // Initialize the message and its header
            applyTorqueMsg = new APPApplyTorque();
            applyTorqueMsg.header = new APPHeader();

            // Set the body of the message using the given parameters
            applyTorqueMsg.actor = new APPActorID();
            applyTorqueMsg.actor.actorID =
                (uint) IPAddress.HostToNetworkOrder((int) actorID);
            applyTorqueMsg.actor.simID =
                (uint) IPAddress.HostToNetworkOrder((int) m_simulationID);
            applyTorqueMsg.torque = new APPVector();
            applyTorqueMsg.torque.x = torque.X;
            applyTorqueMsg.torque.y = torque.Y;
            applyTorqueMsg.torque.z = torque.Z;

            // Convert the message structure into a byte array, so that it
            // can be sent to the remote physics engine
            applyTorqueArray = new byte[m_APPApplyTorqueSize];

            // Convert the header to its byte representation
            ConvertHeaderToBytes(applyTorqueMsg.header,
                ref applyTorqueArray, 0);

            // Convert the rest of the fields, starting with the actorID
            offset = m_APPHeaderSize;
            offset += ConvertAPPActorIDToBytes(applyTorqueMsg.actor,
                ref applyTorqueArray, offset);
            ConvertAPPVectorToBytes(applyTorqueMsg.torque, ref applyTorqueArray,
                offset);

            // Send the packet to the remote physics engine
            m_packetManager.SendPacket(applyTorqueArray);

            // Increment the message index now that a message has been sent
            m_currentMessageIndex++;
        }

        /// <summary>
        /// Partially initializes the header of an APP message.
        /// </summary>
        /// <param name="msgHeader">The header to be initialized</param>
        /// <param name="msgLength">The length of the entire message
        /// (header and body)</param>
        protected void InitializeAPPHeader(ref APPHeader msgHeader,
            uint msgLength)
        {
            // Initialize the given header with the protocol version used by
            // this plugin
            msgHeader.version = (ushort)IPAddress.HostToNetworkOrder(
                m_protocolVersion);

            // TODO: Initialize the timestamp

            // Set the index of the message to the next available index
            msgHeader.msgIndex = (ushort)IPAddress.HostToNetworkOrder(
                m_currentMessageIndex);

            // Initialize the size of the packet with the given length
            msgHeader.length = (uint)IPAddress.HostToNetworkOrder(
                (int)msgLength);
        }

        /// <summary>
        /// Converts an APPHeader structure into its byte array representation.
        /// </summary>
        /// <param name="header">APPHeader structure to be converted</param>
        /// <param name="byteArray">Byte array to which the converted result
        /// will be stored. The array must have been pre-allocated</param>
        /// <param name="resultIndex">The starting index of the location in
        /// the "byteArray" at which the result should be stored</param>
        /// <returns>The number of bytes written to "byteArray"</returns>
        protected int ConvertHeaderToBytes(APPHeader header,
            ref byte[] byteArray, int resultIndex)
        {
            byte[] tempArray;
            int offset;

            // Convert each of the fields into their byte representations
            // Version field
            offset = resultIndex;
            tempArray = BitConverter.GetBytes(header.version);
            Buffer.BlockCopy(tempArray, 0, byteArray, offset, sizeof(ushort));
            offset += sizeof(ushort);

            // Message type
            tempArray = BitConverter.GetBytes(header.msgType);
            Buffer.BlockCopy(tempArray, 0, byteArray, offset, sizeof(ushort));
            offset += sizeof(ushort);

            // Message index
            tempArray = BitConverter.GetBytes(header.msgIndex);
            Buffer.BlockCopy(tempArray, 0, byteArray, offset, sizeof(uint));
            offset += sizeof(uint);

            // Length
            tempArray = BitConverter.GetBytes(header.length);
            Buffer.BlockCopy(tempArray, 0, byteArray, offset, sizeof(uint));
            offset += sizeof(uint);

            // Timestamp
            tempArray = BitConverter.GetBytes(header.timestamp);
            Buffer.BlockCopy(tempArray, 0, byteArray, offset, sizeof(float));
            offset += sizeof(float);

            // Add in the reserved fields
            offset += sizeof(uint) * 2;

            // Calculate how many bytes that have been written and return it
            return (offset - resultIndex);
        }

        /// <summary>
        /// Converts a single-precision float value from host to network
        /// byte order.
        /// </summary>
        /// <param name="hostValue">The float value (in host byte order) to be
        /// converted into network byte order)</param>
        /// <param name="byteArray">The byte array into which the converted
        /// result should be stored</param>
        /// <param name="resultIndex">The location in "byteArray" into which
        /// the result should be stored</param>
        protected void FloatToNetworkOrder(float hostValue,
            ref byte[] byteArray, int resultIndex)
        {
            byte[] tempArray;

            // Convert the float into a byte array
            tempArray = BitConverter.GetBytes(hostValue);

            // Check to see if this machine is little endian
            if (BitConverter.IsLittleEndian)
            {
                // Convert to big endian, since network order is always
                // big endian
                Array.Reverse(tempArray);
            }

            // Copy the converted byte array into the result array
            Buffer.BlockCopy(tempArray, 0, byteArray, resultIndex,
                sizeof(float));
        }

        /// <summary>
        /// Converts a single-precision float value from network to host
        /// byte order.
        /// </summary>
        /// <param name="byteArray">The byte array which contains the desired
        /// float value (in network byte order)</param>
        /// <param name="startIndex">The location in "byteArray" which
        /// contains the float value to be converted</param>
        /// <returns>The converted float value in host byte order</returns>
        protected float FloatToHostOrder(byte[] byteArray, int startIndex)
        {
            float result;

            // Copy the float value to a temporary array
            Buffer.BlockCopy(byteArray, startIndex, m_floatArray, 0,
                sizeof(float));

            // Check to see if this machine is little endian
            if (BitConverter.IsLittleEndian)
            {
                // Convert to little endian, since network order is always
                // big endian
                Array.Reverse(m_floatArray);
            }

            // Convert the bytes into a float
            result = BitConverter.ToSingle(m_floatArray, 0);
            return result;
        }

        /// <summary>
        /// Converts an APPVector structure into its byte array representation.
        /// </summary>
        /// <param name="vector">The vector to be converted</param>
        /// <param name="byteArray">Byte array to which the converted result
        /// will be stored. The array must have been pre-allocated</param>
        /// <param name="resultIndex">The starting index of the location in
        /// the "byteArray" at which the result should be stored</param>
        /// <returns>The number of bytes written to "byteArray"</returns>
        protected int ConvertAPPVectorToBytes(APPVector vector,
            ref byte[] byteArray, int resultIndex)
        {
            int offset;

            // Convert each of the fields into their byte representation
            // X
            offset = resultIndex;
            FloatToNetworkOrder(vector.x, ref byteArray, offset);
            offset += sizeof(float);

            // Y
            FloatToNetworkOrder(vector.y, ref byteArray, offset);
            offset += sizeof(float);

            // Z
            FloatToNetworkOrder(vector.z, ref byteArray, offset);
            offset += sizeof(float);

            // Calculate how many bytes that have been written and return it
            return (offset - resultIndex);
        }

        /// <summary>
        /// Converts an APPQuat structure into its byte array representation.
        /// </summary>
        /// <param name="quat">THe quaternion to be converted</param>
        /// <param name="byteArray">Byte array to which the converted result
        /// will be stored. The array must have been pre-allocated</param>
        /// <param name="resultIndex">The starting index of the location in
        /// the "byteArray" at which the result should be stored</param>
        /// <returns>The number of bytes written to "byteArray"</returns>
        protected int ConvertAPPQuaternionToBytes(APPQuat quat,
            ref byte[] byteArray, int resultIndex)
        {
            int offset;

            // Convert each of the fields into their byte representation
            // X
            offset = resultIndex;
            FloatToNetworkOrder(quat.x, ref byteArray, offset);
            offset += sizeof(float);

            // Y
            FloatToNetworkOrder(quat.y, ref byteArray, offset);
            offset += sizeof(float);

            // Z
            FloatToNetworkOrder(quat.z, ref byteArray, offset);
            offset += sizeof(float);

            // W
            FloatToNetworkOrder(quat.w, ref byteArray, offset);
            offset += sizeof(float);

            // Calculate how many bytes that have been written and return it
            return (offset - resultIndex);
        }

        /// <summary>
        /// Converts an APPIndexVector structure into its byte array
        /// representation.
        /// </summary>
        /// <param name="indexVector">The index vector to be converted</param>
        /// <param name="byteArray">Byte array into which the converted result
        /// will be stored. The array must have been pre-allocated</param>
        /// <param name="resultIndex">The starting index of the location in
        /// the "byteArray" at which the result should be stored</param>
        /// <returns>The number of bytes written to "byteArray"</returns>
        protected int ConvertAPPIndexVectorToBytes(APPIndexVector indexVector,
            ref byte[] byteArray, int resultIndex)
        {
            byte[] tempArray;
            int offset;

            // Convert each of the fields into their byte representation
            // First index
            offset = resultIndex;
            tempArray = BitConverter.GetBytes(indexVector.p1);
            Buffer.BlockCopy(tempArray, 0, byteArray, offset, sizeof(uint));
            offset += sizeof(uint);

            // Second index
            tempArray = BitConverter.GetBytes(indexVector.p2);
            Buffer.BlockCopy(tempArray, 0, byteArray, offset, sizeof(uint));
            offset += sizeof(uint);

            // Third index
            tempArray = BitConverter.GetBytes(indexVector.p3);
            Buffer.BlockCopy(tempArray, 0, byteArray, offset, sizeof(uint));
            offset += sizeof(uint);

            // Calculate how many bytes that have been written and return it
            return (offset - resultIndex);
        }

        /// <summary>
        /// Converts an APPActorID structure into its byte array representation.
        /// </summary>
        /// <param name="actorID">The actor ID to be converted</param>
        /// <param name="byteArray">Byte array to which the converted result
        /// will be stored. The array must have been pre-allocated</param>
        /// <param name="resultIndex">The starting index of the location in the
        /// "byteArray" at which the result should be stored</param>
        /// <returns>The number of bytes written to "byteArray"</returns>
        protected int ConvertAPPActorIDToBytes(APPActorID actorID,
            ref byte[] byteArray, int resultIndex)
        {
            byte[] tempArray;
            int offset;

            // Convert each of the fields into their byte array representation
            // Sim ID
            offset = resultIndex;
            tempArray = BitConverter.GetBytes(actorID.simID);
            Buffer.BlockCopy(tempArray, 0, byteArray, offset, sizeof(uint));
            offset += sizeof(uint);

            // Actor ID
            tempArray = BitConverter.GetBytes(actorID.actorID);
            Buffer.BlockCopy(tempArray, 0, byteArray, offset, sizeof(uint));
            offset += sizeof(uint);

            // Calculate how many bytes that have been written and return it
            return (offset - resultIndex);
        }

        /// <summary>
        /// Converts an APPJointID structure into its byte array representation.
        /// </summary>
        /// <param name="jointID">The joint ID to be converted</param>
        /// <param name="byteArray">Byte array to which the converted result
        /// will be stored. The array must have been pre-allocated</param>
        /// <param name="resultIndex">The starting index of the location in the
        /// "byteArray" at which the result should be stored</param>
        /// <returns>The number of bytes written to "byteArray"</returns>
        protected int ConvertAPPJointIDToBytes(APPJointID jointID,
            ref byte[] byteArray, int resultIndex)
        {
            byte[] tempArray;
            int offset;

            // Convert each of the fields into their byte array representation
            // Sim ID
            offset = resultIndex;
            tempArray = BitConverter.GetBytes(jointID.simID);
            Buffer.BlockCopy(tempArray, 0, byteArray, offset, sizeof(uint));
            offset += sizeof(uint);

            // Joint ID
            tempArray = BitConverter.GetBytes(jointID.jointID);
            Buffer.BlockCopy(tempArray, 0, byteArray, offset, sizeof(uint));
            offset += sizeof(uint);

            // Calculate how many bytes that have been written and return it
            return (offset - resultIndex);
        }

        /// <summary>
        /// Converts an APPShapeID structure into its byte array representation.
        /// </summary>
        /// <param name="shapeID">The shape ID to be converted</param>
        /// <param name="byteArray">Byte array into which the converted result
        /// will be stored. The array must have been pre-allocated</param>
        /// <param name="resultIndex">The starting index of the location in
        /// "byteArray" at which the result should be stored</param>
        /// <returns>The number of bytes written to "byteArray"</returns>
        protected int ConvertAPPShapeIDToBytes(APPShapeID shapeID,
            ref byte[] byteArray, int resultIndex)
        {
            byte[] tempArray;
            int offset;

            // Convert each of the fields into their byte array representation
            // Sim ID
            offset = resultIndex;
            tempArray = BitConverter.GetBytes(shapeID.simID);
            Buffer.BlockCopy(tempArray, 0, byteArray, offset, sizeof(uint));
            offset += sizeof(uint);

            // Shape ID
            tempArray = BitConverter.GetBytes(shapeID.shapeID);
            Buffer.BlockCopy(tempArray, 0, byteArray, offset, sizeof(uint));
            offset += sizeof(uint);

            // Calculate how many bytes that have been written and return it
            return (offset - resultIndex);
        }

        /// <summary>
        /// Converts an APPMaterial structure into its byte array
        /// representation.
        /// </summary>
        /// <param name="material">The material to be converted</param>
        /// <param name="byteArray">Byte array into which the converted result
        /// will be stored.  The array must have been pre-allocated</param>
        /// <param name="resultIndex">The starting index of the location in
        /// "byteArray" at which the result should be stored</param>
        /// <returns>The number of bytes written to "byteArray"</returns>
        protected int ConvertAPPMaterialToBytes(APPMaterial material,
            ref byte[] byteArray, int resultIndex)
        {
            int offset;

            // Convert each of the fields into their byte array representation
            // Density
            offset = resultIndex;
            FloatToNetworkOrder(material.density, ref byteArray, offset);
            offset += sizeof(float);

            // Coefficient of static friction
            FloatToNetworkOrder(material.coeffStaticFriction, ref byteArray,
                offset);
            offset += sizeof(float);

            // Coefficient of kinetic friction
            FloatToNetworkOrder(material.coeffKineticFriction, ref byteArray,
                offset);
            offset += sizeof(float);

            // Coefficient of restitution
            FloatToNetworkOrder(material.coeffRestitution, ref byteArray,
                offset);
            offset += sizeof(float);

            // Calculate how many bytes that have been written and return it
            return (offset - resultIndex);
        }

        /// <summary>
        /// Converts a byte array subset into an APP header.
        /// </summary>
        /// <param name="byteArray">The byte array containing the APP header
        /// data</param>
        /// <param name="startIndex">The index in the "byteArray" at which
        /// to start processing</param>
        /// <param name="header">The resulting structure containing
        /// the header</param>
        /// <returns>Flag indicating whether the header was extracted and
        /// converted successfully</returns>
        protected bool ProcessHeader(byte[] byteArray, int startIndex,
            out APPHeader header)
        {
            int offset;

            // Allocate the header structure that will be the result
            header = new APPHeader();

            // Convert the protocol version
            offset = startIndex;
            header.version = (ushort)IPAddress.NetworkToHostOrder(
                BitConverter.ToInt16(byteArray, offset));
            offset += sizeof(ushort);

            // Check to see if the protocol version is incompatible with
            // this messenger
            if (header.version != m_protocolVersion)
            {
                // This messenger is not designed to handle this protocol
                // version, so display a warning, clean up and exit
                m_log.WarnFormat("{0}: Processed message: ARCHIMEDES Physics " +
                    "Protocol version mismatch.", LogHeader);
                return false;
            }

            // Convert the message type
            header.msgType = (ushort)IPAddress.NetworkToHostOrder(
                BitConverter.ToInt16(byteArray, offset));
            offset += sizeof(ushort);

            // Convert the message index
            header.msgIndex = (uint)IPAddress.NetworkToHostOrder(
                BitConverter.ToInt32(byteArray, offset));
            offset += sizeof(uint);

            // Convert the message length
            header.length = (uint)IPAddress.NetworkToHostOrder(
                BitConverter.ToInt32(byteArray, offset));
            offset += sizeof(uint);

            // Convert the timestamp
            header.timestamp = BitConverter.ToSingle(byteArray, offset);

            // Indicate that the header was successfully converted
            return true;
        }

        /// <summary>
        /// Converts a byte array subsection into an APP actor ID.
        /// </summary>
        /// <param name="byteArray">The byte array containing the actor ID
        /// data</param>
        /// <param name="startIndex">Specifies the starting index in
        /// "byteArray"</param>
        /// <param name="actorID">The resulting structure containing the
        /// actor ID</param>
        protected void ProcessActorID(byte[] byteArray, int startIndex,
            out APPActorID actorID)
        {
            int offset;

            // Allocate the actor ID structure that will be the result
            actorID = new APPActorID();

            // Convert the simulation ID
            offset = startIndex;
            actorID.simID = (uint)IPAddress.NetworkToHostOrder(
                BitConverter.ToInt32(byteArray, offset));
            offset += sizeof(uint);

            // Convert the actor ID
            actorID.actorID = (uint)IPAddress.NetworkToHostOrder(
                BitConverter.ToInt32(byteArray, offset));
        }

        /// <summary>
        /// Processes a byte array as a logon ready message and activates the
        /// appropriate callback.
        /// </summary>
        /// <param name="byteArray">The byte array containing the logon ready
        /// message data</param>
        /// <param name="startIndex">The index in the "byteArray" at which to
        /// start processing</param>
        /// <returns>Flag indicating whether the logon ready message was
        /// extracted and converted successfully</returns>
        protected bool ProcessLogonReadyMessage(byte[] byteArray,
            int startIndex)
        {
            ushort msgType;
            int offset;
            uint simID;
            LogonReadyHandler logonReadyHandler;

            // Ensure that this is a logon ready message by reading in the
            // message type field
            msgType = (ushort)IPAddress.NetworkToHostOrder(
                BitConverter.ToInt16(byteArray, startIndex + 2));
            if (msgType != (ushort)MessageType.LogonReady)
            {
                // This is not a logon ready message, so warn the user and
                // exit unsuccessfully
                m_log.WarnFormat("{0}: Invalid APPLogonReady message received!",
                    LogHeader);
                return false;
            }

            // Make sure that the array length corresponds to the expected
            // size of the messsage
            if (byteArray.Length - startIndex < m_APPLogonReadySize)
            {
                // There is not enough data, so warn the user and exit
                // unsuccessfully
                m_log.WarnFormat("{0}: Insufficient packet data!", LogHeader);
                return false;
            }

            // Convert the simulation ID
            offset = startIndex + m_APPHeaderSize;
            simID = (uint)IPAddress.NetworkToHostOrder(
                BitConverter.ToInt32(byteArray, offset));

            // Get the subscribers that need to be notified of
            // this logon ready message
            logonReadyHandler = OnLogonReadyEvent;

            // Check to see if there are any that are listening
            // to the event
            if (logonReadyHandler != null)
            {
                // Call the logon ready callback
                logonReadyHandler(simID);
            }

            // Indicate that the conversion was successful
            return true;
        }

        /// <summary>
        /// Processes a byte array as a set static actor message and activates
        /// the appropriate callback.
        /// </summary>
        /// <param name="byteArray">The byte array containing the set static
        /// actor message data</param>
        /// <param name="startIndex">The index in the "byteArray" at which to
        /// start processing</param>
        /// <returns>Flag indicating whether the set static actor message was
        /// extracted and converted successfully</returns>
        protected bool ProcessSetStaticActorMessage(byte[] byteArray,
            int startIndex)
        {
            int offset;
            UpdateStaticActorHandler staticActorHandler;
            APPActorID actorID;
            OpenMetaverse.Vector3 position;
            OpenMetaverse.Quaternion orientation;

            // Make sure that the array length corresponds to the expected
            // size of the messsage
            if (byteArray.Length - startIndex < m_APPSetStaticActorSize)
            {
                // There is not enough data, so warn the user and exit
                // unsuccessfully
                m_log.WarnFormat("{0}: Insufficient packet data!", LogHeader);
                return false;
            }

            // Convert the actor ID
            offset = startIndex + m_APPHeaderSize;
            ProcessActorID(byteArray, offset, out actorID);
            offset += m_APPActorIDSize;

            // Convert the position vector portion into three floats
            position.X = FloatToHostOrder(byteArray, offset);
            offset += sizeof(float);
            position.Y = FloatToHostOrder(byteArray, offset);
            offset += sizeof(float);
            position.Z = FloatToHostOrder(byteArray, offset);
            offset += sizeof(float);

            // Convert the orientation quaternion portion into four floats
            orientation.X = FloatToHostOrder(byteArray, offset);
            offset += sizeof(float);
            orientation.Y = FloatToHostOrder(byteArray, offset);
            offset += sizeof(float);
            orientation.Z = FloatToHostOrder(byteArray, offset);
            offset += sizeof(float);
            orientation.W = FloatToHostOrder(byteArray, offset);

            // Get the subscribers that need to be notified of
            // this actor update
            staticActorHandler = OnStaticActorUpdateEvent;

            // Check to see if there are any that are
            // listening to the event
            if (staticActorHandler != null)
            {
                // Call the static actor update callback with
                // the newly-received information
                staticActorHandler(actorID.actorID, position, orientation);
            }

            // Indicate that the conversion was sucessful
            return true;
        }

        /// <summary>
        /// Processes a byte array as a set dynamic actor message and activates
        /// the appropriate callback.
        /// </summary>
        /// <param name="byteArray">The byte array containing the set dynamic
        /// actor message data</param>
        /// <param name="startIndex">The index in the "byteArray" at which to
        /// start processing</param>
        /// <returns>Flag indicating whether the set dynamic actor message
        /// was extracted and converted successfully</returns>
        protected bool ProcessSetDynamicActorMessage(byte[] byteArray,
            int startIndex)
        {
            int offset;
            UpdateDynamicActorHandler dynamicActorHandler;
            APPActorID actorID;
            OpenMetaverse.Vector3 position;
            OpenMetaverse.Quaternion orientation;
            OpenMetaverse.Vector3 linearVelocity;
            OpenMetaverse.Vector3 angularVelocity;
            float gravityModifier;
            
            // Make sure that the array length corresponds to the expected
            // size of the messsage
            if (byteArray.Length - startIndex < m_APPSetDynamicActorSize)
            {
                // There is not enough data, so warn the user and
                // exit unsuccessfully
                m_log.WarnFormat("{0}: Insufficient packet data!", LogHeader);
                return false;
            }

            // Convert the actor ID
            offset = startIndex + m_APPHeaderSize;
            ProcessActorID(byteArray, offset, out actorID);
            offset += m_APPActorIDSize;

            // Convert the position vector portion
            position.X = FloatToHostOrder(byteArray, offset);
            offset += sizeof(float);
            position.Y = FloatToHostOrder(byteArray, offset);
            offset += sizeof(float);
            position.Z = FloatToHostOrder(byteArray, offset);
            offset += sizeof(float);

            // Convert the orientation quaternion portion
            orientation.X = FloatToHostOrder(byteArray, offset);
            offset += sizeof(float);
            orientation.Y = FloatToHostOrder(byteArray, offset);
            offset += sizeof(float);
            orientation.Z = FloatToHostOrder(byteArray, offset);
            offset += sizeof(float);
            orientation.W = FloatToHostOrder(byteArray, offset);
            offset += sizeof(float);

            // Convert the gravity modifier
            gravityModifier = FloatToHostOrder(byteArray,
                offset);
            offset += sizeof(float);

            // Convert the linear velocity portion
            linearVelocity.X = FloatToHostOrder(byteArray,
                offset);
            offset += sizeof(float);
            linearVelocity.Y = FloatToHostOrder(byteArray,
                offset);
            offset += sizeof(float);
            linearVelocity.Z = FloatToHostOrder(byteArray,
                offset);
            offset += sizeof(float);

            // Convert the angular velocity portion
            angularVelocity.X = FloatToHostOrder(byteArray,
                offset);
            offset += sizeof(float);
            angularVelocity.Y = FloatToHostOrder(byteArray,
                offset);
            offset += sizeof(float);
            angularVelocity.Z = FloatToHostOrder(byteArray,
                offset);

            // Get the subscribers that need to be notified of
            // this actor update
            dynamicActorHandler = OnDynamicActorUpdateEvent;

            // Check to see if there are any that are listening to the event
            if (dynamicActorHandler != null)
            {
                // Call the dynamic actor update callback with
                // the newly-received information
                dynamicActorHandler(actorID.actorID, position, orientation,
                    linearVelocity, angularVelocity);
            }

            // Indicate that the conversion was successful
            return true;
        }

        /// <summary>
        /// Processes a byte array as a update dynamic actor mass message and
        /// activates the appropriate callback.
        /// </summary>
        /// <param name="byteArray">The byte array containing the update dynamic
        /// actor mass message data</param>
        /// <param name="startIndex">The index in the "byteArray" at which to
        /// start processing</param>
        /// <returns>Flag indicating whether the update dynamic actor mass
        /// message was extracted and converted successfully</returns>
        protected bool ProcessUpdateDynamicActorMassMessage(byte[] byteArray,
            int startIndex)
        {
            int offset;
            UpdateDynamicActorMassHandler actorMassHandler;
            APPActorID actorID;
            float actorMass;

            // Make sure that the array length corresponds to the expected
            // size of the messsage
            if (byteArray.Length - startIndex < m_APPUpdateDynamicActorMassSize)
            {
                // There is not enough data, so warn the user and
                // exit unsuccessfully
                m_log.WarnFormat("{0}: Insufficient packet data!", LogHeader);
                return false;
            }

            // Convert the actor ID
            offset = startIndex + m_APPHeaderSize;
            ProcessActorID(byteArray, offset, out actorID);
            offset += m_APPActorIDSize;

            // Convert the mass
            actorMass = FloatToHostOrder(byteArray, offset);

            // Get the subscribers that need to be notified of this actor mass
            // update
            actorMassHandler = OnDynamicActorMassUpdateEvent;

            // Check to see if there are any that are listening to the event
            if (actorMassHandler != null)
            {
                // Call the dynamic actor mass update callback
                // with the newly-received information
                actorMassHandler(actorID.actorID, actorMass);
            }

            // Indicate that the conversion was successful
            return true;
        }

        /// <summary>
        /// Processes a byte array as an error message and activates the
        /// appropriate callback.
        /// </summary>
        /// <param name="byteArray">Byte array containing the error message
        /// data</param>
        /// <param name="startIndex">The index in the "byteArray" at which to
        /// start processing</param>
        /// <returns>Flag indicating whether the error message was extracted
        /// and converted successfully</returns>
        protected bool ProcessErrorMessage(byte[] byteArray, int startIndex)
        {
            int offset;
            byte[] tempArray;
            ErrorCallbackHandler errorHandler;
            string reasonString;
            uint msgIndex;

            // Convert the index of the referred message
            offset = startIndex + m_APPHeaderSize;
            msgIndex = (uint)IPAddress.NetworkToHostOrder(
                BitConverter.ToInt32(byteArray, offset));
            offset += sizeof(uint);

            // Convert the error message
            // First, create an intermediate array that will be used to extract
            // error reason string from the byte array
            tempArray = new byte[byteArray.Length - offset];

            // Extract the error reason string from the message byte array
            Buffer.BlockCopy(byteArray, offset, tempArray, 0, tempArray.Length);

            // Convert the byte array into a string using default encoding
            reasonString = System.Text.Encoding.Default.GetString(tempArray);

            // Get the subscribers that need to be notified of
            // this error message
            errorHandler = OnErrorEvent;

            // Check to see if there are any that are
            // listening to the event
            if (errorHandler != null)
            {
                // Call the error callback with the newly-received information
                errorHandler(msgIndex, reasonString);
            }

            // Indicate that the conversion was successful
            return true;
        }

        /// <summary>
        /// Processes a byte array as an actors collided message and activates
        /// the appropriate callback.
        /// </summary>
        /// <param name="byteArray">Byte array containing the actors collided
        /// message data</param>
        /// <param name="startIndex">The index in the "byteArray" at which to
        /// start processing</param>
        /// <returns>Flag indicating whether the actors collided message was
        /// extracted and converted successfully</returns>
        protected bool ProcessActorsCollidedMessage(byte[] byteArray,
            int startIndex)
        {
            int offset;
            ActorsCollidedHandler collisionHandler;
            APPActorID collidingActor;
            APPActorID collidedActor;
            OpenMetaverse.Vector3 contactPoint;
            OpenMetaverse.Vector3 contactNormal;
            float separation;
            
            // Make sure that the array length corresponds to the expected
            // size of the messsage
            if (byteArray.Length - startIndex < m_APPActorsCollidedSize)
            {
                // There is not enough data, so warn the user and exit
                // unsuccessfully
                m_log.WarnFormat("{0}: Insufficient packet data!", LogHeader);
                return false;
            }

            // Convert the first actor ID
            offset = startIndex + m_APPHeaderSize;
            ProcessActorID(byteArray, offset, out collidingActor);
            offset += m_APPActorIDSize;

            // Convert the second actor ID
            ProcessActorID(byteArray, offset, out collidedActor);
            offset += m_APPActorIDSize;

            // Convert the contact point
            contactPoint.X = FloatToHostOrder(byteArray, offset);
            offset += sizeof(float);
            contactPoint.Y = FloatToHostOrder(byteArray, offset);
            offset += sizeof(float);
            contactPoint.Z = FloatToHostOrder(byteArray, offset);
            offset += sizeof(float);

            // Convert the contact normal
            contactNormal.X = FloatToHostOrder(byteArray, offset);
            offset += sizeof(float);
            contactNormal.Y = FloatToHostOrder(byteArray, offset);
            offset += sizeof(float);
            contactNormal.Z = FloatToHostOrder(byteArray, offset);
            offset += sizeof(float);

            // Convert the separation
            separation = FloatToHostOrder(byteArray, offset);

            // Get the subscribers that need to be notified of
            // this collision
            collisionHandler = OnActorsCollidedEvent;

            // Check to see if there are any that are
            // listening to the event
            if (collisionHandler != null)
            {
                // Call the actors collided callback with the
                // newly-received information
                collisionHandler(collidedActor.actorID, collidingActor.actorID,
                    contactPoint, contactNormal, separation);
            }

            // Indicate that the conversion was successful
            return true;
        }

        /// <summary>
        /// Processes a byte array as a time advanced message and activates
        /// the appropriate callback.
        /// </summary>
        /// <param name="byteArray">Byte array containing the actors collided
        /// message data</param>
        /// <param name="startIndex">The index in the "byteArray" at which to
        /// start processing</param>
        /// <returns>Flag indicating whether the actors collided message was
        /// extracted and converted successfully</returns>
        protected bool ProcessTimeAdvancedMessage(byte[] byteArray,
            int startIndex)
        {
            int offset;
            TimeAdvancedHandler timeAdvancedHandler;
            uint simID;

            // Make sure that the array length corresponds to the expected
            // size of the messsage
            if (byteArray.Length - startIndex < m_APPTimeAdvancedSize)
            {
                // There is not enough data, so warn the user and exit
                // unsuccessfully
                m_log.WarnFormat("{0}: Insufficient packet data!", LogHeader);
                return false;
            }

            // Convert the simulation ID
            offset = startIndex + m_APPHeaderSize;
            simID = (uint)IPAddress.NetworkToHostOrder(
                BitConverter.ToInt32(byteArray, offset));

            // Check to see if this time advanced message refers
            // to this simulation
            if (simID == m_simulationID)
            {
                // Get the subscribers that need to be notified of this
                // time advancement
                timeAdvancedHandler = OnTimeAdvancedEvent;

                // Check to see if there are any that are
                // listening to the event
                if (timeAdvancedHandler != null)
                {
                    // Call the time advanced callback with the
                    // newly-received information
                    timeAdvancedHandler();
                }
            }

            // Indicate that the conversion was successful
            return true;
        }

        /// <summary>
        /// Stops the update thread, if one is being by the messenger.
        /// </summary>
        protected void StopUpdates()
        {
            // Set the flag indicating that the update thread should stop
            m_stopUpdates = true;
        }

        /// <summary>
        /// Regularly runs the update method for this messenger
        /// </summary>
        public void RunUpdate()
        {
            Thread.CurrentThread.Priority = ThreadPriority.Highest;

            // Run the update until the stop flag indicates otherwise
            while (!m_stopUpdates)
            {
                Update();

                // Sleep the thread to ensure that it doesn't hog resources
                Thread.Sleep(10);

                Watchdog.UpdateThread();
            }

            Watchdog.RemoveThread();
        }

        /// <summary>
        /// The main update method for the messenger. Processes incoming
        /// packets.
        /// </summary>
        public void Update()
        {
            byte[] currMessage;

            // Check to see if the messenger has been initialized; if it has
            // not, exit out
            if (!m_isInitialized)
                return;

            // Check to see if the packet manager is not using its own internal
            // thread for updates
            if (!m_packetManagerInternalThread)
            {
                // Update the packet managers
                m_packetManager.Update();
                m_udpPacketManager.Update();
            }

            // Keep processing messages till the TCP packet manager has no more
            // incoming packets
            int packetCount = 0;
            while (m_packetManager.HasIncomingPacket() &&
                packetCount < m_maxPackets)
            {
                // Get the next incoming packet from the packet manager
                currMessage = m_packetManager.GetIncomingPacket();

                // Process the message
                ProcessMessage(currMessage);

                // Update the number of packets processed
                packetCount++;
            }

            // Now process messages from the UDP packet manager
            packetCount = 0;
            while (m_udpPacketManager.HasIncomingPacket() &&
                packetCount < m_maxPackets)
            {
                // Get the next incoming packet from the UDP packet manager
                currMessage = m_udpPacketManager.GetIncomingPacket();

                // Process the message
                ProcessMessage(currMessage);

                // Update the number of packets processed
                packetCount++;
            }
        }

        /// <summary>
        /// Helper method that processes a single message from the remote
        /// physics engine.
        /// </summary>
        protected void ProcessMessage(byte[] message)
        {
            ushort msgType;

            // Figure out the type of the message, by converting the
            // third and fourth bytes into an integer
            msgType = (ushort)IPAddress.NetworkToHostOrder(
                BitConverter.ToInt16(message, 2));

            // Construct the proper message structure based on them
            // message type; only look for messages that could be sent
            // back from the remote physics engine
            if (msgType == (short)MessageType.LogonReady)
            {
                // Attempt to convert the byte array into the
                // appropriate message
                ProcessLogonReadyMessage(message, 0);
            }
            else if (msgType ==
                (short)MessageType.SetStaticActor)
            {
                // Attempt to convert the byte array into the
                // appropriate message
                ProcessSetStaticActorMessage(message, 0);
            }
            else if (msgType ==
                (short)MessageType.SetDynamicActor)
            {
                // Attempt to convert the byte array into the
                // appropriate message
                ProcessSetDynamicActorMessage(message, 0);
            }
            else if (msgType ==
                (short)MessageType.DynamicActorUpdateMass)
            {
                // Attempt to convert the byte array into the
                // appropriate message
                ProcessUpdateDynamicActorMassMessage(message, 0);
            }
            else if (msgType == (short)MessageType.Error)
            {
                // Attempt to convert the byte array into the
                // appropriate message
                ProcessErrorMessage(message, 0);
            }
            else if (msgType ==
                (short)MessageType.ActorsCollided)
            {
                // Attempt to convert the byte array into the
                // appropriate message
                ProcessActorsCollidedMessage(message, 0);
            }
            else if (msgType ==
                (short) MessageType.TimeAdvanced)
            {
                // Attempt to convert the byte array into the
                // appropriate message
                ProcessTimeAdvancedMessage(message, 0);
            }
        }

        /// <summary>
        /// The implemenation of the RemoteEngineError callback.
        /// </summary>
        event ErrorCallbackHandler IRemotePhysicsMessenger.RemoteEngineError
        {
            add
            {
                // Add the event in a thread-safe manner
                lock (m_eventLock)
                {
                    OnErrorEvent += value;
                }
            }

            remove
            {
                // Remove the event in a thread-safe manner
                lock (m_eventLock)
                {
                    OnErrorEvent -= value;
                }
            }
        }

        /// <summary>
        /// The implementation of the LogonReady callback.
        /// </summary>
        event LogonReadyHandler IRemotePhysicsMessenger.LogonReady
        {
            add
            {
                // Add the event in a thread-safe manner
                lock (m_eventLock)
                {
                    OnLogonReadyEvent += value;
                }
            }

            remove
            {
                // Remove the event in a thread-safe manner
                lock (m_eventLock)
                {
                    OnLogonReadyEvent -= value;
                }
            }
        }

        /// <summary>
        /// The implementation of the OnStaticActorUpdated callback.
        /// </summary>
        event UpdateStaticActorHandler
            IRemotePhysicsMessenger.StaticActorUpdated
        {
            add
            {
                // Add the event in a thead-safe manner
                lock (m_eventLock)
                {
                    OnStaticActorUpdateEvent += value;
                }
            }

            remove
            {
                // Remove the event in a thread-safe manner
                lock (m_eventLock)
                {
                    OnStaticActorUpdateEvent -= value;
                }
            }
        }

        /// <summary>
        /// The implementation of the OnDynamicActorUpdated callback.
        /// </summary>
        event UpdateDynamicActorHandler
            IRemotePhysicsMessenger.DynamicActorUpdated
        {
            add
            {
                // Add the event in a thread-safe manner
                lock (m_eventLock)
                {
                    OnDynamicActorUpdateEvent += value;
                }
            }

            remove
            {
                // Remove the event in a thread-safe manner
                lock (m_eventLock)
                {
                    OnDynamicActorUpdateEvent -= value;
                }
            }
        }

        /// <summary>
        /// The implementation of the OnDynamicActorMassUpdated callback.
        /// </summary>
        event UpdateDynamicActorMassHandler
            IRemotePhysicsMessenger.DynamicActorMassUpdated
        {
            add
            {
                // Add the event in a thread-safe manner
                lock (m_eventLock)
                {
                   OnDynamicActorMassUpdateEvent += value;
                }
            }

            remove
            {
               /// Remove the event in a thread-safe manner
               lock (m_eventLock)
               {
                  OnDynamicActorMassUpdateEvent -= value;
               }
            }
        }

        /// <summary>
        /// The implementation of the OnActorsCollided callback.
        /// </summary>
        event ActorsCollidedHandler IRemotePhysicsMessenger.ActorsCollided
        {
            add
            {
                // Add the event in a thread-safe manner
                lock (m_eventLock)
                {
                    OnActorsCollidedEvent += value;
                }
            }

            remove
            {
                // Remove the event in a thread-safe manner
                lock (m_eventLock)
                {
                    OnActorsCollidedEvent -= value;
                }
            }
        }

        /// <summary>
        /// The implementation of the OnTimeAdvanced callback.
        /// </summary>
        event TimeAdvancedHandler IRemotePhysicsMessenger.TimeAdvanced
        {
            add
            {
                 // Add the event in a thread-safe manner
                lock (m_eventLock)
                {
                    OnTimeAdvancedEvent += value;
                }
            }

            remove
            {
                // Remove the event in a thread-safe manner
                lock (m_eventLock)
                {
                    OnTimeAdvancedEvent -= value;
                }
            }
        }
    }
}
