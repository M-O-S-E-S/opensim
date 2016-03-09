
// TODO: Create banner

namespace OpenSim.Region.Physics.RemotePhysicsPlugin
{
    /// <summary>
    /// Delegate called when a logon attempt has been processed completely
    /// by the remote physics engine.
    /// </summary>
    /// <param name="simID">The ID assigned to the messenger's
    /// simulation</param>
    public delegate void LogonReadyHandler(uint simID);

    /// <summary>
    /// Delegate called when a static actor has been updated by the remote
    /// physics engine.
    /// </summary>
    /// <param name="actorID">The unique ID of the actor being updated</param>
    /// <param name="position">The new position of the actor</param>
    /// <param name="orientation">The new orientation of the actor</param>
    public delegate void UpdateStaticActorHandler(uint actorID,
        OpenMetaverse.Vector3 position, OpenMetaverse.Quaternion orientation);

    /// <summary>
    /// Delegate called when a dynamic actor has been updated by the remote
    /// physics engine.
    /// </summary>
    /// <param name="actorID">The unique ID of the actor being updated</param>
    /// <param name="position">The new position of the actor</param>
    /// <param name="orientation">The new orientation of the actor</param>
    /// <param name="linearVelocity">The new linear velocity of the
    /// actor</param>
    /// <param name="angularVelocity">The new angular velocity of the
    /// actor</param>
    public delegate void UpdateDynamicActorHandler(uint actorID,
        OpenMetaverse.Vector3 position, OpenMetaverse.Quaternion orientation,
        OpenMetaverse.Vector3 linearVelocity,
        OpenMetaverse.Vector3 angularVelocity);

    /// <summary>
    /// Delegate called when a dynamic actor's mass has been updated by
    /// the remote physics engine.
    /// </summary>
    /// <param name="actorID">The unique identifier of the actor being
    /// updated</param>
    /// <param name="mass">The mass of the actor</param>
    public delegate void UpdateDynamicActorMassHandler(uint actorID,
        float mass);

    /// <summary>
    /// Delegate called when the remote physics engine encounters an error.
    /// </summary>
    /// <param name="msgIndex">The index of the message that is associated
    /// with the error</param>
    /// <param name="errorMsg">The description of the error from the remote
    /// physics engine</param>
    public delegate void ErrorCallbackHandler(uint msgIndex, string errorMsg);

    /// <summary>
    /// Delegate called when actors in the remote physics engine collide.
    /// </summary>
    /// <param name="collidedActor">The unique ID of the actor with which
    /// the collision has occurred</param>
    /// <param name="collidingActor">The unique ID of the actor that has
    /// collided</param>
    /// <param name="contactPoint">The point of contact in world space</param>
    /// <param name="contactNormal">The normal of the surfaces at the
    /// point of contact</param>
    /// <param name="separation">The distance between the two actors. A
    /// negative value means penetration</param>
    public delegate void ActorsCollidedHandler(uint collidedActor,
        uint collidingActor, OpenMetaverse.Vector3 contactPoint,
        OpenMetaverse.Vector3 contactNormal, float separation);

    /// <summary>
    /// Delegate called when a time step has been completed in the remote
    /// physics engine.
    /// </summary>
    public delegate void TimeAdvancedHandler();

    /// <summary>
    /// The interface that defines the functionality for messengers that
    /// process messages to and from the remote physice engine.
    /// </summary>
    public interface IRemotePhysicsMessenger
    {
        /// <summary>
        /// Initialize the messenger. No messages will be sent or processed
        /// by the messenger until this method has been successfully executed.
        /// </summary>
        /// <param name="config">The configuration used to configure this
        /// messenger</param>
        /// <param name="packetManager">The packet manager that the messenger
        /// will use to communicate with the remote server</param>
        /// <param name="udpPacketManager">The packet manager that the messenger
        /// will use to communicate with the remote physics engine over
        /// UDP</param>
        void Initialize(RemotePhysicsConfiguration config,  
            IRemotePhysicsPacketManager packetManager,
            IRemotePhysicsPacketManager udpPacketManager);

        /// <summary>
        /// Log on to the remote physics engine.
        /// </summary>
        /// <param name="simID">The unique identifier of the simulation</param>
        /// <param name="simulationName">The name of the simulation</param>
        void Logon(uint simID, string simulationName);

        /// <summary>
        /// Log off from the remote physics engine.
        /// </summary>
        /// <param name="simID">The unique identifier of the simulation</param>
        void Logoff(uint simID);

        /// <summary>
        /// Initialize the world (or scene) in the remote physics engine.
        /// </summary>
        /// <param name="gravity">The gravity intensity and direction in
        /// the remote physics world</param>
        /// <param name="staticFriction">The default coefficient of static
        /// friction; mainly used for the ground plane</param>
        /// <param name="kineticFriction">The default coefficient of kinetic
        /// friction; mainly used for the ground plane</param>
        /// <param name="collisionMargin">The distance threshold (in meters)
        /// at which actors are considered colliding</param>
        /// <param name="groundPlaneID">The unique identifier for the
        /// ground plane</param>
        /// <param name="groundPlaneHeight">The height at which the ground
        /// plane resides; no objects may fall below the ground plane</param>
        /// <param name="groundPlaneNormal">The normal of the ground
        /// plane</param>
        void InitializeWorld(OpenMetaverse.Vector3 gravity,
            float staticFriction, float kineticFriction, float restitution,
            float collisionMargin, uint groundPlaneID, float groundPlaneHeight,
            OpenMetaverse.Vector3 groundPlaneNormal);

        /// <summary>
        /// Creates a new static actor in the remote physics engine.
        /// </summary>
        /// <param name="actorID">The unique ID of the actor that is being
        /// created</param>
        /// <param name="position">The position of the actor</param>
        /// <param name="orientation">The orientation of the actor</param>
        /// <param name="reportCollisions">Indicates whether collisions
        /// involving the actor should be reported</param>
        void CreateStaticActor(uint actorID, OpenMetaverse.Vector3 position,
            OpenMetaverse.Quaternion orientation, bool reportCollisions);

        /// <summary>
        /// Creates a new dynamic actor in the remote physics engine.
        /// </summary>
        /// <param name="actorID">The unique ID of the actor that is being
        /// created</param>
        /// <param name="position">The position of the actor</param>
        /// <param name="orientation">The orientation of the actor</param>
        /// <param name="gravityModifier">The gravity multiplier affecting
        /// this actor</param>
        /// <param name="linearVelocity">The linear velocity of this
        /// actor</param>
        /// <param name="angularVelocity">The angular velocity of this
        /// actor</param>
        /// <param name="reportCollisions">Indicates whether collisions
        /// involving the actor should be reported</param>
        void CreateDynamicActor(uint actorID, OpenMetaverse.Vector3 position, 
            OpenMetaverse.Quaternion orientation, float gravityModifier, 
            OpenMetaverse.Vector3 linearVelocity,
            OpenMetaverse.Vector3 angularVelocity, bool reportCollisions);

        /// <summary>
        /// Create a new static actor or update the state of an existing
        /// static actor in the remote physics engine.
        /// </summary>
        /// <param name="actorID">The unique ID of the actor that is being
        /// created/updated</param>
        /// <param name="position">The new position of the actor</param>
        /// <param name="orientation">The new orientation of the actor</param>
        void SetStaticActor(uint actorID, OpenMetaverse.Vector3 position,
            OpenMetaverse.Quaternion orientation);

        /// <summary>
        /// Create a new dynamic actor or update the state of an existing
        /// dynamic actor in the remote physics engine.
        /// </summary>
        /// <param name="actorID">The unique ID of the actor that is being
        /// created/updated</param>
        /// <param name="position">The new position of the actor</param>
        /// <param name="orientation">The new orientation of the actor</param>
        /// <param name="gravityModifier">The gravity multiplier affecting
        /// this actor</param>
        /// <param name="linearVelocity">The new linear velocity of this
        /// actor</param>
        /// <param name="angularVelocity">The new angular velocity of this
        /// actor</param>
        void SetDynamicActor(uint actorID, OpenMetaverse.Vector3 position, 
            OpenMetaverse.Quaternion orientation, float gravityModifier, 
            OpenMetaverse.Vector3 linearVelocity,
            OpenMetaverse.Vector3 angularVelocity);
        
        /// <summary>
        /// Updates the position of an actor in the remote physics engine.
        /// </summary>
        /// <param name="actorID">The unique identifier of the actor</param>
        /// <param name="gravityModifier">The new position of the actor</param>
        void UpdateActorPosition(uint actorID, OpenMetaverse.Vector3 position);
        
        /// <summary>
        /// Updates the orientation of an actor in the remote physics engine.
        /// </summary>
        /// <param name="actorID">The unique identifier of the actor</param>
        /// <param name="orientation">The new orientation of the actor</param>
        void UpdateActorOrientation(uint actorID,
            OpenMetaverse.Quaternion orientation);
        
        /// <summary>
        /// Updates the gravity modifier of a dynamic actor in the remote
        /// physics engine.
        /// </summary>
        /// <param name="actorID">The unique identifier of the actor</param>
        /// <param name="gravityModifier">The new gravity modifier</param>
        void UpdateActorGravityModifier(uint actorID, float gravityModifier);
        
        /// <summary>
        /// Updates the linear velocity of an actor in the remote
        /// physics engine.
        /// </summary>
        /// <param name="actorID">The unique identifier of the actor</param>
        /// <param name="velocity">The new linear velocity of the actor</param>
        void UpdateActorVelocity(uint actorID, OpenMetaverse.Vector3 velocity);
        
        /// <summary>
        /// Updates the angular velocity of an actor in the remote
        /// physics engine.
        /// </summary>
        /// <param name="actorID">The unique identifier of the actor</param>
        /// <param name="velocity">The new angular velocity of the actor</param>
        void UpdateActorAngularVelocity(uint actorID,
            OpenMetaverse.Vector3 velocity);

        /// <summary>
        /// Requests the mass of an actor in the remote physics engine.
        /// </summary>
        /// <param name="actorID">The unique ID of the actor whose mass is
        /// being queried</param>
        void GetActorMass(uint actorID);

        /// <summary>
        /// Remove an actor from the remote physics engine.
        /// </summary>
        /// <param name="actorID">The unique ID of the actor being
        /// removed</param>
        void RemoveActor(uint actorID);

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
        /// <param name="actor1Orientation">The orientation relative to
        /// the actor at which the joint is being attached to the second
        /// actor</param>
        /// <param name="actor2ID">The unique identifier of the second
        /// actor to which the joint is being attached</param>
        /// <param name="actor2Translation">The position relative to the
        /// actor at which the joint is being attached to the
        /// second actor</param>
        /// <param name="actor2Orientation">The orientation relative to the
        /// actor at which the joint is being attached to the
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
        /// If lowerLimit is less than the upperLimit, axis is limited to be
        /// between the two limits.  If lowerLimit is greater than upperLimit,
        /// axis is free.</remarks>
        void AddJoint(uint jointID, uint actor1ID,
            OpenMetaverse.Vector3 actor1Translation,
            OpenMetaverse.Quaternion actor1Orientation, uint actor2ID,
            OpenMetaverse.Vector3 actor2Translation,
            OpenMetaverse.Quaternion actor2Orientation,
            OpenMetaverse.Vector3 linearLowerLimits,
            OpenMetaverse.Vector3 linearUpperLimits,
            OpenMetaverse.Vector3 angularLowerLimits,
            OpenMetaverse.Vector3 angularUpperLimits);
        
        /// <summary>
        /// Removes a joint from the remote physics engine.
        /// </summary>
        /// <param name="jointID">The unique identifier of the joint to be
        /// removed</param>
        void RemoveJoint(uint jointID);

        /// <summary>
        /// Add a sphere shape to the remote physics engine.
        /// </summary>
        /// <param name="shapeID">The unique ID of the shape being
        /// created</param>
        /// <param name="origin">The origin of the sphere relative to any actor
        /// to which it is attached</param>
        /// <param name="radius">The radius of the sphere</param>
        void AddSphere(uint shapeID, OpenMetaverse.Vector3 origin,
            float radius);

        /// <summary>
        /// Add a plane shape to the remote physics engine.
        /// </summary>
        /// <param name="shapeID">The unique ID of the plane shape being
        /// created</param>
        /// <param name="planeNormal">The normal of the plane</param>
        /// <param name="planeConstant">The plane constant describing the
        /// distance to the plane from the origin</param>
        void AddPlane(uint shapeID, OpenMetaverse.Vector3 planeNormal,
            float planeConstant);

        /// <summary>
        /// Add a capsule shape to the remote physics engine.
        /// </summary>
        /// <param name="shapeID">The unique ID of the capsule shape being
        /// created</param>
        /// <param name="radius">The radius of the capsule</param>
        /// <param name="height">The height of the capsule</param>
        void AddCapsule(uint shapeID, float radius, float height);

        /// <summary>
        /// Add a box shape to the remote physics engine.
        /// </summary>
        /// <param name="shapeID">The unique ID of the box shape being
        /// created</param>
        /// <param name="length">The length of the box</param>
        /// <param name="width">The width of the box</param>
        /// <param name="height">The height of the box</param>
        void AddBox(uint shapeID, float length, float width, float height);

        /// <summary>
        /// Add a convex mesh shape to the remote physics engine.
        /// </summary>
        /// <param name="shapeID">The unique ID of the convex mesh shape
        /// being created</param>
        /// <param name="points">The points that make up the mesh</param>
        void AddConvexMesh(uint shapeID, OpenMetaverse.Vector3[] points);

        /// <summary>
        /// Add a triangle mesh shape to the remote physics engine.
        /// </summary>
        /// <param name="shapeID">The unique ID of the triangle mesh shape
        /// being created</param>
        /// <param name="points">The points that make up the triangles
        /// in the mesh</param>
        /// <param name="triangles">The indices which define the triangles
        /// created from "points"</param>
        void AddTriangleMesh(uint shapeID, OpenMetaverse.Vector3[] points,
            int[] triangles);

        /// <summary>
        /// Add a height field shape to the remote physics engine.
        /// </summary>
        /// <param name="shapeID">The unique ID of the height field shape
        /// being created</param>
        /// <param name="numRows">The number of rows in the height field</param>
        /// <param name="numColumns">The number of columns in the height
        /// field</param>
        /// <param name="rowSpacing">The space between rows in the height
        /// field</param>
        /// <param name="columnSpacing">The space between columns in the
        /// height field</param>
        /// <param name="posts">The height values that make up the height
        /// field</param>
        void AddHeightField(uint shapeID, uint numRows, uint numColumns,
            float rowSpacing, float columnSpacing, float[] posts);

        /// <summary>
        /// Remove a shape from the remote physics engine.
        /// </summary>
        /// <param name="shapeID">The unique ID of the shape being
        /// removed</param>
        void RemoveShape(uint shapeID);

        /// <summary>
        /// Attach an instance of a shape to an actor in the remote physics
        /// engine.
        /// </summary>
        /// <param name="actorID">The unique ID of the actor to which the
        /// shape is being attached</param>
        /// <param name="shapeID">The unique ID of the shape being
        /// attached</param>
        /// <param name="density">The density of the shape instance</param>
        /// <param name="staticFriction">The coefficient of static
        /// friction of the shape instance</param>
        /// <param name="kineticFriction">The coefficient of kinetic friction
        /// of the shape instance</param>
        /// <param name="restitution">The coefficient of restitution of the
        /// shape instance</param>
        /// <param name="orientation">The orientation (relative to the actor)
        /// of the shape instance</param>
        /// <param name="translation">The position (relative to the actor) of
        /// the shape instance</param>
        void AttachShape(uint actorID, uint shapeID, float density,
            float staticFriction, float kineticFriction, float restitution,
            OpenMetaverse.Quaternion orientation,
            OpenMetaverse.Vector3 translation);

        /// <summary>
        /// Updates the physical properties of a shape attached to an actor
        /// in the remote physics engine.
        /// </summary>
        /// <param name="actorID">The unique identifier of the of the actor to
        /// which the shape is attached</param>
        /// <param name="shapeID">The unique identifier of the shape whose
        /// material properties are to be modified</param>
        /// <param name="staticFriction">The new coefficient of static friction
        /// for the shape</param>
        /// <param name="kineticFriction">The new coefficient of kinetic
        /// friction for the shape</param>
        /// <param name="restitution">The new coefficient of restitution for the
        /// shape</param>
        /// <param name="density">The new density of the shape</param>
        void UpdateShapeMaterial(uint actorID, uint shapeID,
            float staticFriction, float kineticFriction, float restitution,
            float density);

        /// <summary>
        /// Detaches a shape instance from an actor in the remote physics
        /// engine.
        /// </summary>
        /// <param name="actorID">The unique ID of the actor from which the
        /// shape instance is being detached</param>
        /// <param name="shapeID">The unique ID of the shape whose instance   
        /// is being detached</param>
        void DetachShape(uint actorID, uint shapeID);

        /// <summary>
        /// Applies a force to an actor in the remote physics engine.
        /// </summary>
        /// <param name="actorID">The unique identifier of the actor to which
        /// the force is being applied</param>
        /// <param name="force">The force that is being applied</param>
        void ApplyForce(uint actorID, OpenMetaverse.Vector3 force);

        /// <summary>
        /// Applies a torque to an actor in the remote physics engine.
        /// </summary>
        /// <param name="actorID">The unique identifier of the actor to which
        /// the torque is being applied</param>
        /// <param name="torque">The torque that is being applied</param>
        void ApplyTorque(uint actorID, OpenMetaverse.Vector3 torque);

        /// <summary>
        /// Advance the time of the world in the remote physics engine.
        /// </summary>
        /// <param name="time">The number of seconds to advance the
        /// world</param>
        void AdvanceTime(float time);

        /// <summary>
        /// The event that is triggered when a logon attempt has been
        /// completed successfully by the remote physicse engine.
        /// </summary>
        event LogonReadyHandler LogonReady;

        /// <summary>
        /// The event that is triggered when a static actor is updated by the
        /// remote physics engine.
        /// </summary>
        event UpdateStaticActorHandler StaticActorUpdated;

        /// <summary>
        /// The event that is triggered when a dynamic actor is updated by
        /// the remote physics engine.
        /// </summary>
        event UpdateDynamicActorHandler DynamicActorUpdated;

        /// <summary>
        /// The event that is triggered when a dynamic actor's mass is updated
        /// by the remote physics engine.
        /// </summary>
        event UpdateDynamicActorMassHandler DynamicActorMassUpdated;

        /// <summary>
        /// The event that is triggered when the remote physics engine
        /// encounters an error.
        /// </summary>
        event ErrorCallbackHandler RemoteEngineError;

        /// <summary>
        /// The event that is triggered when two actors in the remote physics
        /// engine collide.
        /// </summary>
        event ActorsCollidedHandler ActorsCollided;

        /// <summary>
        /// The event that is triggered when the remote physics engine has
        /// completed a time step.
        /// </summary>
        event TimeAdvancedHandler TimeAdvanced;
    }
}
