
// TODO: Create banner

using System;
using System.Collections.Generic;
using System.Text;

using log4net;

using OpenMetaverse;
using OpenSim.Framework;
using OpenSim.Region.Physics.Manager;


namespace OpenSim.Region.Physics.RemotePhysicsPlugin
{
    public class RemotePhysicsAvatar : RemotePhysicsObject
    {
        // Create the logger used for logging and debugging
        protected static readonly ILog m_log = LogManager.GetLogger(
            System.Reflection.MethodBase.GetCurrentMethod().DeclaringType);

        // This string is used to identify statement from this class in the
        // log
        protected static readonly String LogHeader = "[REMOTE PHYSICS AVATAR]";

        /// <summary>
        /// Indicates the type of object represented by this actor.
        /// </summary>
        protected int m_actorType;

        /// <summary>
        /// The unique identifier of this avatar.
        /// </summary>
        protected uint m_actorID;

        /// <summary>
        /// The size of the avatar.
        /// </summary>
        protected OpenMetaverse.Vector3 m_size;

        /// <summary>
        /// The orientation of the avatar.
        /// </summary>
        protected OpenMetaverse.Quaternion m_orientation;

        /// <summary>
        /// The linear velocity of the avatar.
        /// </summary>
        protected OpenMetaverse.Vector3 m_velocity;

        /// <summary>
        /// The angular velocity of the avatar.
        /// </summary>
        protected OpenMetaverse.Vector3 m_rotationalVelocity;

        /// <summary>
        /// The position of the avatar.
        /// </summary>
        protected OpenMetaverse.Vector3 m_position;

        /// <summary>
        /// The torque being applied to this avatar.
        /// </summary>
        protected OpenMetaverse.Vector3 m_torque;

        /// <summary>
        /// The acceleration of the avatar.
        /// </summary>
        protected OpenMetaverse.Vector3 m_acceleration;

        /// <summary>
        /// The mass of the avatar.
        /// </summary>
        protected float m_mass;

        /// <summary>
        /// The bouyancy of the avatar.
        /// </summary>
        protected float m_bouyancy;

        /// <summary>
        /// The collision score of the avatar.
        /// </summary>
        protected float m_collisionScore;

        /// <summary>
        /// Indicates whether the avatar is flying.
        /// </summary>
        protected bool m_isFlying;

        /// <summary>
        /// Indicates whether the avatar should always run.
        /// </summary>
        protected bool m_alwaysRun;

        /// <summary>
        /// Indicates whether the avatar should float on water.
        /// </summary>
        protected bool m_floatOnWater;

        /// <summary>
        /// Indicates whether the avatar is being acted upon by physical forces.
        /// </summary>
        protected bool m_kinematic;

        /// <summary>
        /// Indicates whether the avatar is stationary.
        /// </summary>
        protected bool m_isStatic;

        /// <summary>
        /// Indicates whether the avatar is represented by a rigid body.
        /// </summary>
        protected bool m_isRigid;

        /// <summary>
        /// Indicates whether the user has grabbed this avatar.
        /// </summary>
        protected bool m_grabbed;

        /// <summary>
        /// Indicates whether the user has selected this avatar.
        /// </summary>
        protected bool m_selected;

        /// <summary>
        /// The static friction of the avatar.
        /// </summary>
        protected float m_staticFriction;

        /// <summary>
        /// The kinetic friction of the avatar.
        /// </summary>
        protected float m_kineticFriction;

        /// <summary>
        /// Indicates whether the avatar shape has been created in the remote
        /// physics configuration.  Once successfully created, the shape can
        /// be used for all avatars.
        /// </summary>
        protected static bool m_avatarShapeCreated = false;

        /// <summary>
        /// The ID of the avatar shape in the remote physics engine. This ID is
        /// shared across all avatars.
        /// </summary>
        protected static uint m_avatarShapeID;

        /// <summary>
        /// The unique identifier of the joint used to keep the avatar upright
        /// against the effects of gravity.
        /// </summary>
        protected uint m_fallJointID;

        /// <summary>
        /// Constructor.
        /// </summary>
        /// <param name="localID">The unique identifier of the avatar</param>
        /// <param name="avatarName">The name of the avatar</param>
        /// <param name="parentScene">The physics scene to which the avatar
        /// belongs</param>
        /// <param name="position">The initial position of the avatar</param>
        /// <param name="velocity">The initial linear velocity of the
        /// avatar</param>
        /// <param name="size">The size of the avatar</param>
        /// <param name="isFlying">Whether the avatar is flying or not</param>
        /// <param name="config">The configuration used to initialize the
        /// avatar</param>
        public RemotePhysicsAvatar(uint localID, String avatarName,
                RemotePhysicsScene parentScene, OpenMetaverse.Vector3 position,
                OpenMetaverse.Vector3 velocity, OpenMetaverse.Vector3 size,
                bool isFlying, RemotePhysicsConfiguration config) 
                : base(parentScene, localID, avatarName,
                    "RemotePhysicsCharacter", config)
        {
            OpenMetaverse.Quaternion localPoseQuat;

            // Initialize the unique ID of this avatar
            m_actorID = localID;

            // Indicate that this object is an avatar
            m_actorType = (int)ActorTypes.Agent;

            // Initialize the position to what's given
            Position = position;

            // Initialize the orientation to the default value
            Orientation = OpenMetaverse.Quaternion.Identity;

            // Initialize the velocity based on what's given
            m_velocity = velocity;

            // Initialize the friction values based on the parent scene's
            // friction coefficient for avatars
            Friction = ParentConfiguration.AvatarKineticFriction;
            m_kineticFriction = ParentConfiguration.AvatarKineticFriction;
            m_staticFriction = ParentConfiguration.AvatarStaticFriction;

            // Initialize the density based on the parent scene's density
            // value for avatars
            Density = ParentConfiguration.AvatarDensity;

            // Initialize the size of this avatar using the given size
            m_size = size;

            // Check to see if any of the dimensions are zero
            // If they are, use the default sizes
            if (m_size.X == 0.0f)
                m_size.X = ParentConfiguration.AvatarShapeDepth;
            if (m_size.Y == 0.0f)
                m_size.Y = ParentConfiguration.AvatarShapeWidth;
            if (m_size.Z == 0.0f)
                m_size.Z = ParentConfiguration.AvatarShapeHeight;

            // Compute the mass of the avatar, so that it can be referenced
            // for later computations
            ComputeAvatarMass();

            // Send out a message to the remote physics engine to create an
            // actor
            ParentScene.RemoteMessenger.CreateDynamicActor(m_actorID, Position,
                Orientation, 1.0f, m_velocity,
                new OpenMetaverse.Vector3(0.0f, 0.0f, 0.0f), true);

            // Fetch a unique identifier for this avatar's shape
            m_avatarShapeID = ParentScene.GetNewShapeID();

            // Build the avatar's shape in the remote physics engine
            BuildAvatarShape();

            // Add a constraint between the new actor and ground plane such
            // that the actor doesn't fall over on its side due to gravity
            // Use an unused actor ID to denote that this joint is between the
            // avatar and the world frame
            m_fallJointID = ParentScene.GetNewJointID();
            ParentScene.RemoteMessenger.AddJoint(m_fallJointID, m_actorID,
                OpenMetaverse.Vector3.Zero, OpenMetaverse.Quaternion.Identity,
                ParentScene.GetNewActorID(), OpenMetaverse.Vector3.Zero,
                OpenMetaverse.Quaternion.Identity,
                new OpenMetaverse.Vector3(1.0f, 1.0f, 1.0f),
                new OpenMetaverse.Vector3(0.0f, 0.0f, 0.0f),
                new OpenMetaverse.Vector3(0.0f, 0.0f, 0.0f),
                new OpenMetaverse.Vector3(0.0f, 0.0f, 0.0f));

            // Indicate that this object is now initialized
            IsInitialized = true;
        }

        /// <summary>
        /// Releases resources used by this avatar.
        /// </summary>
        public override void Destroy()
        {
            // Indicate that this object is no longer fully initialized
            IsInitialized = false;

            // Remove the avatar's actor and shape from the remote physics
            // engine
            ParentScene.RemoteMessenger.RemoveJoint(m_fallJointID);
            ParentScene.RemoteMessenger.RemoveActor(m_actorID);
            ParentScene.RemoteMessenger.RemoveShape(m_avatarShapeID);

            // Destroy the super class members
            base.Destroy();
        }

        /// <summary>
        /// Builds the avatar's shape in the remote physics engine.
        /// </summary>
        protected void BuildAvatarShape()
        {
            OpenMetaverse.Quaternion localPoseQuat;

            // Check to see if the avatar shape has not been created in the
            // remote physics engine
            if (!m_avatarShapeCreated)
            {
                // Create the shape based on which type is configured
                // for avatars
                switch (ParentConfiguration.AvatarShape)
                {
                    case RemotePhysicsShape.SHAPE_CAPSULE:
                        ParentScene.RemoteMessenger.AddCapsule(m_avatarShapeID,
                            Math.Min(Size.X, Size.Y) / 2.0f, Size.Z);

                        // Attach the avatar shape to the new actor
                        // NOTE: The capsule is initially lying along the
                        // x-axis, so it has to be rotated upright
                        localPoseQuat =
                            OpenMetaverse.Quaternion.CreateFromEulers(0.0f,
                            (float)Math.PI / 2.0f, 0.0f);
                        ParentScene.RemoteMessenger.AttachShape(m_actorID,
                            m_avatarShapeID, Density, m_staticFriction,
                            m_kineticFriction, Restitution, localPoseQuat,
                            Vector3.Zero);
                        break;
                    case RemotePhysicsShape.SHAPE_BOX:
                        ParentScene.RemoteMessenger.AddBox(m_avatarShapeID,
                            Size.X, Size.Y, Size.Z);

                        // Attach the avatar shape to the new actor
                        ParentScene.RemoteMessenger.AttachShape(m_actorID,
                            m_avatarShapeID, Density,
                            m_staticFriction, m_kineticFriction, Restitution,
                            Quaternion.Identity, Vector3.Zero);
                        break;

                    // Indicate that the avatar's shape has now been created in
                    // remote physics engine
                    m_avatarShapeCreated = true;
                }
            }
        }

        /// <summary>
        /// Compute avatar's mass and volume based on the density and
        /// shape information.
        /// </summary>
        protected void ComputeAvatarMass()
        {
            float avatarVolume;

            // Compute the volume and mass of the avatar based on the primitve
            // shape that is being used
            switch (ParentConfiguration.AvatarShape)
            {
                case RemotePhysicsShape.SHAPE_UNKNOWN:
                case RemotePhysicsShape.SHAPE_MESH:
                    // Cannot compute the volume (and therefore the mass), if
                    // the shape is unknown
                    break;
                case RemotePhysicsShape.SHAPE_CAPSULE:
                    // The capsule shape is a cylinder with two semi-ellipsoids
                    // at each end
                    // First, calculate the volume of the ellipsoid
                    // The height of the ellipsoid is the smaller of the other
                    // two dimensions of the ellipsoid, in order to minimize its
                    // height
                    // Use an approximation of 4/3 in order to optimize the
                    // calculation
                    avatarVolume = (float)Math.PI * 1.333333333f *
                        (Size.X / 2.0f) * (Math.Min(Size.X, Size.Y) / 2.0f) *
                        (Size.Y  / 2.0f);

                    // Now add in the volume of the cylindrical center part
                    // of the capsule
                    avatarVolume += (float)Math.PI * Size.X / 2.0f *
                        Size.Y / 2.0f * Size.Z;

                    // Now that the volume has been calculated, use it to
                    // calculate the mass of the avatar
                    m_mass = Density * avatarVolume;

                    break;
                case RemotePhysicsShape.SHAPE_BOX:
                    // Caculate the volume of the box shape based on the
                    // dimensions that have been provided
                    avatarVolume = Size.X * Size.Y * Size.Z;

                    // Calculate the mass using the volume computed above
                    m_mass = Density * avatarVolume;

                    break;
            }
        }

        // SDS: Don't know what this does
        public override bool Stopped { get { return false; } }

        /// <summary>
        /// Indicates the type of this physical object.
        /// </summary>
        public override int PhysicsActorType
        {
            get
            {
                return m_actorType;
            }

            set
            {
                m_actorType = value;
            }
        }

        /// <summary>
        /// Indicates whether the avatar is stationary.
        /// </summary>
        public override bool IsStatic
        {
            get
            {
                // Avatars are never static
                return false;
            }
        }

        /// <summary>
        /// Indicates whether the avatar should always be running instead of
        /// walking.
        /// </summary>
        public override bool SetAlwaysRun
        {
            get
            {
                return m_alwaysRun;
            }
            set
            {
                m_alwaysRun = value;
            }
        }

        /// <summary>
        /// Indicates whether the avatar is flying.
        /// </summary>
        public override bool Flying
        {
            get
            {
                return m_isFlying;
            }
            set
            {
                m_isFlying = value;

                // Schedule an update to be sent right before the time is
                // advanced
                ParentScene.AddActorTaintCallback(delegate()
                {
                    ParentScene.RemoteMessenger.UpdateActorGravityModifier(
                        LocalID, m_isFlying ? 0.0f : 1.0f);
                });
            }
        }

        /// <summary>
        /// The size of the avatar.
        /// </summary>
        public override OpenMetaverse.Vector3 Size
        {
            get
            {
                // Return the the dimensions of the avatar's shape
                return m_size;
            }

            set
            {
                // Check to see if the size has changed
                if (m_size != value)
                {
                    // Update the size value
                    m_size = value;
 
                    // Remove the old shape, which was created with the old
                    // size value
                    ParentScene.RemoteMessenger.DetachShape(LocalID,
                        m_avatarShapeID);
                    ParentScene.RemoteMessenger.RemoveShape(m_avatarShapeID);
                    m_avatarShapeCreated = false;

                    // Rebuild the vatar's shape
                    BuildAvatarShape();
                }
            }
        }

        /// <summary>
        /// The mass of the avatar.
        /// </summary>
        public override float Mass
        {
            get
            {
                // Return the pre-computed mass of the avatar
                return m_mass;
            }
        }

        /// <summary>
        /// Indicates whether the avatar floats on water.
        /// </summary>
        public override bool FloatOnWater
        {
            set 
            {
                m_floatOnWater = value;

                // TODO: Add collision flags based on given value
            }
        }

        /// <summary>
        /// The vehicle type. Not used, since avatars are not vehicles.
        /// </summary>
        public override int VehicleType
        {
            get
            {
                // Avatars aren't vehicles
                return (int)Vehicle.TYPE_NONE;
            }

            set
            {
                // Avatars aren't vehicles, so disregard the incoming value
                return;
            }
        }

        /// <summary>
        /// Sets a vehicle parameter. Not used, since avatars are not vehicles.
        /// </summary>
        /// <param name="param">The index of the parameter</param>
        /// <param name="value">The new value for the parameter</param>
        public override void VehicleFloatParam(int param, float value)
        {
            // Don't do anything here, since avatars aren't vehicles
        }

        /// <summary>
        /// Sets a vehicle parameter. Not used, since avatars are not vehicles.
        /// </summary>
        /// <param name="param">The index of the parameter</param>
        /// <param name="value">The new value for the parameter</param>
        public override void VehicleVectorParam(int param,
            OpenMetaverse.Vector3 value)
        {
            // Don't do anything here, since avatars aren't vehicles
        }

        /// <summary>
        /// Sets a vehicle parameter. Not used, since avatars are not vehicles.
        /// </summary>
        /// <param name="param">The index of the parameter</param>
        /// <param name="rotation">The new value for the parameter</param>
        public override void VehicleRotationParam(int param,
            OpenMetaverse.Quaternion rotation)
        {
            // Don't do anything here, since avatars aren't vehicles
        }

        /// <summary>
        /// Sets a vehicle flag. Not sued, since avatars are not vehicles.
        /// </summary>
        /// <param name="param">The index of the parameter</param>
        /// <param name="remove">The index of the parameter</param>
        public override void VehicleFlags(int param, bool remove)
        {
            // Don't do anything here, since avatars aren't vehicles
        }

        /// <summary>
        /// Indicates whether this avatar can collide with non-physical
        /// primitives (not supported).
        /// </summary>
        /// <param name="param"></param>
        public override void SetVolumeDetect(int param)
        {
            // This feature is not supported (it allows collision with
            // inherently non-physical prims)
            return;
        }

        /// <summary>
        /// The center of the avatar in object space.
        /// </summary>
        public override OpenMetaverse.Vector3 GeometricCenter
        {
            get
            {
                // Return the raw position of the avatar, since that is also
                // it gemoetric center in world space
                return Position;
            }
        }

        /// <summary>
        /// The center of mass of the avatar.
        /// </summary>
        public override OpenMetaverse.Vector3 CenterOfMass
        {
            get
            {
                // Return the raw position of the avatar, since that is also
                // its center of mass in world space
                return Position;
            }
        }

        /// <summary>
        /// The linear velocity of the avatar.
        /// </summary>
        public override OpenMetaverse.Vector3 Velocity
        {
            get
            {
                // Return the current velocity of this avatar
                return m_velocity;
            }

            set
            {
                // Update the current and target velocity of this avatar,
                // because the user wants to force the avatar to this velocity
                m_velocity = value;
                m_targetVelocity = value;

                // Check if this avatar is currently running and not flying
                if (m_alwaysRun && !m_isFlying)
                {
                    // Amplify the velocity by the run factor
                    m_velocity *= new Vector3(
                        ParentConfiguration.AvatarRunFactor,
                        ParentConfiguration.AvatarRunFactor, 1.0f);
                }

                // Schedule an update to be sent right before the time is
                // advanced
                ParentScene.AddActorTaintCallback(delegate()
                {
                    ParentScene.RemoteMessenger.UpdateActorVelocity(LocalID,
                        m_velocity);
                });
            }
        }

        /// <summary>
        /// Method used by the remote physics engine to update the velocity of
        /// the avatar without issuing a taint callback.
        /// </summary>
        /// <param name="newVelocity">The new velocity of the avatar</param>
        public void UpdateVelocity(OpenMetaverse.Vector3 newVelocity)
        {
            // Update the linear velocity of the avatar
            m_velocity = newVelocity;
        }

        /// <summary>
        /// The angular velocity of the avatar.
        /// </summary>
        public override OpenMetaverse.Vector3 RotationalVelocity
        {
            get
            {
                // Return the current rotational velocity of this avatar
                return m_rotationalVelocity;
            }

            set
            {
                // Update the current rotational velocity of the this avatar
                m_rotationalVelocity = value;

                // Schedule an update to be sent right before the time is
                // advanced
                ParentScene.AddActorTaintCallback(delegate()
                {
                    ParentScene.RemoteMessenger.UpdateActorAngularVelocity(
                        LocalID, m_rotationalVelocity);
                });
            }
        }

        /// <summary>
        /// Method used by the remote physics engine to update the angular
        /// (rotational) velocity of the avatar without issuing a taint
        /// callback.
        /// </summary>
        /// <param name="newVelocity">The new angular velocity of the
        /// avatar</param>
        public void UpdateRotationalVelocity(OpenMetaverse.Vector3 newVelocity)
        {
            // Update the angular (rotational) velocity of the avatar
            m_rotationalVelocity = newVelocity;
        }

        /// <summary>
        /// The position of the avatar.
        /// </summary>
        public override OpenMetaverse.Vector3 Position
        {
            get
            {
                // Return the current position of this avatar
                return m_position;
            }

            set
            {
                m_position = value;

                // Schedule an update to be sent right before the time is
                // advanced
                ParentScene.AddActorTaintCallback(delegate()
                {
                    ParentScene.RemoteMessenger.UpdateActorPosition(LocalID,
                        m_position);
                });
            }
        }

        /// <summary>
        /// Method used by the remote physics engine to update the position of
        /// the avatar without issuing a taint callback.
        /// </summary>
        /// <param name="newPosition">The new position of the avatar</param>
        public void UpdatePosition(OpenMetaverse.Vector3 newPosition)
        {
            // Update the position of the avatar
            m_position = newPosition;
        }

        /// <summary>
        /// The orientation of the avatar.
        /// </summary>
        public override OpenMetaverse.Quaternion Orientation
        {
            get
            {
                // Return the current orientation of this avatar
                return m_orientation;
            }

            set
            {
                // Update the current orientation of this avatar
                m_orientation = value;

                // Schedule an update to be sent right before the time is
                // advanced
                ParentScene.AddActorTaintCallback(delegate()
                {
                    ParentScene.RemoteMessenger.UpdateActorOrientation(
                        LocalID, m_orientation);
                });
            }
        }

        /// <summary>
        /// Method used by the remote physics engine to update the
        /// orientation of the avatar without issuing a taint callback.
        /// </summary>
        /// <param name="newOrientation">The new orientation of the
        /// avatar</param>
        public void UpdateOrientation(OpenMetaverse.Quaternion newOrientation)
        {
            // Update the orientation of the avatar
            m_orientation = newOrientation;
        }

        /// <summary>
        /// The acceleration of the avatar
        /// </summary>
        public override OpenMetaverse.Vector3 Acceleration
        {
            get
            {
                // Return the current acceleration of this avatar
                return m_acceleration;
            }

            set
            {
                // Update the current acceleration of this avatar
                m_acceleration = value;
            }
        }

        /// <summary>
        /// The torque being applied to the avatar.
        /// </summary>
        public override OpenMetaverse.Vector3 Torque
        {
            get
            {
                // Return the torque being applied to this avatar
                return m_torque;
            }

            set
            {
                // Update the torque being applied to this avatar
                m_torque = value;

                // Apply the torque in the remote physics engine
                ParentScene.RemoteMessenger.ApplyTorque(LocalID, m_torque);
            }
        }

        /// <summary>
        /// The bouyancy of the avatar.
        /// </summary>
        public override float Buoyancy
        {
            get { return m_bouyancy; }

            set
            {
                // Update the bouyancy of this object
                m_bouyancy = value;

                // TODO: Send a message to the remote physics engine
                // indicating the bouyancy change
            }
        }

        /// <summary>
        /// Indicates whether the user has grabbed the avatar.
        /// </summary>
        public override bool Grabbed
        {
            set
            {
                 // Update the value indicating whether this avatar has been
                 // grabbed, though this won't be used since avatars can't be
                 // grabbed
                 m_grabbed = value;
            }
        }

        /// <summary>
        /// Indicates whether user has selected the avatar.
        /// </summary>
        public override bool Selected
        {
            set
            {
                // Update the value indicating whether this avatar has been
                // selected, though this won't be used since selection doesn't
                // affect the physical properties of the avatar
                m_selected = value;
            }
        }

        /// <summary>
        /// Indicates whether physical forces act upon the avatar.
        /// </summary>
        public override bool Kinematic
        {
            get
            {
                return m_kinematic;
            }
            set
            {
                m_kinematic = value;
            }
        }

        /// <summary>
        /// Link the avatar to another actor (not supported).
        /// </summary>
        /// <param name="actor">The other actor to which the avatar should be
        /// linked</param>
        public override void link(PhysicsActor actor)
        {
            // Avatars are not supposed to be linked, so just exit
            return;
        }

        /// <summary>
        /// Remove the link between this avatar and another actor (not
        /// supported).
        /// </summary>
        public override void delink()
        {
            // Avatars are not supposed to be linked, so just exit
            return;
        }

        /// <summary>
        /// Stop angular motion of the avatar (not supported).
        /// </summary>
        /// <param name="axis">The axis around which angular motion should be
        /// stopped</param>
        public override void LockAngularMotion(OpenMetaverse.Vector3 axis)
        {
            // This feature is not supported, so just exit
            return;
        }

        /// <summary>
        /// Add an angular force to the avatar (not supported).
        /// </summary>
        /// <param name="force">The force to be applied</param>
        /// <param name="pushforce">Indicates whether the force is a
        /// push</param>
        public override void AddAngularForce(Vector3 force, bool pushforce)
        {
            // This feature is not supported, so just exit
            return;
        }

        /// <summary>
        /// Set the momentum of the avatar (not supported).
        /// </summary>
        /// <param name="momentum">The new momentum of the avatar</param>
        public override void SetMomentum(Vector3 momentum)
        {
            // This feature is not supported, so just exit
            return;
        }

        /// <summary>
        /// Add a force to the avatar.
        /// </summary>
        /// <param name="force">The force to be applied</param>
        /// <param name="pushforce">Indicates whether the force is a
        /// push</param>
        public override void AddForce(Vector3 force, bool pushforce)
        {
            // Send a message to the remote physics engine with the new force
            ParentScene.RemoteMessenger.ApplyForce(LocalID, force);
        }

        /// <summary>
        /// Property for setting the base shape; not used by avatars
        /// </summary>
        public PrimitiveBaseShape BaseShape { get; protected set; }

        /// <summary>
        /// Property for setting the shape; currently use PhysicsShape
        /// instead. Not used by avatars.
        /// </summary>
        public override PrimitiveBaseShape Shape
        {
            set { BaseShape = value; }
        }
    }
}
