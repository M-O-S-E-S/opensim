
using System;
using System.Collections.Generic;
using System.Text;

using log4net;

using OpenMetaverse;
using OpenSim.Framework;
using OpenSim.Region.Physics.Manager;


namespace OpenSim.Region.Physics.RemotePhysicsPlugin
{
    public class RemotePhysicsPrimitive : RemotePhysicsObject
    {
        /// <summary>
        /// Enumeration used to indicate the state of a mesh asset.
        /// </summary>
        public enum AssetState : int
        {
            UNKNOWN = 0,
            WAITING = 1,
            FAILED_ASSET_FETCH = 2,
            FAILED_MESHING = 3,
            FETCHED = 4
        }

        /// <summary>
        /// The logger used for logging and debugging.
        /// </summary>
        protected static readonly ILog m_log = LogManager.GetLogger(
            System.Reflection.MethodBase.GetCurrentMethod().DeclaringType);

        /// <summary>
        /// String is used to identify the log statements from this class
        /// </summary>
        protected static readonly string LogHeader =
            "[REMOTE PHYSICS PRIMITIVE]";

        /// <summary>
        /// Indicates the type of object represented by this actor.
        /// </summary>
        protected int m_actorType;

        /// <summary>
        /// The position of this primitive.
        /// </summary>
        protected Vector3 m_position;

        /// <summary>
        /// The orientation of this primitive.
        /// </summary>
        protected Quaternion m_orientation;

        /// <summary>
        /// The linear velocity of this primitive.
        /// </summary>
        protected Vector3 m_velocity;

        /// <summary>
        /// The angular velocity of this primitive.
        /// </summary>
        protected Vector3 m_rotationalVelocity;

        /// <summary>
        /// The size of this primitive.
        /// </summary>
        protected Vector3 m_size;

        /// <summary>
        /// The mesh of this primitive (if it is not a regular shape).
        /// </summary>
        protected IMesh m_primitiveMesh = null;

        /// <summary>
        /// The acceleration of this primtive.
        /// </summary>
        protected Vector3 m_acceleration;

        /// <summary>
        /// The torque being applied to this primitive.
        /// </summary>
        protected Vector3 m_torque;

        /// <summary>
        /// Indicates whether this primitive is flying.
        /// </summary>
        protected bool m_isFlying;

        /// <summary>
        /// Indicates whether this primitive should float on water.
        /// </summary>
        protected bool m_floatOnWater;

        /// <summary>
        /// Indicates whether this primitive is being acted upon by
        /// physical forces.
        /// </summary>
        protected bool m_kinematic;

        /// <summary>
        /// The bouyancy of the primitive.
        /// </summary>
        protected float m_bouyancy;

        /// <summary>
        /// Indicates whether a user has selected this primitive.
        /// </summary>
        protected bool m_selected;

        /// <summary>
        /// Indicates whether a user has grabbed this primitive.
        /// </summary>
        protected bool m_grabbed;

        /// <summary>
        /// The mass of the primitive.
        /// </summary>
        protected float m_mass;

        /// <summary>
        /// The coefficient of static and kinetic friction of this primitive.
        /// </summary>
        protected float m_friction;

        /// <summary>   
        /// The coefficient of restitution of this primitive.
        /// </summary>
        protected float m_restitution;

        /// <summary>
        /// The density of the primitive.
        /// </summary>
        protected float m_density;

        /// <summary>
        /// Indicates whether this primitive should always move at a run speed.
        /// </summary>
        protected bool m_alwaysRun;

        /// <summary>
        /// The unique identifier used to represent the shape of this primitive
        /// in the remote physics engine.
        /// </summary>
        protected uint m_primShapeID;

        /// <summary>
        /// Indicates the state of assets required by this primitive.
        /// </summary>
        public AssetState PrimAssetState { get; protected set; }

        /// <summary>
        /// If the primitive is part of a linkset, this field will be a
        /// reference to the parent of the linkset.
        /// </summary>
        protected RemotePhysicsPrimitive m_linkParent = null;

        /// <summary>
        /// A list of child primitives that are linked to this primtive.
        /// </summary>
        protected List<RemotePhysicsPrimitive> m_childPrimitives =
            new List<RemotePhysicsPrimitive>();

        /// <summary>
        /// If the primitive is part of a linkset, this field will denote the
        /// position of the primitive relative to the linkset parent.
        /// </summary>
        protected OpenMetaverse.Vector3 m_linkPosition;

        /// <summary>
        /// If the primitive is part of a linkset, this field will denote the
        /// orientation of the primitive relative to the linkset parent.
        /// </summary>
        protected OpenMetaverse.Quaternion m_linkOrientation =
            Quaternion.Identity;

        /// <summary>
        /// Constructor.
        /// </summary>
        /// <param name="localID">The unique identifier of this
        /// primitive</param>
        /// <param name="primName">The name of this primitive</param>
        /// <param name="parentScene">The scene to which this primitive
        /// belongs</param>
        /// <param name="position">The initial position of the primitive</param>
        /// <param name="orientation">The initial orientation of the
        /// primitive</param>
        /// <param name="size">The size of the primitive</param>
        /// <param name="baseShape">The description of the shape of this
        /// primitive</param>
        /// <param name="isPhysical">Indicates whether this primitive should be
        /// acted upon by other forces</param>
        public RemotePhysicsPrimitive(uint localID, string primName,
            RemotePhysicsScene parentScene,
            Vector3 position, Quaternion orientation, Vector3 size,
            PrimitiveBaseShape baseShape, bool isPhysical) :
            base(parentScene, localID, primName, "RemotePhysicsPrimitive",
            parentScene.RemoteConfiguration)
        {
            // Indicate that this object is a primitive
            m_actorType = (int)ActorTypes.Prim;

            // Initialize the physical properties of the primitive
            Position = position;
            Orientation = orientation;
            Velocity = Vector3.Zero;
            BaseShape = baseShape;
            m_isPhysical = isPhysical;
            m_rotationalVelocity = Vector3.Zero;
            m_size = size;
  
            // Fetch a new ID for this primitive's shape
            m_primShapeID = ParentScene.GetNewShapeID();

            // Initialize the physical parameters of this primitive to their
            // default values
            m_friction = ParentConfiguration.DefaultFriction;
            m_restitution = ParentConfiguration.DefaultRestitution;
            m_density = ParentConfiguration.DefaultDensity;

            // Build the primitive actor in the remote phsyics engine;
            // indicate that the shape should also be rebuilt
            BuildPrimitive(true);

            // Fetch the mass of this primitive from the remote physics engine
            ParentScene.RemoteMessenger.GetActorMass(LocalID);

            // Indicate that the primitive has been initialized
            IsInitialized = true;
        }

        /// <summary>
        /// Clean up resources used by this primitive.
        /// </summary>
        public override void Destroy()
        {
            // Indicate that this object is no longer fully initialized
            IsInitialized = false;

            // Send out a message to the remote physics engine that removes
            // the actor and the shape associated with this primitive
            ParentScene.RemoteMessenger.RemoveActor(LocalID);
            ParentScene.RemoteMessenger.RemoveShape(m_primShapeID);

            // Check to see if this primitive is part of a linkset
            if (m_linkParent != null)
            {
                // Detach this primitive's shape from the linkset parent
                ParentScene.RemoteMessenger.DetachShape(m_linkParent.LocalID,
                    m_primShapeID);
            }

            // Destroy the super class members
            base.Destroy();
        }

        /// <summary>   
        /// Callback method that is called once an asset has been fetched.
        /// This method is used to ensure that assets for meshes have been
        /// fetched before attempting a recreation of the mesh.
        /// </summary>
        public void MeshAssetFetched(AssetBase asset)
        {
            bool assetFound;

            // Start out assuming that the asset has not been found
            assetFound = false;

            // Check to see if the asset is valid and that this primitive is a
            // sculpt mesh
            if (asset != null && BaseShape.SculptEntry)
            {
                // Check to see if the given asset is one for which this
                // primitive is looking
                if (BaseShape.SculptTexture.ToString() == asset.ID)
                {
                    // Initialize the asset data
                    BaseShape.SculptData = asset.Data;

                    // Indicate that the asset has been fetched
                    this.PrimAssetState = AssetState.FETCHED;
                    assetFound = true;

                    // Rebuild the primitive; make sure to also rebuild the
                    // shape with the new data
                    BuildPrimitive(true);

                    // Fetch the mass of this primitive from the remote
                    // physics engine
                    ParentScene.RemoteMessenger.GetActorMass(LocalID);
                }
            }

            // Check to see if the asset not found
            if (!assetFound)
            {
                // Indicate that the attempt to fetch the asset for this
                // primitive has failed
                this.PrimAssetState = AssetState.FAILED_ASSET_FETCH;
            }
        }

        public override bool Stopped { get { return false; } }

        /// <summary>
        /// Indicates the type of the object represented by this primitive.
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
        /// Indicates whether this primitive is static or dynamic.
        /// </summary>
        public override bool IsStatic
        {
            get 
            {
                // The primitive is static, if it's selected by the user or
                // is explicitly set to be static
                return m_selected || !IsPhysical;
            }
        }

        /// <summary>
        /// The mass of the primitive.
        /// </summary>
        public override float Mass
        {
            get
            {
                // Return the pre-computed mass of the primitive
                return m_mass;
            }
        }

        /// <summary>
        /// Method used by the remote physics engine to update the mass of
        /// the primitive.
        /// </summary>
        /// <param name="mass">The new mass of the primitive</param>
        public void UpdateMass(float mass)
        {
            // Update the mass of this primitive
            m_mass = mass;
        }

        /// <summary>
        /// The size of the primitive.
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
                // Check to see if the size of this primitive has changed
                if (m_size != value)
                {
                    // Update the size of the primitive
                    m_size = value;

                    // If this primitive has a mesh, it has to also be rebuilt
                    // according to the new size
                    m_primitiveMesh = null;

                    // Rebuild the primitive's actor and shape with the new
                    // shape size
                    BuildPrimitive(true);

                    // Fetch the mass of this primitive from the remote
                    // physics engine
                    ParentScene.RemoteMessenger.GetActorMass(LocalID);
                }
            }
        }

        /// <summary>
        /// The descriptor of the shape represented by this primitive.
        /// </summary>
        public PrimitiveBaseShape BaseShape { get; protected set; }

        /// <summary>
        /// Indicates whether the base shape of the primitive is a regular
        /// shape that does not require a mesh to represent. This means that
        /// the primitive shape has no cuts,
        /// tapers, twists, shears or irregular scaling.
        /// </summary>
        /// <returns>Boolean indicating whether the shape is regular</returns>
        public bool IsRegularShape()
        {
            return (BaseShape.ProfileBegin == 0 && BaseShape.ProfileEnd == 0 &&
                    BaseShape.ProfileHollow == 0 && BaseShape.PathTwist == 0 &&
                    BaseShape.PathTwistBegin == 0 && BaseShape.PathBegin == 0 &&
                    BaseShape.PathEnd == 0 && BaseShape.PathTaperX == 0 &&
                    BaseShape.PathTaperY == 0 && BaseShape.PathScaleX == 100 &&
                    BaseShape.PathScaleY == 100 && BaseShape.PathShearX == 0 &&
                    BaseShape.PathShearY == 0);
        }

        /// <summary>
        /// Indicates whether the primitive should float on water.
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
        /// Indicates whether the primitive should move at a runnig speed.
        /// NOTE: Not used by primitives, since primitives cannot run.
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
        /// Indicates whether the primitive is flying. Primitives are not meant
        /// to fly, so this property is ignored.
        /// </summary>
        public override bool Flying { get; set; }

        /// <summary>
        /// The type of the vehicle represented by this primitve
        /// (not supported).
        /// </summary>
        public override int VehicleType
        {
            get
            {
                return (int)Vehicle.TYPE_NONE;
            }

            set
            {
                return;
            }
        }

        /// <summary>
        /// Update a vehicle parameter (not supported).
        /// </summary>
        /// <param name="param">The parameter to be updated</param>
        /// <param name="value">The new value of the parameter</param>
        public override void VehicleFloatParam(int param, float value)
        {
            // Don't do anything here, since avatars aren't vehicles
        }

        /// <summary>
        /// Update a vehicle parameter (not supported).
        /// </summary>
        /// <param name="param">The parameter to be updated</param>
        /// <param name="value">The new value of the parameter</param>
        public override void VehicleVectorParam(int param,
            OpenMetaverse.Vector3 value)
        {
            // Don't do anything here, since avatars aren't vehicles
        }

        /// <summary>
        /// Update a vehicle parameter (not supported).
        /// </summary>
        /// <param name="param">The parameter to be updated</param>
        /// <param name="value">The new value of the parameter</param>
        public override void VehicleRotationParam(int param,
            OpenMetaverse.Quaternion rotation)
        {
        }

        /// <summary>
        /// Update a vehicle parameter (not supported).
        /// </summary>
        /// <param name="param">The parameter to be updated</param>
        /// <param name="value">The new value of the parameter</param>
        public override void VehicleFlags(int param, bool remove)
        {
        }

        /// <summary>
        /// Indicates whether this primitive can collide with non-physical
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
        /// The geomtric center of the object.
        /// </summary>
        public override OpenMetaverse.Vector3 GeometricCenter
        {
            get
            {
                OpenMetaverse.Vector3 sum;

                // Check to see if this object has any children objects that are
                // linked to it
                if (m_childPrimitives.Count == 0)
                {
                    // There are no children, so return the position of this
                    // primitive
                    return Position;
                }
                else
                {
                    // Since there are children objects, so average the
                    // positions of this primitive and the children primitives
                    sum = Position;
                    foreach (RemotePhysicsPrimitive currChild in
                        m_childPrimitives)
                    {
                        sum += currChild.Position;
                    }
                    sum /= (m_childPrimitives.Count + 1);

                    // Return the average position
                    return sum;
                }
            }
        }

        /// <summary>
        /// The location of the primitive's center of mass.
        /// </summary>
        public override OpenMetaverse.Vector3 CenterOfMass
        {
            get
            {
                Vector3 sum;
                float totalMass;

                // Check to see if this primitive is the parent of a linkset
                if (m_childPrimitives.Count > 0)
                {
                    // Calculate the weighted sum of the masses of each object
                    // in the linkset; also calculate the total mass of the
                    // linkset
                    sum = Position * Mass;
                    totalMass = Mass;
                    foreach (RemotePhysicsPrimitive currPrim in
                        m_childPrimitives)
                    {
                        sum += currPrim.Position * currPrim.Mass;
                        totalMass += currPrim.Mass;
                    }

                    // Average the weighted mass over the total mass to find the
                    // center of mass; make sure not to divide by 0
                    if (totalMass > 0.0f)
                        sum /= totalMass;

                    // Return the resulting position
                    return sum;
                }
                else
                {
                    // This is not a linkset, so return the regular position
                    return Position;
                }
            }
        }

        /// <summary>
        /// The linear velocity of the primitive.
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

                // Schedule an update to be sent right before the time is
                // advanced
                ParentScene.AddActorTaintCallback(delegate()
                {
                    ParentScene.RemoteMessenger.UpdateActorVelocity(
                        LocalID, m_velocity);
                });
            }
        }

        /// <summary>
        /// Method used by the remote physics engine to update the velocity of
        /// the primitive without issuing a taint callback.
        /// </summary>
        /// <param name="newVelocity">The new velocity of the primitive</param>
        public void UpdateVelocity(OpenMetaverse.Vector3 newVelocity)
        {
            // Update the linear velocity of the primitive if it is not part
            // of a linkset (in which case it will use its parent's velocity)
            if (m_linkParent != null)
                m_velocity = newVelocity;
        }

        /// <summary>
        /// The angular velocity of this primitive.
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
                    ParentScene.RemoteMessenger.UpdateActorAngularVelocity(                             LocalID, m_rotationalVelocity);
                });
            }
        }

        /// <summary>
        /// Method used by the remote physics engine to update the angular
        /// (rotational) velocity of the primitive without issuing a
        /// taint callback.
        /// </summary>
        /// <param name="newVelocity">The new angular velocity of the
        /// primitive</param>
        public void UpdateRotationalVelocity(OpenMetaverse.Vector3 newVelocity)
        {
            // Update the angular (rotational) velocity of the primitive if it
            // is not part a linkset (in which case it will use its parent's
            // rotational velocity)
            if (m_linkParent != null)
                m_rotationalVelocity = newVelocity;
        }

        /// <summary>
        /// The position of this primitive.
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
                // Update the position of this avatar
                m_position = value;

                // Schedule an update to be sent right before the time is
                // advanced, if this primitive is not a child of a linkset
                if (m_linkParent != null)
                {
                    ParentScene.AddActorTaintCallback(delegate()
                    {
                        ParentScene.RemoteMessenger.UpdateActorPosition(
                            LocalID, m_position);
                    });
                }
            }
        }

        /// <summary>
        /// Method used by the remote physics engine to update the position of
        /// the primitive without issuing a taint callback.
        /// </summary>
        /// <param name="newPosition">The new position of the primitive</param>
        public void UpdatePosition(OpenMetaverse.Vector3 newPosition)
        {
            // Update the position of the primitive
            m_position = newPosition;
        }

        /// <summary>
        /// The orientation of the primitive.
        /// </summary>
        public override OpenMetaverse.Quaternion Orientation
        {
            get
            {
                // Return the current orientation of this primitive
                return m_orientation;
            }

            set
            {
                // Update the current orientation of this primitive
                m_orientation = value;

                // Schedule an update to be sent right before the time is
                // advanced, if this primitive is not a child of a linkset
                if (m_linkParent != null)
                {
                    ParentScene.AddActorTaintCallback(delegate()
                    {
                        ParentScene.RemoteMessenger.UpdateActorOrientation(
                            LocalID, m_orientation);
                    });
                }
            }
        }

        /// <summary>
        /// Method used by the remote physics engine to update the orientation
        /// of the primitive without issuing a taint callback.
        /// </summary>
        /// <param name="newOrientation">The new orientation of the
        /// primitive</param>
        public void UpdateOrientation(OpenMetaverse.Quaternion newOrientation)
        {
            // Update the orientation of the primitive
            m_orientation = newOrientation;
        }

        /// <summary>
        /// Indicates whether this primitive is acted upon by physical forces.
        /// </summary>
        public override bool IsPhysical
        {
            get
            {
                return base.IsPhysical;
            }

            set
            {
                bool oldValue;

                // Store the previous value for future reference
                oldValue = IsStatic;

                // Update the physical state
                base.IsPhysical = value;

                // Check to see value has changed
                if (oldValue != IsStatic)
                {
                    // Check to see if this primitive has a regular shape
                    if (IsRegularShape())
                    {
                        // Only the actor needs to be rebuilt
                        BuildPrimitive(false);
                    }
                    else
                    {
                        // Rebuild the primitive's actor and shape, as the
                        // shape type may have to change in the remote
                        // physics engine
                        BuildPrimitive(true);
                    }

                    // Fetch the mass of this primitive from the remote
                    // physics engine
                    ParentScene.RemoteMessenger.GetActorMass(LocalID);
                }
            }
        }

        /// <summary>
        /// Sends updated properties of the primitive to the simulator.
        /// </summary>
        public override void RequestPhysicsterseUpdate()
        {
            // Only send the updates, if this primitive is not a child of a
            // linkset
            if (m_linkParent == null)
                base.RequestPhysicsterseUpdate();
        }

        /// <summary>
        /// Rebuilds the primitive's representation in the remote physics
        /// engine.
        /// </summary>
        /// <param name="buildShape">Indicates whether the shape for this
        /// primitive should be rebuilt in the remote physics engine</param>
        public void BuildPrimitive(bool buildShape)
        {
            int[] indices;
            List<OpenMetaverse.Vector3> vertexList;
            RequestAssetDelegate assetDelegate;
            uint actorID;
            Vector3 shapePos;
            Quaternion shapeOrient;
            bool shapeSuccess;

            // Remove the actor from the remote engine
            ParentScene.RemoteMessenger.RemoveActor(LocalID);

            // Start out assuming that the shape has been successfully
            // rebuilt
            shapeSuccess = true;

            // Check to see if the shape of this primitive has to be rebuilt
            // in the remote engine
            if (buildShape)
            {
                // Remove the old shape from the remote engine
                ParentScene.RemoteMessenger.RemoveShape(m_primShapeID);

                // Check to see if this primitive's shape is simple
                if (IsRegularShape())
                {
                    // Check to see the type of the primitive
                    if (BaseShape.ProfileShape == ProfileShape.HalfCircle &&
                        BaseShape.PathCurve == (byte)Extrusion.Curve1 &&
                        BaseShape.Scale.X == BaseShape.Scale.Y &&
                        BaseShape.Scale.Y == BaseShape.Scale.Z)
                    {
                        // Create a sphere in the remote physics engine
                        ParentScene.RemoteMessenger.AddSphere(m_primShapeID,
                            Vector3.Zero, Size.X / 2.0f);
                    }
                    else if (BaseShape.ProfileShape == ProfileShape.Square &&
                        BaseShape.PathCurve == (byte)Extrusion.Straight)
                    {
                        // Create a box in the remote physics engine
                        ParentScene.RemoteMessenger.AddBox(m_primShapeID,
                            Size.X, Size.Y, Size.Z);
                    }
                }
                else
                {
                    // Check to see if this primitive already has the mesh
                    // it needs
                    if (m_primitiveMesh == null)
                    {
                        // Re-create the mesh so that it can be added to the
                        // remote physics engine in its new form
                        m_primitiveMesh = ParentScene.SceneMesher.CreateMesh(
                            Name, BaseShape, Size, 32, m_isPhysical, false);
                    }
 
                    // Check to see if the mesh was successfully created
                    if (m_primitiveMesh != null)
                    {
                        // The primitive has to be switched from static to
                        // dynamic or vice versa
                        if (!IsStatic)
                        {
                            // Since the mesh must now be physically active,
                            // it can no longer be a triangle mesh;
                            // triangle meshes can't be dynamic in the
                            // remote physics engine
                            // Re-create the mesh as a convex mesh
                            vertexList = m_primitiveMesh.getVertexList();
                            ParentScene.RemoteMessenger.AddConvexMesh(
                                m_primShapeID, vertexList.ToArray());
                        }
                        else
                        {
                            // The mesh is no longer physical, and can now
                            // now be switched back to being a triangle
                            // mesh, which has a higher level of detail
                            vertexList = m_primitiveMesh.getVertexList();
                            indices = m_primitiveMesh.getIndexListAsInt();
                            ParentScene.RemoteMessenger.AddTriangleMesh(
                                m_primShapeID, vertexList.ToArray(),
                                indices);
                        }
                    }
                    else
                    {
                        // Check to see if the conditions are right to fetch
                        // the assets for the mesh. Don't attempt to
                        // fetch the texture , if this object is already
                        // waiting for the texture fetch
                        if (BaseShape.SculptEntry &&
                            PrimAssetState !=
                            AssetState.FAILED_ASSET_FETCH &&
                            PrimAssetState != AssetState.FAILED_MESHING &&
                            PrimAssetState != AssetState.WAITING &&
                            PrimAssetState != AssetState.FETCHED &&
                            BaseShape.SculptTexture != UUID.Zero)
                        {
                            // Indicate that this primitive is now
                            // waiting on an asset
                            PrimAssetState = AssetState.WAITING;
 
                            // Get the request asset from the parent scene
                            assetDelegate = ParentScene.RequestAssetMethod;
 
                            // If the request asset method is valid,
                            // go ahead and request the texture asset for
                            // the mesh
                            if (assetDelegate != null)
                            {
                                assetDelegate(BaseShape.SculptTexture,
                                    new AssetReceivedDelegate(
                                        MeshAssetFetched));
                            }
                        }
                        else if (PrimAssetState == AssetState.FETCHED)
                        {
                            // The mesher has failed to create the mesh even
                            // with all the assets, so mark the state as such
                            PrimAssetState = AssetState.FAILED_MESHING;
                        }

                        // Indicate that the shape was not built successfully
                        shapeSuccess = false;
                    }
                }
            }

            // Check to see if this primitive is part of a linkset
            if (m_linkParent != null)
            {
                // This primitive is part of a linkset, so attach the
                // shape to the parent of the linkset
                // The actor for this primitive does not have to be
                // re-created as it won't have any shapes attached to it
                actorID = m_linkParent.LocalID;
                shapePos = m_linkPosition;
                shapeOrient = m_linkOrientation;
            }
            else
            {
                // This primitive is not part of a linkset, so re-create
                // the primitive's actor
                if (!IsStatic)
                {
                    // Re-add the primitive as a dynamic actor
                    ParentScene.RemoteMessenger.CreateDynamicActor(
                        LocalID, Position, Orientation, 1.0f,
                        Velocity, m_rotationalVelocity,
                        ParentScene.RemoteConfiguration.
                            ReportNonAvatarCollisions);
                }
                else
                {
                    // Re-add the primitive as a static actor
                    ParentScene.RemoteMessenger.CreateStaticActor(
                        LocalID, Position, Orientation,
                        ParentScene.RemoteConfiguration.
                            ReportNonAvatarCollisions);
                }

                // Use the default position to re-attach the shape
                actorID = LocalID;
                shapePos = Vector3.Zero;
                shapeOrient = Quaternion.Identity;
            }

            // Re-attach the shape
            if (shapeSuccess)
            {
                ParentScene.RemoteMessenger.AttachShape(actorID,
                    m_primShapeID, Density, Friction, Friction, Restitution,
                    shapeOrient, shapePos);
            }
        }

        /// <summary>
        /// Property representing the density of this primitive.
        /// </summary>
        public override float Density
        {
            get
            {
                // Return the density of this primitive
                return m_density;
            }

            set
            {
                uint actor;

                // Check to see if the given value is a valid density
                // If not, exit out
                if (value <= 0.0f)
                   return;

                // Update the density of this primitive
                m_density = value;

                // Check to see if this primitive is part of a linkset
                if (m_linkParent != null)
                {
                    // The primitive is attached to the linkset parent, so use
                    // the parent's ID
                    actor = m_linkParent.LocalID;
                }
                else
                {
                    // This primitive is not part of a linkset, so use its ID
                    // as normal
                    actor = LocalID;
                }

                // Update the material of this primitive's shape in the
                // remote physics engine
                ParentScene.RemoteMessenger.UpdateShapeMaterial(actor,
                    m_primShapeID, Friction, Friction, Restitution,
                    m_density);
            }
        }

        /// <summary>
        /// Property representing the coefficients of static and kinematic
        /// friction of this primitive.
        /// </summary>
        public override float Friction
        {
            get
            {
                // Return the coefficient of friction of this primitive
                return m_friction;
            }

            set
            {
                uint actor;

                // Update the friction of this primitive
                m_friction = value;

                // Check to see if this primitive is part of a linkset
                if (m_linkParent != null)
                {
                    // The primitive is attached to the linkset parent, so
                    // use the parent's ID
                    actor = m_linkParent.LocalID;
                }
                else
                {
                    // The primitive is not part of a linkset, so use its ID
                    // as normal
                    actor = LocalID;
                }

                // Update the material of this primitive's shape in the remote
                // physics engine
                ParentScene.RemoteMessenger.UpdateShapeMaterial(actor,
                    m_primShapeID, m_friction, m_friction, Restitution,
                    Density);
            }
        }

        /// <summary>
        /// Property representing the coefficient of restitution of the
        /// primitive.
        /// </summary>
        public override float Restitution
        {
            get
            {
                // Return the coefficient of restitution of this primitive
                return m_restitution;
            }

            set
            {
                uint actor;

                // Update the restitution of this primitive
                m_restitution = value;

                // Check to see if this primitive is part of a linkset
                if (m_linkParent != null)
                {
                    // The primitive is attached to the linkset parent, so
                    // use the parent's ID
                    actor = m_linkParent.LocalID;
                }
                else
                {
                    // The primitive is not part of a linkset, so use its ID
                    // as normal
                    actor = LocalID;
                }

                // Update the material of this primitive's shape in the remote
                // physics engine
                ParentScene.RemoteMessenger.UpdateShapeMaterial(actor,
                    m_primShapeID, Friction, Friction, m_restitution, Density);
            }
        }

        /// <summary>
        /// The acceleration of the primitive.
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
        /// The torque being applied to the primitive.
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
        /// The bouyancy of the primitive.
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
        /// Indicates whether the primitive is being grabbed by a user.
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
        /// Indicates whether the primitive is being selected by a user.
        /// </summary>
        public override bool Selected
        {
            set
            {
                bool oldValue;

                // Check to see whether the selected value is actually changing
                if (m_selected != value)
                {
                    // Store the previous static status of this primitive, so
                    // that it can be used to see if there has been any change
                    oldValue = IsStatic;

                    // Update the value indicating whether this primitive has
                    // been selected by the user
                    m_selected = value;
 
                    // Check to see if there has been a change in the static
                    // status of the object
                    if (oldValue != IsStatic)
                    {
                        // Check to see if this primitive has a regular shape
                        if (IsRegularShape())
                        {
                            // Only the actor of this primitive has to be
                            // rebuilt
                            BuildPrimitive(false);
                        }
                        else
                        {
                            // Rebuild the actor and the shape, as the shape
                            // type may have to change in the remote physics
                            // engine
                            BuildPrimitive(true);
                        }

                        // Fetch the mass of this primitive from the remote
                        // physics engine
                        ParentScene.RemoteMessenger.GetActorMass(LocalID);
                    }
                }
            }
        }

        /// <summary>
        /// Indicates whether this primitive is being acted upon by physical
        /// forces.
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
        /// Link this primitive to a specified parent primitive.
        /// </summary>
        /// <param name="actor">The other actor with which to link</param>
        public override void link(PhysicsActor actor)
        {
            // Attempt to convert the given actor into a RemotePhysicsPrimitive
            m_linkParent = actor as RemotePhysicsPrimitive;

            // Check to see if the given actor was successfully converted
            if (m_linkParent != null)
            {
                // Add this object as a child object to the given primitive's
                // linkset
                m_linkParent.AddToLinkset(this);

                // Add a taint callback to calculate the linkset position of
                // this primitive just before the next simulation period.
                // This is done to avoid conflicting linking use-cases provided
                // by OpenSim
                ParentScene.AddActorTaintCallback(ComputeLinkset);

                // Clear the velocities on this primitive as it will now start
                // behaving according to the velocity of its parent
                m_velocity = OpenMetaverse.Vector3.Zero;
                m_rotationalVelocity = OpenMetaverse.Vector3.Zero;
            }
        }

        /// <summary>
        /// Computes the position and orientation of the primitive relative to
        /// a linkset parent.
        /// </summary>
        protected void ComputeLinkset()
        {
            // Calculate the position of this primitive relative to the
            // parent primitive
            m_linkPosition = Position - m_linkParent.Position;
            m_linkPosition *= Quaternion.Inverse(m_linkParent.Orientation);

            // Calculate the relative orientation in a similar manner
            m_linkOrientation = Quaternion.Inverse(
                m_linkParent.Orientation) * Orientation;

            BuildPrimitive(false);
        }

        /// <summary>
        /// Delink this primitive from actors.
        /// </summary>
        public override void delink()
        {
            // Check to see if the object has a valid parent from which to
            // delink
            if (m_linkParent != null)
            {
                // Remove this primitive from the parent's children list
                m_linkParent.RemoveFromLinkset(this);

                // Detach this primitive's shape from the linkset parent
                ParentScene.RemoteMessenger.DetachShape(m_linkParent.LocalID,
                    m_primShapeID);

                // Re-calculate the position and orientation of this primitive
                // based on the current position and orientation of the
                // parent orientation
                UpdatePosition((m_linkPosition * m_linkParent.Orientation) +
                    m_linkParent.Position);
                UpdateOrientation(m_linkOrientation * m_linkParent.Orientation);

                // Indicate that this primitive no longer has a parent
                m_linkParent = null;

                // Rebuild the primitive in the remote physics engine;
                // the shape does not need to be rebuilt
                BuildPrimitive(false);
            }
        }

        /// <summary>
        /// Adds the given physics primitive to the list of children primitives
        /// that are considered to be linked under the primitive.
        /// </summary>
        /// <param name="childPrim">The child primitive to be added</param>
        public void AddToLinkset(RemotePhysicsPrimitive childPrim)
        {
            // Check to see if the given primitive is valid, and that it is
            // not this primitive (linking to self is not allowed)
            if (childPrim != null && childPrim != this)
            {
                // Add the new child primitive to the list
                m_childPrimitives.Add(childPrim);
            }
        }

        /// <summary>
        /// Removes the given physics primitive from the list of children
        /// primitives.
        /// </summary>
        /// <param name="childPrim">The child primitive to be removed</param>
        public void RemoveFromLinkset(RemotePhysicsPrimitive childPrim)
        {
            // Remove the given child from the list of children objects that are
            // linked under this object
            m_childPrimitives.Remove(childPrim);
        }

        /// <summary>
        /// Stop angular motion of this primitive (not supported).
        /// </summary>
        /// <param name="axis">The axis around which angular motion should be
        /// stopped</param>
        public override void LockAngularMotion(OpenMetaverse.Vector3 axis)
        {
            // This feature is not supported, so just exit
            return;
        }

        /// <summary>
        /// Add an angular force to the primitive (not supported).
        /// </summary>
        /// <param name="force">The force being applied</param>
        /// <param name="pushforce">Indicates whether the force is a
        /// push</param>
        public override void AddAngularForce(Vector3 force, bool pushforce)
        {
            // This feature is not supported, so just exit
            return;
        }

        /// <summary>
        /// Update the momentum of the primitive.
        /// </summary>
        /// <param name="momentum">The new momentum</param>
        public override void SetMomentum(Vector3 momentum)
        {
            // This feature is not supported, so just exit
            return;
        }

        /// <summary>
        /// Add a force to the primitive.
        /// </summary>
        /// <param name="force">The force being applied</param>
        /// <param name="pushforce">Indicates whether the force is a
        /// push</param>
        public override void AddForce(Vector3 force, bool pushforce)
        {
            float forceCorrectionFactor;

            // Check to see if this avatar has a physical representation in
            // the engine
            if (IsPhysical)
            {
                // PhysX is able to handle the normal density value of objects,
                // however OpenSim sends its force values assuming that the
                // density value has been decreased by a 0.1 factor, so this
                // variable corrects the issue
                forceCorrectionFactor = 10.0f;

                // Send a message to the remote physics engine with the
                // new force
                ParentScene.RemoteMessenger.ApplyForce(LocalID, force *
                    forceCorrectionFactor);
            }
        }

        /// <summary>
        /// Property for setting the shape description. Currently use
        /// PhysicsShape instead.
        /// </summary>
        public override PrimitiveBaseShape Shape
        {
            set { BaseShape = value; }
        }

        /// <summary>
        /// Update the physical material archetype of the primitive.
        /// </summary>
        /// <param name="material">The index of the material to which the
        /// primitive should be changed</param>
        public override void SetMaterial(int material)
        {
            RemotePhysicsMaterialAttributes attributes;

            // Fetch the attributes for the given material from the library
            attributes = ParentScene.MaterialLibrary.GetAttributes(
                (RemotePhysicsMaterialLibrary.Material) material);

            // Update this object's material properties
            Density = attributes.m_density;
            Friction = attributes.m_friction;
            Restitution = attributes.m_restitution;
        }
    }
}
