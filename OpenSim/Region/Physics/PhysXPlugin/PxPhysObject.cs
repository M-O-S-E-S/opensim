
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

using OpenMetaverse;
using OpenSim.Framework;
using OpenSim.Region.Physics.Manager;

using log4net;

namespace OpenSim.Region.Physics.PhysXPlugin
{

    // The actor type, used when any of the actor's physical properties such
    // as restitution, friction, and size are being updated; since a new actor 
    // is created with these updated values, the actor type lets PhysX know 
    // which type of actor to recreate with these updated values
    public enum ActorType : int
    {
        UNKNOWN = 0,
        AVATAR = 1,
        PRIM = 2
    }

    public enum AssetState : int
    {
        UNKNOWN = 0,
        WAITING = 1,
        FAILED_ASSET_FETCH = 2,
        FAILED_MESHING = 3,
        FETCHED = 4
    }


    /// <summary>
    /// Class that represents physical objects in the physical scene.
    /// </summary>
    public class PxPhysObject : PhysicsActor
    {
        #region Logging

        /// <summary>
        /// The logger for this plugin.
        /// </summary>
        internal static readonly ILog m_log = LogManager.GetLogger(
            System.Reflection.MethodBase.GetCurrentMethod().DeclaringType);

        /// <summary>
        /// Header used for logger to highlight logs made in this class.
        /// </summary>
        internal static readonly string LogHeader = "[PHYSX PXPHYSOBJECT]";

        #endregion // Logging

        public const double TOLERANCE = 0.000001;
        
        #region Instance Fields

        /// <summary>
        /// A local instance field that will store a reference to the scene
        /// that created this physical object.
        /// </summary>
        private PxScene m_pxScene;

        /// <summary>
        /// The last vector3 given to this PhysXObject, this is needed because
        /// the scaling of the object has to be done on a per shape basis, and
        /// the current scaling value will not be able to be calculated.
        /// </summary>
        private Vector3 m_currentSize;

        /// <summary>
        /// The unique identifier of the primary shape attached to this object.
        /// </summary>
        private uint m_shapeID;

        /// <summary>
        /// The density given to this PhysXObject, used in calculations of
        /// the mass of the PhysXObject.
        /// </summary>
        private float m_density;

        /// <summary>
        /// This is the shape provided by opensim that will need to be
        /// converted to a PhysX shape for physics calculations.
        /// </summary>
        private PrimitiveBaseShape m_opensimBaseShape;

        /// <summary>
        /// Determine if this object has been selected by the user.
        /// </summary>
        private bool m_isSelected;

        /// <summary>
        /// The internal grabbed variable. I don't know much more than this
        /// because their are no comments and the value doesn't appear to be in
        /// use anymore.
        /// </summary>
        private bool m_grabbed;

        /// <summary>
        /// A locally stored position variable that will reduce the lookup time
        /// of the position due to the large number of times the position is
        /// changed and requested.
        /// </summary>
        private Vector3 m_rawPosition;

        /// <summary>
        /// A locally stored variable to track the current velocity of the
        /// physical object.
        /// </summary>
        private Vector3 m_velocity;

        /// <summary>
        /// A locally stored variable to track the current torque being applied
        /// to the physical object.
        /// </summary>
        private Vector3 m_torque;

        /// <summary>
        /// A variable to store the current mass of the physical object.
        /// </summary>
        private float m_mass;

        /// <summary>
        /// A variable to store the force being applied to the physical object.
        /// </summary>
        private Vector3 m_rawForce;

        /// <summary>
        /// The current acceleration of the physical object.
        /// </summary>
        private Vector3 m_acceleration;

        /// <summary>
        /// The current orientation of the physical object.
        /// </summary>
        private Quaternion m_orientation;

        /// <summary>
        /// The type of physics actor that this physical object represents.
        /// </summary>
        private int m_physicsActorType;

        /// <summary>
        /// Flag that determines if the object should act according to physics.
        /// </summary>
        private bool m_isPhysical;

        /// <summary>
        /// Flag to determine if the object is currently flying.
        /// </summary>
        private bool m_isFlying;

        /// <summary>
        /// Flag used to determine if the actor has been initialized
        /// </summary>
        private bool m_isActorInitialized;

        /// <summary>
        /// Indicates the state of the assets required by this object.
        /// Only applies to Primitives.
        /// </summary>
        public AssetState PrimAssetState { get; protected set; }

        /// <summary>
        /// Indicates the name of the vehicle actor.
        /// </summary>
        public const string VehicleActorName = "BasicVehicle";

        /// <summary>
        /// Caches a constructed mesh, such that it does not have to re-created
        /// unless necessary. Will need to be re-created if physical properties
        /// of the object or the level of detail changes.
        /// <summary>
        private IMesh m_primMesh;

        /// <summary>
        /// The current level of detail the mesh (if this object has one).
        /// Only applies to Primitives and is used to ensure that convex meshes
        /// are not too complex for PhysX.
        /// </summary>
        private float m_meshLOD;

        /// <summary>
        /// The default level of detail for meshes. This is the highest
        /// level of detail supported by the mesher.
        /// </summary>
        private static readonly float m_defaultMeshLOD = 32.0f;

        /// <summary>
        /// Lock used to ensure that the process of building the physical shape
        /// of the object is thread-safe.
        /// </summary>
        private Object m_shapeLock = new Object();

        /// <summary>
        /// Determines if the avatar is currently running and multiplies the
        /// avatar velocity by the run factor set by the user.
        /// </summary>
        private bool m_isAlwaysRun;

        /// <summary>
        /// 
        /// </summary>
        private bool m_throttleUpdates;

        /// <summary>
        /// Flag that is used to determine whether the object is floating on
        /// water. This is here to prevent unecessary updates when the flag is
        /// being set to the same value.
        /// </summary>
        private bool m_floatOnWater;

        /// <summary>
        /// 
        /// </summary>
        private Vector3 m_rotationalVelocity;

        /// <summary>
        ///
        /// </summary>
        private bool m_kinematic;

        /// <summary>
        /// 
        /// </summary>
        private float m_buoyancy;

        /// <summary>
        /// The volume of the physical object.
        /// </summary>
        private float m_avatarVolume;

        /// <summary>
        /// The unique identifier of the joint used to keep the avatar
        /// upright against the effects of gravity.
        /// </summary>
        private uint m_fallJointID;

        /// <summary>
        /// If the object is part of a linkset, this field will denote the
        /// position of this object relative to the linkset parent.
        /// </summary>
        private Vector3 m_linkPos;

        /// <summary>
        /// If the object is part of a linkset, this field will denote the
        /// orientation of this object relative to the linkset parent.
        /// </summary>
        private Quaternion m_linkOrient;

        /// <summary>
        /// If the object is part of a linkset, this field will be a reference
        /// to the parent of the linkset.
        /// </summary>
        private PxPhysObject m_linkParent = null;

        /// <summary>
        /// A list of the child objects that are linked to the object.
        /// </summary>
        private List<PxPhysObject> m_childObjects = new List<PxPhysObject>();

        /// <summary>
        /// A mutex object used to ensure that the linkset children list is
        /// thread-safe.
        /// </summary>
        private Object m_childLock = new Object();

        /// <summary>
        /// Stores the collisions that have been collected for the new
        /// collision report to OpenSim.
        /// </summary>
        private CollisionEventUpdate m_collisionCollection;

        /// <summary>
        /// The collision collection that was last reported to OpenSim.
        /// </summary>
        private CollisionEventUpdate m_prevCollisionCollection;

        /// <summary>
        /// Stores the next time that the collisions will be reported to
        /// OpenSim.
        /// </summary>
        private int m_nextCollisionOkTime;

        /// <summary>
        /// Tracks the number of collisions that have happened with this object.
        /// </summary>
        private long NumCollisions { get; set; }

        /// <summary>
        /// Requested number of milliseconds between collision events. If this
        /// value is zero collision events are disabled.
        /// </summary>
        private int m_subscribedEventsTime;

        /// <summary>
        /// Indicates the number of times the object has failed to cross
        /// a region boundary into a new region.
        /// </summary>
        private int CrossingFailures { get; set; }

        /// <summary>
        /// A collection of physical actors that are acting upon this physics
        /// object within the scene.
        /// </summary>
        private PxActorCollection m_physicalActors;

        /// <summary>
        /// The physical actors collection for this physics object.
        /// </summary>
        public PxActorCollection PhysicalActors
        {
            get 
            { 
                return m_physicalActors;
            }

            protected set 
            {
                m_physicalActors = value;
            }
        }

        /// <summary>
        /// The float on water boolean for this physical object.
        /// </summary>
        public bool FloatsOnWater
        {
            get
            {
                return m_floatOnWater;
            }

            protected set
            {
                m_floatOnWater = value;
            }
        }

        /// <summary>
        /// Internal field that indicates whether the shape for this object
        /// has been built inside the PhysX scene.
        /// </summary>
        private bool m_isObjectBuilt = false;

        #endregion // Instance Fields

        #region 

        /// <summary>
        /// Indicates the simulation step at which the last collision with the
        /// ground has occurred.
        /// </summary>
        protected int GroundCollisionTime { get; set; }

        /// <summary>
        /// Indicates the simulation step at which the last collision with an
        /// object occurred.
        /// </summary>
        protected int ObjectCollisionTime { get; set; }
        
        /// <summary>
        /// Indicates the simulation step at which any type of collision 
        /// occurred.
        /// </summary>
        public int LastCollisionTime { get; set; }

        #endregion

        #region Constructors


        /// <summary>
        /// Physical object that represents an avatar in the scene.
        /// </summary>
        /// <param name="localID">Unique ID for the physical object</param>
        /// <param name="name">Name of the physical object</param>
        /// <param name="parentScene">The scene that this object belongs to
        /// </param>
        /// <param name="position">Current position of the new object</param>
        /// <param name="velocity">Current velocity of the new object</param>
        /// <param name="size">Size of the object; used to scale it</param>
        /// <param name="isFlying">Whether the object is currently
        /// flying or not</param>
        public PxPhysObject(uint localID, string name, PxScene parentScene, 
            Vector3 position, Vector3 velocity, Vector3 size, bool isFlying)
        {
            // Initialize the physical actors collection
            m_physicalActors = new PxActorCollection(parentScene);

            // Actor has not been initialized yet
            m_isActorInitialized = false;

            // Keep a reference to the parent scene of this physical object,
            // mostly to make calls to the physx api
            m_pxScene = parentScene;
            
            // Save the unique ID that will be used by the unmanaged and
            // managed code to track changes in the physical object
            LocalID = localID;
            
            // Store the name of the actor given by opensim
            Name = name;

            // Store the starting position of the physical object
            m_rawPosition = position;

            // Store the starting velocity of the physical object
            m_velocity = velocity;

            // Store the size of the physical object
            m_currentSize = size;

            // Store the starting orientation of the physical object
            m_orientation = Quaternion.Identity;

            // Fetch a new unique identifier for the primary shape that will
            // be associated with this actor
            m_shapeID = m_pxScene.GetNewShapeID();

            // Initialise the collision event update collections
            m_collisionCollection = new CollisionEventUpdate();
            m_prevCollisionCollection = m_collisionCollection;

            // No collisions have occurred, so start the collision count at 0
            NumCollisions = 0;

            // Set the physics actor type to avatar
            m_physicsActorType = (int) ActorType.AVATAR;

            // Use the default values for the avatar friction
            base.Friction = m_pxScene.UserConfig.AvatarStaticFriction;

            // Old versions of ScenePresence pass only the height of the
            // physical object, so check the width and depth and assign a
            // default value if that is the case
            if (m_currentSize.X == 0.0f)
            {
                // Assign a default depth to the physical object
                m_currentSize.X = m_pxScene.UserConfig.AvatarShapeDepth;
            }
            if (m_currentSize.Y == 0.0f)
            {
                // Assign a default width to the physical object
                m_currentSize.Y = m_pxScene.UserConfig.AvatarShapeWidth;
            }

            // Compute the volume and mass of the avatar
            ComputeAvatarVolumeAndMass();

            // Set the density of the avatar after it is initialized
            m_density = m_pxScene.UserConfig.AvatarDensity;

            // Send the information over to PhysX unmannaged and make
            // sure to set initialized to true
            // Collisions involving avatars should always be reported
            m_pxScene.PhysX.CreateCharacterCapsule(LocalID, Name,
                m_rawPosition, m_orientation, m_shapeID, Friction, Friction,
                Restitution,  ComputeAvatarHalfHeight(), 
                Math.Min(m_currentSize.X, m_currentSize.Y) / 2.0f, Density,
                true, true);

            // Add a joint between this avatar and the ground plane in
            // order to keep the avatar always standing up
            m_fallJointID = m_pxScene.GetNewJointID();
            m_pxScene.PhysX.AddGlobalFrameJoint(m_fallJointID, LocalID,
                Vector3.Zero, Quaternion.Identity, Vector3.One,
                Vector3.Zero, Vector3.Zero, Vector3.Zero);

            // Indicate that the shape has been built for this avatar
            m_isObjectBuilt = true;

            // Set whether the avatar is flying after it is initialized 
            Flying = isFlying;

            // Send info about the object's transform data
            m_pxScene.PhysX.SetTransformation(
                LocalID, m_rawPosition, m_orientation);

            // The actor has been initialized, set flag to true
            m_isActorInitialized = true;
        }


        /// <summary>
        /// Constructor for a physical object that is not an avatar.
        /// </summary>
        /// <param name="localID">Unique ID for the physical object</param>
        /// <param name="name">Name of the physical object</param>
        /// <param name="parentScene">The scene that this object belongs to
        /// </param>
        /// <param name="position">Current position of the new object</param>
        /// <param name="size">Size of the object; used to scale it</param>
        /// <param name="rotation">Current rotation of the object</param>
        /// <param name="pbs">Basic prim shape info about the object's mass and
        /// geometry</param>
        /// <param name="isPhysical">Whether object should react to
        /// forces and collisions</param>
        public PxPhysObject(uint localID, string name, PxScene parentScene,
            Vector3 position, Vector3 size, Quaternion rotation,
            PrimitiveBaseShape pbs, bool isPhysical)
        {
            // Initialize the physical actors collection
            m_physicalActors = new PxActorCollection(parentScene);            

            // Actor has not been initialized yet
            m_isActorInitialized = false;

            // Keep a reference to the parent scene of this physical object,
            // mostly to make calls to the physx api
            m_pxScene = parentScene;

            // Since this is a prim it can not run
            m_isAlwaysRun = false;

            // Save the unique ID that will be used by the unmanaged and
            // managed code to track changes in the physical object
            LocalID = localID;

            // Store the name of the actor given by opensim
            Name = name;

            // Set the physics actor type to primitive
            m_physicsActorType = (int) ActorType.PRIM;

            // Store the starting position of the physical object
            m_rawPosition = position;

            // Store the size of the physical object
            m_currentSize = size;

            // Store the starting orientation of the physical object
            m_orientation = rotation;

            // Store the basic prim shape information for determining mass and
            // geometry creation
            m_opensimBaseShape = pbs;

            // Buoyancy should be set to zero initially and allow OpenSim to
            // update the value later
            m_buoyancy = 0.0f;

            // Store whether the object should act like a rigid actor or
            // dynamic actor inside of PhysX
            m_isPhysical = isPhysical;

            // Start the object velocity at zero
            m_velocity = Vector3.Zero;

            // Fetch a new unique identifier for the primary shape that will
            // be associated with this actor
            m_shapeID = m_pxScene.GetNewShapeID();

            // Assign a default friction, and resititution to the prim
            base.Friction = m_pxScene.UserConfig.DefaultFriction;
            base.Restitution = m_pxScene.UserConfig.DefaultRestitution;

            // Indicate that the state of any assets needed by this object is
            // unknown
            PrimAssetState = AssetState.UNKNOWN;

            // The mesh for this object (if any) has not yet been created,
            // so initialize to null
            m_primMesh = null;

            // The default level of detail for meshes is 32, which is also the
            // highest
            m_meshLOD = m_defaultMeshLOD;

            // Create the PhysX Wrapper physical shape for this physical object
            m_pxScene.AddTaintedObject(this);

            // Initialise the collision event update collections
            m_collisionCollection = new CollisionEventUpdate();
            m_prevCollisionCollection = m_collisionCollection;
        }


        #endregion // Constructors

        #region PhysicsActor Overrides


        /// <summary>
        /// Apparently this method is not used inside of OpenSim anymore, but 
        /// is still included inside the PhysicsActor
        /// </summary>
        public override bool Stopped
        {
            get
            {
                // Since this is no longer used just return false
                return false;
            }
        }

    
        /// <summary>
        /// Allows opensim access to the size of the physical actor, either to
        /// change the size or to get the current size.
        /// </summary>
        public override Vector3 Size
        {
            get
            {
                // Return the previously set size of the physical object
                return m_currentSize;
            }
            set
            {
                // Only update the size if the value has changed
                if (m_currentSize != value)
                {
                    // Set the size of the current physical object, this will be
                    // used in recreating the object on the PhysX side
                    m_currentSize = value;

                    // Check to see if this object is not part of a linkset;
                    // the object does not have a corresponding actor in the
                    // PhysX scene, if it is part of a linkset
                    if (m_linkParent == null)
                    {
                        // Remove the actor from the scene, so that it
                        // can be properly re-built
                        m_pxScene.PhysX.RemoveActor(LocalID);
                    }
                    else
                    {
                        // Since this object is part of a linkset, remove
                        // its shape from the linkset parent instead, since
                        // it does not have an actor in the PhysX scene
                        m_pxScene.PhysX.RemoveShape(m_linkParent.LocalID,
                            m_shapeID);
                    }

                    // Indicate that this object is no longer built in the
                    // physics scene
                    m_isObjectBuilt = false;

                    // If there is a mesh for this object's shape, it needs
                    // to be rebuilt; so release the reference to the old mesh
                    m_primMesh = null;

                    // Reset the asset state, because they need to be fetched
                    // again
                    PrimAssetState = AssetState.UNKNOWN;
                    
                    // Update the actor with updated size value
                    if (m_linkParent == null)
                        m_pxScene.AddTaintedObject(this);
                    else
                        m_pxScene.AddTaintedObject(m_linkParent);
                }
            }
        }


        /// <summary>
        /// Allows opensim to create or modify a physical object with a new
        /// shape. 
        /// </summary>
        public override PrimitiveBaseShape Shape
        {
            set
            {
                // Store the new shape sent by opensim
                m_opensimBaseShape = value;
            }
        }


        /// <summary>
        /// Allows opensim to change the value of the internal m_grabbed
        /// variable
        /// </summary>
        public override bool Grabbed
        {
            set
            {
                // Allow opensim to set the value as to whether this object is
                // grabbed
                // NOTE: I don't think this is being used anymore
                m_grabbed = value;
            }
        }

        
        /// <summary>
        /// Called when this object is selected to be modified.
        /// </summary>
        public override bool Selected
        {
            set
            {
                // Since this will require the physical shape to be rebuilt
                // make sure that the value has changed before doing the work
                if (m_isSelected != value)
                {
                    // Update the value of the internal variable to the new
                    // value
                    m_isSelected = value;

                    // Only when the object is physical will it need to rebuild
                    // the physical object, this will save some computation
                    // time by not rebuilding during the duplicate sending of
                    // selected by OpenSim
                    // NOTE: Only physical objects need to be rebuilt because
                    // a static object would remain frozen during modification
                    // and would not need to unfreeze when the user is finished
                    if (m_isPhysical)
                    {
                        // Check to see if this objct is part of a linkset
                        if (m_linkParent == null)
                        {
                            // Remove the actor from the scene, so that it
                            // can be properly re-built
                            m_pxScene.PhysX.RemoveActor(LocalID);
                        }
                        else
                        {
                            // Since this object is part of a linkset, remove
                            // its shape from the linkset parent instead, since
                            // it does not have an actor in the PhysX scene
                            m_pxScene.PhysX.RemoveShape(m_linkParent.LocalID,
                                m_shapeID);
                        }

                        // Indicate that this object is no longer built in the
                        // PhysX scene
                        m_isObjectBuilt = false;

                        // Check to see if this object is part of a linkset;
                        // if it is, have the parent do the rebuild, which
                        // will cause the entire linkset to build properly
                        if (m_linkParent == null)
                            m_pxScene.AddTaintedObject(this);
                        else
                            m_pxScene.AddTaintedObject(m_linkParent);
                    }
                }
            }
        }

        
        /// <summary>
        /// This is called when a prim or character has failed to cross into a
        /// new region.
        /// </summary>
        public override void CrossingFailure()
        {
            // Increment the number of crossing failures that this object
            // has experienced
            CrossingFailures++;

            // Check to see if this object has exceeded the number of region
            // crossing failures
            if (CrossingFailures >
                m_pxScene.UserConfig.CrossingFailuresBeforeOutOfBounds)
            {
                // Indicate that this object is now out of bounds
                base.RaiseOutOfBounds(m_rawPosition);
            }
        }

        /// <summary>
        /// Add the collision to the PxPhysObject as a CollisionEventUpdate.
        /// </summary>
        public virtual bool Collide(uint collidingWith, PxPhysObject collidee,
            Vector3 contactPoint, Vector3 contactNormal, float penetrationDepth)
        {
            // Update the time at which a collision has occured
            LastCollisionTime = m_pxScene.m_simulationTime;

            // Check if this is a collision with the terrain, otherwise
            // this is a collision with a physical object
            if (collidingWith == m_pxScene.TerrainID)
            {
                GroundCollisionTime = m_pxScene.m_simulationTime;
            }
            else
            {
                ObjectCollisionTime = m_pxScene.m_simulationTime;
            }

            // Update the number of collisions for this object
            NumCollisions++;

            // First check to see if anything is subscribed to events
            // from this object
            if (SubscribedEvents())
            {
                // Add this collision to the collection of collisions to
                // be sent back to the simulator at the next update
                m_collisionCollection.AddCollider(
                    collidingWith, new ContactPoint(
                    contactPoint, contactNormal, penetrationDepth));
            }

            // Successfully handled collision
            return true;
        }


        /// <summary>
        /// This will link to the specified parent.
        /// </summary>
        public override void link(PhysicsActor obj)
        {
            PxPhysObject otherObj;

            // Attempt to convert the given object into PhysX plugin object
            m_linkParent = obj as PxPhysObject;

            // Check to see if the object was successfully converted
            if (m_linkParent != null)
            {
                // Add this object as child to the given object's linkset
                m_linkParent.AddToLinkset(this);

                // Remove the old static or dynamic actor before creating
                // the new actor
                m_pxScene.PhysX.RemoveActor(LocalID);

                // Indicate that this object is no longer built in the
                // PhysX scene
                m_isObjectBuilt = false;

                // Schedule a delegate that will compute the linkset right
                // before the next simulation step occurs; this is done
                // account for inconsistencies in relative position caused
                // by certain OpenSim link cases
                m_pxScene.BeforeStep += ComputeLinkset;
            }

            // Bullet did nothing so we are also going to do nothing for now
            return;
        }

        public void ComputeLinkset(float timestep)
        {
            // Check to see if this object is linked to a parent
            if (m_linkParent != null)
            {
                // Calculate the position relative to the
                // parent that preserves the overall global
                // position of this object
                m_linkPos = m_rawPosition -
                    m_linkParent.Position;
                m_linkPos *= Quaternion.Inverse(
                    m_linkParent.Orientation);

                // Calculate the relative orientation in a
                // similar manner
                m_linkOrient = Quaternion.Inverse(
                    m_linkParent.Orientation) * m_orientation;
             }

             // Remove this method from the before step event, so that it
             // doesn't keep firing on each simulate
             m_pxScene.BeforeStep -= ComputeLinkset;
        }


        /// <summary>
        /// This will remove the physical object from the linkset.
        /// </summary>
        public override void delink()
        {
            // Check to see if the object has a valid parent from which to
            // delink
            if (m_linkParent != null)
            {
                // Remove the shape from the linkset parent
                m_pxScene.PhysX.RemoveShape(m_linkParent.LocalID, m_shapeID);

                // Indicate that the shape is no longer built for this object
                m_isObjectBuilt = false;

                // Recalculate the position based on the current position
                // and orientation of the parent object
                m_rawPosition = (m_linkPos * m_linkParent.Orientation) +
                    m_linkParent.Position;
                m_orientation = m_linkOrient * m_linkParent.Orientation;

                // Indicate that this object no longer has a parent
                m_linkParent = null;

                // Rebuild the object without the linkset
                m_pxScene.AddTaintedObject(this);
            }

            return;
        }


        /// <summary>
        /// Adds the given physics object to the list of children objects that
        /// are considered to be linked under this object.
        /// </summary>
        /// <param name="childObj">The child object to be added</param>
        public void AddToLinkset(PxPhysObject childObj)
        {
            // If the given physics object is valid, add it to the list
            // children objects that are linked under this object
            if (childObj != null)
            {
                // Add the child in a thread-safe manner
                lock (m_childLock)
                {
                    m_childObjects.Add(childObj);
                }
            }
        }


        /// <summary>
        /// Removes the given physics object from the list of children objects.
        /// </summary>
        /// <param name="childObj">The child object to be removed</param>
        public void RemoveFromLinkset(PxPhysObject childObj)
        {
            // Remove the given child from the list of children objects that
            // are linked under this object (in a thread-safe manner)
            lock (m_childLock)
            {
                m_childObjects.Remove(childObj);
            }
        }

        /// <summary>
        /// Removes this object's shape from the linkset parent, if it is part
        /// of a linkset
        /// </summary>
        public void RemoveShapeFromLinkset()
        {
            // Check to see if this object is part of a linkset
            if (m_linkParent != null)
            {
                // Remove this object's shape from the linkset parent
                m_pxScene.PhysX.RemoveShape(m_linkParent.LocalID, m_shapeID);
            }
        }

        /// <summary>
        /// Stops motion of the object on given axes, both linear and angular.
        /// <param name="linear">A vector indicating around which axes linear
        /// motion should be locked. 1.0 indicates no locking; less than 1.0
        /// indicates locking</param>
        /// <param name="angular">A vector indicating around which axes angular
        /// motion should be locked. 1.0 indicates no locking; less than 1.0
        /// indicates locking</param>
        /// </summary>
        public void LockMotion(Vector3 linear, Vector3 angular)
        {
            Vector3 angularLowerLimits, linearLowerLimits;

            // Check to see if this actor is a prim; if not exit out, as this
            // operation is only available for prims
            if (m_physicsActorType != (int) ActorType.PRIM)
            {
                return;
            }

            // Check to see if the linear motion arround the axes should be
            // locked; start off with all 3 axes unlocked
            linearLowerLimits = Vector3.One;
            if (linear.X < 1.0f)
            {
                // Lock linear around x-axis
                linearLowerLimits.X = 0.0f;
            }
            if (linear.Y < 1.0f)
            {
                // Lock linear around y-axis
                linearLowerLimits.Y = 0.0f;
            }
            if (linear.Z < 1.0f)
            {
                // Lock linear around z-axis
                linearLowerLimits.Z = 0.0f;
            }

            // Check to see if the angular motion arround the x-axis should be
            // locked; start off with all 3 axes unlocked
            angularLowerLimits = Vector3.One;
            if (angular.X < 1.0f)
            {
                // Lock angular around x-axis
                angularLowerLimits.X = 0.0f;
            }
            if (angular.Y < 1.0f)
            {
                // Lock angular around y-axis
                angularLowerLimits.Y = 0.0f;
            }
            if (angular.Z < 1.0f)
            {
                // Lock angular around z-axis
                angularLowerLimits.Z = 0.0f;
            }

            // Create a joint that will restrict angular motion around the
            // axes specified above
            m_pxScene.PhysX.AddGlobalFrameJoint(m_fallJointID, LocalID,
                Vector3.Zero, Quaternion.Identity, linearLowerLimits,
                Vector3.Zero, angularLowerLimits, Vector3.Zero);

        }

        /// <summary>
        /// Stops angular motion of the object on given axes.
        /// <param name="axis">A vector indicating around which axes angular
        /// motion should be locked. 1.0 indicates no locking; less than 1.0
        /// indicates locking</param>
        /// </summary>
        public override void LockAngularMotion(Vector3 axis)
        {
            Vector3 angularLowerLimits;

            // Check to see if this object is a prim; if not exit out, as this
            // operation is only available for prims
            if (m_physicsActorType != (int) ActorType.PRIM)
            {
                return;
            }

            // Check to see if the angular motion around the x-axis should
            // be locked; start of with all 3 axes unlocked
            angularLowerLimits = Vector3.One;
            if (axis.X < 1.0f)
            {
               // Lock angular around x-axis
               angularLowerLimits.X = 0.0f;
            }
            if (axis.Y < 1.0f)
            {
               // Lock angular around y-axis
               angularLowerLimits.Y = 0.0f;
            }
            if (axis.Z < 1.0f)
            {
               // Lock angular around z-axis
               angularLowerLimits.Z = 0.0f;
            }

            // Create a joint that will restrict angular motion around the
            // axes specified above
            m_pxScene.PhysX.AddGlobalFrameJoint(m_fallJointID, LocalID,
                Vector3.Zero, Quaternion.Identity, Vector3.One,
                Vector3.Zero, angularLowerLimits, Vector3.Zero);
        }


        public override void SetMaterial(int material)
        {
            PxMaterialAttributes attributes;

            // Fetch the attributes for the given material from the library
            attributes = m_pxScene.MaterialLibrary.GetAttributes(
                (PxMaterialLibrary.Material )material);

            // Update this object's material properties
            Density = attributes.m_density;
            Friction = attributes.m_friction;
            Restitution = attributes.m_restitution;
        }


        /// <summary>
        /// The density of the physical object used to calculate the mass of
        /// the object inside of PhysX.
        /// </summary>
        public override float Density
        {
            get
            {
                // Return the density value of the physical object
                return m_density;
            }
            set
            {
                uint   actorID;

                // Check to see if the density is valid; if not, exit
                if (value <= 0.0f)
                    return;

                // Update the density value for the object
                m_density = value;

                // Check to see if this object is linked to another object;
                // if so, use its actor ID, since this object's shape is
                // attached to it
                if (m_linkParent != null)
                    actorID = m_linkParent.LocalID;
                else
                    actorID = LocalID;

                // Update the density of this object's shape
                m_pxScene.PhysX.UpdateShapeDensity(actorID, m_shapeID,
                    m_density);
            }
        }


        public override float GravModifier
        {
            get
            {
                // Return the grav modifier from the inherited class
                return base.GravModifier;
            }
            set
            {
                // Record the gravity modifier value
                base.GravModifier = value;

                // Enable or disable gravity based on the given modifier
                if (Math.Abs(base.GravModifier) <= TOLERANCE)
                {
                    m_pxScene.PhysX.EnableGravity(LocalID, false);
                }
                else
                {
                    m_pxScene.PhysX.EnableGravity(LocalID, true);
                }
            }
        }


        /// <summary>
        /// The friction of the physical object.
        /// </summary>
        public override float Friction
        {
            get
            {
                // Return the friction from the inherited class
                return base.Friction;
            }
            set
            {
                // Only update the static friction if the value has changed
                // and if the updated value is not negative
                if (base.Friction != value && value >= 0)
                {
                    // Notify the super class of the updated friction value
                    base.Friction = value;

                    // Check to see if the shape for this object has been built
                    if (m_isObjectBuilt)
                    {
                        // Check to see if this object is part of a linkset
                        if (m_linkParent != null)
                        {
                            // The shape for this object is attached to the
                            // parent of the linkset, so update the shape there
                            m_pxScene.PhysX.UpdateMaterialProperties(
                                m_linkParent.LocalID, m_shapeID, base.Friction,
                                base.Friction, base.Restitution);
                        }
                        else
                        {
                            // Update the actor with the updated friction value
                            m_pxScene.PhysX.UpdateMaterialProperties(LocalID,
                                m_shapeID, base.Friction, base.Friction,
                                base.Restitution);
                        }
                    }
                }
            }
        }


        /// <summary>
        /// The restitution of the physical object.
        /// </summary>
        public override float Restitution
        {
            get
            {
                // Return the restitution from the inherited class
                return base.Restitution;
            }
            set
            {
                // Only update the restitution if the value has changed, and if
                // the updated value is not negative
                if (base.Restitution != value && value >= 0)
                {
                    // Notify the super class of the updated restitution value
                    base.Restitution = value;

                    // Check to see if the shape for this object has been built
                    if (m_isObjectBuilt)
                    {
                        // Check to see if this object is part of a linkset
                        if (m_linkParent != null)
                        {
                            // The shape for this object is attached to the
                            // parent of the linkset, so update the shape there
                            m_pxScene.PhysX.UpdateMaterialProperties(
                                m_linkParent.LocalID, m_shapeID, base.Friction,
                                base.Friction, base.Restitution);
                        }
                        else
                        {
                            // Update the actor with the updated friction value
                            m_pxScene.PhysX.UpdateMaterialProperties(LocalID,
                                m_shapeID, base.Friction, base.Friction,
                                base.Restitution);
                        }
                    }
                }
            }
        }


        /// <summary>
        /// Get an aproximate position of the physical object. 
        /// </summary>
        public override Vector3 Position
        {
            get
            {
                // Don't refetch the position from the physics scene in order
                // to speed up the runtime of the program, this estimate will
                // be close enough for most
                return m_rawPosition;
            }
            set
            {
                // Set the position to the new value 
                m_rawPosition = value;

                // Check to see if the object has been constructed in the
                // PhysX scene and that it is not part of a linkset;
                // If the object is part of a linkset, it does not have its
                // own actor in the PhysX scene
                if (m_isObjectBuilt && m_linkParent == null)
                {
                    // Update the position in the PhysX scene
                    m_pxScene.PhysX.SetPosition(LocalID, m_rawPosition);
                }
            }
        }


        /// <summary>
        /// The current mass of the physical object.
        /// </summary>
        public override float Mass
        {
            get
            {
                // Return the current mass of the physical object
                return m_pxScene.PhysX.GetActorMass(LocalID);
            }
        }


        /// <summary>
        /// The current force acting on the physical object.
        /// </summary>
        public override Vector3 Force
        {
            get
            {
                // Return the current force acting on this physical object
                return m_rawForce;
            }
            set
            {
                // Set the current force acting on this physical object
                m_rawForce = value;

                // PhysX is able to handle the normal density value of objects,
                // however OpenSim sends its force values assuming that the
                // density value has been decreased by a 0.1 factor, so this
                // variable corrects the issue
                float forceCorrection = 10.0f;

                // Apply the force to the object in the PhysX scene
                m_pxScene.PhysX.AddForce(LocalID, m_rawForce * forceCorrection);
            }
        }


        /// <summary>
        /// This is an enumeration that will determine the type of motion a
        /// vehicle should follow. This will be set to none for the character
        /// and most other physical objects.
        /// </summary>
        public override int VehicleType
        {
            get
            {
                int ret = (int) Vehicle.TYPE_NONE;

                // Get the vehicle actor, and don't create one if there no type
                PxActorVehicle vehicleActor = GetVehicleActor(false);

                // If we have a vehicle actor, then return its type
                if (vehicleActor != null)
                {
                    ret = (int) vehicleActor.Type;
                }

                return ret;
            }
            set
            {
                PxActorVehicle vehicleActor;
                Vehicle type = (Vehicle) value;
                
                if (type == Vehicle.TYPE_NONE)
                {
                    // Vehicle is of type 'none' so get rid of the actor if it exists
                    vehicleActor = GetVehicleActor(false);
                    if (vehicleActor != null)
                    {
                        PhysicalActors.RemoveAndRelease(vehicleActor.ActorName);
                    }
                }
                else
                {
                    // Vehicle is not of type 'none' so create an actor
                    vehicleActor = GetVehicleActor(true);
                    if (vehicleActor != null)
                    {
                        vehicleActor.ProcessTypeChange(type);
                    }    
                }
            }
        }


        /// <summary>
        /// Uses the parameters given to update a float parameter associated
        /// with the vehicle type of this physical object.
        /// </summary>
        /// <param name="param">The enum value of the parameter that the
        /// calling location wished to change to the given parameter</param>
        /// <param name="value">The floating point value that the parameter
        /// given should be updated to</param>
        public override void VehicleFloatParam(int param, float value)
        {
            // Get the vehicle actor and force it to be created by
            // passing true
            PxActorVehicle vehicleActor = GetVehicleActor(true);

            // If we have an actual vehicle actor, tell it to process updating
            // the vector parameter
            if (vehicleActor != null)
            {
                vehicleActor.ProcessFloatVehicleParam((Vehicle) param, value);
            }
        }


        /// <summary>
        /// Uses the parameters given to update a vector parameter associated
        /// with the vehicle type of this physical object.
        /// </summary>
        /// <param name="param">The enum value of the parameter that the
        /// calling location wished to change to the given parameter</param>
        /// <param name="value">The Vector3 value that the parameter
        /// given should be updated to</param>
        public override void VehicleVectorParam(int param, Vector3 value)
        {
            // Get the vehicle actor and force it to be created by
            // passing true
            PxActorVehicle vehicleActor = GetVehicleActor(true);

            // If we have an actual vehicle actor, tell it to process updating
            // the vector parameter
            if (vehicleActor != null)
            {
                vehicleActor.ProcessVectorVehicleParam((Vehicle) param, value);
            }
        }


        /// <summary>
        /// Uses the parameters given to update a quaternion parameter 
        /// associated with the vehicle type of this physical object.
        /// </summary>
        /// <param name="param">The enum value of the parameter that the
        /// calling location wished to change to the given parameter</param>
        /// <param name="value">The quaternion value that the parameter
        /// given should be updated to</param>
        public override void VehicleRotationParam(int param, Quaternion value)
        {
            // Get the vehicle actor and force it to be created by
            // passing true
            PxActorVehicle vehicleActor = GetVehicleActor(true);

            // If we have an actual vehicle actor, tell it to process updating
            // the rotational parameter
            if (vehicleActor != null)
            {
                vehicleActor.ProcessRotationVehicleParam((Vehicle) param, value);
            }
        }

        /// <summary>
        /// Find and return a handle to the current vehicle actor.
        /// </summary>
        /// <param name="createIfNone"> Create the actor if it was not found
        /// within the physical actors</param>
        public PxActorVehicle GetVehicleActor(bool createIfNone)
        {
            PxActorVehicle ret = null;
            PxActor actor;

            // If the physical actors dictionary contains this physics
            // object vehicle actor, then set ret as the PxActorVehicle
            if (PhysicalActors.TryGetActor(VehicleActorName, out actor))
            {
                ret = actor as PxActorVehicle;
            }
            else
            {
                // If there is no created actor, then create the
                // new vehicle and add it to the physical actors
                if (createIfNone)
                {
                    ret = new PxActorVehicle(m_pxScene, this, VehicleActorName);
                    PhysicalActors.Add(ret.ActorName, ret);
                }
            }

            return ret;
        }


        /// <summary>
        /// Uses the parameters given to update a boolean parameter associated
        /// with the vehicle type of this physical object.
        /// </summary>
        /// <param name="param">The enum value of the parameter that the
        /// calling location wished to change to the given parameter</param>
        /// <param name="value">To remove or add the flag specified</param>
        public override void VehicleFlags(int param, bool value)
        {
            // Get the vehicle actor and force it to be created by
            // passing true
            PxActorVehicle vehicleActor = GetVehicleActor(true);

            // If we have an actual vehicle actor, tell it to prccess updating
            // the rotational parameter
            if (vehicleActor != null)
            {
                vehicleActor.ProcessVehicleFlags(param, value);
            }
        }


        /// <summary>
        /// Allows the detection of collisions with inherently non-physical
        /// prims. 
        /// </summary>
        /// <param name="param"></param>
        public override void SetVolumeDetect(int param)
        {
            // Volume detect only works with prims, so return if this object
            // is not a prim
            if (m_physicsActorType != (int)ActorType.PRIM)
                return;

            // Check to see if volume detect should be turned on
            if (param != 0)
            {
                // Turn the object into a physical object, so that it can
                // collide
                IsPhysical = true;
            }
        }


        /// <summary>
        /// Finds the geometric center of a physical object.
        /// </summary>
        /// <returns>The geometric center of the object</returns>
        public override Vector3 GeometricCenter
        {
            get
            {
                Vector3 sum;

                // Check to see if this object has any children objects that are
                // linked to it in a thread-safe manner
                lock (m_childLock)
                {
                    if (m_childObjects.Count == 0)
                    {
                        // There are no children, so return the position of the
                        // object
                        return m_rawPosition;
                    }
                    else
                    {
                        // Since there are children objects, average the
                        // positions of this object and the children
                        sum = m_rawPosition;
                        foreach (PxPhysObject currChild in m_childObjects)
                        {
                            sum += currChild.Position;
                        }
                        sum /= (m_childObjects.Count + 1);
 
                        // Return the average position
                        return sum;
                    }
                }
            }
        }


        /// <summary>
        /// Finds the center of mass of a physical object.
        /// </summary>
        public override Vector3 CenterOfMass
        {
            get
            {
                Vector3 sum;
                float totalMass;

                // Check to see if this object is the parent object of a
                // linkset in a thread-safe manner
                lock (m_childLock)
                {
                if (m_childObjects.Count > 0)
                {
                        // Calculate the weighted sum of the masses of each
                        // object in the linkset; also calculate the total
                        // mass of the linkset
                        sum = m_rawPosition * Mass;
                        totalMass = Mass;
                        foreach(PxPhysObject currObj in m_childObjects)
                        {
                            sum += currObj.Position * currObj.Mass;
                            totalMass += currObj.Mass;
                        }
 
                        // Average the weighted mass over the total mass to
                        // find the center of mass; make sure not to divide by 0
                        if (totalMass > 0.0f)
                            sum /= totalMass;
 
                        // Return the resulting position
                        return sum;
                    }
                    else
                    {
                        // This is not a linkset, so return the regular position
                        return m_rawPosition;
                    }
                }
            }
        }


        /// <summary>
        /// The current velocity of the physical object.
        /// </summary>
        public override Vector3 Velocity
        {
            get
            {
                // Return the current velocity of the physical object.
                return m_velocity;
            }
            set
            {
                // Set the local velocity value to the new velocity
                m_velocity = value;
                m_targetVelocity = value;

                // Check if this avatar is currently running, a prim will never
                // be set to running
                if (m_isAlwaysRun && !m_isFlying)
                {
                    // Take the current velocity and multiply by the run factor
                    // provided by the user inside of the config file
                    m_targetVelocity *= new Vector3(
                        m_pxScene.UserConfig.RunFactor,
                        m_pxScene.UserConfig.RunFactor, 1.0f);
                }

                // Set the PhysX velocity value of the physical object to
                // the new value
                m_pxScene.PhysX.SetLinearVelocity(LocalID, m_velocity);
            }
        }


        /// <summary>
        /// The current torque being applied to the physical object.
        /// </summary>
        public override Vector3 Torque
        {
            get
            {
                // Return the current toque being applied to the physical 
                // object
                return m_torque;
            }
            set
            {
                // Update the torque applied to this object
                m_torque = value;

                // Apply the torque to the object in the PhysX scene
                m_pxScene.PhysX.AddTorque(LocalID, m_torque);
            }
        }


        /// <summary>
        /// Evaluates the collision score for the object based the frequency
        /// of collisions. This method allows for fewer computations of score.
        /// </summary>
        public void EvaluateCollisionScore()
        {
            // Check to see if collision has occurred yet for this object
            if (LastCollisionTime == PxScene.InvalidTime)
            {
                // No collision has occurred for this object yet, so the score
                // is zero
                CollisionScore = 0;
            }

            // Determine the frequency of collisions while avoiding
            // division by zero
            if (m_pxScene.m_simulationTime - LastCollisionTime == 0)
            {
                CollisionScore = NumCollisions;
            }
            else
            {
                CollisionScore = NumCollisions /
                    (m_pxScene.m_simulationTime - LastCollisionTime);
            }
        }


        /// <summary>
        /// Returns a score which indicates the likelihood of collisions 
        /// with this object based on the frequency of past collisions.
        /// </summary>
        public override float CollisionScore { get; set; }


        /// <summary>
        /// Accessor methods for the acceleration of the physical object.
        /// </summary>
        public override Vector3 Acceleration
        {
            get
            {
                // Return the current acceleration of the physical object
                return m_acceleration;
            }
            set
            {
                // Save the new acceleration value to the local acceleration
                // variable
                m_acceleration = value;
            }
        }


        /// <summary>
        /// Accesor methods for the orientation of the physical object.
        /// </summary>
        public override Quaternion Orientation
        {
            get
            {
                // Return the current orientation of the physical object
                return m_orientation;
            }
            set
            {
                // If the new quaternion doesn't present a change, exit 
                if (value == m_orientation)
                    return;

                // Set the local orientation variable to the new value
                m_orientation = value;

                // Check to see if this object is not part of a linkset;
                // the object does not have a corresponding actor in the
                // PhysX scene, if it is part of a linkset
                if (m_linkParent == null)
                {
                    // Update the PhysX orientation value for this physical
                    // object
                    m_pxScene.PhysX.SetRotation(LocalID, m_orientation);
                }
            }
        }


        /// <summary>
        /// Accessor methods for the physics actor type of the physical object.
        /// </summary>
        public override int PhysicsActorType
        {
            get
            {
                // Return the physics actor type of this physical object
                return m_physicsActorType;
            }
            set
            {
                // Set the physics actor type to the new value
                m_physicsActorType = value;
            }
        }


        /// <summary>
        /// Accessor methods for whether the physical object acts like a
        /// physical object.
        /// </summary>
        public override bool IsPhysical
        {
            get
            {
                // Return whether this object acts like a physical object
                return m_isPhysical;
            }
            set
            {
                // Check whether the value has changed to save computer
                // cycles
                if (m_isPhysical != value)
                {
                    // Set the local value to the new value
                    m_isPhysical = value;

                    // Check to see if this object is part of a linkset;
                    // if it is part of a linkset, it does not have an actor
                    // in the PhysX scene
                    if (m_linkParent == null)
                    {
                        // Remove the old static or dynamic actor before
                        // creating the new actor
                        m_pxScene.PhysX.RemoveActor(LocalID);
                    }
                    else
                    {
                        // Since this object is part of a linkset, remove its
                        // shape from the linkset parent
                        m_pxScene.PhysX.RemoveShape(m_linkParent.LocalID,
                            m_shapeID);
                    }


                    // Indicate that this object is no longer built in the
                    // PhysX scene
                    m_isObjectBuilt = false;

                    // Add the new physical object to the PhysX scene
                    if (m_linkParent == null)
                        m_pxScene.AddTaintedObject(this);
                    else
                        m_pxScene.AddTaintedObject(m_linkParent);
                }
            }
        }


        /// <summary>
        /// Determines if this object should be physically moving through the
        /// scene.
        /// </summary>
        public bool IsPhysicallyActive
        {
            get
            {
                // Return true if the object is not currently being held by a
                // user and is currently a physical object
                return !m_isSelected && IsPhysical;
            }
        }


        /// <summary>
        /// Accessor methods for the is flying flag.
        /// </summary>
        public override bool Flying
        {
            get
            {
                // Return whether the physical object is currently flying
                return m_isFlying;
            }
            set
            {
                // Update the flag for whether the physical object is currently
                // flying
                m_isFlying = value;

                // Change how gravity works on this object based on
                // on its buoyancy
                ComputeBuoyancyFromFlying();

                // TODO: Update the gravity value instead of directly
                // enabling/disabling gravity

                // Use the actor's given buoyancy to determine its local
                // gravity; send data to PhysX
                if (Math.Abs(m_buoyancy - 1.0f) <= TOLERANCE)
                {
                    m_pxScene.PhysX.EnableGravity(LocalID, false);
                }
                else
                {
                    m_pxScene.PhysX.EnableGravity(LocalID, true);
                }
            }
        }
     

        /// <summary>
        /// Accessor methods for the is always running flag.
        /// </summary>
        public override bool SetAlwaysRun
        {
            get
            {
                // Return whether the physical object is set to always run
                return m_isAlwaysRun;
            }
            set
            {
                // Update the flag for whether the physical object is set to 
                // always run
                m_isAlwaysRun = value;
            }
        }


        /// <summary>
        /// Accessor methods for the throttle updates flag.
        /// </summary>
        public override bool ThrottleUpdates
        {
            get
            {
                // Return whether the physical object is currently throttling
                // it's updates
                return m_throttleUpdates;
            }
            set
            {
                // Update the flag for whether the physical object is currently
                // throttling it's updates
                m_throttleUpdates = value;
            }
        }

        /// <summary>
        /// Determines if the physical object is currently colliding with
        /// anything.
        /// </summary>
        public override bool IsColliding
        {
            get
            {
                // The object is colliding if the last collision time
                // matches the last simulated time
                return (LastCollisionTime == m_pxScene.m_simulationTime);
            }
            set
            {
                // Set the last collision time to the current simulated time
                // or invalid time based on the given value
                if (value)
                {
                    LastCollisionTime = m_pxScene.m_simulationTime;
                }
                else
                {
                    LastCollisionTime = PxScene.InvalidTime;
                }
            }
        }


        /// <summary>
        /// Determines if the physical object is currently colliding with the
        /// ground.
        /// </summary>
        public override bool CollidingGround
        {
            get
            {
                // The object is colliding with the terrain if the last
                // collision time matches the last simulated time
                return (GroundCollisionTime == m_pxScene.m_simulationTime);
            }
            set
            {
                // Set the ground collision time to the current simulated time
                // or invalid time based on the given value
                if (value)
                {
                    GroundCollisionTime = m_pxScene.m_simulationTime;
                }
                else
                {
                    GroundCollisionTime = PxScene.InvalidTime;
                }
            }
        }


        /// <summary>
        /// Determines if the physical object is currently colliding with
        /// another object.
        /// </summary>
        public override bool CollidingObj
        {
            get
            {
                // This physical object is colliding with another object if
                // the last collision time matches the last simulated time
                return (ObjectCollisionTime == m_pxScene.m_simulationTime);
            }
            set
            {
                // Set the object collision time to the current simulation time
                // or an invalid time based on the given value
                if (value)
                {
                    ObjectCollisionTime = m_pxScene.m_simulationTime;
                }
                else
                {
                    ObjectCollisionTime = PxScene.InvalidTime;
                }
            }
        }


        /// <summary>
        /// Use this method to change if the object should float on the water.
        /// </summary>
        public override bool FloatOnWater
        {
            set
            {
                // Check if there is a need to update the float on water flag
                if (m_floatOnWater != value)
                {
                    // Remember the new value, so we can prevent unecessary
                    // updates
                    m_floatOnWater = value;

                    // TODO: Update the collision list of this physical object
                    // to include the water.
                }
            }
        }


        /// <summary>
        /// Accessors for the rotational velocity of the physical object.
        /// </summary>
        public override Vector3 RotationalVelocity
        {
            get
            {
                // Return the current rotational velocity of the physical
                // object
                return m_rotationalVelocity;
            }
            set
            {
                // Update the internal rotational velocity value to match the
                // given value
                m_rotationalVelocity = value;

                // Send the new rotational velocity to PhysX
                m_pxScene.PhysX.SetAngularVelocity(LocalID, 
                    m_rotationalVelocity);
            }
        }


        /// <summary>
        /// This method is not supported in the current version.
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
        /// Accessor methods for buoyancy which changes the effects of gravity
        /// on the physical object.
        /// </summary>
        public override float Buoyancy
        {
            get
            {
                // Return the current buoyancy value of the physical object
                return m_buoyancy;
            }
            set
            {
                // Update the buoyancy local value with the new value
                m_buoyancy = value;

                // TODO: Send the value to PhysX. For bullet characters apply a
                // change on their gravity, while prims change their mass.
            }
        }


        public override Vector3 PIDTarget
        {
            set
            {
                // TODO: Current design returns having done nothing, while
                // bullet sets MoveToTargetTarget
            }
        }


        public override bool PIDActive
        {
            get
            {
                // TODO: Current design returns false while Bullet returns
                // MoveToTargetActive
                return false;
            }
            set
            {
                // TODO: Current design sets value to PIDActive while bullet
                // sets MoveToTargetActive
            }
        }


        public override float PIDTau
        {
            set
            {
                // TODO: Current design returns having done nothing, while
                // bullet sets MoveToTargetTau
            }
        }


        public override bool PIDHoverActive
        {
            set
            {
                // TODO: Current design returns having done nothing, while
                // bullet sets HoverActive
            }
        }


        public override float PIDHoverHeight
        {
            set
            {
                // TODO: Current design returns having done nothing, while
                // bullet sets HoverHeight
            }
        }


        public override PIDHoverType PIDHoverType
        {
            set
            {
                // TODO: Current design returns having done nothing, while
                // bullet sets HoverType
            }
        }


        public override float PIDHoverTau
        {
            set
            {
                // TODO: Current design returns having done nothing, while
                // bullet sets HoverTau
            }
        }


        public override Quaternion APIDTarget
        {
            set
            {
                // TODO: Current design returns having done nothing
            }
        }


        public override bool APIDActive
        {
            set
            {
                // TODO: Current design returns having done nothing
            }
        }


        public override float APIDStrength
        {
            set
            {
                // TODO: Current design returns having done nothing
            }
        }


        public override float APIDDamping
        {
            set
            {
                // TODO: Current design returns having done nothing
            }
        }
		
        /// <summary>
        /// Applied the vector force to the PxPhysObject.
        /// </summary>
        /// <param name='force'>Force vector for the force to be applied.</param>
        /// <param name='pushForce'>If it is a push force.</param>
        public override void AddForce(Vector3 force, bool pushForce)
        {
            // PhysX is able to handle the normal density value of objects,
            // however OpenSim sends its force values assuming that the
            // density value has been decreased by a 0.1 factor, so this
            // variable corrects the issue
            float forceCorrection = 10.0f;

            // Tell PhysX to add a force on the current PhysX object
            m_pxScene.PhysX.AddForce(LocalID, force * forceCorrection);
        }
		

        public override void AddAngularForce(Vector3 force, bool pushForce)
        {
            // TODO: Determine how to add angular force to a physics object
        }


        /// <summary>
        /// Currently this method is here as a place holder and does nothing.
        /// </summary>
        public override void SetMomentum(Vector3 momentum)
        {
            // Bullet choose to ignore this so for now we will do the same
        }


        /// <summary>
        /// Method to allow OpenSim to ask to be sent events from this physical
        /// object.
        /// </summary>
        /// <param name="ms">The number of milliseconds between events sent
        /// from the physical object to OpenSim</param>
        public override void SubscribeEvents(int ms)
        {
            // Save the amount of time between events
            m_subscribedEventsTime = ms;

            // Check that a valid subscribed events time was given
            if (ms > 0)
            {
                // Make the next collision time before the current time to
                // make certain the first collision happens
                m_nextCollisionOkTime = Util.EnvironmentTickCountSubtract(
                    m_subscribedEventsTime);
            }
            else
            {
                // Don't send events to OpenSim
                UnSubscribeEvents();
            }
        }


        public override void UnSubscribeEvents()
        {
            // No time required for events since no one is subscribed
            m_subscribedEventsTime = 0;
        }


        public override bool SubscribedEvents()
        {
            // If the event time is greater than zero then the simulator
            // is subscribed for events from this object
            return (m_subscribedEventsTime > 0);
        }


        #endregion // PhysicsActor Overrides
        

        /// <summary>
        /// Release resources used by this physical object.
        /// </summary>
        public void Destroy()
        {
            m_physicalActors.Enable(false);
            m_physicalActors.Dispose();

            // Remove this object's joint
            m_pxScene.PhysX.RemoveJoint(m_fallJointID);

            // Check to see if this object is part of a linkset
            if (m_linkParent != null)
            {
                m_pxScene.PhysX.RemoveShape(m_linkParent.LocalID, m_shapeID);
            }
            else
            {
                m_pxScene.PhysX.RemoveActor(LocalID);
            }
        }

        /// <summary>
        /// Update the various physical properties of this object to
        /// reflect the same updates from the physics engine.
        /// </summary>
        /// <param name="entityProperties">Data structure containing
        /// the physical parameters of this object</param>
        public void UpdateProperties(EntityProperties entityProperties)
        {
            // Update the position
            m_rawPosition.X = entityProperties.PositionX;
            m_rawPosition.Y = entityProperties.PositionY;
            m_rawPosition.Z = entityProperties.PositionZ;

            // Update the orientation
            m_orientation.X = entityProperties.RotationX;
            m_orientation.Y = entityProperties.RotationY;
            m_orientation.Z = entityProperties.RotationZ;
            m_orientation.W = entityProperties.RotationW;

            // Update the velocity
            m_velocity.X = entityProperties.VelocityX;
            m_velocity.Y = entityProperties.VelocityY;
            m_velocity.Z = entityProperties.VelocityZ;

            // Update the angular velocity
            m_rotationalVelocity.X = entityProperties.AngularVelocityX;
            m_rotationalVelocity.Y = entityProperties.AngularVelocityY;
            m_rotationalVelocity.Z = entityProperties.AngularVelocityZ;

            // Refresh the physical actors on this physics object
            m_physicalActors.Refresh();
        }

        #region Helper Methods

        /// <summary>
        /// The delegate that models the function that will be used
        /// to create the PxActor on the call of PxPhysObject.EnableActor().
        /// </summary>
        public delegate PxActor CreateActor();

        /// <summary>
        /// Enable a physics actor on the physics object by way
        /// of using a delegate as a callback to initialize the
        /// physics actor to be added.
        /// </summary>
        /// <param name="enableActor"> If the physics actor is enabled. </param>
        /// <param name="actorName"> The physical actor name. </param>
        /// <param name="creator"> The delegate to create the PxActor. </param>
        public void EnableActor(bool enableActor, string actorName, CreateActor creator)
        {
            PxActor theActor;

            lock(PhysicalActors)
            {
                // If getting the physical actor out of the collection
                // then we set it's enabled attribute as given, otherwise
                // we will call upon the delegate function to create the
                // PxActor and add it to the actor collection
                if(PhysicalActors.TryGetActor(actorName, out theActor))
                {
                    theActor.Enabled = enableActor;
                }
                else
                {
                    // Create and enable the actor from the given values
                    // and the delegate callback
                    theActor = creator();
                    theActor.Enabled = enableActor;

                    // Add the created physical actor to the collection
                    PhysicalActors.Add(actorName, theActor);
                }

                // Finally we go ahead an refresh the physical actors
                theActor.Refresh();
            }
        }

        /// <summary>
        /// Flying is done by removing the effects of gravity on the physical
        /// object. This method uses the current flying value of this physical
        /// object to determine the buoyancy value. Buoyancy will then affect
        /// the gravity value to cause a flying effect.
        /// </summary>
        private void ComputeBuoyancyFromFlying()
        {
            // Determine the value of buoyancy on the current physical object
            // based on whether the object is currently flying
            if (m_isFlying)
            {
                // Remove all gravity acting on this object by maxing out
                // buoyancy; this will be subtracted from gravity to remove
                // the effect
                m_buoyancy = 1.0f;
            }
            else
            {
                // Turn off all buoyancy effects so that it will not effect how
                // gravity acts on the physical object
                m_buoyancy = 0.0f;
            }
        }


        /// <summary>
        /// Use the Size and density of the physical object to calculate it's
        /// volume and mass. This is currently set for a character using the
        /// sphere shape.
        /// </summary>
        private void ComputeAvatarVolumeAndMass()
        {
            // This formula assumes that a capsule is being used and thus takes
            // the area of the capsule and multiplies by the height(size.Z)
            // then it adds the volume of the end caps
            m_avatarVolume = (float) (Math.PI * Size.X / 2.0f * Size.Y / 2.0f * 
                Size.Z + 1.33333333f * Math.PI * Size.X / 2.0f * 
                Math.Min(Size.X, Size.Y) / 2 * Size.Y / 2.0f);

            // TODO: This is commented out until Chandler has a chance to
            // commit his set mass fix
            // To get the mass take the volume of the shape and multiply by the
            // density
            //Mass = Density * m_avatarVolume;
        }
    

        /// <summary>
        /// Send the collected collisions to OpenSim.
        /// </summary>
        /// <returns>A boolean value that is true if all collisions were
        /// processed and false if an actual collision was passed over</returns>
        public bool SendCollisions()
        {
            bool ret;
            bool sendEndEvent;

            // Initialise the return value with true until proven that all
            // collisions were handled
            ret = true;

            // Check to see if there are currently no more collisions but
            // collisions were sent out during the last call; if so, then
            // send an empty collision to signify the end of the collisions
            sendEndEvent = false;
            if (m_collisionCollection.Count == 0 &&
                m_prevCollisionCollection.Count != 0)
            {
                // Signal end of collisions
                sendEndEvent = true;
            }

            // Check if an end event needs to be sent or if the minimum
            // interval event time has passed
            if (sendEndEvent ||
                m_pxScene.m_simulationTime >= m_nextCollisionOkTime)
            {
                // Calculate the next time the plugin should send collision
                // updates to OpenSim
                m_nextCollisionOkTime = m_pxScene.m_simulationTime + 
                    m_subscribedEventsTime;

                // If a collision is being processed on this timestep the
                // object should remain in the list of collisions so that
                // OpenSim receives an empty event to signal the end of
                // collisions for this object
                if (m_collisionCollection.Count == 0 &&
                    m_prevCollisionCollection.Count == 0)
                {
                    // There has been no collisions for two updates, so return
                    // false to remove this object from the collision list
                    ret = false;
                }

                // Send the collision data to OpenSim
                base.SendCollisionUpdate(m_collisionCollection);

                // Keep the collisions reported in this update for
                // future reference
                m_prevCollisionCollection = m_collisionCollection;

                // Create a new collision collection instance to prevent race
                // conditions inside of OpenSim
                m_collisionCollection = new CollisionEventUpdate();
            }

            return ret;
        }

        /// <summary>
        /// Creates the physical objects in the PhysX Wrapper by going through
        /// the PxAPIManager with the given values. This also determines the
        /// shape of the physical object when creating the physical object.
        /// </summary>
        public void BuildPhysicalShape()
        {
            int[] indices;
            Vector3[] vertices;
            float currLOD;
            RequestAssetDelegate assetDelegate;
            bool isPhysical = false;
            bool reportCollisions;

            // Check to see if this object is part of a linkset
            if (m_linkParent != null)
            {
               // Only allow this object to be rigid dynamic (physical) object
               // if it is attached to a physical objected
               if (m_linkParent.IsPhysical && !m_isSelected)
               {
                   isPhysical = true;
               }
            }
            else
            {
                // Only allow an object to be a rigid dynamic(physical) object
                // if it is currently a physical object and not selected by
                // the user for modification
                if (m_isPhysical && !m_isSelected)
                {
                    isPhysical = true;
                }
            }

            // Indicate that the shape has not yet been built
            m_isObjectBuilt = false;

            // Record whether collisions not involving avatars
            // should be reported
            reportCollisions =
                m_pxScene.UserConfig.ReportNonAvatarCollisions;
           
            lock (m_shapeLock)
            {
                // Check it this is an avatar or a prim 
                if (m_physicsActorType == (int) ActorType.AVATAR)
                {
                    // Compute volume and mass of the avatar
                    ComputeAvatarVolumeAndMass();

                    // Remove the joint that is holding the avatar up and 
                    // prevents the avatar from sliding across the ground
                    m_pxScene.PhysX.RemoveJoint(m_fallJointID);

                    // Create a capsule actor
                    m_pxScene.PhysX.CreateCharacterCapsule(LocalID, Name,
                        m_rawPosition, m_orientation, m_shapeID, base.Friction,
                        base.Friction, base.Restitution,
                        ComputeAvatarHalfHeight(), 
                        Math.Min(m_currentSize.X, m_currentSize.Y) / 2.0f, 
                        Density, true, true);

                    // Now that the physical object has been created add the 
                    // joint holding up the avatar back to the avatar
                    // physical object
                    m_pxScene.PhysX.AddGlobalFrameJoint(m_fallJointID, LocalID,
                        Vector3.Zero, Quaternion.Identity, Vector3.One,
                        Vector3.Zero, Vector3.Zero, Vector3.Zero);

                    // Update the flying state of the actor in the PhysX scene
                    Flying = m_isFlying;

                    // Mark that the object has been created to prevent the 
                    // creation of a mesh for an object that already exists
                    m_isObjectBuilt = true;
                }
                else
                {
                    // Check to see if the shape has no cuts, twists or hollows
                    if (m_opensimBaseShape.ProfileBegin == 0 &&
                        m_opensimBaseShape.ProfileEnd == 0 &&
                        m_opensimBaseShape.ProfileHollow == 0 &&
                        m_opensimBaseShape.PathTwist == 0 &&
                        m_opensimBaseShape.PathTwistBegin == 0 &&
                        m_opensimBaseShape.PathBegin == 0 &&
                        m_opensimBaseShape.PathEnd == 0 &&
                        m_opensimBaseShape.PathTaperX == 0 &&
                        m_opensimBaseShape.PathTaperY == 0 &&
                        m_opensimBaseShape.PathScaleX == 100 &&
                        m_opensimBaseShape.PathScaleY == 100 &&
                        m_opensimBaseShape.PathShearX == 0 &&
                        m_opensimBaseShape.PathShearY == 0)
                    {
                        // Find out which shape this is by comparing the given 
                        // profile and curve path to that of the known 
                        // primitive shapes
                        if ((m_opensimBaseShape.ProfileShape == 
                            ProfileShape.HalfCircle && 
                            m_opensimBaseShape.PathCurve == 
                            (byte)Extrusion.Curve1) && 
                            m_opensimBaseShape.Scale.X == 
                            m_opensimBaseShape.Scale.Y && 
                            m_opensimBaseShape.Scale.Y == 
                            m_opensimBaseShape.Scale.Z)
                        {
                            // Check to see if this object is linked to a parent
                            if (m_linkParent != null)
                            {
                                // Attach this object's sphere shape to the
                                // linkset parent
                                m_pxScene.PhysX.AttachSphere(
                                    m_linkParent.LocalID,
                                    m_shapeID, Friction, Friction, Restitution,
                                    m_currentSize.X / 2.0f, m_linkPos,
                                    Density);
                            }
                            else
                            {
                                // This is a sphere object: send info for
                                // PhysX to create sphere object in the scene
                                m_pxScene.PhysX.CreateObjectSphere(LocalID,
                                    Name, m_rawPosition, m_shapeID, Friction,
                                    Friction, Restitution,
                                    m_currentSize.X / 2.0f, Density,
                                    isPhysical, reportCollisions);
                            }

                            // Mark that the object has been created to prevent
                            // the creation of a mesh for an object that
                            // already exists
                            m_isObjectBuilt = true;
                        }
                        else if (m_opensimBaseShape.ProfileShape == 
                            ProfileShape.Square && 
                            m_opensimBaseShape.PathCurve == 
                            (byte)Extrusion.Straight)
                        {
                            // Check to see if this object is linked to a parent
                            if (m_linkParent != null)
                            {
                                // Attach this object's box shape to the
                                // linkset parent
                                m_pxScene.PhysX.AttachBox(m_linkParent.LocalID,
                                    m_shapeID, Friction, Friction, Restitution,
                                    m_currentSize.X / 2.0f,
                                    m_currentSize.Y / 2.0f,
                                    m_currentSize.Z / 2.0f, m_linkPos,
                                    m_linkOrient, Density);
                            }
                            else
                            {
                                // This is a box object: send info for
                                // PhysX to create box object in the scene
                                m_pxScene.PhysX.CreateObjectBox(LocalID, Name,
                                    m_rawPosition, m_shapeID, Friction,
                                    Friction, Restitution,
                                    (m_currentSize.X / 2.0f), 
                                    (m_currentSize.Y / 2.0f),
                                    (m_currentSize.Z / 2.0f), Density, 
                                    isPhysical, reportCollisions);
                            }

                            // Mark that the object has been created to
                            // prevent the creation of a mesh for an object
                            // that already exists
                            m_isObjectBuilt = true;
                        }
                    }
                }
                
                // This checks to see if the object was created by being a
                // predefined sphere, box, or avatar capsule, if not a mesh
                // should be created to represent the object
                if (!m_isObjectBuilt)
                {
                    // This is a mesh, so use the scene's mesher
                    // to create a mesh with the given shape info
                    vertices = null;
                    indices = null;

                    // Check to see if the mesh needs to be reconstructed
                    if (m_primMesh == null)
                    {
                        // Construct the mesh using the mesher and the
                        // current level of detail
                        m_primMesh = m_pxScene.SceneMesher.CreateMesh(Name,
                            m_opensimBaseShape, m_currentSize, m_meshLOD, 
                            isPhysical, false);
                    }

                    // Check to see if the mesh was successfully constructed
                    if (m_primMesh != null)
                    {
                        // Fetch the list of vertices and indices that will
                        // be used to construct the mesh in PhysX
                        vertices = m_primMesh.getVertexList().ToArray();
                        indices = m_primMesh.getIndexListAsInt();

                        // Check to see if the physics object is not 
                        // physical
                        if (!isPhysical)
                        {
                            // Check to see if this object is linked to a
                            // parent
                            if (m_linkParent != null)
                            {
                                // Attach this object's mesh shape to the
                                // linkset parent
                                m_pxScene.PhysX.AttachConvexMesh(
                                   m_linkParent.LocalID, m_shapeID,
                                   Friction, Friction, Restitution,
                                   vertices, m_linkPos, m_linkOrient,
                                   Density);
                            }
                            else
                            {
                                // Since the object is non-physical, it can
                                // be represented as a triangle mesh in
                                // PhysX
                                m_pxScene.PhysX.CreateObjectTriangleMesh(
                                    LocalID, Name, m_rawPosition, m_shapeID,
                                    Friction, Friction, Restitution,
                                    vertices, indices, isPhysical,
                                    reportCollisions);
                            }
                        }
                        else
                        {
                            // Check to see if this object is linked to a
                            // parent
                            if (m_linkParent != null)
                            {
                                // Attach this object's mesh shape to the
                                // linkset parent
                                m_pxScene.PhysX.AttachConvexMesh(
                                   m_linkParent.LocalID, m_shapeID,
                                   Friction, Friction, Restitution,
                                   vertices, m_linkPos, m_linkOrient,
                                   Density);
                            }
                            else
                            {
                                // This object is physical, so it has to be
                                // represented by a convex mesh in PhysX
                                // Check to see if this object is linked to
                                // a parent
                                m_pxScene.PhysX.CreateObjectConvexMesh(
                                    LocalID, Name, m_rawPosition, m_shapeID,
                                    Friction, Friction, Restitution,
                                    vertices, Density, isPhysical,
                                    reportCollisions);
                            }
                        }

                        // Indicate that the shape for this object has been
                        // built
                        m_isObjectBuilt = true;
                    }
                    else
                    {
                        // Check to see if the assets for the mesh have
                        // been fetched
                        if (PrimAssetState == AssetState.FETCHED)
                        {
                            // If the mesher failed to create a mesh with
                            // all the assets available, it means that
                            // the mesh is not meant to be
                            PrimAssetState = AssetState.FAILED_MESHING;
                        }
                        else
                        {
                            // Check to see if the conditions are right
                            // to fetch the assets for the mesh.
                            // Don't attempt to fetch the texture, if the
                            // this object is already waiting for the
                            // texture fetch
                            if (m_opensimBaseShape.SculptEntry &&
                                PrimAssetState != 
                                AssetState.FAILED_ASSET_FETCH &&
                                PrimAssetState !=
                                AssetState.FAILED_MESHING &&
                                PrimAssetState != AssetState.WAITING &&
                                m_opensimBaseShape.SculptTexture !=
                                UUID.Zero)
                            {
                                // Indicate that this object is now waiting
                                // on the asset
                                PrimAssetState = AssetState.WAITING;

                                // Get the request asset method from the
                                // parent scene
                                assetDelegate = m_pxScene.
                                                RequestAssetMethod;

                                // If the request asset method is valid,
                                // go ahead and request the texture asset
                                // for the mesh
                                if (assetDelegate != null)
                                {
                                    assetDelegate(
                                        m_opensimBaseShape.SculptTexture,
                                        new AssetReceivedDelegate(
                                                            MeshAssetFetched));
                                }
                            }
                        }
                    } 
                }

                // Check to see if the shape was successfully built
                if (m_isObjectBuilt)
                {
                    // Send info about the object's transform data
                    m_pxScene.PhysX.SetTransformation(
                        LocalID, m_rawPosition, m_orientation);

                    // If this object has any children linkset objects,
                    // go ahead and build them now so that they can be
                    // properly attached
                    lock (m_childLock)
                    {
                        foreach (PxPhysObject currObj in m_childObjects)
                        {
                            // Remove the old shape, so that the new shape
                            // can be properly attached
                            currObj.RemoveShapeFromLinkset();
                            currObj.BuildPhysicalShape();
                        }
                    }
                }

                // Enable the buoyancy actor on this physics object
                EnableActor(true, "PxActorBuoyancy", delegate() {
                    return new PxActorBuoyancy(m_pxScene, this,
                        "PxActorBuoyancy");
                });
            }
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

            // Check to see if the asset is valid and that this physics object
            // is a sculpt mesh
            if (asset != null && m_opensimBaseShape.SculptEntry)
            {
                // Check to see if the given asset is the one for which this
                // physics object is looking
                if (m_opensimBaseShape.SculptTexture.ToString() == asset.ID)
                {
                    // Initialize the asset data
                    m_opensimBaseShape.SculptData = asset.Data;

                    // Indicate that the asset has been fetched
                    this.PrimAssetState = AssetState.FETCHED;
                    assetFound = true;

                    // Check to see if this object is part of linkset, and have
                    // the parent rebuild the shape using the new asset data;
                    // otherwise, directly schedule a rebuild for the object
                    if (m_linkParent == null)
                       m_pxScene.AddTaintedObject(this);
                    else
                       m_pxScene.AddTaintedObject(m_linkParent);
                }
            }

            // If this is not the correct asset, indicate failure to fetch the
            // asset
            if (!assetFound)
            {
                this.PrimAssetState = AssetState.FAILED_ASSET_FETCH;
            }
        }

        /// <summary>
        /// Adjust the avatar height to account for how PhysX creates avatar
        /// capsules. This will remove the spheres at the edges of the capsule
        /// and then cut the height in half, because OpenSim passes the height
        /// with the radius included and PhysX wants the half height of the
        /// cylinder and will add the radius of the spheres to the capsule.
        /// </summary>
        private float ComputeAvatarHalfHeight()
        {
            float halfHeight;

            // Remove the sphere from the capsule height and then cut the
            // height in half 
            halfHeight = (m_currentSize.Z - Math.Min(m_currentSize.X, 
                m_currentSize.Y)) / 2.0f;

            // There is a noticable jitter for small scale characters, if the
            // configuration options are enabled this will prevent the avatar
            // from experiencing the jitter by keeping the physical capsule at
            // a larger size than the avatar
            if (m_pxScene.UserConfig.AvatarJitterFix && halfHeight < 0.685f)
            {
                halfHeight = 0.685f;
            }

            // Return the half height that will be used to construct the
            // character capsule geometry
            return halfHeight;
        }

        #endregion // Helper Methods
    }
}
