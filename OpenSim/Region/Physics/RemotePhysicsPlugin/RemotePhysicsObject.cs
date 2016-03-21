
// TODO: Create banner

using System;
using System.Collections.Generic;
using System.Text;

using OpenMetaverse;
using OpenSim.Framework;
using OpenSim.Region.Physics.Manager;

namespace OpenSim.Region.Physics.RemotePhysicsPlugin
{
    public abstract class RemotePhysicsObject : PhysicsActor
    {
        /// <summary>
        /// The parent scene within which this object is contained.
        /// </summary>
        public RemotePhysicsScene ParentScene { get; private set;}

        /// <summary>
        /// The parent scene's configuration used for initializing various
        /// properties throughout the object.
        /// </summary>
        public RemotePhysicsConfiguration ParentConfiguration { get; set; }

        /// <summary>
        /// The type of this object in string form.
        /// </summary>
        public String TypeName { get; protected set; }

        /// <summary>
        /// Inidcates whether this object has been initialized.
        /// </summary>
        public bool IsInitialized { get; protected set; }

        /// <summary>
        /// Indicates the physical shape type of this object.
        /// </summary>
        public RemotePhysicsShape PhysicalShape { get; set; }

        /// <summary>
        /// Indicates whether this object is moving or not.
        /// </summary>
        public abstract bool IsStatic { get; }

        /// <summary>
        /// The gravity of the scene in which this object resides.
        /// </summary>
        public OpenMetaverse.Vector3 Gravity { get; set; }

        /// <summary>
        // The net force acting upon this object.
        /// </summary>
        private OpenMetaverse.Vector3 m_force;

        /// <summary>
        /// The interval between event updates in milliseconds. Used to
        /// throttle rate of outgoing events.
        /// </summary>
        protected int SubscribedEventInterval { get; set; }

        /// <summary>
        /// The next time at which a collision event should be sent. Used to
        /// throttle collision event updates.
        /// </summary>
        protected double NextCollisionEventTime { get; set; }

        /// <summary>
        /// The simulation step at which the last collision occurred.
        /// </summary>
        protected long CollidingStep { get; set; }

        /// <summary>
        /// The simulation step at which the last collision with the ground
        /// occurred.
        /// </summary>
        protected long CollidingGroundStep { get; set; }

        /// <summary>
        /// The simulation step at which the last collision with the 
        /// </summary>
        protected long CollidingObjectStep { get; set; }

        /// <summary>
        /// The number of collisions that have happened with this object.
        /// </summary>
        protected long NumCollisions { get; set; }

        /// <summary>
        /// Keeps track of collision events to be sent back to the simulator.
        /// </summary>
        protected CollisionEventUpdate Collisions;

        /// <summary>
        /// Keeps track of collisions that were reported in the last sending.
        /// </summary>
        protected CollisionEventUpdate CollisionsLastReported;

        /// <summary>
        /// Indicates the number of times the object has failed to cross a
        /// region boundary into a new region.
        /// </summary>
        private int CrossingFailures { get; set; }

        /// <summary>
        /// Indicates whether move to target mode is active.
        /// </summary>
        public override bool PIDActive
        {
            get { return MoveToTargetActive; }
            set { MoveToTargetActive = value; }
        }

        /// <summary>
        /// Indicates whether move to target mode is active.
        /// </summary>
        public bool MoveToTargetActive { get; set; }

        /// <summary>
        /// Indicates the the target location to which this object is moving.
        /// </summary>
        public override OpenMetaverse.Vector3 PIDTarget {
            set { PIDTarget = value; } }

        /// <summary>
        /// Indicates the seconds to critically damping, which prevents
        /// oscillation.
        /// </summary>
        public override float PIDTau { set { MoveToTargetTau = value; } }

        /// <summary>
        /// Indicates the seconds to critically damping, which prevents
        /// oscillation.
        /// </summary>
        public float MoveToTargetTau { get; set; }

        /// <summary>
        /// Indicates whether hovering is active when moving to a target.
        /// </summary>
        public override bool PIDHoverActive { set { HoverActive = value; } }

        /// <summary>
        /// Indicates whether hovering is active when moving to a target.
        /// </summary>
        public bool HoverActive { get; set; }

        /// <summary>
        /// Indicates the height at which hovering may be occurring when
        /// moving towards a target.
        /// </summary>
        public override float PIDHoverHeight { set { HoverHeight = value; } }

        /// <summary>
        /// Indicates the height at which hovering may be occurring when
        /// moving towards a target.
        /// </summary>
        public float HoverHeight { get; set; }

        /// <summary>
        /// Indicates where hovering can occur when moving to a target.
        /// </summary>
        public override PIDHoverType PIDHoverType { set { HoverType = value; } }

        /// <summary>
        /// Indicates where hovering can occur when moving to a target.
        /// </summary>
        public PIDHoverType HoverType { get; set; }

        /// <summary>
        /// Indicates the seconds to critically damping, which prevents
        /// oscillation.
        /// </summary>
        public override float PIDHoverTau { set { HoverTau = value; } }

        /// <summary>
        /// Indicates the seconds to critically damping, which prevents
        /// oscillation.
        /// </summary>
        public float HoverTau { get; set; }

        /// <summary>
        /// llRotLookAt functionality. Not supported at this time.
        /// </summary>
        public override OpenMetaverse.Quaternion APIDTarget { set { return; } }

        /// <summary>
        /// llRotLookAt functionality. Not supported at this time.
        /// </summary>
        public override bool APIDActive { set { return; } }

        /// <summary>
        /// llRotLookAt functionality. Not supported at this time.
        /// </summary>
        public override float APIDStrength { set { return; } }

        /// <summary>
        /// llRotLookAt functionality. Not supported at this time.
        /// </summary>
        public override float APIDDamping { set { return; } }

        /// <summary>
        /// Indicates the simulation step at which the last collision with the
        /// ground has occurred.
        /// </summary>
        protected long GroundCollisionStep { get; set; }

        /// <summary>
        /// Indicates the simulation step at which the last collision with
        /// another object has occurred.
        /// </summary>
        protected long ObjectCollisionStep { get; set; }

        /// <summary>
        /// Indicates the simulation step at which any type of collision
        /// occurred.
        /// </summary>
        protected long LastCollisionStep { get; set; }

        /// <summary>
        /// Object used to ensure that the collision step data is thread-safe.
        /// </summary>
        protected Object m_collisionStepLock;

        /// <summary>
        /// Indicates whether updates from this object should be throttled.
        /// </summary>
        protected bool m_throttleUpdates;

        /// <summary>
        /// Indicates whether this object is being acted upon by the remote
        /// physics engine.
        /// </summary>
        protected bool m_isPhysical;

        /// <summary>
        /// Constructor.
        /// </summary>
        /// <param name="parentScene">The scene to which this object
        /// belongs</param>
        /// <param name="localID">The unique identifier of the object</param>
        /// <param name="name">The name of the object</param>
        /// <param name="typeName">The type of the object</param>
        /// <param name="config">The configuration used to initialize this
        /// object</param>
        protected RemotePhysicsObject(RemotePhysicsScene parentScene,
            uint localID, String name, String typeName,
            RemotePhysicsConfiguration config)
        {
            // Initialize the parent physics scene
            ParentScene = parentScene;

            // Indicate that this object has not been fully initialized
            IsInitialized = false;

            // Initialize the handle to the configuration that will be used
            // initialize other properties
            ParentConfiguration = config;

            // Initialize the local ID of this object
            LocalID = localID;

            // Initialize the name of this physics object
            // Usually this is the same name as the OpenSim object
            Name = name;

            // Initialize the type of this object in string form
            TypeName = typeName;

            // Initialize the gravity modifier to be 1.0, so that the
            // gravity of the scene will be unmodified when it affects
            // this object
            GravModifier = 1.0f;

            // Initialize the gravity vector
            Gravity = new OpenMetaverse.Vector3(0.0f, 0.0f, config.Gravity);

            // Indicate that the object is not hovering
            HoverActive = false;

            // Create the list of pending and sent collisions
            Collisions = new CollisionEventUpdate();
            CollisionsLastReported = Collisions;

            // Initialize the collision steps to be invalid steps, since no
            // collision has occurred yet
            GroundCollisionStep = RemotePhysicsScene.InvalidStep;
            ObjectCollisionStep = RemotePhysicsScene.InvalidStep;
            LastCollisionStep = RemotePhysicsScene.InvalidStep;

            // No collisions have occurred, so start the collision count at 0
            NumCollisions = 0;

            // Initialize the object that will ensure the thread safety of the
            // collision step data
            m_collisionStepLock = new Object();
        }


        /// <summary>
        /// Releases resources used by this object.
        /// </summary>
        public virtual void Destroy()
        {
            // Nothing to clean up here
        }

        public override bool Stopped
        {
            // NOTE: No one seems to know what this property is supposed to
            // mean
            get
            {
                return false;
            }
        }

        /// <summary>
        /// Determines whether a collision is valid, and updates appropriate
        /// collision flags.
        /// </summary>
        /// <param name="colliderID">The unique identifier of the object that
        /// is colliding</param>
        /// <param name="collider">The object that is colliding</param>
        /// <param name="contactPoint">The point at which contact was
        /// made</param>
        /// <param name="contactNormal">The normal of the contact point</param>
        /// <param name="penetrationDepth">How far one collider has penetrated
        /// the other</param>
        /// <returns>Whether the collision is valid</returns>
        public virtual bool Collide(uint colliderID,
            RemotePhysicsObject collider, OpenMetaverse.Vector3 contactPoint,
            OpenMetaverse.Vector3 contactNormal, float penetrationDepth)
        {
            lock (m_collisionStepLock)
            {
                // Update the step at which a collision has occurred
                LastCollisionStep = ParentScene.CurrentSimulationStep;

                // Check to see if this is a collision with terrain
                if (colliderID <= ParentScene.TerrainID)
                {
                    GroundCollisionStep = ParentScene.CurrentSimulationStep;
                }
                else
                {
                    ObjectCollisionStep = ParentScene.CurrentSimulationStep;
                }
            }

            // Update the number of collisions for this object
            NumCollisions++;

            // Check to see if something is subscribed to events from
            // this object
            if (SubscribedEvents())
            {
                // Add this collision to the collection of collisions to be sent
                // back to the simulator at the next update, in a
                // thread-safe manner
                lock (Collisions)
                {
                    Collisions.AddCollider(colliderID,
                        new ContactPoint(contactPoint, contactNormal,
                            penetrationDepth));
                }

                // Indicate that the collision has been successfully processed
                return true;
            }
            else
            {
                // Indicate that the collision wasn't processed, since there are
                // no subscribers to this object
                return false;
            }
        }

        /// <summary>
        /// Sends collision events concerning this object to the simulator.
        /// </summary>
        /// <returns>Whether any collisions were sent to the simulator</returns>
        public virtual bool SendCollisions()
        {
            bool sendEndEvent;
            bool moreEvents = true;

            // Check to see if this object is supposed to be sending out
            // collision events
            if (SubscribedEventInterval <= 0)
            {
                // This means that events should not be sent out, so exit
                return false;
            }

            // Check to see if there are no more collisions, but collisions
            // were sent out last call; if so, go ahead and send an empty
            // collision to signify the end of the collisions
            sendEndEvent = false;
            if (Collisions.Count == 0 && CollisionsLastReported.Count != 0)
            {
                sendEndEvent = true;
            }

            // Check to see if an end event needs to be sent or if the minimum
            // event interval has passed
            if (sendEndEvent || ParentScene.CurrentSimulationTime >=
                NextCollisionEventTime)
            {
                // Calculate the time for sending out the next collision update
                NextCollisionEventTime = ParentScene.CurrentSimulationTime +
                    SubscribedEventInterval;

                // Send the collision event
                base.SendCollisionUpdate(Collisions);

                // Check to see if there are no more collision events to send
                if (Collisions.Count == 0)
                {
                    // Flag that there are no more collision events to send
                    moreEvents = false;
                }

                // Store the collisions reported in this update for later
                // reference
                CollisionsLastReported = Collisions;

                // Create a new event update that will accrue future collisions
                Collisions = new CollisionEventUpdate();
            }

            // Return whether this object has more collision events pending
            return moreEvents;
        }

        /// <summary>
        /// Evaluates the collision score for the object based on the frequency
        /// of collisions. This method allows for computations of the score.
        /// </summary>
        public void EvaluateCollisionScore()
        {
            // Check to see if a collision has occurred yet for this primitive
            if (CollidingStep  <= RemotePhysicsScene.InvalidStep)
            {
                // No collision has occurred for this object yet, so the score
                // is zero
                CollisionScore = 0;
                return;
            }

            // Determine the frequency of collisions while avoid division
            // by zero
            if (ParentScene.CurrentSimulationStep - CollidingStep == 0)
            {
                CollisionScore = NumCollisions;
            }
            else
            {
                CollisionScore = NumCollisions /
                    (ParentScene.CurrentSimulationStep - CollidingStep);
            }
        }

        /// <summary>
        /// The collision score of the primitive.
        /// </summary>
        public override float CollisionScore { get; set; }

        /// <summary>
        /// Sets the interval between events from this object.
        /// </summary>
        /// <param name="updateInterval">The desired interval between events
        /// from this object in milliseconds </param>
        public override void SubscribeEvents(int updateInterval)
        {
            // Store the minimum interval that should occur between collision
            // event updates
            SubscribedEventInterval = updateInterval;

            // Check to see if there is to be any interval between events
            if (updateInterval > 0)
            {
                // Calculate the time for sending out the next collision
                // update event
                NextCollisionEventTime =
                    Util.EnvironmentTickCountSubtract(SubscribedEventInterval);
            }
            else
            {
                // If the given interval is 0 or less, this means that the
                // caller wants to unsubscribe
                UnSubscribeEvents();
            }
        }

        /// <summary>
        /// Unsubscribe from events from this object.
        /// </summary>
        public override void UnSubscribeEvents()
        {
            // No delay required for events, since no one is subscribed
            SubscribedEventInterval = 0;
        }

        /// <summary>
        /// Returns whether there is an event subscription for this object.
        /// </summary>
        /// <returns>Whether there is an event subscription for this
        /// object</returns>
        public override bool SubscribedEvents()
        {
            // If the event interval is greater than zero, that means that the
            // simulator is subscribed
            // for events from this object
            return SubscribedEventInterval > 0;
        }

        /// <summary>
        /// Indicates whether this object has been colliding with the ground.
        /// </summary>
        public override bool CollidingGround
        {
            get
            {
                bool result;

                lock (m_collisionStepLock)
                {
                    // Check to see if the object is colliding with the terrain
                    // right now
                    result = (GroundCollisionStep >
                        RemotePhysicsScene.InvalidStep && 
                        GroundCollisionStep ==
                        ParentScene.CurrentSimulationStep);
                }

                return result;
            }

            set
            {
                lock (m_collisionStepLock)
                {
                    // Set the ground collision step to the current simulated
                    // step or invalid step based on the given value
                    if (value)
                    {
                        GroundCollisionStep = ParentScene.CurrentSimulationStep;
                    }
                    else
                    {
                        GroundCollisionStep = RemotePhysicsScene.InvalidStep;
                    }
                }
            }
        }

        /// <summary>
        /// Indicates whether this object has been colliding with
        /// another object.
        /// </summary>
        public override bool CollidingObj
        {
            get
            {
                bool result;

                lock (m_collisionStepLock)
                {
                    // Check to see if the object is colliding with another
                    // object right now
                    result = (ObjectCollisionStep >
                        RemotePhysicsScene.InvalidStep &&
                        ObjectCollisionStep ==
                        ParentScene.CurrentSimulationStep);
                }

                return result;
            }

            set
            {
                lock (m_collisionStepLock)
                {
                    // Set the object collision step to the current simulated
                    // step or invalid step based on the given value
                    if (value)
                    {
                        ObjectCollisionStep = ParentScene.CurrentSimulationStep;
                    }
                    else
                    {
                        ObjectCollisionStep = RemotePhysicsScene.InvalidStep;
                    }
                }
            }
        }

        /// <summary>
        /// Indicates whether this object has been colliding with anything.
        /// </summary>
        public override bool IsColliding
        {
            get
            {
                bool result;

                lock (m_collisionStepLock)
                {
                    // If the last collision time matches the last simulated
                    // time, this object is colliding
                    result = (LastCollisionStep >
                        RemotePhysicsScene.InvalidStep &&
                        LastCollisionStep == ParentScene.CurrentSimulationStep);
                }

                return result;
            }

            set
            {
                lock (m_collisionStepLock)
                {
                    // Check to see if object should be colliding based on the
                    // given value
                    if (value)
                    {
                        // Force the object to appear colliding by setting
                        // the last collision step to be the last step
                        // simulated by the engine
                        LastCollisionStep = ParentScene.CurrentSimulationStep;
                    }
                    else
                    {
                        // Force the object to appear as not colliding by
                        // setting the last collision step to be an invalid step
                        LastCollisionStep = RemotePhysicsScene.InvalidStep;
                    }
                }
            }
        }

        /// <summary>
        /// Property for throttling the number of updates from this object.
        /// </summary>
        public override bool ThrottleUpdates
        {
            get
            {
                return m_throttleUpdates;
            }

            set
            {
                m_throttleUpdates = value;
            }
        }

        /// <summary>
        /// Indicates whether this object is being acted upon by physical
        /// forces in the remote engine.
        /// </summary>
        public override bool IsPhysical
        {
            get
            {
                return m_isPhysical;
            }
            set
            {
                m_isPhysical = value;
            }
        }

        /// <summary>
        /// The net force acting upon this object.
        /// </summary>
        public override Vector3 Force
        {
            get
            {
                return m_force;
            }
            set
            {
                m_force = value;
            }
        }


        /// <summary>
        /// This is called when an object has failed to cross into a new region.
        /// </summary>
        public override void CrossingFailure()
        {
            // Increment the number of crossing failures that this object has
            // experienced
            CrossingFailures++;
 
            // Check to see if this object has exceeded the number of region
            // crossing failures
            if (CrossingFailures >
                ParentScene.RemoteConfiguration.
                    CrossingFailuresBeforeOutOfBounds)
            {
                // Indicate that this object is now out of bounds
                base.RaiseOutOfBounds(Position);
            }
        }
    }
}

