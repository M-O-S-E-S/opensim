
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
using System.Runtime.InteropServices;
using System.Security;
using System.Text;

using OpenSim.Framework;
using OpenMetaverse;

namespace OpenSim.Region.Physics.PhysXPlugin
{
    /// <summary>
    /// Interface that provides the connection to the unmanaged assembly
    /// of the PhysX physics engine.
    /// </summary>
    public sealed class PxAPIManager
    {
        /// <summary>
        /// Pinned memory that is passed between the managed and
        /// unmanaged code. Will contain updated entity data from PhysX.
        /// </summary>
        GCHandle m_updateArrayPinnedHandle;

        /// <summary>
        /// Pinned memory that is passed between the managed and
        /// unmanaged code. Will contain updated collision data from PhysX.
        /// </summary>
        GCHandle m_collisionArrayPinnedHandle;

        /// <summary>
        /// Basic structure used to denote an actor's position.
        /// </summary>
        public struct ActorPosition
        {
            public float x;
            public float y;
            public float z;
        }

        /// <summary>
        /// Basic structure used to denote an actor's orientation.
        /// </summary>
        public struct ActorOrientation
        {
            public float x;
            public float y;
            public float z;
            public float w;
        }

        #region Expose API calls

        /// <summary>
        /// Constructor of the PhysX API manager. Loads the PhysX assembly.
        /// </summary>
        public PxAPIManager()
        {
            // Load the PhysX DLL for Windows platform; if not Windows,
            // loading is performed by the Mono loader as specified in
            // "bin/Physics/OpenSim.Region.Physics.PhysXPlugin.dll.config"
            if (Util.IsWindows())
            {
                Util.LoadArchSpecificWindowsDll("PhysXWrapper.dll");
            }
        }


        /// <summary>
        /// Initialize the PhysX foundation.
        /// </summary>
        /// <returns>Flag indicating whether the PhysX startup was
        /// successful or not.</returns>
        public bool Initialize()
        {
            int success;

            // Initialize the PhysX engine
            success = PxAPI.initialize();

            // Return whether the PhysX successfully initialized or not
            if (success == 1)
            {
                return true;
            }
            else
            {
                return false;
            }
        }


        /// <summary>
        /// Clean up all PhysX objects.
        /// </summary>
        public void Uninitialize()
        {
            // Clean up all PhysX objects
            PxAPI.releasePhysics();

            // Free the update array that is currently pinned in memory
            if (m_updateArrayPinnedHandle.IsAllocated)
            {
               m_updateArrayPinnedHandle.Free();
            }

            // Free the collision array that is currently pinned in memory
            if (m_collisionArrayPinnedHandle.IsAllocated)
            {
                m_updateArrayPinnedHandle.Free();
            }
        }


        /// <summary>
        /// Pass the array that will be shared between the managed and
        /// unmanaged code. PhysX will use array to pass back updated
        /// physical properties of physical actors.
        /// </summary>
        /// <param name="updateArray">Array to contain the properties
        /// of active physical actors in the scene</param>
        /// <param name="maxUpdatesPerFrame">Max amount of updates
        /// that are allowed to be processed each frame</param>
        public void InitEntityUpdate(
            ref EntityProperties[] updateArray, int maxUpdatesPerFrame)
        {
            // Pin down the memory that will be used to pass object updates
            // back from unmanaged code
            m_updateArrayPinnedHandle =
                GCHandle.Alloc(updateArray, GCHandleType.Pinned);

            PxAPI.initEntityUpdate(
                m_updateArrayPinnedHandle.AddrOfPinnedObject(),
                maxUpdatesPerFrame);
        }

        /// <summary>
        /// Pass the array that will be shared between the managed and
        /// unmanaged code. PhysX will use array to pass back updated
        /// collisions between the physical actors.
        /// </summary>
        /// <param name="collisionArray">Array to contain the properties
        /// of active collisions that have occured in the scene</param>
        /// <param name="maxCollisionsPerFrame">Max amount of collisions
        /// that are allowed to be processed each frame</param>
        public void InitCollisionUpdate(
            ref CollisionProperties[] collisionArray, int maxCollisionsPerFrame)
        {
            // Pin down the memory that will be used to pass object updates
            // back from unmanaged code
            m_collisionArrayPinnedHandle =
                GCHandle.Alloc(collisionArray, GCHandleType.Pinned);

            // Initialize the collision update array on unmanaged code side
            PxAPI.initCollisionUpdate(
                m_collisionArrayPinnedHandle.AddrOfPinnedObject(),
                maxCollisionsPerFrame);
        }


        /// <summary>
        /// Create the PhysX Scene.
        /// </summary>
        /// <param name="gpuEnabled">Controls whether PhysX should attempt to
        /// initialise the GPU dispatcher</param>
        /// <param name="cpuEnabled">Controls whether PhysX should attempt to
        /// initialise the CPU dispatcher</param>
        /// <param name="cpuMaxThreads">The number of threads that the cpu will
        /// attempt to run in parallel for PhysX, if and only if the cpuEnabled
        /// value is set to true</param>
        /// <returns>Flag indicating whether the scene was successfully
        /// created or not.</returns>
        /// <remarks>The create scene function will do everything it can to
        /// keep PhysX running. This means that should CPU be set to false and
        /// the GPU fails or is also false the program will enable the CPU with
        /// no extra threads.</remarks>
        public bool CreateScene(bool gpuEnabled, int cpuMaxThreads)
        {
            int success;

            // Create the PhysX Scene
            success = PxAPI.createScene(gpuEnabled, cpuMaxThreads);

            // Return whether the scene was successfully created or not
            if (success == 1)
            {
                return true;
            }
            else
            {
                return false;
            }
        }


        /// <summary>
        /// Dispose of the scene.
        /// </summary>
        public void DisposeScene()
        {
            // Dispose of the scene
            PxAPI.releaseScene();
        }


        /// <summary>
        /// Run the main update of the physics engine for the amount of time
        /// given.
        /// </summary>
        /// <param name="time">The amount of time that the physics engine
        /// should advance inside of the simulation</param>
        /// <param name="updatedEntityCount">The number of entity
        /// objects that were updated during the simulation step</param>
        /// <param name="updatedCollisionCount">The number of collision
        /// updates that were made during the simulation step</param>
        public void RunSimulation(float time,
            out uint updatedEntityCount, out uint updatedCollisionCount)
        {
            // Tell PhysX to advance the simulation
            PxAPI.simulate(
                time, out updatedEntityCount, out updatedCollisionCount);
        }


        /// <summary>
        /// Set the world space transform of a physical actor.
        /// </summary>
        /// <param name="ID">Unique ID of the phsyical actor"</param>
        /// <param name="position">New position of actor</param>
        /// <param name="rotation">New rotation of actor</param>
        public void SetTransformation(
            uint ID, Vector3 position, Quaternion rotation)
        {
            // Pass the given actor's translation info to PhysX
            PxAPI.setTransformation(ID, position.X, position.Y, position.Z,
                rotation.X, rotation.Y, rotation.Z, rotation.W);
        }


        /// <summary>
        /// Directly set the position for the actor.
        /// </summary>
        /// <param name="ID">Unique ID of the physical actor</param>
        /// <param name="position">New position of actor</param>
        public void SetPosition(uint ID, Vector3 position)
        {
            ActorPosition actorPos;

            // Pass the given actor's new position to PhysX
            actorPos.x = position.X;
            actorPos.y = position.Y;
            actorPos.z = position.Z;
            PxAPI.setPosition(ID, actorPos);
        }


        /// <summary>
        /// Directly set the rotation for the actor.
        /// </summary>
        /// <param name="ID">Unique ID of the physical actor</param>
        /// <param name="position">New rotation of actor</param>
        public void SetRotation(uint ID, Quaternion rotation)
        {
            ActorOrientation actorOrient;

            // Pass the given actor's new rotation to PhysX
            actorOrient.x = rotation.X;
            actorOrient.y = rotation.Y;
            actorOrient.z = rotation.Z;
            actorOrient.w = rotation.W;
            PxAPI.setRotation(ID, actorOrient);
        }


        /// <summary>
        /// Set the velocity for the actor to move in a linear direction.
        /// </summary>
        /// <param name="ID">Unique ID of the physical actor</param>
        /// <param name="velocity">New linear velocity of actor</param>
        public void SetLinearVelocity(uint ID, Vector3 velocity)
        {
            // Pass the given actor's new velocity to PhysX
            PxAPI.setLinearVelocity(ID, velocity.X, velocity.Y, velocity.Z);
        }


        /// <summary>
        /// Set the angular velocity for the actor 
        /// which defines actor's angular rotation.
        /// </summary>
        /// <param name="ID">Unique ID of the physical actor</param>
        /// <param name="angularVelocity">New angular velocity of the actor
        /// </param>
        public void SetAngularVelocity(uint ID, Vector3 angularVelocity)
        {
            // Pass the given actor's new angular velocity to PhysX
            PxAPI.setAngularVelocity(ID, angularVelocity.X, angularVelocity.Y,
                angularVelocity.Z);
        }


        /// <summary>
        /// Create an empty physical actor inside the PhysX scene.
        /// </summary>
        /// <param name="id">Unique identifier of the actor</param>
        /// <param name="name">Name of the actor</param>
        /// <param name="position">The global position of the actor</param>
        /// <param name="isDynamic">Indicates whether the actor will be
        /// static or dynamic; true means dynamic</param>
        /// <param name="reportCollisions">Indicates whether collisions
        /// involving this actor should be reported</param>
        public void CreateObject(uint id, string name, Vector3 position,
            bool isDynamic, bool reportCollisions)
        {
            // Create actor inside of the wrapper
            PxAPI.createActor(id, name, position.X, position.Y,
            position.Z, isDynamic, reportCollisions);
        }


        /// <summary>
        /// Attaches a sphere shape to an existing actor inside the PhysX scene.
        /// </summary>
        /// <param name="id">Unique identifier of the actor to which the
        /// shape will be attached</param>
        /// <param name="shapeId">Unique identifier of the shape being
        /// attached.</param>
        /// <param name="staticFriction">Coefficient of friction between
        /// the shape and other objects when stationary</param>
        /// <param name="dynamicFriction">Coefficient of friction between
        /// the shape and other objects when moving</param>
        /// <param name="restitution">Coefficient that indicates the bounciness
        /// of the shape</param>
        /// <param name="relativePosition">The position of the sphere relative
        /// to the actor</param>
        /// <param name="radius">The radius of the sphere</param>
        /// <param name="density">The density of the sphere</param>
        public void AttachSphere(uint id, uint shapeId, float staticFriction,
            float dynamicFriction, float restitution, float radius,
            Vector3 relativePosition, float density)
        {
            // Attach the sphere to an actor inside the PhysX scene
            PxAPI.attachSphere(id, shapeId, staticFriction, dynamicFriction,
                restitution, radius,relativePosition.X, relativePosition.Y,
                relativePosition.Z, density);
        }


        /// <summary>
        /// Attaches a box shape to an existing actor inside the PhysX scene.
        /// </summary>
        /// <param name="id">Unique identifier of the actor to which the
        /// shape will be attached</param>
        /// <param name="shapeId">Unique identifier of the shape being
        /// attached</param>
        /// <param name="staticFriction">Coefficient of friction between
        /// the box and other objects when stationary</param>
        /// <param name="dynamicFriction">Coefficient of friction between
        /// the box and other objects when moving</param>
        /// <param name="restitution">Coefficient that indicates the bounciness
        /// of the shape</param>
        /// <param name="halfX">Half the length of the box</param>
        /// <param name="halfY">Half the width of the box</param>
        /// <param name="halfZ">Half the height of the box</param>
        /// <param name="relativePosition">The position of the box relative
        /// to the actor</param>
        /// <param name="relativeOrientation">The orientation of the box
        /// relative to the actor</param>
        /// <param name="density">The density of the box</param>
        public void AttachBox(uint id, uint shapeId, float staticFriction,
            float dynamicFriction, float restitution, float halfX, float halfY,
            float halfZ, Vector3 relativePosition,
            Quaternion relativeOrientation, float density)
        {
            // Attach the box to an actor in the PhysX scene
            PxAPI.attachBox(id, shapeId, staticFriction, dynamicFriction,
                restitution, halfX, halfY, halfZ, relativePosition.X,
                relativePosition.Y, relativePosition.Z, relativeOrientation.X,
                relativeOrientation.Y, relativeOrientation.Z,
                relativeOrientation.W, density);
        }


        /// <summary>
        /// Attaches a capsule shape to an existing actor inside the PhysX
        /// scene.
        /// </summary>
        /// <param name="id">Unique identifier of the actor to which the
        /// shape will be attached</param>
        /// <param name="shapeId">Unique identifier of the shape being
        /// attached</param>
        /// <param name="staticFriction">Coefficient of friction between the
        /// capsule and other objects when stationary</param>
        /// <param name="dynamicFriction">Coefficient of friction between the
        /// capsule and other objects when moving</param>
        /// <param name="restitution">Coefficient that indicates the bounciness
        /// of the box</param>
        /// <param name="halfHeight">Half the height of the capsule</param>
        /// <param name="radius">The raidus of the capsule</param>
        /// <param name="relativePosition">The position of the box relative
        /// to the actor</param>
        /// <param name="relativeOrientation">The orientation of the box
        /// relative to the actor</param>
        /// <param name="density">The density of the box</param>
        public void AttachCapsule(uint id, uint shapeId, float staticFriction,
            float dynamicFriction, float restitution, float halfHeight,
            float radius, Vector3 relativePosition,
            Quaternion relativeOrientation, float density)
        {
            // Attach the capsule to an actor in the PhysX scene
            PxAPI.attachCapsule(id, shapeId, staticFriction, dynamicFriction,
                restitution, halfHeight, radius, relativePosition.X,
                relativePosition.Y, relativePosition.Z, relativeOrientation.X,
                relativeOrientation.Y, relativeOrientation.Z,
                relativeOrientation.Z, density);
        }


        /// <summary>
        /// Attaches a triangle mesh shape to an existing actor inside the
        /// PhysX scene.
        /// </summary>
        /// <param name="id">Unique identifier of the actor to which the shape
        /// will be attached</param>
        /// <param name="shapeId">Unique identifier of the shape being
        /// attached</param>
        /// <param name="staticFriction">Coefficient of friction between the
        /// mesh and other objects when stationary</param>
        /// <param name="dynamicFriction">Coefficient of friction between the
        /// mesh and other objects when moving</param>
        /// <param name="restitution">Coefficient that indicates the bounciness
        /// of the mesh</param>
        /// <param name="vertices">Array of triangle points that make up
        /// this triangle mesh</param>
        /// <param name="indices">The index array that map out the triangle
        /// points in the vertices array</param>
        /// <param name="relativePosition">The position of the mesh relative
        /// to the actor</param>
        /// <param name="relativeOrientation">The orientation of the mesh
        /// relative to the actor</param>
        /// </param>
        public void AttachTriangleMesh(uint id, uint shapeId,
            float staticFriction, float dynamicFriction, float restitution,
            Vector3[] vertices, int[] indices, Vector3 relativePosition,
            Quaternion relativeOrientation)
        {
           float[] vertexArray;

            // Convert the array of vertices to an array of floats
            vertexArray = new float[vertices.Length * 3];
            for (int i = 0; i < vertices.Length; i++)
            {
                vertexArray[i * 3] = vertices[i].X;
                vertexArray[i * 3 + 1] = vertices[i].Y;
                vertexArray[i * 3 + 2] = vertices[i].Z;
            }

            // Attach the triangle mesh to an actor in the PhysX scene
            PxAPI.attachTriangleMesh(id, shapeId, staticFriction,
                dynamicFriction, restitution, vertexArray, indices,
                vertices.Length, indices.Length, relativePosition.X,
                relativePosition.Y, relativePosition.Z, relativeOrientation.X,
                relativeOrientation.Y, relativeOrientation.Z,
                relativeOrientation.W);
        }


        /// <summary>
        /// Attaches a convex mesh shape to an existing actor inside the
        /// PhysX scene.
        /// </summary>
        /// <param name="id">Unique identifier of the actor to which the shape
        /// will be attached</param>
        /// <param name="shapeId">Unique identifier of the shape being
        /// attached</param>
        /// <param name="staticFriction">Coefficient of friction between the
        /// mesh and other objects when stationary</param>
        /// <param name="dynamicFriction">Coefficient of friction between the
        /// mesh and other objects when moving</param>
        /// <param name="restitution">Coefficient that indicates the bounciness
        /// of the mesh</param>
        /// <param name="vertices">Array of points that make up the mesh</param>
        /// <param name="relativePositon">The position of the mesh relative
        /// to the actor</param>
        /// <param name="relativeOrientation">The orientation of the mesh
        /// relative to the actor</param>
        /// <param name="density">The density of the mesh</param>
        public void AttachConvexMesh(uint id, uint shapeId,
            float staticFriction, float dynamicFriction, float restitution,
            Vector3[] vertices, Vector3 relativePosition,
            Quaternion relativeOrientation, float density)
        {
           float[] vertexArray;

            // Convert the array of vertices to an array of floats
            vertexArray = new float[vertices.Length * 3];
            for (int i = 0; i < vertices.Length; i++)
            {
                vertexArray[i * 3] = vertices[i].X;
                vertexArray[i * 3 + 1] = vertices[i].Y;
                vertexArray[i * 3 + 2] = vertices[i].Z;
            }

            // Attach the convex mesh to an actor in the PhysX scene
            PxAPI.attachConvexMesh(id, shapeId, staticFriction, dynamicFriction,
                restitution, vertexArray, vertices.Length,
                relativePosition.X, relativePosition.Y, relativePosition.Z,
                relativeOrientation.X, relativeOrientation.Y,
                relativeOrientation.Z, relativeOrientation.W, density);
        }


        /// <summary>
        /// Removes a shape from an actor and deletes it from the PhysX scene.
        /// </summary>
        /// <param name="id">Unique identifier of the actor from which the
        /// the shape is being removed</param>
        /// <param name="shapeId">Unique identifier of the shape being
        /// removed and deleted</param>
        public void RemoveShape(uint id, uint shapeId)
        {
            // Remove and delete the given shape from the PhysX scene
            PxAPI.removeShape(id, shapeId);
        }


        /// <summary>
        /// Create a physical actor inside of the PhysX unmanaged code that
        /// will use the sphere shape.
        /// </summary>
        /// <param name="id">Unique identifier of actor</param>
        /// <param name="name">Name of actor</param>
        /// <param name="pos">Initial position of actor</param>
        /// <param name="shapeId">Unique identifier of the box shape</param>
        /// <param name="staticFriction">Coefficient of friction between the
        /// actor and stationary objects</param>
        /// <param name="dynamicFriction">Coefficient of friction between the
        /// actor and dynamic objects</param>
        /// <param name="restitution">Coefficient to indicate how much
        /// the actor bounces off of surfaces</param>
        /// <param name="radius">The radius of the actor's shape</param>
        /// <param name="density">Actor's density</param>
        /// <param name="isDynamic">Whether the actor is static or dynamic
        /// </param>
        /// <param name="reportCollisions">Indicates whether collisions
        /// involving this actor should be reported</param>
        public void CreateObjectSphere(uint id, string name, Vector3 pos, 
            uint shapeId, float staticFriction, float dynamicFriction,
            float restitution, float radius, float density, bool isDynamic,
            bool reportCollisions)
        {
            // Create sphere actor inside of the wrapper
            PxAPI.createActorSphere(id, name, pos.X, pos.Y, pos.Z,
                shapeId, staticFriction, dynamicFriction, restitution, radius,
                density, isDynamic, reportCollisions);
        }


        /// <summary>
        /// Create a physical actor inside of the PhysX unmanaged code that
        /// will use the box shape.
        /// </summary>
        /// <param name="id">Unique identifier of actor</param>
        /// <param name="name">Name of actor</param>
        /// <param name="pos">Initial position of actor</param>
        /// <param name="shapeId">Unique identifier of the box shape</param>
        /// <param name="staticFriction">Coefficient of friction between the
        /// actor and stationary objects</param>
        /// <param name="dynamicFriction">Coefficient of friction between the
        /// actor and dynamic objects</param>
        /// <param name="restitution">Coefficient to indicate how much
        /// the actor bounces off of surfaces</param>
        /// <param name="halfX">One-half of the box's size in the x-axis</param>
        /// <param name="halfY">One-half of the box's size in the y-axis</param>
        /// <param name="halfZ">One-half of the box's size in the z-axis</param>
        /// <param name="density">Actor's density</param>
        /// <param name="isDynamic">Whether the actor is static or dynamic
        /// </param>
        /// <param name="reportCollisions">Indicates whether collisions
        /// involving this actor should be reported</param>
        public void CreateObjectBox(uint id, string name, Vector3 pos,
            uint shapeId, float staticFriction, float dynamicFriction,
            float restitution, float halfX, float halfY, float halfZ,
            float density, bool isDynamic, bool reportCollisions)
        {
            // Create box actor inside of the wrapper
            PxAPI.createActorBox(id, name, pos.X, pos.Y, pos.Z,
                shapeId, staticFriction, dynamicFriction, restitution,
                halfX, halfY, halfZ, density, isDynamic, reportCollisions);
        }


        /// <summary>
        /// Create a physical actor inside of the PhysX unmanaged code that
        /// will use the capsule shape.
        /// </summary>
        /// <param name="id">Unique identifier of actor</param>
        /// <param name="name">Name of actor</param>
        /// <param name="pos">Initial position of actor</param>
        /// <param name="rot">Initial rotation of the actor</param>
        /// <param name="shapeId">Unique identifier of the capsule shape</param>
        /// <param name="staticFriction">Coefficient of friction between the
        /// actor and stationary objects</param>
        /// <param name="dynamicFriction">Coefficient of friction between the
        /// actor and dynamic objects</param>
        /// <param name="restitution">Coefficient to indicate how much
        /// the actor bounces off of surfaces</param>
        /// <param name="halfHeight">One-half of the capsule's height</param>
        /// <param name="radius">The radius of the capsule's shape</param>
        /// <param name="density">Actor's density</param>
        /// <param name="isDynamic">Whether the actor is static or dynamic
        /// </param>
        /// <param name="reportCollisions">Indicates whether collisions
        /// involving this actor should be reported</param>
        public void CreateCharacterCapsule(uint id, string name, Vector3 pos, 
            Quaternion rot, uint shapeId, float staticFriction,
            float dynamicFriction, float restitution, float halfHeight,
            float radius, float density, bool isDynamic, bool reportCollisions)
        {
            // Create capsule actor inside of the wrapper
            PxAPI.createActorCapsule(id, name, pos.X, pos.Y, pos.Z,
                rot.X, rot.Y, rot.Z, rot.W, shapeId, staticFriction,
                dynamicFriction, restitution, halfHeight,
                radius, density, isDynamic, reportCollisions);
        }


        /// <summary>
        /// Create a physical actor inside of the PhysX unmanaged code that
        /// is a triangle mesh.
        /// </summary>
        /// <param name="id">Unique identifier of actor</param>
        /// <param name="name">Name of actor</param>
        /// <param name="pos">Initial position of actor</param>
        /// <param name="shapeId">Unique identifier of the mesh shape</param>
        /// <param name="staticFriction">Coefficient of friction between the
        /// actor and stationary objects</param>
        /// <param name="dynamicFriction">Coefficient of friction between the
        /// actor and dynamic objects</param>
        /// <param name="restitution">Coefficient to indicate how much
        /// the actor bounces off of surfaces</param>
        /// <param name="vertices">Array of triangle points that make up
        /// this triangle points</param>
        /// <param name="indices">The index array that map out the triangle
        /// points in the vertices array</param>
        /// <param name="density">Actor's density</param>
        /// <param name="isDynamic">Whether the actor is static or dynamic
        /// </param>
        /// <param name="reportCollisions">Indicates whether collisions
        /// involving this actor should be reported</param>
        public void CreateObjectTriangleMesh(uint id, string name, Vector3 pos,
            uint shapeId, float staticFriction, float dynamicFriction,
            float restitution, Vector3[] vertices, int[] indices,
            bool isDynamic, bool reportCollisions)
        {
           float[] vertexArray;

            // Convert the array of vertices to an array of floats
            vertexArray = new float[vertices.Length * 3];
            for (int i = 0; i < vertices.Length; i++)
            {
                vertexArray[i * 3] = vertices[i].X;
                vertexArray[i * 3 + 1] = vertices[i].Y;
                vertexArray[i * 3 + 2] = vertices[i].Z;
            }

            // Send the info to create the new triangle mesh actor
            // to PhysX
            PxAPI.createActorTriangleMesh(id, name, pos.X, pos.Y, pos.Z,
                shapeId, staticFriction, dynamicFriction, restitution,
                vertexArray, indices, vertices.Length, indices.Length,
                isDynamic, reportCollisions);
        }


        /// <summary>
        /// Create a physical actor inside of the PhysX unmanaged code that
        /// is a convex mesh.
        /// </summary>
        /// <param name="id">Unique identifier of actor</param>
        /// <param name="name">Name of actor</param>
        /// <param name="pos">Initial position of actor</param>
        /// <param name="shapeId">Unique identifier of the mesh shape</param>
        /// <param name="staticFriction">Coefficient of friction between the
        /// actor and stationary objects</param>
        /// <param name="dynamicFriction">Coefficient of friction between the
        /// actor and dynamic objects</param>
        /// <param name="restitution">Coefficient to indicate how much
        /// the actor bounces off of surfaces</param>
        /// <param name="vertices">Array of convex points that make up
        /// this actor</param>
        /// <param name="density">Actor's density</param>
        /// <param name="isDynamic">Whether the actor is static or dynamic
        /// </param>
        /// <param name="reportCollisions">Indicates whether collisions
        /// involving this actor should be reported</param>
        public void CreateObjectConvexMesh(uint id, string name, Vector3 pos,
            uint shapeId, float staticFriction, float dynamicFriction,
            float restitution, Vector3[] vertices, float density,
            bool isDynamic, bool reportCollisions)
        {
           float[] vertexArray;

            // Convert the array of vertices to an array of floats
            vertexArray = new float[vertices.Length * 3];
            for (int i = 0; i < vertices.Length; i++)
            {
                vertexArray[i * 3] = vertices[i].X;
                vertexArray[i * 3 + 1] = vertices[i].Y;
                vertexArray[i * 3 + 2] = vertices[i].Z;
            }

            // Send the info to create the new convex mesh actor
            // to PhysX
            PxAPI.createActorConvexMesh(id, name, pos.X, pos.Y, pos.Z,
                shapeId, staticFriction, dynamicFriction, restitution,
                vertexArray, vertices.Length, density, isDynamic,
                reportCollisions);
        }


        /// <summary>
        /// Remove an actor from the physics engine.
        /// </summary>
        /// <param name="id">Unique ID of actor</param>
        public void RemoveActor(uint id)
        {
            // Remove an actor from the PhysX wrapper
            PxAPI.removeActor(id);
        }


        /// <summary>
        /// Updates the physical material properties of a shape.
        /// </summary>
        /// <param name="id">Unique identifier of the actor to which the
        /// shape being modified is attached</param>
        /// <param name="shapeId">Unique identifier of the shape, which is
        /// being modified</param>
        /// <param name="staticFriction">Coefficient of friction between the
        /// shape and other objects when stationary</param>
        /// <param name="dynamicFriction">Coefficient of friction between the
        /// shape and other objects when moving</param>
        /// <param name="restitution">Coefficient that indicates the bounciness
        /// of the shape</param>
        public void UpdateMaterialProperties(uint id, uint shapeId,
            float staticFriction, float dynamicFriction, float restitution)
        {
            // Update the physical properties of the shape's material
            PxAPI.updateMaterialProperties(id, shapeId, staticFriction,
                dynamicFriction, restitution);
        }


        /// <summary>
        /// Create an initial ground plane for the physics world to sit on.
        /// </summary>
        public void CreateGroundPlane(float x, float y, float z)
        {
            // Create an initial ground plane in PhysX
            PxAPI.createGroundPlane(x, y, z);
        }


        /// <summary>
        /// Remove the ground plane from the physics engine. This will most
        /// likely be done right before the terrain mesh is generated.
        /// </summary>
        public void ReleaseGroundPlane()
        {
            // Release the initial ground plane from the wrapper
            PxAPI.releaseGroundPlane();
        }


        /// <summary>
        /// Build the terrain using the height map provided by OpenSim.
        /// </summary>
        /// <param name="terrainActorID">Currently this is set to 0 to identify
        /// it as part of the terrain; When the code for mega-regions is added
        /// this will change</param>
        /// <param name="terrainShapeID">Unique identifier of the terrain
        /// height field</param>
        /// <param name="regionSizeX">The width of the region</param>
        /// <param name="regionSizeY">The length of the region</param>
        /// <param name="rowSpacing">The distance between height positions
        /// along the rows</param>
        /// <param name="columnSpacing">The distance between height positions
        /// along the columns</param>
        /// <param name="posts">The heights of the terrain at intervals given
        /// by the row and column spacing</param>
        /// <param name="heightScaleFactor">The height value scale factor to
        /// be used. Determines the range and precision of height values stored
        /// in the field. A lower value preserves more precision, but reduces
        /// the range of height values in the field. This value must be
        /// greater than 0</param>
        public void SetHeightField(uint terrainActorID, uint terrainShapeID,
            int regionSizeX, int regionSizeY, float rowSpacing,
            float columnSpacing, float[] posts, float heightScaleFactor)
        {
            // Update the height field for the terrain inside of the wrapper
            PxAPI.setHeightField(terrainActorID, terrainShapeID, regionSizeX,
                regionSizeY, rowSpacing, columnSpacing, posts,
                heightScaleFactor);
        }


        /// <summary>
        /// Enable or disable the gravity for the specified physical object.
        /// </summary>
        /// <param name="id">Unique ID of the physical object</param>
        /// <param name="enabled">Whether to enable or disable gravity</param>
        public void EnableGravity(uint id, bool enabled)
        {
            PxAPI.enableGravity(id, enabled);
        }


        /// <summary>
        /// Updates the density of a given shape that is attached to a given
        /// actor.
        /// </summary>
        /// <param name="id">Unique identifier of the actor to which the
        /// desired shape is attached</param>
        /// <param name="shapeID">Unique identifier of the desired shape</param>
        /// <param name="density">The new density of the shape</param>
        public void UpdateShapeDensity(uint id, uint shapeID, float density)
        {
            // Update the density of the shape in the PhysX scene
            PxAPI.updateShapeDensity(id, shapeID, density);
        }


        /// <summary>
        /// Update the mass of the specified PhysX actor.
        /// </summary>
        /// <param name="actorID">The actor id for which the mass is to be
        /// updated</param>
        /// <param name="mass">The updated mass of the actor</param>
        public bool UpdateActorMass(uint actorID, float mass)
        {
            // Tell PhysX to update the mass of the actor
            return PxAPI.updateActorMass(actorID, mass);
        }


        /// <summary>
        /// Add a joint between two actors.
        /// </summary>
        /// <param name="jointID">The ID of the joint being added</param>
        /// <param name="actorID1">The ID of the first actor being joined
        /// </param>
        /// <param name="actorID2">The ID of the second actor being joined
        /// </param>
        /// <param name="actorPos1">The position of joint relative to the
        /// first actor</param>
        /// <param name="actorRot1">The orientation of joint relative to the
        /// first actor</param>
        /// <param name="actorPos2">The position of joint relative to the
        /// second actor</param>
        /// <param name="actorRot2">The orientation of joint relative to the
        /// second actor</param>
        /// <param name="linearLowerLimit">Lower limits of each of the 3
        /// translation axes</param>
        /// <param name="linearUpperLimit">Upper limits of each of the 3
        /// translation axes</param>
        /// <param name="angularLowerLimit">Lower limits of each of the 3
        /// rotational axes</param>
        /// <param name="angularUpperLimit">Upper limits of each of the 3
        /// rotational axes</param>
        public void AddJoint(uint jointID, uint actorID1, uint actorID2,
            Vector3 actorPos1, Quaternion actorRot1, Vector3 actorPos2,
            Quaternion actorRot2, Vector3 linearLowerLimit,
            Vector3 linearUpperLimit, Vector3 angularLowerLimit,
            Vector3 angularUpperLimit)
        {
            float[] actorPosition1;
            float[] actorPosition2;
            float[] actorRotation1;
            float[] actorRotation2;
            float[] linLowerLimit;
            float[] linUpperLimit;
            float[] angLowerLimit;
            float[] angUpperLimit;

            // Initialize the float array for the given positions, rotations,
            // and linear and angular limits
            actorPosition1 = new float[3];
            actorPosition2 = new float[3];
            actorRotation1 = new float[4];
            actorRotation2 = new float[4];
            linLowerLimit = new float[3];
            linUpperLimit = new float[3];
            angLowerLimit = new float[3];
            angUpperLimit = new float[3];

            // Convert first actor's position vector to array of floats
            actorPosition1[0] = actorPos1.X;
            actorPosition1[1] = actorPos1.Y;
            actorPosition1[2] = actorPos1.Z;

            // Convert first actor's rotation quaternion to array of floats
            actorRotation1[0] = actorRot1.X;
            actorRotation1[1] = actorRot1.Y;
            actorRotation1[2] = actorRot1.Z;
            actorRotation1[3] = actorRot1.W;

            // Convert second actor's position vector to array of floats
            actorPosition2[0] = actorPos2.X;
            actorPosition2[1] = actorPos2.Y;
            actorPosition2[2] = actorPos2.Z;

            // Convert second actor's rotation quaternion to array of floats
            actorRotation2[0] = actorRot2.X;
            actorRotation2[1] = actorRot2.Y;
            actorRotation2[2] = actorRot2.Z;
            actorRotation2[3] = actorRot2.W;

            // Convert the linear lower and upper limit vectors to array
            // of floats
            linLowerLimit[0] = linearLowerLimit.X;
            linLowerLimit[1] = linearLowerLimit.Y;
            linLowerLimit[2] = linearLowerLimit.Z;
            linUpperLimit[0] = linearUpperLimit.X;
            linUpperLimit[1] = linearUpperLimit.Y;
            linUpperLimit[2] = linearUpperLimit.Z;

            // Convert the rotational lower and upper limit vectors to array
            // of floats
            angLowerLimit[0] = angularLowerLimit.X;
            angLowerLimit[1] = angularLowerLimit.Y;
            angLowerLimit[2] = angularLowerLimit.Z;
            angUpperLimit[0] = angularUpperLimit.X;
            angUpperLimit[1] = angularUpperLimit.Y;
            angUpperLimit[2] = angularUpperLimit.Z;
            
            // Send the joint data to PhysX to create a joint between
            // the two given actors
            PxAPI.addJoint(jointID, actorID1, actorID2, actorPosition1,
                actorRotation1, actorPosition2, actorRotation2,
                linLowerLimit, linUpperLimit, angLowerLimit, angUpperLimit);
        }

        /// <summary>
        /// Add a joint between an actor and the global frame.
        /// </summary>
        /// <param name="jointID">The ID of the joint being added</param>
        /// <param name="actorID">The ID of the actor being joined.</param>
        /// <param name="actorPos">The position of joint relative to the
        /// actor</param>
        /// <param name="actorRot">The orientation of joint relative to the
        /// actor</param>
        /// <param name="linearLowerLimit">Lower limits of each of the 3
        /// translation axes</param>
        /// <param name="linearUpperLimit">Upper limits of each of the 3
        /// translation axes</param>
        /// <param name="angularLowerLimit">Lower limits of each of the 3
        /// rotational axes</param>
        /// <param name="angularUpperLimit">Upper limits of each of the 3
        /// rotational axes</param>
        public void AddGlobalFrameJoint(uint jointID, uint actorID,
            Vector3 actorPos, Quaternion actorRot, Vector3 linearLowerLimit,
            Vector3 linearUpperLimit, Vector3 angularLowerLimit,
            Vector3 angularUpperLimit)
        {
            float[] actorPosition;
            float[] actorRotation;
            float[] linLowerLimit;
            float[] linUpperLimit;
            float[] angLowerLimit;
            float[] angUpperLimit;

            // Initialize the float array for the given position, rotation,
            // and linear and angular limits
            actorPosition = new float[3];
            actorRotation = new float[4];
            linLowerLimit = new float[3];
            linUpperLimit = new float[3];
            angLowerLimit = new float[3];
            angUpperLimit = new float[3];

            // Convert the actor's position vector to array of floats
            actorPosition[0] = actorPos.X;
            actorPosition[1] = actorPos.Y;
            actorPosition[2] = actorPos.Z;

            // Convert the actor's rotation quaternion to array of floats
            actorRotation[0] = actorRot.X;
            actorRotation[1] = actorRot.Y;
            actorRotation[2] = actorRot.Z;
            actorRotation[3] = actorRot.W;

            // Convert the linear lower and upper limit vectors to array
            // of floats
            linLowerLimit[0] = linearLowerLimit.X;
            linLowerLimit[1] = linearLowerLimit.Y;
            linLowerLimit[2] = linearLowerLimit.Z;
            linUpperLimit[0] = linearUpperLimit.X;
            linUpperLimit[1] = linearUpperLimit.Y;
            linUpperLimit[2] = linearUpperLimit.Z;

            // Convert the rotational lower and upper limit vectors to array
            // of floats
            angLowerLimit[0] = angularLowerLimit.X;
            angLowerLimit[1] = angularLowerLimit.Y;
            angLowerLimit[2] = angularLowerLimit.Z;
            angUpperLimit[0] = angularUpperLimit.X;
            angUpperLimit[1] = angularUpperLimit.Y;
            angUpperLimit[2] = angularUpperLimit.Z;
            
            // Send the joint data to PhysX to create a joint between
            // the two given actors
            PxAPI.addGlobalFrameJoint(jointID, actorID, actorPosition,
                actorRotation, linLowerLimit, linUpperLimit, angLowerLimit,
                angUpperLimit);
        }


        /// <summary>
        /// Remove joint from the physics scene.
        /// </summary>
        /// <param name="jointID">The ID of the join to be removed</param>
        public void RemoveJoint(uint jointID)
        {
            // Tell PhysX to remove the given joint
            PxAPI.removeJoint(jointID);
        }

        
        /// <summary>
        /// Gets the mass of the specified actor.
        /// </summary>
        /// <param name='actorID'>The actor id for which the mass is to be returned.</param>
        public float GetActorMass(uint actorID)
        {
            // Tell PhysX to get and return the mass of the actor
            return PxAPI.getActorMass(actorID);
        }
    
        
        /// <summary>
        /// Adds the force.
        /// </summary>
        /// <param name='actorID'> The actor id for the force to be added to.</param>
        /// <param name='force'> The force as a vector. </param>
        public bool AddForce(uint actorID, Vector3 force)
        {
            // Tell PhysX to add the force to the actor
            return PxAPI.addForce(actorID, force.X, force.Y, force.Z);
        }

        /// <summary>
        /// Adds the force impulse.
        /// </summary>
        /// <param name='actorID'> The actor id for the force to be added to.</param>
        /// <param name='force'> The force impulse as a vector. </param>
        public bool AddForceImpulse(uint actorID, Vector3 force)
        {
            // Tell PhysX to add the force to the actor
            return PxAPI.addForceImpulse(actorID, force.X, force.Y, force.Z);
        }


        /// <summary>
        /// Adds torque to an actor.
        /// </summary>
        /// <param name='actorID'>The unique identifier of the actor to
        /// which the torque is applied</param>
        /// <param name='torque'> The torque as a vector. </param>
        public void AddTorque(uint actorID, Vector3 torque)
        {
            // Tell PhysX to add the torque to the actor
            PxAPI.addTorque(actorID, torque.X, torque.Y, torque.Z);
        }


        /// <summary>
        /// Adds torque impulse to an actor.
        /// </summary>
        /// <param name='actorID'>The unique identifier of the actor to
        /// which the torque is applied</param>
        /// <param name='torque'> The torque impulse as a vector. </param>
        public void AddTorqueImpulse(uint actorID, Vector3 torque)
        {
            // Tell PhysX to add the torque impulse to the actor
            PxAPI.addTorqueImpulse(actorID, torque.X, torque.Y, torque.Z);
        }

        /// <summary>
        /// Sets the linear damping coefficient of an actor.
        /// </summary>
        /// <param name="actorID"> The unique identifier of the actor to
        /// which the linear damping coefficient is updated.</param>
        /// <param name="damping"> The linear damping coefficient. </param>
        public void SetLinearDamping(uint actorID, float damping)
        {
            // Tell PhysX to set the linear damping coefficient of the actor
            PxAPI.setLinearDamping(actorID, damping);
        }

        /// <summary>
        /// Sets the angular damping coefficient of an actor.
        /// </summary>
        /// <param name="actorID"> The unique identifier of the actor to
        /// which the angular damping coefficient is updated.</param>
        /// <param name="damping"> The angular damping coefficient. </param>
        public void SetAngularDamping(uint actorID, float damping)
        {
            // Tell PhysX to set the angular damping coefficient of the actor
            PxAPI.setAngularDamping(actorID, damping);
        }

        #endregion

        /// <summary>
        /// The actual interface to the unmanaged code of PhysX.
        /// </summary>
        static class PxAPI
        {
            [DllImport("PhysXWrapper")]
                public static extern int initialize();
            [DllImport("PhysXWrapper")]
                public static extern void releasePhysics();

            [DllImport("PhysXWrapper")]
                public static extern int createScene(bool gpuEnabled, 
                    int cpuMaxThreads);
            [DllImport("PhysXWrapper")]
                public static extern void releaseScene();

            [DllImport("PhysXWrapper")]
                public static extern void createActor(uint id, string name,
                    float x, float y, float z, bool isDynamic,
                    bool reportCollisions);

            [DllImport("PhysXWrapper")]
                public static extern void attachSphere(uint id, uint shapeId,
                    float staticFriction, float dynamicFriction,
                    float restitution, float radius, float x, float y, float z,
                    float density);

            [DllImport("PhysXWrapper")]
                public static extern void attachBox(uint id, uint shapeId,
                    float staticFriction, float dynamicFriction,
                    float restitution, float halfX, float halfY, float halfZ,
                    float x, float y, float z, float rotX, float rotY,
                    float rotZ, float rotW, float density);

            [DllImport("PhysXWrapper")]
                public static extern void attachCapsule(uint id, uint shapeId,
                    float staticFriction, float dynamicFriction,
                    float restitution, float halfHeight, float radius,
                    float x, float y, float z, float rotX, float rotY,
                    float rotZ, float rotW, float density);

            [DllImport("PhysXWrapper")]
                public static extern void attachTriangleMesh(uint id,
                    uint shapeId, float staticFriction, float dynamicFriction,
                    float restitution, float[] vertices, int[] indices,
                    int vertexCount, int indexCount, float x, float y, float z,
                    float rotX, float rotY, float rotZ, float rotW);

            [DllImport("PhysXWrapper")]
                public static extern void attachConvexMesh(uint id,
                    uint shapeId, float staticFriction, float dynamicFriction,
                    float restitution, float[] vertices, int vertexCount,
                    float x, float y, float z, float rotX, float rotY,
                    float rotZ, float rotW, float density);

            [DllImport("PhysXWrapper")]
                public static extern void removeShape(uint id, uint shapeId);

            [DllImport("PhysXWrapper")]
                public static extern void createActorSphere(uint id,
                    string name, float x, float y, float z,
                    uint shapeId, float staticFriction, float dynamicFriction,
                    float restitution, float radius, float density,
                    bool isDynamic, bool reportCollisions);

            [DllImport("PhysXWrapper")]
                public static extern void createActorBox(uint id,
                    string name, float posX, float posY, float posZ,
                    uint shapeId, float staticFriction, float dynamicFriction,
                    float restitution, float halfX, float halfY,
                    float halfZ, float density, bool isDynamic,
                    bool reportCollisions);

            [DllImport("PhysXWrapper")]
                public static extern void createActorCapsule(uint id,
                    string name, float posX, float posY, float posZ,
                    float rotX, float rotY, float rotZ, float rotW,
                    uint shapeId, float staticFriction, float dynamicFriction,
                    float restitution, float halfHeight, float radius,
                    float density, bool isDynamic, bool reportCollisions);

            [DllImport("PhysXWrapper")]
                public static extern void createActorTriangleMesh(uint id,
                    string name, float posX, float posY, float posZ,
                    uint shapeId, float staticFriction, float dynamicFriction,
                    float restitution, float[] vertices, int[] indices,
                    int vertexCount, int indexCount, bool isDynamic,
                    bool reportCollisions);

            [DllImport("PhysXWrapper")]
                public static extern void createActorConvexMesh(uint id,
                    string name, float posX, float posY, float posZ,
                    uint shapeId, float staticFriction, float dynamicFriction,
                    float restitution, float[] vertices, 
                    int vertexCount, float density, bool isDynamic,
                    bool reportCollisions);

            [DllImport("PhysXWrapper")]
                public static extern void removeActor(uint id);

            [DllImport("PhysXWrapper")]
                public static extern void updateMaterialProperties(uint id,
                    uint shapeId, float staticFriction, float dynamicFriction,
                    float restitution);

            [DllImport("PhysXWrapper")]
                public static extern void createGroundPlane(
                    float x, float y, float z);

            [DllImport("PhysXWrapper")]
                public static extern void releaseGroundPlane();

            [DllImport("PhysXWrapper")]
                public static extern void simulate(
                    float timeStep, out uint updatedEntityCount,
                    out uint updatedCollisionCount);

            [DllImport("PhysXWrapper")]
                public static extern void initEntityUpdate(
                    IntPtr updateArray, int maxUpdates);

            [DllImport("PhysXWrapper")]
                public static extern void initCollisionUpdate(
                    IntPtr collisionArray, int maxCollisions);

            [DllImport("PhysXWrapper")]
                public static extern void setTransformation(
                    uint id, float posX, float posY, float posZ,
                    float rotX, float rotY, float rotZ, float rotW);

            [DllImport("PhysXWrapper")]
                public static extern void setPosition(
                    uint id, ActorPosition actorPos);

            [DllImport("PhysXWrapper")]
                public static extern void setRotation(
                    uint id, ActorOrientation actorOrient);

            [DllImport("PhysXWrapper")]
                public static extern void setHeightField(uint terrainActorID,
                    uint terrainShapeID, int regionSizeX, int regionSizeY,
                    float rowSpacing, float columnSpacing, float[] posts,
                    float heightScaleFactor);

            [DllImport("PhysXWrapper")]
                public static extern void setLinearVelocity(
                   uint id, float x, float y, float z);

            [DllImport("PhysXWrapper")]
                public static extern void setAngularVelocity(
                    uint id, float x, float y, float z);

            [DllImport("PhysXWrapper")]
                public static extern void enableGravity(uint id, bool enabled);

            [DllImport("PhysXWrapper")]
                public static extern void updateShapeDensity(uint id,
                    uint shapeID, float density);

            [DllImport("PhysXWrapper")]
                public static extern bool updateActorMass(uint id, float mass);

            [DllImport("PhysXWrapper")]
                public static extern void addJoint(
                    uint jointID, uint actorID1, uint actorID2,
                    float[] actorPosition1, float[] actorRotation1,
                    float[] actorPosition2, float[] actorRotation2,
                    float[] linearLowerLimit, float[] linearUpperLimit,
                    float[] angularLowerLimt, float[] angularUpperLimit);

            [DllImport("PhysXWrapper")]
                public static extern void addGlobalFrameJoint(
                    uint jointID, uint actorID, float[] actorPosition,
                    float[] actorRotation, float[] linearLowerLimit,
                    float[] linearUpperLimit, float[] angularLowerLimit,
                    float[] angularUpperLimit);

            [DllImport("PhysXWrapper")]
                public static extern void removeJoint(uint jointID);

            [DllImport("PhysXWrapper")]
                public static extern float getActorMass(uint id);
      
            [DllImport("PhysXWrapper")]
                public static extern bool addForce(uint id, float x, float y, float z);

            [DllImport("PhysXWrapper")]
                public static extern bool addForceImpulse(uint id, float x, float y, float z);

            [DllImport("PhysXWrapper")]
                public static extern void addTorque(uint id, float x, float y,
                   float z);

            [DllImport("PhysXWrapper")]
                public static extern void addTorqueImpulse(uint id, float x, float y,
                   float z);

            [DllImport("PhysXWrapper")]
                public static extern void setLinearDamping(uint id, float damp);

            [DllImport("PhysXWrapper")]
                public static extern void setAngularDamping(uint id, float damp);
        }
    }
}

