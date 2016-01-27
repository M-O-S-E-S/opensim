
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
using System.Text;

namespace OpenSim.Region.Physics.PhysXPlugin
{
    public class PxActorCollection
    {
        /// <summary>
        /// The physics scene that the actors contained within
        /// this collection of PxActors will act within/inside of.
        /// </summary>
        private PxScene m_physicsScene;

        /// <summary>
        /// A dictionary relating the names of PxActors to physics actors
        /// that act upon physical object within the physics scene.
        /// </summary>
        private Dictionary<string, PxActor> m_physicalActors;

        /// <summary>
        /// Getter and protected setter for the physics scene that
        /// the actors contained within this collection of PxActors
        /// will act within/inside of.
        /// </summary>
        public PxScene PhysicsScene 
        {
            get 
            {
                return m_physicsScene;
            }

            protected set 
            {
                m_physicsScene = value;
            }
        }

        /// <summary>
        /// The getter and setter for the dictionary of physical
        /// actors tha will act upon physical object within the
        /// physics scene.
        /// </summary>
        public Dictionary<string, PxActor> PhysicalActors
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
        /// Constructor for the PxActorCollection, which takes in the
        /// physics scene and initializes everything need for the 
        /// PxActorCollection.
        /// </summary>
        public PxActorCollection(PxScene physicsScene)
        {
            PhysicsScene = physicsScene;
            PhysicalActors = new Dictionary<string, PxActor>();
        }
        
        /// <summary>
        /// Add a PxActor to the PxActorCollection by its name.
        /// </summary>
        /// <param name="name"> The physics actor name. </param>
        /// <param name="actor"> The physics actor. </param>
        public void Add(string name, PxActor actor)
        {
            lock (PhysicalActors)
            {
                // If the physical actor's name is not in our dictionary
                // add the physics actor to the dictionary by name
                if (!PhysicalActors.ContainsKey(name))
                {
                   PhysicalActors[name] = actor;
                }
            }
        }

        /// <summary>
        /// Removes and releases the physical actor associated within this
        /// PxActorCollection by the given name, and returns the result of
        /// remove and releasing the physics actor.
        /// </summary>
        /// <param name="name"> The name of the physics actor. </param>
        public bool RemoveAndRelease(string name)
        {
            bool ret = false;
            
            // First, lock the physical actors dictionary
            lock (PhysicalActors)
            {
                // If the physical actor is contained within our dictionary
                // then get the physics actor and dispose of it as well as
                // remove it from the dictionary by name
                if (PhysicalActors.ContainsKey(name))
                {
                    PxActor beingRemoved = PhysicalActors[name];
                    PhysicalActors.Remove(name);
                    beingRemoved.Dispose();
                    ret = true;
                }
            }

            return ret;
        }

        /// <summary>
        /// Removes any and all actors from this PxActorCollection, and
        /// goes ahead and disposes each actor.
        /// </summary>
        public void Clear()
        {
            // Lock the physical actors dictionary
            lock (PhysicalActors)
            {
                // For each, actor, call its dispose method
                // and clear the physical actors dictionary
                ForEachActor(actors => actors.Dispose());
                PhysicalActors.Clear();
            }
        }

        /// <summary>
        /// Disposes and clears the PxActorCollection.
        /// </summary>
        public void Dispose()
        {
            // Clear the physical actors dictionary and dispose
            Clear();
        }

        /// <summary>
        /// Returns the result of checking if the physics actor dictionary
        /// contains a physics actors with the specified name.
        /// </summary>
        /// <param name="name"> The physics actor name. </param>
        public bool HasActor(string name)
        {
            // Return the result of inquiring the physical actors dictionary
            // to see if it contains an entry of a key with the given name
            return PhysicalActors.ContainsKey(name);
        }

        /// <summary>
        /// Return the result of getting the actor out of the dictionary
        /// and returns in via the reference of the PxActor.
        /// </summary>
        /// <param name="actorName"> The physics actor name. </param>
        /// <param name="theActor"> The physics actor reference. </param>
        public bool TryGetActor(string actorName, out PxActor theActor)
        {
            // Try to get the result of getting the actor by the given
            // actor name, and return the result, if true theActor will
            // be the retrieved physical actor
            return PhysicalActors.TryGetValue(actorName, out theActor);
        }

        /// <summary>
        /// Iterates through and preforms the passed in action on each of 
        /// the PxActors within this PxActorCollection.
        /// </summary>
        /// <param name="act"> The action to preform. </param>
        public void ForEachActor(Action<PxActor> act)
        {
            // Lock the physical actors dictionary
            lock (PhysicalActors)
            {
                // For each of the physical actor keypair within the dictionary
                // go ahead and call the action on the physical actor
                foreach (KeyValuePair<string, PxActor> kvp in PhysicalActors)
                {
                    act(kvp.Value);
                }
            }
        }

        /// <summary>
        /// Iterates through and enables/disables the physical actors within this
        /// PxActorCollection.
        /// </summary>
        /// <param name="enabled"> The boolean representing to disable/enable. </param>
        public void Enable(bool enabled)
        {
            // For each actor, call the SetEnabled method with the given value
            ForEachActor(actor => actor.SetEnabled(enabled));
        }
        
        /// <summary>
        /// Iterates through and refreshes the state of each of the PxActors within
        /// this PxActorCollection.
        /// </summary>
        public void Refresh()
        {
            // For each actor, call the Refresh method
            ForEachActor(actor => actor.Refresh());
        }

        /// <summary>
        /// Iterates through and calls the remove dependencies of each of the 
        /// PxActors within this PxActorCollection.
        /// </summary>
        public void RemoveDependencies()
        {
            // For each actor, call the RemoveDependencies method
            ForEachActor(a => a.RemoveDependencies());
        }

    }
}
