
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


using System.Collections.Generic;
using System.Text;
using System;


namespace OpenSim.Region.Physics.PhysXPlugin
{
    /// <summary>
    /// Structure that describes the physical properties of a material type.
    /// </summary>
    public struct PxMaterialAttributes
    {
        /// <summary>
        /// The name of the type of the material.
        /// </summary>
        public string m_type;

        /// <summary>
        /// The density of the material.
        /// </summary>
        public float m_density;

        /// <summary>
        /// The friction of the material.
        /// </summary>
        public float m_friction;

        /// <summary>
        /// The restitution of the material.
        /// </summary>
        public float m_restitution;

        /// <summary>
        /// Constructor.
        /// <summary>
        /// <param name="type">String that names the type of the
        /// material</param>
        /// <param name="density">The density of the material</param>
        /// <param name="friction">The coefficient of friction of the
        /// material</param>
        /// <param name="restitution">The restitution of the material</param>
        public PxMaterialAttributes(string type, float density, float friction,
            float restitution)
        {
            // Initialize the material properties based on the given parameters
            m_type = type;
            m_density = density;
            m_friction = friction;
            m_restitution = restitution;
        }
    }


    /// <summary>
    /// Class that holds physical property data for various material archetypes.
    /// </summary>
    public class PxMaterialLibrary
    {
        public enum Material : int
        {
            Stone = 0,
            Metal = 1,
            Glass = 2,
            Wood = 3,
            Flesh = 4,
            Plastic = 5,
            Rubber = 6,
            Light = 7,
            NumberOfTypes = 8
        }

        /// <summary>
        /// Array containing the various materials in the library.
        /// </summary>
        protected PxMaterialAttributes[] m_attributes =
                     new PxMaterialAttributes[(int)Material.NumberOfTypes];

        /// <summary>
        /// Constructor.
        /// Initializes default values for various materials. The default
        /// values are from http://wiki.secondlife.com/wiki/PRIM_MATERIAL
        /// <summary>
        public PxMaterialLibrary(PxConfiguration config)
        {
            float defaultDensity;

            // Retrieve the default density for objects
            defaultDensity = config.DefaultDensity;

            // Initialize each of the default material attributes using the
            // values from http://wiki.secondlife.com/wiki/PRIM_MATERIAL
            m_attributes[(int)Material.Stone] = new
                PxMaterialAttributes("Stone", defaultDensity, 0.8f, 0.4f);
            m_attributes[(int)Material.Metal] = new
                PxMaterialAttributes("Metal", defaultDensity, 0.3f, 0.4f);
            m_attributes[(int)Material.Glass] = new
                PxMaterialAttributes("Glass", defaultDensity, 0.2f, 0.7f);
            m_attributes[(int)Material.Wood] = new
                PxMaterialAttributes("Wood", defaultDensity, 0.6f, 0.5f);
            m_attributes[(int)Material.Flesh] = new
                PxMaterialAttributes("Flesh", defaultDensity, 0.9f, 0.3f);
            m_attributes[(int)Material.Plastic] = new
                PxMaterialAttributes("Plastic", defaultDensity, 0.4f, 0.7f);
            m_attributes[(int)Material.Rubber] = new
                PxMaterialAttributes("Rubber", defaultDensity, 0.9f, 0.9f);
            m_attributes[(int)Material.Light] = new
                PxMaterialAttributes("Light", defaultDensity,
                    config.DefaultFriction, config.DefaultRestitution);
        }

        /// <summary>
        /// Returns the attributes of a material.
        /// </summary>
        /// <param name="matType">The desired material</param>
        /// <returns>Structure containing various physical attributes</returns>
        public PxMaterialAttributes GetAttributes(Material matType)
        {
            // Check to see if the given material type is valid
            if ((int) matType >= 0 &&
                (int) matType < (int) Material.NumberOfTypes)
            {
                // Fetch & return the material attributes for the given type
                return m_attributes[(int) matType];
            }
            else
            {
                // Return the a default Stone material
                return m_attributes[(int) Material.Stone];
            }
        }
    }
}

