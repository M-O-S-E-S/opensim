
using System.Collections.Generic;
using System.Text;
using System;


namespace OpenSim.Region.Physics.RemotePhysicsPlugin
{
    /// <summary>
    /// Structure that describes the physical properties of a material type.
    /// </summary>
    public struct RemotePhysicsMaterialAttributes
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
        public RemotePhysicsMaterialAttributes(string type, float density,
            float friction, float restitution)
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
    public class RemotePhysicsMaterialLibrary
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
        protected RemotePhysicsMaterialAttributes[] m_attributes =
                     new RemotePhysicsMaterialAttributes[
                         (int)Material.NumberOfTypes];

        /// <summary>
        /// Constructor.
        /// Initializes default values for various materials. The default
        /// values are from http://wiki.secondlife.com/wiki/PRIM_MATERIAL
        /// <summary>
        public RemotePhysicsMaterialLibrary(RemotePhysicsConfiguration config)
        {
            float defaultDensity;

            // Retrieve the default density for objects
            defaultDensity = config.DefaultDensity;

            // Initialize each of the default material attributes using the
            // values from http://wiki.secondlife.com/wiki/PRIM_MATERIAL
            m_attributes[(int)Material.Stone] = new
                RemotePhysicsMaterialAttributes("Stone", defaultDensity, 0.8f,
                    0.4f);
            m_attributes[(int)Material.Metal] = new
                RemotePhysicsMaterialAttributes("Metal", defaultDensity, 0.3f,
                    0.4f);
            m_attributes[(int)Material.Glass] = new
                RemotePhysicsMaterialAttributes("Glass", defaultDensity, 0.2f,
                    0.7f);
            m_attributes[(int)Material.Wood] = new
                RemotePhysicsMaterialAttributes("Wood", defaultDensity, 0.6f,
                    0.5f);
            m_attributes[(int)Material.Flesh] = new
                RemotePhysicsMaterialAttributes("Flesh", defaultDensity, 0.9f,
                    0.3f);
            m_attributes[(int)Material.Plastic] = new
                RemotePhysicsMaterialAttributes("Plastic", defaultDensity, 0.4f,
                    0.7f);
            m_attributes[(int)Material.Rubber] = new
                RemotePhysicsMaterialAttributes("Rubber", defaultDensity, 0.9f,
                    0.9f);
            m_attributes[(int)Material.Light] = new
                RemotePhysicsMaterialAttributes("Light", defaultDensity,
                    config.DefaultFriction, config.DefaultRestitution);
        }


        /// <summary>
        /// Returns the attributes of a material.
        /// </summary>
        /// <param name="matType">The desired material</param>
        /// <returns>Structure containing various physical attributes</returns>
        public RemotePhysicsMaterialAttributes GetAttributes(Material matType)
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

