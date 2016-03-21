
// TODO: Create a banner

using System;

using OpenSim.Framework;
using OpenSim.Region.Physics.Manager;

namespace OpenSim.Region.Physics.RemotePhysicsPlugin
{
    public class RemotePhysicsPlugin : IPhysicsPlugin
    {
        /// <summary>
        /// The physics scene held by this plugin.
        /// </summary>
        protected RemotePhysicsScene m_remoteScene;

        /// <summary>
        /// Constructor.
        /// </summary>
        public RemotePhysicsPlugin()
        {
            // Nothing to initialize here
        }

        /// <summary>
        /// Initializes the plugin.
        /// </summary>
        /// <returns>Whether the plugin was successfully initialized</returns>
        public bool Init()
        {
            // Nothing to initialize here
            return true;
        }

        /// <summary>
        /// Gets the physics scene held by this plugin.
        /// </summary>
        /// <param name="sceneIdentifier">The name of the scene</param>
        /// <returns>The physics scene</returns>
        public PhysicsScene GetScene(String sceneIdentifier)
        {
            // Check to see if the scene has been created
            if (m_remoteScene == null)
            {
                // Create the remote scene
                m_remoteScene = new RemotePhysicsScene(sceneIdentifier);
            }

            // Return the physics scene held by this plugin
            return m_remoteScene;
        }

        /// <summary>
        /// Returns the name of the plugin.
        /// </summary>
        /// <returns>The name of the plugin</returns>
        public string GetName()
        {
            return ("RemotePhysics");
        }

        /// <summary>
        /// Clean up resources held by the plugin.
        /// </summary>
        public void Dispose()
        {
            // Nothing to clean up here
        }
    }
}
