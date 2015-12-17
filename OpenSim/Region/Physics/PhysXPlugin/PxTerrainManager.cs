
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

using OpenMetaverse;

namespace OpenSim.Region.Physics.PhysXPlugin
{
    public class PxTerrainManager
    {
        /// <summary> 
        /// The height map that PhysX is currently using to simulate the
        /// terrain.
        /// </summary>
        private float[] m_heightMap;

        /// <summary>
        /// The current region size in the X coordinate.
        /// </summary>
        private float m_regionSizeX;
        /// <summary>
        /// The current region size in the Y coordinate.
        /// </summary>
        private float m_regionSizeY;

        /// <summary>
        /// The last value in the X coordinate that a physical object requested
        /// the terrain height for.
        /// </summary>
        private float m_lastHeightTX;
        /// <summary>
        /// The last value in the Y coordinate that a physical object requested
        /// the terrain height for.
        /// </summary>
        private float m_lastHeightTY;
        /// <summary>
        /// The last height returned for the lastHeightTX and TY.
        /// </summary>
        private float m_lastHeight;

        /// <summary>
        /// This tracks whether the lastHeight value is still valid by letting
        /// the terrain manager know when the terrain has been modified.
        /// </summary>
        private bool m_terrainModified;

        /// <summary>
        /// The initial height used for the last height instance field.
        /// </summary>
        public const float HEIGHT_INITIAL_LASTHEIGHT = 24.876f;
        /// <summary>
        /// Initial height used when looking for the height of the terrain at a
        /// given location. This value will be returned if something goes wrong
        /// with looking up the actual height of the terrain.
        /// </summary>
        public const float INITIAL_GETHEIGHT_HEIGHT = 24.765f;

        /// <summary>
        /// Constructor.
        /// </summary>
        /// <param name="heightMap"> The new height map that PhysX will be
        /// using to simulate the terrain</param>
        /// <param name="regionSizeX"> The current size of the region for the X
        /// coordinate</param>
        /// <param name="regionSizeY"> The current size of the region for the Y
        /// coordinate</param>
        public PxTerrainManager(float[] heightMap, float regionSizeX, 
            float regionSizeY)
        {
            // Store the current height map so that the height can be looked up
            // for a given position
            m_heightMap = heightMap;

            // Store the region sizes as these will be used to determine the
            // legitimacy of the location needed and to help find the location
            // inside of the array
            m_regionSizeX = regionSizeX;
            m_regionSizeY = regionSizeY;

            // Since a new height field has been added the terrain has been
            // modified so set it to true
            m_terrainModified = true;

            // Store the initial values for the last position requested and the
            // last height returned
            m_lastHeightTX = 999999f;
            m_lastHeightTY = 999999f;
            m_lastHeight = HEIGHT_INITIAL_LASTHEIGHT;
        }

        /// <summary>
        /// Determines the height of the terrain at a given position.
        /// </summary>
        public float GetTerrainHeightAtXYZ(Vector3 position)
        {
            float terrainHeightAtXYZ;
            Vector3 terrainBaseXYZ = Vector3.Zero;
            int offsetX;
            int offsetY;
            int mapIndex;

            // Check that the terrain has not changed and that the position
            // requested is the same as last time, according to the bullet
            // plugin repetitive requests to this method happen a lot and this
            // is an optimization for that
            if (!m_terrainModified && position.X == m_lastHeightTX && 
                position.Y == m_lastHeightTY)
            {
                // Return the same height as last time since the position
                // hasn't changed and the terrain hasn't changed
                return m_lastHeight;
            }

            // The terrain is now considered to be the same until the next time
            // this class is constructed
            m_terrainModified = false;

            // Store the current position in case a duplicate request is
            // received
            m_lastHeightTX = position.X;
            m_lastHeightTY = position.Y;

            // Check that the positions exist within the terrain bounds
            if (position.X < 0.0f || position.Y < 0.0f)
            {
                // Create a fake value for the base vector that will force the
                // use of the default terrain height
                terrainBaseXYZ = new Vector3(-m_regionSizeX, -m_regionSizeY, 
                    0.0f);
            }
            else
            {
                // Use the position to find the offset value of the terrain
                offsetX = ((int)(position.X / (int)m_regionSizeX)) * 
                    (int)m_regionSizeX;
                offsetY = ((int)(position.Y / (int)m_regionSizeY)) *
                    (int)m_regionSizeY;
                terrainBaseXYZ = new Vector3(offsetX, offsetY, 0.0f);
            }

            // Determine the location inside the height map that equals the
            // position that was provided
            mapIndex = (int)(position.Y - terrainBaseXYZ.Y) * 
                (int)m_regionSizeY + (int)(position.X - terrainBaseXYZ.X);

            try
            {
                // Look in the height map for the correct terrain height
                terrainHeightAtXYZ = m_heightMap[mapIndex];
            }
            catch
            {
                // The correct terrain height was not found so use the default
                // terrain height
                terrainHeightAtXYZ = INITIAL_GETHEIGHT_HEIGHT;
            }

            // Return the terrain height that was found for this position
            return terrainHeightAtXYZ;
        }

        /// <summary>
        /// Determines whether the position is within the region bounds.
        /// </summary>
        /// <param name="position"> The position that is being checked against
        /// the terrain bounds</param>
        public bool IsWithinKnownTerrain(Vector3 position)
        {
            // Check the position against the bounds of the region
            if ((position.X < 0.0f || position.X > m_regionSizeX) ||
                (position.Y < 0.0f || position.Y > m_regionSizeY))
            {
                // The position was outside the bounds of the region so return
                // false
                return false;
            }
            else
            {
                // Since the position was not outside of the region it must be
                // inside of the region so return true
                return true;
            }
        }
    }
}
