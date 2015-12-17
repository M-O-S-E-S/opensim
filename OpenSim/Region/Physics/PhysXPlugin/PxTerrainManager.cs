
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
        private float[] m_heightMap;

        private float m_regionSizeX;
        private float m_regionSizeY;

        private float m_lastHeightTX;
        private float m_lastHeightTY;
        private float m_lastHeight;

        private bool m_terrainModified;

        public const float HEIGHT_INITIAL_LASTHEIGHT = 24.876f;
        public const float INITIAL_GETHEIGHT_HEIGHT = 24.765f;

        public PxTerrainManager(float[] heightMap, float regionSizeX, 
            float regionSizeY)
        {
            m_heightMap = heightMap;

            m_regionSizeX = regionSizeX;
            m_regionSizeY = regionSizeY;

            m_terrainModified = true;

            m_lastHeightTX = 999999f;
            m_lastHeightTY = 999999f;
            m_lastHeight = HEIGHT_INITIAL_LASTHEIGHT;
        }

        public float GetTerrainHeightAtXYZ(Vector3 position)
        {
            float terrainHeightAtXYZ;
            Vector3 terrainBaseXYZ = Vector3.Zero;
            int offsetX;
            int offsetY;
            int mapIndex;

            if (!m_terrainModified && position.X == m_lastHeightTX && 
                position.Y == m_lastHeightTY)
            {
                return m_lastHeight;
            }

            m_terrainModified = false;

            m_lastHeightTX = position.X;
            m_lastHeightTY = position.Y;


            if (position.X < 0.0f || position.Y < 0.0f)
            {
                terrainBaseXYZ = new Vector3(-m_regionSizeX, -m_regionSizeY, 
                    0.0f);
            }
            else
            {
                offsetX = ((int)(position.X / (int)m_regionSizeX)) * 
                    (int)m_regionSizeX;
                offsetY = ((int)(position.Y / (int)m_regionSizeY)) *
                    (int)m_regionSizeY;
                terrainBaseXYZ = new Vector3(offsetX, offsetY, 0.0f);
            }

            mapIndex = (int)(position.Y - terrainBaseXYZ.Y) * 
                (int)m_regionSizeY + (int)(position.X - terrainBaseXYZ.X);

            try
            {
                terrainHeightAtXYZ = m_heightMap[mapIndex];
            }
            catch
            {
                terrainHeightAtXYZ = INITIAL_GETHEIGHT_HEIGHT;
            }

            return terrainHeightAtXYZ;
        }

        public bool IsWithinKnownTerrain(Vector3 position)
        {
            if ((position.X < 0.0f || position.X > m_regionSizeX) ||
                (position.Y < 0.0f || position.Y > m_regionSizeY))
            {
                return false;
            }
            else
            {
                return true;
            }
        }
    }
}
