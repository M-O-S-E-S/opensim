
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
using System.Runtime.InteropServices;
using System.Text;

using OpenMetaverse;

namespace OpenSim.Region.Physics
{
    /// <summary>
    /// Structure that will be shared with the PhysX unmanaged code to allow
    /// the transfer of data through pinned memory to prevent issues with
    /// updating the physical object parameters after the collision updates
    /// have occurred.
    /// </summary>
    [StructLayout (LayoutKind.Sequential)]
    public struct EntityProperties
    {
        public UInt32 ID;

        public float PositionX;
        public float PositionY;
        public float PositionZ;

        public float RotationX;
        public float RotationY;
        public float RotationZ;
        public float RotationW;

        public float VelocityX;
        public float VelocityY;
        public float VelocityZ;

        public float AngularVelocityX;
        public float AngularVelocityY;
        public float AngularVelocityZ;

        public override string ToString()
        {
            // Convert to string buffer and then return the string
            StringBuilder buff = new StringBuilder();
            buff.Append("<id = ");
            buff.Append(ID.ToString());
            buff.Append(", pos = ");
            buff.Append("<" + Convert.ToString(PositionX) + ", ");
            buff.Append(Convert.ToString(PositionY) + ", ");
            buff.Append(Convert.ToString(PositionZ) + ">");
            buff.Append(", rot = ");
            buff.Append("<" + Convert.ToString(RotationX) + ", ");
            buff.Append(Convert.ToString(RotationY) + ", ");
            buff.Append(Convert.ToString(RotationZ) + ", ");
            buff.Append(Convert.ToString(RotationW) + ">");
            buff.Append(", vel = ");
            buff.Append("<" + Convert.ToString(VelocityX) + ", ");
            buff.Append(Convert.ToString(VelocityY) + ", ");
            buff.Append(Convert.ToString(VelocityZ) + ">");
            buff.Append(", ang vel = ");
            buff.Append("<" + Convert.ToString(AngularVelocityX) + ", ");
            buff.Append(Convert.ToString(AngularVelocityY) + ", ");
            buff.Append(Convert.ToString(AngularVelocityZ) + ">");
            buff.Append(">");
            return buff.ToString();
        }
    }

    /// <summary>
    /// Structure that will be shared with the PhysX unmanaged code to allow
    /// the transfer of data through pinned memory to prevent issues with
    /// updating the physical collision parameters after the collision updates
    /// have occurred.
    /// </summary>
    [StructLayout (LayoutKind.Sequential)]
    public struct CollisionProperties
    {
        public uint ActorId1;
        public uint ActorId2;

        public float PositionX;
        public float PositionY;
        public float PositionZ;

        public float NormalX;
        public float NormalY;
        public float NormalZ;

        public float Penetration;

        public override string ToString()
        {
            // Convert to string buffer and then return the string
            StringBuilder buff = new StringBuilder();
            return buff.ToString();
        }
    }
}

