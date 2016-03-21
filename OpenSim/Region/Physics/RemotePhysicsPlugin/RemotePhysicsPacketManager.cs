
// TODO: Create banner

using System;

namespace OpenSim.Region.Physics.RemotePhysicsPlugin
{
    /// <summary>
    /// The interface that defines the functionality for packet managers that
    /// are used to connect to and communicate with the remote physics engine.
    /// </summary>
    public interface IRemotePhysicsPacketManager
    {
        /// <summary>
        /// Initialize basic information about packets that will be handled
        /// by the packet manager.
        /// </summary>
        /// <param name="headerSize">The size of a packet header in bytes.
        /// Must be non-zero</param>
        /// <param name="packetLengthOffset">The offset in the header
        /// (in bytes) at which the length of the packet resides.
        /// </param>
        void InitializePacketParameters(uint headerSize, uint packetLengthOffset);

        /// <summary>
        /// Sends a packet of data to the remote physics engine.
        /// </summary>
        /// <param name="packetData">The data to be sent to the remote
        /// physics engine</param>
        void SendPacket(Byte[] packetData);

        /// <summary>
        /// Indicates whether there is a packet(s) from the remote physics
        /// engine waiting to be processed.
        /// </summary>
        /// <returns>A boolean indicating whether there is a packet(s) to be
        /// processed; false if there are not packets</returns>
        bool HasIncomingPacket();

        /// <summary>
        /// Fetches the next packet from the remote physics engine that is
        /// waiting to be processed.
        /// </summary>
        /// <returns>The next packet to be processed; will be null if
        /// there are no packets</returns>
        Byte[] GetIncomingPacket();

        /// <summary>
        /// The update method that synchronizes the packet manager. Will need
        /// to be manually called if the packet manager is not using an
        /// internal update mechanism.
        /// </summary>
        void Update();
    }
}
