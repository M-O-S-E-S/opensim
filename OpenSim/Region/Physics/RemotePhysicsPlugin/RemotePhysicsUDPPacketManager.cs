
using System;
using System.IO;
using System.Collections.Generic;
using System.Threading;
using System.Text;
using System.Net;
using System.Net.Sockets;
using OpenSim.Framework;
using log4net;


namespace OpenSim.Region.Physics.RemotePhysicsPlugin
{
    public class RemotePhysicsUDPPacketManager : IDisposable,
        IRemotePhysicsPacketManager
    {
        /// <summary>
        /// The logger that will be used for the packet manager.
        /// </summary>
        internal static readonly ILog m_log = LogManager.GetLogger(
            System.Reflection.MethodBase.GetCurrentMethod().DeclaringType);

        /// <summary>
        /// The tag that will be used to denote log messages from this class.
        /// </summary>
        internal static readonly string LogHeader =
            "[REMOTE PHYSICS UDP PACKET MANAGER]";

        /// <summary>
        /// The queue of packets that have been received.
        /// </summary>
        protected Queue<byte[]> m_incomingPackets = new Queue<byte[]>();

        /// <summary>
        /// The queue of packets that are ready to be sent.
        /// </summary>
        protected Queue<byte[]> m_outgoingPackets = new Queue<byte[]>();

        /// <summary>
        /// The lock used to ensure that the incoming packet queue is
        /// thread-safe.
        /// </summary>
        protected Mutex m_incomingMutex = new Mutex();

        /// <summary>
        /// The lock used to ensure that the outgoing packet queue is
        /// thread-safe.
        /// </summary>
        protected Mutex m_outgoingMutex = new Mutex();

        /// <summary>
        /// The maximum message size that this packet manager will process.
        /// </summary>
        protected static readonly int m_receiveBufferLength = 65536;

        /// <summary>
        /// The ubffer used for receiving packets from the remote engine.
        /// </summary>
        protected byte[] m_readBuffer = new byte[m_receiveBufferLength];

        /// <summary>
        /// The UDP client used to communicate with the remote physics engine.
        /// </summary>
        protected UdpClient m_udpClient = null;

        /// <summary>
        /// The remote host to which the UDP client will be connecting.
        /// </summary>
        protected IPEndPoint m_remoteHost = null;

        /// <summary>
        /// Indicates whether the internal update thread should stop
        /// (if an internal thread is being used).
        /// </summary>
        protected bool m_stopUpdates = false;

        /// <summary>
        /// The expected size of a message header. Used to extract headers
        /// from incoming packets. The default value is 24 bytes.
        /// </summary>
        protected uint m_headerSize = 24;

        /// <summary>
        /// The byte offset at which the packet is expected to contain the
        /// message length field. Used to determine the size of incoming
        /// messages. The default value is 8 bytes.
        /// </summary>
        protected uint m_messageLengthOffset = 8;

        /// <summary>
        /// Indicates whether this packet manager should use its own internal
        /// thread for updating.
        /// </summary>
        protected bool m_useInternalThread;

        /// <summary>
        /// The internal thread used for updating this packet manager
        /// (if it is enabled).
        /// </summary>
        protected Thread m_updateThread;

        /// <summary>
        /// Flag that indicates that the previous receive operation has
        /// completed and the manager is ready to start a new receive operation.
        /// </summary>
        protected bool m_receiveReady = true;

        /// <summary>
        /// Constructor of the UDP packet manager.
        /// </summary>
        public RemotePhysicsUDPPacketManager(RemotePhysicsConfiguration config)
        {
            IPAddress remoteAddress;

            // Configure the connection point using configuration address
            // and port
            remoteAddress = IPAddress.Parse(config.RemoteAddress);
            m_remoteHost = new IPEndPoint(remoteAddress, config.RemotePort);

            // Create the UDP client that will be used to communicate with the
            // remote physics engine
            m_udpClient = new UdpClient();

            // Check to see if the configuration states whether this packet
            // manager should use its own internal update thread
            m_useInternalThread = config.PacketManagerInternalThread;
            if (m_useInternalThread)
            {
               // Create the thread that will send and receive messages from
               // the remote physics engine
               m_updateThread = new Thread(new ThreadStart(RunUpdate));
               m_updateThread.Priority = ThreadPriority.AboveNormal;
               m_updateThread.Start();
            }
        }

        /// <summary>
        /// Constructor of the UDP packet manager.
        /// </summary>
        public RemotePhysicsUDPPacketManager(string remoteAddress,
            int remotePort, bool useInternalThread)
        {
            IPAddress engineAddress;

            // Configure the connection point using the given address and port
            engineAddress = IPAddress.Parse(remoteAddress);
            m_remoteHost = new IPEndPoint(engineAddress, remotePort);

            // Create the UDP client that will be used to communicate with the
            // remote physics engine
            m_udpClient = new UdpClient();

            // Check to see if the configuration states whether this packet
            // manager should use its own internal update thread
            m_useInternalThread = useInternalThread;
            if (m_useInternalThread)
            {
               // Create the thread that will send and receive messages from
               // the remote physics engine
               m_updateThread = new Thread(new ThreadStart(RunUpdate));
               m_updateThread.Start();
            }

        }

        /// <summary>
        /// Releases resources and closes threads.
        /// </summary>
        public void Dispose()
        {
            // Close the update thread, if one is being used
            if (m_useInternalThread && m_updateThread != null)
            {
                // Indicate that the update thread should be stopped
                StopUpdates();

                // Wait half a second for the thread to finish executing and
                // stop it
                m_updateThread.Join(500);
            }
        }

        /// <summary>
        /// Initializes basic information about the packets that will be
        /// handled by the packet manager.
        /// </summary>
        /// <param name="headerSize">The size of the packet headers in bytes.
        /// Must be non-zero</param>
        /// <param name="messageLengthOffset">The offset in the header
        /// (in bytes) at which the length of the message resides</param>
        public void InitializePacketParameters(uint headerSize,
            uint messageLengthOffset)
        {
            // Initialize the header length and message length offset
            m_headerSize = headerSize;
            m_messageLengthOffset = messageLengthOffset;
        }

        /// <summary>   
        /// Sends a packet to the remote physics engine.
        /// </summary>
        /// <param name="packet">The byte array to be sent as a packet</param>
        public void SendPacket(byte[] packet)
        {
            // Attempt to send the packet to the remote physics engine
            try
            {
                // Start the non-blocking send operation for the dequeued
                // outgoing packet
                m_udpClient.BeginSend(packet, packet.Length,
                    m_remoteHost,
                    new AsyncCallback(SendCallback), m_udpClient);
            }
            catch (SocketException socketException)
            {
                // Inform the user that the plugin has failed to establish
                // an UDP connection to the remote physics engine
                m_log.ErrorFormat("{0}: Unable to establish UDP " +
                    "connection to remote physics engine.", LogHeader);
            }
        }

        /// <summary>
        /// Fetch the next received packet.
        /// </summary>
        /// <returns>The next packet in the incoming packet. Null if there are
        /// no incoming packets</returns>
        public byte[] GetIncomingPacket()
        {
            byte[] incomingPacket;

            // Dequeue a packet from the incoming packet queue and return it
            // in a thread-safe manner
            incomingPacket = null;
            m_incomingMutex.WaitOne();

            // If there is a packet available, fetch it
            if (m_incomingPackets.Count > 0)
                incomingPacket = m_incomingPackets.Dequeue();

            // Now that the packet has been retrieved, release the mutex
            m_incomingMutex.ReleaseMutex();

            // Return the fetched packet (even if it is null)
            return incomingPacket;
        }

        /// <summary>
        /// Indicates whether there any packets that have been received.
        /// </summary>
        /// <returns>True if there are unprocessed packets that have been
        /// received; false if not</returns>
        public bool HasIncomingPacket()
        {
            // Check to see if the incoming packet queue has any packets
            // waiting to be read
            return (m_incomingPackets.Count > 0);
        }

        /// <summary>
        /// Regularly runs the update method for this packet manager
        /// (if the packet manager is using its own internal thread).
        /// </summary>
        public void RunUpdate()
        {
            // Run the update method until the stop flag indicates otherwise
            while (!m_stopUpdates)
            {
                Update();
            }
        }

        /// <summary>
        /// The main update method for the packet manager. Called by the
        /// internal thread if one is used, and called by the owner of this
        /// manager if an internal thread is not used.
        /// </summary>
        public void Update()
        {
            byte[] currPacket;

            // Attempt to read in data from the remote physics engine
            try
            {
                 currPacket = m_udpClient.Receive(ref m_remoteHost);

                 if (currPacket != null)
                 {
                     // Add the new packet to the incoming queue in a
                     // thread-safe manner
                     m_incomingMutex.WaitOne();
                     m_incomingPackets.Enqueue(currPacket);
                     m_incomingMutex.ReleaseMutex();
                 }
            }
            catch (SocketException socketException)
            {
                // Inform the user that the plugin has failed to establish
                // an UDP connection to the remote physics engine
                m_log.ErrorFormat("{0}: Unable to establish UDP " +
                    "connection to remote physics engine.", LogHeader);
            }
        }

        /// <summary>
        /// Callback used to finish a send operation to the remote physics
        /// engine.
        /// </summary>
        /// <param name="udpClient">The UDP client object used to communicate
        /// with the remote physics engine</param>
        protected void SendCallback(IAsyncResult udpClient)
        {
            UdpClient client;

            // Convert the given temporary object into the udp client
            client = (UdpClient) udpClient.AsyncState;

            // Attempt to finish the send operation
            try
            {
                // Signal that the send operation is complete
                client.EndSend(udpClient);
            }
            catch (SocketException socketException)
            {
                // Inform the user that the plugin has failed to communicate
                // with the remote physics engine
                m_log.ErrorFormat("{0}: Unable to send data to remote " +
                    "physics engine over UDP.", LogHeader);
            }
        }

        /// <summary>
        /// Callback used to finish a receive operation from the remote physics
        /// engine (if asynchronous receive operations are used).
        /// </summary>
        /// <param name="udpClient">The UDP client object used to communicate
        /// with the remote physics engine</param>
        protected void ReceiveCallback(IAsyncResult udpClient)
        {
            UdpClient client;
            byte[] newPacket;

            // Convert the given temporary objcet int othe udp client
            client = (UdpClient) udpClient.AsyncState;

            // Attempt to finsh the read operation
            try
            {
                // Finish the read operation, and fetch how many bytes were
                // read
                newPacket = client.EndReceive(udpClient, ref m_remoteHost);

                if (newPacket != null)
                {
                    // Add the new packet to the incoming queue in a thread-safe
                    // manner
                    m_incomingMutex.WaitOne();
                    m_incomingPackets.Enqueue(newPacket);
                    m_incomingMutex.ReleaseMutex();
                }
            }
            catch (SocketException socketException)
            {
                // Inform the user that the plugin has failed to communicate
                // with the remote physics engine
                m_log.ErrorFormat("{0}: Unable to receive data from remote " +
                    "physics engine over UDP.", LogHeader);
            }

            m_receiveReady = true;
        }

        /// <summary>
        /// Stops the update thread, if one is being used by the packet manager.
        /// </summary>
        protected void StopUpdates()
        {
            // Set the flag indicating that the update thread should stop
            m_stopUpdates = true;
        }
    }
}
