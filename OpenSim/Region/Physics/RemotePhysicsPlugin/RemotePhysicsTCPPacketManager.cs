
// TODO: Create banner

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
    public class RemotePhysicsTCPPacketManager : IDisposable,
        IRemotePhysicsPacketManager
    {
        /// <summary>
        /// The logger that will be used for the packet manager.
        /// </summary>
        internal static readonly ILog m_log = LogManager.GetLogger(
            System.Reflection.MethodBase.GetCurrentMethod().DeclaringType);

        /// <summary>
        /// This is the tag that will be used to deonte log messages from
        /// this class
        /// </summary>
        internal static readonly string LogHeader =
            "[REMOTE PHYSICS MESSENGER]";

        /// <summary>
        /// The queue of packets that have been received.
        /// </summary>
        protected Queue<byte[]> m_incomingPackets;

        /// <summary>
        /// The queue of packets that are ready to be sent.
        /// </summary>
        protected Queue<byte[]> m_outgoingPackets;

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
        /// The maximum message size that this packet manager will process.
        /// </summary>
        protected readonly int m_receiveBufferLength = 65536;

        /// <summary>
        /// The buffer used for receiving packets from the remote engine.
        /// </summary>
        protected byte[] m_readBuffer;

        /// <summary>
        /// Buffer used to store messages whose parts oveflow the read buffer.
        /// <summary>
        protected byte[] m_overflowBuffer;

        /// <summary>
        /// The index of the m_overflowBuffer to which new data should be
        /// written.
        /// </summary>
        protected int m_overflowWritePosition = 0;

        /// <summary>
        /// The lock used to ensure that the outgoing packet queue is
        /// thread-safe.
        /// </summary>
        protected Mutex m_outgoingMutex;

        /// <summary>
        /// The lock used to ensure that the incoming packet queue is
        /// thread-safe.
        /// </summary>
        protected Mutex m_incomingMutex;

        /// <summary>
        /// The network socket used to communicate with the remote physics
        /// engine.
        /// </summary>
        protected Socket m_remoteSocket;

        /// <summary>
        /// Synchronization event used to ensure that the packet manager's
        /// thread does not proceed until connection is complete.
        /// </summary>
        protected ManualResetEvent connectDone;

        /// <summary>
        /// Indicates whether the internal update thread should
        /// (if one is being used).
        /// </summary>
        protected bool m_stopUpdates = false;

        /// <summary>
        /// Flag used to ensure that packets are received in order.
        /// </summary>
        protected bool m_receiveReady = true;

        /// <summary>
        /// Flag used to ensure that packets are sent in order.
        /// </summary>
        protected bool m_sendReady = true;

        /// <summary>
        /// The expected size of a packet header. Used to extract headers out
        /// of the incoming data stream.
        /// </summary>
        protected uint m_headerSize;

        /// <summary>
        /// The default packet header size used, if none is specified.
        /// </summary>
        protected static readonly uint m_defaultHeaderSize = 24;

        /// <summary>
        /// The byte offset at which the packet contains the length field.
        /// Used to determine incoming packet size.
        /// </summary>
        protected uint m_packetLengthOffset;

        /// <summary>
        /// The default offset location at which the packet length field
        /// resides in bytes.
        /// </summary>
        protected static readonly uint m_defaultPacketLengthOffset = 8;
        
        /// <summary>
        /// The maximum number of outgoing packets that should be processed in
        /// one update.
        /// </summary>
        protected static int m_maxOutgoingPackets = 50;

        /// <summary>
        /// Flag indicating whether the manager has successfully connected to
        /// the remote physics engine.
        /// </summary>
        protected bool m_isConnected = false;

        /// <summary>
        /// Mutex object for ensuring that the connected flag is accessed in
        /// thread-safe manner.
        /// </summary>
        protected Object m_connectedLock = new Object();

        /// <summary>
        /// Constructor of the TCP packet manager.
        /// </summary>
        /// <param name="config">The configuration which is used to
        /// initialize the packet manager</param>
        public RemotePhysicsTCPPacketManager(RemotePhysicsConfiguration config)
        {
            IPAddress remoteAddress;
            IPEndPoint remoteEP;

            // Create the incoming/outgoing packet threads and locks
            m_incomingPackets = new Queue<byte[]>();
            m_outgoingPackets = new Queue<byte[]>();
            m_outgoingMutex = new Mutex();
            m_incomingMutex = new Mutex();

            // Configure the connection point using the configuration address
            // and port
            remoteAddress = IPAddress.Parse(config.RemoteAddress);
            remoteEP = new IPEndPoint(remoteAddress, config.RemotePort);

            // Create the socket that will allow for communication with the
            // remote physics engine
            m_remoteSocket = new Socket(AddressFamily.InterNetwork,
                SocketType.Stream, ProtocolType.Tcp);

            // Initialize the synchronization event used to block this
            // thread during the connection process
            connectDone = new ManualResetEvent(false);

            // Start the connection to the remote physics engine
            m_remoteSocket.BeginConnect(remoteEP,
                new AsyncCallback(ConnectCallback), m_remoteSocket);

            // Block the thread until the connection is complete
            connectDone.WaitOne();

            // Initialize the buffer that will be used for reading messages
            // from the remote physics engine
            m_readBuffer = new byte[m_receiveBufferLength];

            // Initialize the buffer that will be used to hold messages whose
            // data overflow the read buffer
            m_overflowBuffer = new byte[m_receiveBufferLength];

            // Check to see if the configuration states that this packet
            // manager should use its own internal thread
            m_useInternalThread = config.PacketManagerInternalThread;
            if (m_useInternalThread)
            {
                // Create thread that will send and receive messages from the
                // remote server
                m_updateThread = new Thread(new ThreadStart(RunUpdate));
                m_updateThread.Start();
            }

            // Initialize the header length and packet length offset to
            // their defaults
            m_headerSize = m_defaultHeaderSize;
            m_packetLengthOffset = m_defaultPacketLengthOffset;
        }

        /// <summary>
        /// Constructor of the TCP packet manager.
        /// </summary>
        /// <param name="remoteAddress">The address of the remote physics
        /// engine server</param>
        /// <param name="remotePort">The port used to communicate with the
        /// remote physics engine</param>
        /// <param name="useInternalThread">Indicates whether the packet
        /// manager should use its own update thread</param>
        public RemotePhysicsTCPPacketManager(string remoteAddress,
            int remotePort, bool useInternalThread)
        {
            IPAddress engineAddress;
            IPEndPoint remoteEP;

            // Create the incoming/outgoing packet threads and locks
            m_incomingPackets = new Queue<byte[]>();
            m_outgoingPackets = new Queue<byte[]>();

            // Configure the connection point using the configuration address
            // and port
            engineAddress = IPAddress.Parse(remoteAddress);
            remoteEP = new IPEndPoint(engineAddress, remotePort);

            // Create the socket that will allow for communication with the
            // remote physics engine
            m_remoteSocket = new Socket(AddressFamily.InterNetwork,
                SocketType.Stream, ProtocolType.Tcp);

            // Initialize the synchronization event used to block this thread
            // during the connection process
            connectDone = new ManualResetEvent(false);

            // Start the connection to the remote physics engine
            m_remoteSocket.BeginConnect(remoteEP,
                new AsyncCallback(ConnectCallback), m_remoteSocket);

            // Block the thread until the connection is complete
            connectDone.WaitOne();

            // Initialize the buffer that will be used for reading messages
            // from the remote physics engine
            m_readBuffer = new byte[m_receiveBufferLength];

            // Initialize the buffer that will be used to hold messages whose
            // data overflow the read buffer
            m_overflowBuffer = new byte[m_receiveBufferLength];

            // Check to see if the configuration states that this packet
            // manager should use its own internal thread
            m_useInternalThread = useInternalThread;
            if (m_useInternalThread)
            {
                // Create thread that will send and receive messages from the
                // remote server
                m_updateThread = new Thread(new ThreadStart(RunUpdate));
            }

            // Initialize the header length and packet length offset to
            // their defaults
            m_headerSize = m_defaultHeaderSize;
            m_packetLengthOffset = m_defaultPacketLengthOffset;
        }

        /// <summary>
        /// Releases resources and closes threads.
        /// </summary>
        public void Dispose()
        {
            bool isConnected;

            lock (m_connectedLock)
            {
               isConnected = m_isConnected;

               // Indicate that this manager should no longer be connected
               m_isConnected = false;
            }

            // Close the update thread, if one is being used
            if (m_useInternalThread && m_updateThread != null)
            {
                // Indicate that update thread should be stopped
                StopUpdates();

                // Wait half a second for the thread to finish executing
                m_updateThread.Join(500);
            }

            // Check to see if the packet manager still has a connection to
            // the remote physics engine
            if (isConnected)
            {
                // Close the socket
                m_remoteSocket.Shutdown(SocketShutdown.Both);
                m_remoteSocket.Close();
            }

            // Indicate that the manager should no longer be connected
            // to the remote physics engine in a thread-safe manner
            lock (m_connectedLock)
            {
                m_isConnected = false;
            }
        }

        /// <summary>
        /// Initialize basic information about packets that will be handled by
        /// the packet manager.
        /// </summary>
        /// <param name="headerSize">The size of a packet header in bytes.
        /// Must be non-zero</param>
        /// <param name="packetLengthOffset">The offset in the header
        /// (in bytes) at which the length of the packet resides.
        /// </param>
        public void InitializePacketParameters(uint headerSize,
            uint packetLengthOffset)
        {
            // Initialize the header length and packet length offset
            m_headerSize = headerSize;
            m_packetLengthOffset = packetLengthOffset;
        }

        /// <summary>
        /// Callback used to finish the connection process to the remote
        /// physics engine.
        /// </summary>
        /// <param name="remoteSocketObject">The socket object used to
        /// connect with the remote physics engine</param>
        protected void ConnectCallback(IAsyncResult remoteSocketObject)
        {
            Socket remoteSocket;
            bool connected;

            // Start off assuming that the connection operation succeeds in a
            // thread-safe manner
            lock (m_connectedLock)
            {
                m_isConnected = true;
            }

            // Obtain the socket object used to connect to the remote
            // physics engine
            remoteSocket = (Socket)remoteSocketObject.AsyncState;

            // Attempt to complete the connection operation
            try
            {
                // Signal that the connection process is complete
                remoteSocket.EndConnect(remoteSocketObject);
            }
            catch (SocketException socketExcpetion)
            {
                // Inform the user that the plugin has failed to connect to
                // the remote physics engine
                m_log.ErrorFormat("Unable to connect to " +
                   "remote physics engine.");

                // Indicate that the connection attempt failed
                lock (m_connectedLock)
                {
                    m_isConnected = false;
                }
            }

            // Check to see if the connection was successful
            lock (m_connectedLock)
            {
               connected = m_isConnected;
            }
            if (connected)
            {
                // Log the connection event
                m_log.InfoFormat("{0}: Connected to remote physics engine " +
                    "at {1}", LogHeader,
                    remoteSocket.RemoteEndPoint.ToString());
            }

            // Unblock the execution of the packet manager's thread
            connectDone.Set();
        }

        /// <summary>
        /// Callback used to finish a send operation to the remote
        /// physics engine.
        /// </summary>
        /// <param name="remoteSocketObject">The socket object used to
        /// connect with the remote physics engine</param>
        protected void SendCallback(IAsyncResult remoteSocketObject)
        {
            Socket remoteSocket;
            int bytesSent;

            // Obtain the socket object used to connect to the remote
            // physics engine
            remoteSocket = (Socket)remoteSocketObject.AsyncState;

            try
            {
                // Signal that the send operation is complete
                bytesSent = remoteSocket.EndSend(remoteSocketObject);
            }
            catch (SocketException socketExcpetion)
            {
                // Inform the user that the plugin has failed to connect to
                // the remote physics engine
                m_log.ErrorFormat("Unable to connect to " +
                   "remote physics engine.");

                // Indicate that the connection attempt failed
                lock (m_connectedLock)
                {
                    m_isConnected = false;
                }
            }

            // Indicate that the manager is ready to send the next packet
            m_sendReady = true;
        }

        /// <summary>
        /// Callback used to finish a receive operation from the remote
        /// physics engine.
        /// </summary>
        /// <param name="asyncResult">The socket object used to connect with
        /// the remote physics engine</param>
        protected void ReceiveCallback(IAsyncResult asyncResult)
        {
            Socket client;
            int bytesRead;
            byte[] newPacket;
            int packetCount;
            uint msgLen;
            byte[] tempBuf;
            bool connected;
            int readPos;
            bool doneReading;

            // Check to see if the manager is no longer connected, in a
            // thread-safe manner
            lock (m_connectedLock)
            {
                connected = m_isConnected;
            }

            // If the manager is not connected, exit out
            if (!connected)
                return;

            // Obtain the socket object used to connect to the remote
            // physics engine
            client = (Socket)asyncResult.AsyncState;
            bytesRead = 0;

            try
            {
                // Signal that the receive operation is complete and figure out
                // how many bytes were read
                bytesRead = client.EndReceive(asyncResult);
            }
            catch (SocketException socketExcpetion)
            {
                // Inform the user that the plugin has failed to connect to
                // the remote physics engine
                m_log.ErrorFormat("Unable to connect to " +
                   "remote physics engine.");

                // Indicate that the connection attempt failed
                lock (m_connectedLock)
                {
                    m_isConnected = false;
                }
            }

            // Acess the incoming stream and packet queue in a thread-safe
            // manner
            m_incomingMutex.WaitOne();

            // Check to see if there is any data in the overflow
            readPos = 0;
            if (m_overflowWritePosition > 0)
            {
                // Check to see if there isn't enough data in the overflow
                // buffer for a header
                if (m_overflowWritePosition < m_headerSize)
                {
                    // Copy enough data from the read buffer to form a header
                    // in the overflow buffer
                    Buffer.BlockCopy(m_readBuffer, (int) readPos,
                        m_overflowBuffer, m_overflowWritePosition,
                        (int) (m_headerSize - m_overflowWritePosition));

                    // Update the read position of the read buffer and write
                    // position of overflow buffer
                    readPos += (int) (m_headerSize - m_overflowWritePosition);
                    m_overflowWritePosition += (int) (m_headerSize -
                        m_overflowWritePosition);
                }

                // Check the length of packet stored in the overflow buffer
                msgLen = 0;
                msgLen = (uint) IPAddress.NetworkToHostOrder(
                    BitConverter.ToInt32(m_overflowBuffer,
                    (int) m_packetLengthOffset));
                
                // Check to see if there is enough data now to read in the
                // rest of the message
                if (m_overflowWritePosition + bytesRead >= msgLen)
                {
                    // Create a byte array to hold the new packet's data
                    newPacket = new byte[msgLen];

                    // Copy the bytes from the overflow into the packet
                    Buffer.BlockCopy(m_overflowBuffer, 0, newPacket, 0,
                        m_overflowWritePosition);

                    // Now copy the remainder of the message from the read
                    // buffer
                    Buffer.BlockCopy(m_readBuffer, (int) readPos, newPacket,
                        (int) m_overflowWritePosition,
                        (int) (msgLen - m_overflowWritePosition));

                    // Update the read position of the read buffer to account
                    // for the bytes that were read above
                    readPos += (int) (msgLen - m_overflowWritePosition);

                    // Indicate that the overflow buffer is cleared
                    m_overflowWritePosition = 0;
                }
            }

            // Check to see if enough bytes were read to process a header
            doneReading = false;
            while ((bytesRead - readPos) >= m_headerSize && !doneReading)
            {
                // Read in the length of the current packet
                // Make sure to convert the field to host byte order
                msgLen = 0;
                msgLen = (uint) IPAddress.NetworkToHostOrder(
                    BitConverter.ToInt32(m_readBuffer,
                    (int) (m_packetLengthOffset + readPos)));

                // Check to see if there is enough data in the read buffer
                // to process the entire message
                if ((bytesRead - readPos) >= msgLen)
                {
                    // Create a byte array to hold the new packet's data
                    newPacket = new byte[msgLen];

                    // Copy the bytes for the packet into the new byte array
                    Buffer.BlockCopy(m_readBuffer, (int) readPos, newPacket, 0,
                        (int) msgLen);

                    // Queue up the new packet into the packet queue, such
                    // that it can be consumed by the plugin
                    m_incomingPackets.Enqueue(newPacket);

                    // Move up the read position, now that message has been
                    // processed
                    readPos += (int) msgLen;
                }
                else
                {
                    // Check to see if all the new data has not been consumed
                    // Note that the read position is 0 indexed
                    if ((bytesRead - readPos) > 0)
                    {
                        // Store the remaining data in the overflow buffer
                        Buffer.BlockCopy(m_readBuffer, (int) readPos,
                            m_overflowBuffer, m_overflowWritePosition,
                            (int) (bytesRead - readPos));

                        // Update the amount of data stored in the overflow
                        // buffer
                        m_overflowWritePosition += (int) (bytesRead - readPos);
                    }

                    doneReading = true;
                }
            }

            // Check to see if all the new data has not been consumed and that
            // reading has not yet finished
            // Note that the read position is 0 indexed
            if ((bytesRead - readPos) > 0 && !doneReading)
            {
                // Store the remaining data in the overflow buffer
                Buffer.BlockCopy(m_readBuffer, (int) readPos, m_overflowBuffer,
                    m_overflowWritePosition, (int) (bytesRead - readPos));

                // Update the amount of data stored in the overflow buffer
                m_overflowWritePosition += (int) (bytesRead - readPos);
            }

            // Now that the data has been processed, release the lock on the
            // data
            m_incomingMutex.ReleaseMutex();

            // Indicate that the manager is ready to receive the next packet
            m_receiveReady = true;
        }

        /// <summary>
        /// Sends a packet to the remote physics engine.
        /// </summary>
        /// <param name="packet">The byte array to be sent as a packet</param>
        public void SendPacket(byte[] packet)
        {
            // Enqueue the packet into the outgoing queue in a thread-safe
            // manner, so that it can be sent in subsequent updates 
            m_outgoingMutex.WaitOne();
            m_outgoingPackets.Enqueue(packet);
            m_outgoingMutex.ReleaseMutex();
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

            if (m_incomingPackets.Count > 0)
                incomingPacket = m_incomingPackets.Dequeue();

            m_incomingMutex.ReleaseMutex();

            // Return the fetched packet (even if it is null)
            return incomingPacket;
        }

        /// <summary>
        /// Indicates whether there are any packets that have been received.
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
        /// Regularly runs the update method for this messenger
        /// </summary>
        public void RunUpdate()
        {
            // Run the update until the stop flag indicates otherwise
            while (!m_stopUpdates)
            {
                Update();

                // Sleep the thread to ensure that it doesn't hog resources
                // (maintain an update rate of apporximately 30 updates per
                // second)
                Thread.Sleep(30);
            }
        }

        /// <summary>
        /// The main update method for the packet manager. Called by the
        /// internal thread if one is used, and called by the owner of this
        /// manager if an internal thread is not used.
        /// </summary>
        public void Update()
        {
            byte[] curPacket;

            // Check to see if the packet manager has not successfully connected
            // to the remote physics engine
            if (!m_isConnected)
            {
                // No further operations can be performed, because there is
                // no connection to the remote physics engine
                return;
            }

            // Keep sending until there are no more messages to send or enough
            // of them have been sent in this cycle
            curPacket = null;
            int packetSentCount = 0;
            while (m_outgoingPackets.Count > 0 &&
                packetSentCount < m_maxOutgoingPackets)
            {
                // Indicate that the manager has already started sending data,
                // and must wait before starting any additional send
                // operations beyond this one
                m_sendReady = false;

                // Retrieve the next packet in the queue in a thread-safe manner
                m_outgoingMutex.WaitOne();
                curPacket = m_outgoingPackets.Dequeue();
                m_outgoingMutex.ReleaseMutex();

                try
                {
                    // Start the non-blocking send operation for the dequeued
                    // outgoing packet
                    m_remoteSocket.BeginSend(curPacket, 0, curPacket.Length, 0,
                        new AsyncCallback(SendCallback), m_remoteSocket);
                }
                catch (SocketException socketExcpetion)
                {
                    // Inform the user that the plugin has failed to connect to
                    // the remote physics engine
                    m_log.ErrorFormat("Unable to connect to " +
                       "remote physics engine.");
 
                    // Indicate that the connection attempt failed
                    lock (m_connectedLock)
                    {
                        m_isConnected = false;
                    }
                }

                // Update the number of sent packets
                packetSentCount++;
            }

            // Check to see if the manager is ready to receive more data
            if (m_receiveReady)
            {
                // Indicate that the manager has already started receiving data,
                // and must wait before starting any additional recieve
                // operations beyond this one
                m_receiveReady = false;

                try
                {
                    // Begin a receive operation; the operation will be
                    // completed in the callback that is passed in
                    m_remoteSocket.BeginReceive(m_readBuffer, 0,
                        m_readBuffer.Length, 0,
                        new AsyncCallback(ReceiveCallback), m_remoteSocket);
                }
                catch (SocketException socketExcpetion)
                {
                    // Inform the user that the plugin has failed to connect to
                    // the remote physics engine
                    m_log.ErrorFormat("Unable to connect to " +
                       "remote physics engine.");
 
                    // Indicate that the connection attempt failed
                    lock (m_connectedLock)
                    {
                        m_isConnected = false;
                    }
                }
            }
        }

        /// <summary>
        /// Stops the update thread, if one is being by the messenger.
        /// </summary>
        protected void StopUpdates()
        {
            // Set the flag indicating that the update thread should stop
            m_stopUpdates = true;
        }
    }
}
