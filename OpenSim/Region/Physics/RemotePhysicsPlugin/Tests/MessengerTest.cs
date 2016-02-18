
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading;
using System.Net;
using System.Net.Sockets;

using NUnit.Framework;
using log4net;

using OpenSim.Tests.Common;
using OpenSim.Region.Physics.RemotePhysicsPlugin;

namespace OpenSim.Region.Physics.RemotePhysicsPlugin.Tests
{
    [TestFixture]
    public class MessengerTest : OpenSimTestCase
    {
        protected uint m_testStaticID = 13;
        protected uint m_testDynamicID = 37;
        protected OpenMetaverse.Vector3 m_position = new OpenMetaverse.Vector3(5.0f, 6.0f, 7.0f);
        protected OpenMetaverse.Quaternion m_orientation = new OpenMetaverse.Quaternion(8.0f, 9.0f, 10.0f, 11.0f);
        protected OpenMetaverse.Vector3 m_linearVelocity = new OpenMetaverse.Vector3(1.0f, 2.0f, 3.0f);
        protected OpenMetaverse.Vector3 m_angularVelocity = new OpenMetaverse.Vector3(20.0f, 21.0f, 22.0f);
        protected float m_gravMod = 5.0f;

        protected bool m_staticResultsReceived = false;
        protected uint m_resultStaticID;
        protected OpenMetaverse.Vector3 m_resultPosition;
        protected OpenMetaverse.Quaternion m_resultOrientation;

        protected bool m_dynamicResultsReceived = false;
        protected uint m_resultDynamicID;
        protected OpenMetaverse.Vector3 m_resultLinearVelocity;
        protected OpenMetaverse.Vector3 m_resultAngularVelocity;

        protected bool m_testDone = false;

        [Test]
        public void StaticActorMsgTest()
        {
            IRemotePhysicsMessenger testMessenger;
            RemotePhysicsConfiguration defaultConfig;
            IRemotePhysicsPacketManager testPacketManager;
            IRemotePhysicsPacketManager testUdpPacketManager;
            Thread serverThread;

            // Indicate that the test isn't done, and the server thread should not be shut down
            m_testDone = false;

            // Create the mock server
            Console.WriteLine("Launching server thread...");
            serverThread = new Thread(new ThreadStart(MockServer));
            serverThread.Start();

            // Create the configuration that will be used for the test; the default values are sufficient
            defaultConfig = new RemotePhysicsConfiguration();

            // Create the packet manager that will be used for receiving messages
            testPacketManager = new RemotePhysicsTCPPacketManager(defaultConfig);
            testUdpPacketManager = new RemotePhysicsUDPPacketManager(defaultConfig);

            // Create the messenger that will be used to receive messages
            testMessenger = new RemotePhysicsAPPMessenger();
            testMessenger.Initialize(defaultConfig, testPacketManager,
                testUdpPacketManager);
            testMessenger.StaticActorUpdated += new UpdateStaticActorHandler(StaticUpdated);

            // Send out a static actor update
            testMessenger.SetStaticActor(m_testStaticID, m_position, m_orientation);

            // Wait for the results to be sent back
            while (!m_staticResultsReceived)
            {
                // Sleep a bit
                Thread.Sleep(2000);
            }

            // Indicate that the test is done, so that the server thread can clean up
            m_testDone = true;

            // Wait for the mock server to clean up
            Thread.Sleep(2000);

            // Compare the results
            Assert.AreEqual(m_testStaticID, m_resultStaticID);
            Assert.AreEqual(m_position.X, m_resultPosition.X);
            Assert.AreEqual(m_position.Y, m_resultPosition.Y);
            Assert.AreEqual(m_position.Z, m_resultPosition.Z);
            Assert.AreEqual(m_orientation.X, m_resultOrientation.X);
            Assert.AreEqual(m_orientation.Y, m_resultOrientation.Y);
            Assert.AreEqual(m_orientation.Z, m_resultOrientation.Z);
            Assert.AreEqual(m_orientation.W, m_resultOrientation.W);
        }

        [Test]
        public void DynamicActorMsgTest()
        {
            IRemotePhysicsMessenger testMessenger;
            RemotePhysicsConfiguration defaultConfig;
            IRemotePhysicsPacketManager testPacketManager;
            IRemotePhysicsPacketManager testUdpPacketManager;
            Thread serverThread;

            // Indicate that the test ins't done, and the server thread should not be shut down
            m_testDone = false;

            // Create the mock server
            Console.WriteLine("Launching server thread...");
            serverThread = new Thread(new ThreadStart(MockServer));
            serverThread.Start();

            // Create the configuration that will be used for the test; the default values are sufficient
            defaultConfig = new RemotePhysicsConfiguration();

            // Create the packet manager that will be used for receiving messages
            testPacketManager = new RemotePhysicsTCPPacketManager(defaultConfig);
            testUdpPacketManager = new RemotePhysicsUDPPacketManager(defaultConfig);

            // Create the messenger that will be used to receive messages
            testMessenger = new RemotePhysicsAPPMessenger();
            testMessenger.Initialize(defaultConfig, testPacketManager,
                testUdpPacketManager);
            testMessenger.DynamicActorUpdated += new UpdateDynamicActorHandler(DynamicUpdated);

            // Send out a static actor update
            testMessenger.SetDynamicActor(m_testDynamicID, m_position, m_orientation, m_gravMod, m_linearVelocity, m_angularVelocity);

            // Wait for the results to be sent back
            while (!m_dynamicResultsReceived)
            {
                // Sleep a bit
                Thread.Sleep(2000);
            }

            // Indicate that the test is done, so that the server thread can clean up
            m_testDone = true;

            // Wait for the mock server to clean up
            Thread.Sleep(2000);

            // Compare the results
            Assert.AreEqual(m_testDynamicID, m_resultDynamicID);
            Assert.AreEqual(m_position.X, m_resultPosition.X);
            Assert.AreEqual(m_position.Y, m_resultPosition.Y);
            Assert.AreEqual(m_position.Z, m_resultPosition.Z);
            Assert.AreEqual(m_orientation.X, m_resultOrientation.X);
            Assert.AreEqual(m_orientation.Y, m_resultOrientation.Y);
            Assert.AreEqual(m_orientation.Z, m_resultOrientation.Z);
            Assert.AreEqual(m_orientation.W, m_resultOrientation.W);
            Assert.AreEqual(m_linearVelocity.X, m_resultLinearVelocity.X);
            Assert.AreEqual(m_linearVelocity.Y, m_resultLinearVelocity.Y);
            Assert.AreEqual(m_linearVelocity.Z, m_resultLinearVelocity.Z);
            Assert.AreEqual(m_angularVelocity.X, m_resultAngularVelocity.X);
            Assert.AreEqual(m_angularVelocity.Y, m_resultAngularVelocity.Y);
            Assert.AreEqual(m_angularVelocity.Z, m_resultAngularVelocity.Z);
        }

        public void MockServer()
        {
            int port;
            IPAddress localHost;
            TcpListener mockServer;
            Socket clientSocket;
            byte[] incBuffer;
            int bytesRead;

            // Open up a connection on the local host with the default RemotePhysics port
            Console.WriteLine("Creating server...");
            localHost = IPAddress.Parse("127.0.0.1");
            port = 9003;

            // Create the mock server
            mockServer = new TcpListener(localHost, port);
            Console.WriteLine("Server created");

            // Start the server
            mockServer.Start();
            Console.WriteLine("Server started!");

            // Initialize the buffer that will be used to store incoming data
            incBuffer = new byte[65536];

            // Main server loop
            while (!m_testDone)
            {
                // Accept incoming connection
                clientSocket = mockServer.AcceptSocket();
                Console.WriteLine("Connected! from " + clientSocket.RemoteEndPoint);

                // Wait for data from the client
                while (!m_testDone)
                {
                    if (clientSocket.Available != 0)
                    {
                        // Read in the incoming data from the client
                        Console.WriteLine("Data available");
                        bytesRead = clientSocket.Receive(incBuffer);

                        // Send it back
                        clientSocket.Send(incBuffer, bytesRead, 0);
                    }
                    
                    // Sleep the thread for half a second, so that it doesn't hog resources
                    Thread.Sleep(500);
                }

                // Close the socket for this client
                if (clientSocket != null)
                    clientSocket.Close();
            }

            // Stop the server
            mockServer.Stop();
            Console.WriteLine("Mock server stopped");
        }

        public void StaticUpdated(uint actorID, OpenMetaverse.Vector3 position, OpenMetaverse.Quaternion orientation)
        {
            // Update the result variables
            m_resultStaticID = actorID;
            m_resultPosition = position;
            m_resultOrientation = orientation;
            m_staticResultsReceived = true;
        }

        public void DynamicUpdated(uint actorID, OpenMetaverse.Vector3 position, OpenMetaverse.Quaternion orientation, 
            OpenMetaverse.Vector3 linearVelocity, OpenMetaverse.Vector3 angularVelocity)
        {
            Console.WriteLine("Received dynamic results!");
            // Update the result variables
            m_resultDynamicID = actorID;
            m_resultPosition = position;
            m_resultOrientation = orientation;
            m_resultLinearVelocity = linearVelocity;
            m_resultAngularVelocity = angularVelocity;
            m_dynamicResultsReceived = true;
        }
    }
}
