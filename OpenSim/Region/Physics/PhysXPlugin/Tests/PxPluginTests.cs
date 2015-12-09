using System;
using System.Collections.Generic;
using System.IO;
using System.Text;
using NUnit.Framework;
using OpenMetaverse;
using OpenSim.Tests.Common;
using OpenSim.Region.Physics.PhysXPlugin;

namespace OpenSim.Region.Physics.PhysXPlugin.Tests
{
    [TestFixture]
    public class PxPluginTests : OpenSimTestCase
    {
        /// <summary>
        /// Tests the initialize function to validate it returns true.
        /// </summary>
        [Test]
        public void TestInit()
        {
            // Allows debug to find what method we are in
            TestHelpers.InMethod();

            PxPlugin plugin = new PxPlugin();

            Assert.That(plugin.Init() == true);
        }
    }
}
