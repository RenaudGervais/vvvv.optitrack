#region usings
using System;
using System.ComponentModel.Composition;
using System.Collections;
using System.Collections.Generic;

using VVVV.PluginInterfaces.V1;
using VVVV.PluginInterfaces.V2;
using VVVV.Utils.VColor;
using VVVV.Utils.VMath;

using VVVV.Core.Logging;

using NatNetML;
#endregion usings

namespace VVVV.Optitrack
{
	public class MathTools{
		

        // Convert a quaternion (TrackingTools type) to euler angles (Y, Z, X)
        // Y = Heading  (Yaw)
        // Z = Attitude (Pitch)
        // X = Bank     (Roll)
        // From Martin Baker (http://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToEuler/index.htm)
        // conventions:
        //  - input and output units are both in radians
        //  - euler order is YZX
        //  - euler angles are about global axes
        //  - euler + angle is right-handed
        public static void QuaternionToEuler(double qx, double qy, double qz, double qw, out double y, out double z, out double x)
        {
            double test = qx * qy + qz * qw;
            if (test > 0.499)                            // singularity at north pole
            {
                y = 2.0F * System.Math.Atan2(qx, qw);
                z = Math.PI / 2.0F;
                x = 0.0F;
                return;
            }

            if (test < -0.499)                           // singularity at south pole
            {
                y = -2.0F * System.Math.Atan2(qx, qw);
                z = -Math.PI / 2.0F;
                x = 0.0F;
                return;
            }

            double sqx = qx * qx;
            double sqy = qy * qy;
            double sqz = qz * qz;
            y = System.Math.Atan2(2.0F * qy * qw - 2.0 * qx * qz, 1.0F - 2.0 * sqy - 2.0 * sqz);
            z = System.Math.Asin(2.0F * test);
            x = System.Math.Atan2(2.0 * qx * qw - 2.0 * qy * qz, 1.0F - 2.0 * sqx - 2.0 * sqz);
        }


        public static double  RadiansToDegrees(double dRads)
        {
            return dRads * (180.0f / Math.PI);
        }
		
		
		// normalising a quaternion works similar to a vector. This method will not do anything
        // if the quaternion is close enough to being unit-length. define TOLERANCE as something
        // small like 0.00001f to get accurate results
        public static void QuaternionNormalise(ref float x, ref float y, ref float z, ref float w)
        {
	        // Don't normalize if we don't have to
            float tolerance = 0.00001f;
            float mag2 = w * w + x * x + y * y + z * z;
            if (mag2 != 0.0f && (Math.Abs(mag2 - 1.0f) > tolerance))
            {
                float mag = (float)Math.Sqrt(mag2);
                w /= mag;
                x /= mag;
                y /= mag;
                z /= mag;
            }
	    }
		public static Matrix4x4 ToVVVVMatrix(Vector3D position, Vector3D orientation){
			
			Matrix4x4 mat =  VMath.IdentityMatrix;
			Matrix4x4 scale = VMath.Scale(-1,1,1);
			Matrix4x4 rotate = VMath.Rotate(orientation.x, orientation.y, orientation.z);
			Matrix4x4 translate = VMath.Translate(position.x,position.y, position.z);
			return scale*rotate*translate*scale;
		}
	}
	
	
	#region PluginInfo
	[PluginInfo(Name = "Device (OptiTrack)", Category = "Devices", Help = "Get information from OptiTrack camera through NatNet protocol", Tags = "tracking, OptiTrack, NatNet")]
	#endregion PluginInfo
	public class DeviceOptiTrackNode : IPluginEvaluate
	{
		#region fields & pins
		[Input("Local", DefaultString = "192.168.0.1", IsSingle = true)]
		public ISpread<String> FLocal;
		
		[Input("Server", DefaultString = "192.168.1.1", IsSingle = true)]
		public ISpread<String> FServer;
		
		// Use multicast by default
		// WARNING: will use default address "239.255.42.99" because of the poor API in the managed dll
		// FIXME: of no use at the moment ; data comes from multicast only
	    //[Input("EnableMulticast", DefaultValue = 0, IsSingle = true)]
		//public ISpread<bool> FEnableMulticast;
		
		// Default command port for retrieving list of rigid bodies
		// NB: data stream on the other could not be configured...
		[Input("Port", DefaultValue = 1510, IsSingle = true)]
		public ISpread<int> FPort;

        [Input("Update", DefaultBoolean = false, IsSingle = true, IsBang = true)]
        public IDiffSpread<bool> FUpdate;
		
		[Input("Enabled", DefaultValue = 0, IsSingle = true)]
		public ISpread<bool> FEnabled;

        [Output("Rigid body name")]
        public ISpread<String> FRigidBody;

		[Output("Frame")]
        public ISpread<NatNetML.FrameOfMocapData> FFrame;
		
		[Output("Connected", DefaultValue = 0, IsSingle = true)]
		public ISpread<bool> FConnected;

		
		[Import()]
		public ILogger FLogger;
		#endregion fields & pins
		
		
		//Private member
		// NatNet object
		private NatNetML.NatNetClientML mNatNet;
		
		// NatNet Frame of Data
		private NatNetML.FrameOfMocapData mFrameOfData;

        private static object syncLock = new object();
        private Queue<NatNetML.FrameOfMocapData> m_FrameQueue;
		private NatNetML.ServerDescription mDesc;
		
		//List containing the names of all the tracked rigid bodies
		private List<String> mRBNames = new List<String>();
		
		
		//Constructor
		public DeviceOptiTrackNode() {
			//The client argument:
			//	0: Multicast
			//	1: Unicast
			mNatNet = new NatNetClientML(0);
			
			mFrameOfData = new NatNetML.FrameOfMocapData();
			m_FrameQueue = new Queue<NatNetML.FrameOfMocapData>();
			mDesc = new NatNetML.ServerDescription();
			
			// [NatNet] set a "Frame Ready" callback function (event handler) handler that will be
			// called by NatNet when NatNet receives a frame of data from the server application
			mNatNet.OnFrameReady += new NatNetML.FrameReadyEventHandler(m_NatNet_OnFrameReady);
			//mNatNet.OnFrameReady2 += new FrameReadyEventHandler2(m_NatNet_OnFrameReady2);
		}
		

		//called when data for any output pin is requested
		public void Evaluate(int SpreadMax)
		{
            int returnCode = 0;
            if(FEnabled[0]) {

                //If not connected already, try to connect
                if(!FConnected[0]) {

                    // [NatNet] connect to a NatNet server
                    returnCode = mNatNet.Initialize(FLocal[0], FServer[0], FPort[0]);
                    FLogger.Log(LogType.Debug, "Trying to connect...");
                	
                	if(returnCode == 0) {
                		FLogger.Log(LogType.Debug, "Connected!");

                		// [NatNet] validate the connection
            			returnCode = mNatNet.GetServerDescription(mDesc);
                		
                		if(returnCode == 0) {
                			FLogger.Log(LogType.Debug, "Server App Name: " + mDesc.HostApp);
                			
                			
                			// [NatNet] [optional] send a test/response message
			                FLogger.Log(LogType.Debug, "Sending TestRequest");
			                int nBytes = 0;
			                byte[] response = new byte[10000];
			                int rc = mNatNet.SendMessageAndWait("TestRequest", out response, out nBytes);
			                if (rc == 0)
			                {
			                    string str = "   Server: " + System.Text.Encoding.ASCII.GetString(response, 0, nBytes);
			                    FLogger.Log(LogType.Debug, str);
			                }
			                else
			                {
			                    FLogger.Log(LogType.Debug, "   Server: No Response.");
			                }
			
			                rc = mNatNet.SendMessageAndWait("FrameRate", out response, out nBytes);
			                if (rc == 0)
			                {
			                    try
			                    {
			                        double serverFramerate = BitConverter.ToSingle(response, 0);
			                        FLogger.Log(LogType.Debug, String.Format("   Server Framerate: {0}", serverFramerate));
			                    }
			                    catch (System.Exception ex)
			                    {
			                        FLogger.Log(LogType.Debug, ex.Message);
			                    }
			                }
                			
   
                			//Connected
                			FConnected[0] = true;
                    		FEnabled[0] = FConnected[0];
                		}
                	}

                    //Get the rigid body names
                    this.RequestDataDescription();
                } else {
                    // Already connected, update output

                    // Upon update, query server for data description
                    if (FUpdate.IsChanged)
                        this.RequestDataDescription();
                	lock(syncLock) {
                		//FLogger.Log(LogType.Debug, "" + m_FrameQueue.Count);
                		while(m_FrameQueue.Count > 0) {
                			mFrameOfData = m_FrameQueue.Dequeue();
                			
                			if(m_FrameQueue.Count > 0)
                				continue;
                			FFrame.SliceCount = 0;
                			FFrame.Add(mFrameOfData);
            			
            			}
            		}
                	
                	
                	// Update the rigid bodies names
                	FRigidBody.SliceCount = mRBNames.Count;
                	for(int i = 0; i < mRBNames.Count; ++i)
                		FRigidBody[i] = mRBNames[i];
                }
            } else {
                //If connected already, disconnect, else, do nothing
                if(FConnected[0]) {
                    mRBNames.Clear();
                    returnCode = mNatNet.Uninitialize();
                    FConnected[0] = (0 == returnCode);
                }
            }
		}


        
        private void RequestDataDescription()
        {
            //Clear previous rigid bodies records
            mRBNames.Clear();

            // [NatNet] request data descriptions from server app.
            List<NatNetML.DataDescriptor> descs = new List<NatNetML.DataDescriptor>();
            bool bSuccess = mNatNet.GetDataDescriptions(out descs);
            if (bSuccess)
            {
                foreach (NatNetML.DataDescriptor d in descs)
                {
                    // RigidBodies
                    if (d.type == (int)NatNetML.DataDescriptorType.eRigidbodyData)
                    {
                        NatNetML.RigidBody rb = (NatNetML.RigidBody)d;
                        //FLogger.Log(LogType.Debug, "RigidBody: " + rb.Name);
                        mRBNames.Add(rb.Name);
                    }
                }
            }
        }

		
		// [NatNet] m_NatNet_OnFrameReady will be called when a frame of Mocap
        // data has is received from the server application.
        //
        // Note: This callback is on the network service thread, so it is
        // important  to return from this function quickly as possible 
        // to prevent incoming frames of data from buffering up on the
        // network socket.
        //
        // Note: "data" is a reference structure to the current frame of data.
        // NatNet re-uses this same instance for each incoming frame, so it should
        // not be kept (the values contained in "data" will become replaced after
        // this callback function has exited).
        void m_NatNet_OnFrameReady(NatNetML.FrameOfMocapData data, NatNetML.NatNetClientML client)
        {
            lock (syncLock)
            {
                m_FrameQueue.Clear();
                m_FrameQueue.Enqueue(data);
            }
        }
		
		
		// [NatNet] m_NatNet_OnFrameReady2 will be called when a frame of Mocap
        // data has is received from the server application.
        void m_NatNet_OnFrameReady2(object sender, NatNetEventArgs e)
        {
            m_NatNet_OnFrameReady(e.data, e.client);
        }

	}
	
	
	
	#region PluginInfo
	[PluginInfo(Name = "RigidBodies (OptiTrack)", Category = "Devices", Help = "Get information from OptiTrack camera through NatNet protocol", Tags = "tracking, OptiTrack, NatNet")]
	#endregion PluginInfo
	public class RigidBodiesOptiTrackNode : IPluginEvaluate{
	
		#region fields & pins
		[Input("Frame")]
        public ISpread<NatNetML.FrameOfMocapData> FFrame;
		
		[Output("Transform")]
        public ISpread<Matrix4x4> FTransform;
		
		[Output("Rigid body name")]
        public ISpread<String> FRigidBody;
		
		[Output("Marker Position")]
        public ISpread<Vector3D> FMarkerPosition;
		
		[Output("Tracked")]
		public ISpread<bool> FTracked;
		
		[Import()]
		public ILogger FLogger;
		#endregion fields & pins
		
		
		// NatNet Frame of Data
		private NatNetML.FrameOfMocapData mFrameOfData;

        private static object syncLock = new object();
		
		
		//Constructor
		public RigidBodiesOptiTrackNode() {

		}
		

		//called when data for any output pin is requested
		public void Evaluate(int SpreadMax)
		{
           if(FFrame.SliceCount>0) {
                // There are avaliable frames

            	lock(syncLock) {
            		//FLogger.Log(LogType.Debug, "" + m_FrameQueue.Count);
            		
        			mFrameOfData = FFrame[0];
        	
        			FTracked.SliceCount = mFrameOfData.nRigidBodies;
            		FTransform.SliceCount = mFrameOfData.nRigidBodies;
            		FLogger.Log(LogType.Debug, "There are " + mFrameOfData.nRigidBodies + " rigid bodies detected");
            		FMarkerPosition.SliceCount = 0;
        			for (int i = 0; i < mFrameOfData.nRigidBodies; ++i)
        			{
        				NatNetML.RigidBodyData rb = mFrameOfData.RigidBodies[i];
        				
        				// Is tracked
        				FTracked[i] = rb.Tracked;
        				
        				// Positions
        				Vector3D newPos = new Vector3D(rb.x, rb.y, rb.z);
        				
        				// Orientations (quaternions)
        				float qx, qy, qw, qz;
        				qx = rb.qx;
        				qy = rb.qy;
        				qz = rb.qz;
        				qw = rb.qw;
        				MathTools.QuaternionNormalise(ref qx, ref qy, ref qz, ref qw);
        				Vector4D newQuat = new Vector4D(rb.qx, rb.qy, rb.qz, rb.qw);

        				//Orientations (Euler)
                        Vector3D euler = VMath.QuaternionToEulerYawPitchRoll(newQuat);
        				
        				//Matrix transform
        				FTransform[i] = MathTools.ToVVVVMatrix(newPos, euler);
        				
       					// Marker positions
        				FMarkerPosition.SliceCount += rb.nMarkers;
	            		for(int j = 0; j < FMarkerPosition.SliceCount; ++j){
	            			Marker marker = rb.Markers[j];
	            			FMarkerPosition[j] = new Vector3D(-marker.x, marker.y, marker.z);
	            		}
            			
            		}
            	}

       		}
		}

	}
	#region PluginInfo
	[PluginInfo(Name = "RigidBody (OptiTrack)", Category = "Devices", Help = "Get information from OptiTrack camera through NatNet protocol", Tags = "tracking, OptiTrack, NatNet")]
	#endregion PluginInfo
	public class RigidBodyOptiTrackNode : IPluginEvaluate{
	
		#region fields & pins
		[Input("Frame")]
        public ISpread<NatNetML.FrameOfMocapData> FFrame;
		
		[Input("Index")]
        public ISpread<int> FFIndex;
		
		[Output("Transform")]
        public ISpread<Matrix4x4> FTransform;
		
		[Output("Rigid body name")]
        public ISpread<String> FRigidBody;
		
		[Output("Marker Position")]
        public ISpread<Vector3D> FMarkerPosition;
		
		[Output("Tracked")]
		public ISpread<bool> FTracked;
		
		[Import()]
		public ILogger FLogger;
		#endregion fields & pins
		
		
		// NatNet Frame of Data
		private NatNetML.FrameOfMocapData mFrameOfData;

        private static object syncLock = new object();
		
		
		//Constructor
		public RigidBodyOptiTrackNode() {

		}
		

		//called when data for any output pin is requested
		public void Evaluate(int SpreadMax)
		{
           if(FFrame.SliceCount>0) {
                // There are avaliable frames

            	lock(syncLock) {
            		//FLogger.Log(LogType.Debug, "" + m_FrameQueue.Count);
            		
        			mFrameOfData = FFrame[0];
        	
        			FTracked.SliceCount = FFIndex.SliceCount;
            		FTransform.SliceCount = FFIndex.SliceCount;
            		FLogger.Log(LogType.Debug, "There are " + mFrameOfData.nRigidBodies + " rigid bodies detected");
            		FMarkerPosition.SliceCount = 0;
        			for (int c = 0; c < FFIndex.SliceCount; ++c)
        			{
        				int i = FFIndex[c]%mFrameOfData.nRigidBodies;
        				NatNetML.RigidBodyData rb = mFrameOfData.RigidBodies[i];
        				
        				// Is tracked
        				FTracked[i] = rb.Tracked;
        				
        				// Positions
        				Vector3D newPos = new Vector3D(rb.x, rb.y, rb.z);
        				
        				// Orientations (quaternions)
        				float qx, qy, qw, qz;
        				qx = rb.qx;
        				qy = rb.qy;
        				qz = rb.qz;
        				qw = rb.qw;
        				MathTools.QuaternionNormalise(ref qx, ref qy, ref qz, ref qw);
        				Vector4D newQuat = new Vector4D(rb.qx, rb.qy, rb.qz, rb.qw);

        				//Orientations (Euler)
                        Vector3D euler = VMath.QuaternionToEulerYawPitchRoll(newQuat);
        				
        				//Matrix transform
        				FTransform[i] = MathTools.ToVVVVMatrix(newPos, euler);
        				
       					// Marker positions
        				FMarkerPosition.SliceCount += rb.nMarkers;
	            		for(int j = 0; j < FMarkerPosition.SliceCount; ++j){
	            			Marker marker = rb.Markers[j];
	            			FMarkerPosition[j] = new Vector3D(-marker.x, marker.y, marker.z);
	            		}
            			
            		}
            	}

       		}
		}

	}
}
