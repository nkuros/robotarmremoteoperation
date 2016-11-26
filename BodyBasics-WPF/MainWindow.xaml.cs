//------------------------------------------------------------------------------
// <copyright file="MainWindow.xaml.cs" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

namespace Microsoft.Samples.Kinect.BodyBasics
{
    using System;
    using System.Windows;
    using System.Windows.Media;
    using Microsoft.Kinect;
    using System.Windows.Controls;
    using System.IO.Ports;
    using System.Threading;
    using System.Collections.Generic;
    using System.ComponentModel;
    using System.IO;
    using System.Linq;
    using System.Text;
    using System.Windows.Threading;
    using System.Windows.Documents;
    using System.Windows.Media.Media3D;
    using System.Net;
    using System.Net.Sockets;


    /// <summary>
    /// Interaction logic for MainWindow
    /// </summary>
    public partial class MainWindow : Window, INotifyPropertyChanged
    {
        #region variables
        SerialPort sp = new SerialPort();
        IPAddress send_to = IPAddress.Parse("207.168.1.107");
        int port = 8888;
        IPEndPoint ep = new IPEndPoint(IPAddress.Parse("192.168.1.107"), 8888);
        Socket sending_socket = new Socket(AddressFamily.InterNetwork, SocketType.Dgram,
        ProtocolType.Udp);
        private UdpClient client = new UdpClient();
        //Richtextbox
        FlowDocument mcFlowDoc = new FlowDocument();
        Paragraph para = new Paragraph();
        //Serial 
        //string received_data;
        #endregion
        /// <summary>
        /// Radius of drawn hand circles
        /// </summary>
        private const double HandSize = 30;

 
        /// <summary>
        /// Thickness of drawn joint lines
        /// </summary>
        private const double JointThickness = 3;

        /// <summary>
        /// Thickness of clip edge rectangles
        /// </summary>
        private const double ClipBoundsThickness = 10;

        /// <summary>
        /// Constant for clamping Z values of camera space points from being negative
        /// </summary>
        private const float InferredZPositionClamp = 0.1f;

        /// <summary>
        /// Brush used for drawing hands that are currently tracked as closed
        /// </summary>
        private readonly Brush handClosedBrush = new SolidColorBrush(Color.FromArgb(128, 255, 0, 0));

        /// <summary>
        /// Brush used for drawing hands that are currently tracked as opened
        /// </summary>
        private readonly Brush handOpenBrush = new SolidColorBrush(Color.FromArgb(128, 0, 255, 0));

        /// <summary>
        /// Brush used for drawing hands that are currently tracked as in lasso (pointer) position
        /// </summary>
        private readonly Brush handLassoBrush = new SolidColorBrush(Color.FromArgb(128, 0, 0, 255));

        /// <summary>
        /// Brush used for drawing joints that are currently tracked
        /// </summary>
        private readonly Brush trackedJointBrush = new SolidColorBrush(Color.FromArgb(255, 68, 192, 68));

        /// <summary>
        /// Brush used for drawing joints that are currently inferred
        /// </summary>        
        private readonly Brush inferredJointBrush = Brushes.Yellow;

        /// <summary>
        /// Pen used for drawing bones that are currently inferred
        /// </summary>        
        private readonly Pen inferredBonePen = new Pen(Brushes.Gray, 1);

        /// <summary>
        /// Drawing group for body rendering output
        /// </summary>
        private DrawingGroup drawingGroup;

        /// <summary>
        /// Drawing image that we will display
        /// </summary>
        private DrawingImage imageSource;

        /// <summary>
        /// Active Kinect sensor
        /// </summary>
        private KinectSensor kinectSensor = null;

        /// <summary>
        /// Coordinate mapper to map one type of point to another
        /// </summary>
        private CoordinateMapper coordinateMapper = null;

        /// <summary>
        /// Reader for body frames
        /// </summary>
        private BodyFrameReader bodyFrameReader = null;

        /// <summary>
        /// Array for the bodies
        /// </summary>
        private Body[] bodies = null;

        /// <summary>
        /// definition of bones
        /// </summary>
        private List<Tuple<JointType, JointType>> bones;

        /// <summary>
        /// Width of display (depth space)
        /// </summary>
        private int displayWidth;

        /// <summary>
        /// Height of display (depth space)
        /// </summary>
        private int displayHeight;

        /// <summary>
        /// List of colors for each body tracked
        /// </summary>
        private List<Pen> bodyColors;

        /// <summary>
        /// Current status text to display
        /// </summary>
        private string statusText = null;

        /// <summary>
        /// Initializes a new instance of the MainWindow class.
        /// </summary>
        public MainWindow()
        {
            InitializeComponent();
            Connect.Content = "Connect";
            Connectwifi.Content = "Connect WiFi";

            // one sensor is currently supported
            this.kinectSensor = KinectSensor.GetDefault();

            // get the coordinate mapper
            this.coordinateMapper = this.kinectSensor.CoordinateMapper;

            // get the depth (display) extents
            FrameDescription frameDescription = this.kinectSensor.DepthFrameSource.FrameDescription;

            // get size of joint space
            this.displayWidth = frameDescription.Width;
            this.displayHeight = frameDescription.Height;

            // open the reader for the body frames
            this.bodyFrameReader = this.kinectSensor.BodyFrameSource.OpenReader();

            // a bone defined as a line between two joints
            this.bones = new List<Tuple<JointType, JointType>>();

            // Torso
            this.bones.Add(new Tuple<JointType, JointType>(JointType.Head, JointType.Neck));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.Neck, JointType.SpineShoulder));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.SpineMid));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineMid, JointType.SpineBase));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.ShoulderRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.ShoulderLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineBase, JointType.HipRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineBase, JointType.HipLeft));

            // Right Arm
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ShoulderRight, JointType.ElbowRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ElbowRight, JointType.WristRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristRight, JointType.HandRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HandRight, JointType.HandTipRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristRight, JointType.ThumbRight));

            // Left Arm
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ShoulderLeft, JointType.ElbowLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ElbowLeft, JointType.WristLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristLeft, JointType.HandLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HandLeft, JointType.HandTipLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristLeft, JointType.ThumbLeft));

            // Right Leg
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HipRight, JointType.KneeRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.KneeRight, JointType.AnkleRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.AnkleRight, JointType.FootRight));

            // Left Leg
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HipLeft, JointType.KneeLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.KneeLeft, JointType.AnkleLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.AnkleLeft, JointType.FootLeft));

            // populate body colors, one for each BodyIndex
            this.bodyColors = new List<Pen>();

            this.bodyColors.Add(new Pen(Brushes.Red, 6));
            this.bodyColors.Add(new Pen(Brushes.Orange, 6));
            this.bodyColors.Add(new Pen(Brushes.Green, 6));
            this.bodyColors.Add(new Pen(Brushes.Blue, 6));
            this.bodyColors.Add(new Pen(Brushes.Indigo, 6));
            this.bodyColors.Add(new Pen(Brushes.Violet, 6));

            // set IsAvailableChanged event notifier
            this.kinectSensor.IsAvailableChanged += this.Sensor_IsAvailableChanged;

            // open the sensor
            this.kinectSensor.Open();

            // set the status text
            this.StatusText = this.kinectSensor.IsAvailable ? Properties.Resources.RunningStatusText
                                                            : Properties.Resources.NoSensorStatusText;

            // Create the drawing group we'll use for drawing
            this.drawingGroup = new DrawingGroup();

            // Create an image source that we can use in our image control
            this.imageSource = new DrawingImage(this.drawingGroup);

            // use the window object as the view model in this simple example
            this.DataContext = this;

            // initialize the components (controls) of the window
            this.InitializeComponent();
        }

        /// <summary>
        /// INotifyPropertyChangedPropertyChanged event to allow window controls to bind to changeable data
        /// </summary>
        public event PropertyChangedEventHandler PropertyChanged;

        /// <summary>
        /// Gets the bitmap to display
        /// </summary>
        public ImageSource ImageSource
        {
            get
            {
                return this.imageSource;
            }
        }

        /// <summary>
        /// Gets or sets the current status text to display
        /// </summary>
        public string StatusText
        {
            get
            {
                return this.statusText;
            }

            set
            {
                if (this.statusText != value)
                {
                    this.statusText = value;

                    // notify any bound elements that the text has changed
                    if (this.PropertyChanged != null)
                    {
                        this.PropertyChanged(this, new PropertyChangedEventArgs("StatusText"));
                    }
                }
            }
        }

        /// <summary>
        /// Execute start up tasks
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void MainWindow_Loaded(object sender, RoutedEventArgs e)
        {
            if (this.bodyFrameReader != null)
            {
                this.bodyFrameReader.FrameArrived += this.Reader_FrameArrived;
            }
        }

        /// <summary>
        /// Execute shutdown tasks
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void MainWindow_Closing(object sender, CancelEventArgs e)
        {
            if (this.bodyFrameReader != null)
            {
                // BodyFrameReader is IDisposable
                this.bodyFrameReader.Dispose();
                this.bodyFrameReader = null;
            }

            if (this.kinectSensor != null)
            {
                this.kinectSensor.Close();
                this.kinectSensor = null;
            }
        }

        /// <summary>
        /// Handles the body frame data arriving from the sensor
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void Reader_FrameArrived(object sender, BodyFrameArrivedEventArgs e)
        {
            bool dataReceived = false;

            using (BodyFrame bodyFrame = e.FrameReference.AcquireFrame())
            {
                if (bodyFrame != null)
                {
                    if (this.bodies == null)
                    {
                        this.bodies = new Body[bodyFrame.BodyCount];
                    }

                    // The first time GetAndRefreshBodyData is called, Kinect will allocate each Body in the array.
                    // As long as those body objects are not disposed and not set to null in the array,
                    // those body objects will be re-used.
                    bodyFrame.GetAndRefreshBodyData(this.bodies);
                    dataReceived = true;
                }
                if (bodies.Length == 0) { return; }
                 var skel = this.bodies.FirstOrDefault(b => b.IsTracked);
                if (skel == null) { return; }
                Vector3D ShoulderCenter = new Vector3D(skel.Joints[JointType.SpineShoulder].Position.X, skel.Joints[JointType.SpineShoulder].Position.Y, skel.Joints[JointType.SpineShoulder].Position.Z);
                Vector3D RightShoulder = new Vector3D(skel.Joints[JointType.ShoulderRight].Position.X, skel.Joints[JointType.ShoulderRight].Position.Y, skel.Joints[JointType.ShoulderRight].Position.Z);
                Vector3D LeftShoulder = new Vector3D(skel.Joints[JointType.ShoulderLeft].Position.X, skel.Joints[JointType.ShoulderLeft].Position.Y, skel.Joints[JointType.ShoulderLeft].Position.Z);
                Vector3D RightElbow = new Vector3D(skel.Joints[JointType.ElbowRight].Position.X, skel.Joints[JointType.ElbowRight].Position.Y, skel.Joints[JointType.ElbowRight].Position.Z);
                Vector3D LeftElbow = new Vector3D(skel.Joints[JointType.ElbowLeft].Position.X, skel.Joints[JointType.ElbowLeft].Position.Y, skel.Joints[JointType.ElbowLeft].Position.Z);
                Vector3D RightWrist = new Vector3D(skel.Joints[JointType.WristRight].Position.X, skel.Joints[JointType.WristRight].Position.Y, skel.Joints[JointType.WristRight].Position.Z);
                Vector3D LeftWrist = new Vector3D(skel.Joints[JointType.WristLeft].Position.X, skel.Joints[JointType.WristLeft].Position.Y, skel.Joints[JointType.WristLeft].Position.Z);
                Vector3D RightHip = new Vector3D(skel.Joints[JointType.HipRight].Position.X, skel.Joints[JointType.HipRight].Position.Y, skel.Joints[JointType.HipRight].Position.Z);
                Vector3D LeftHip = new Vector3D(skel.Joints[JointType.HipLeft].Position.X, skel.Joints[JointType.HipLeft].Position.Y, skel.Joints[JointType.HipLeft].Position.Z);
                Vector3D SpineMid = new Vector3D(skel.Joints[JointType.SpineMid].Position.X, skel.Joints[JointType.SpineMid].Position.Y, skel.Joints[JointType.SpineMid].Position.Z);
                Vector3D RightHand = new Vector3D(skel.Joints[JointType.HandRight].Position.X, skel.Joints[JointType.HandRight].Position.Y, skel.Joints[JointType.HandRight].Position.Z);
                Vector3D LeftHand = new Vector3D(skel.Joints[JointType.HandLeft].Position.X, skel.Joints[JointType.HandLeft].Position.Y, skel.Joints[JointType.HandLeft].Position.Z);
                HandState handStateRight = skel.HandRightState;
                int RightHandState =0;
                switch (handStateRight)
                {
                    case HandState.Closed:
                         RightHandState = 10;

                        break;

                    case HandState.Open:
                        RightHandState = 90;

                        break;

                    case HandState.Lasso:
                        RightHandState = 45;

                        break;
                }
                var rightShoulder = skel.Joints[JointType.ShoulderRight];
                var leftShoulder =  skel.Joints[JointType.ShoulderLeft];

                var rightWrist = skel.Joints[JointType.WristRight];
                var leftWrist = skel.Joints[JointType.WristLeft];

                var rightHand = skel.Joints[JointType.HandRight];
                var leftHand = skel.Joints[JointType.HandLeft];

                var rightHip = skel.Joints[JointType.HipRight];
                var leftHip = skel.Joints[JointType.HipLeft];

                var rightElbow = skel.Joints[JointType.ElbowRight];
                var leftElbow = skel.Joints[JointType.ElbowLeft];


                XValueRight.Text = rightWrist.Position.X.ToString();
                YValueRight.Text = (rightWrist.Position.Y - rightShoulder.Position.Y).ToString();
                ZValueRight.Text = rightWrist.Position.Z.ToString();

                XValueLeft.Text = leftWrist.Position.X.ToString();
                YValueLeft.Text = (leftWrist.Position.Y - leftShoulder.Position.Y).ToString();
                ZValueLeft.Text = leftWrist.Position.Z.ToString();

                var ShouldertoArduinoRight = Convert.ToInt32(AngleBetweenTwoVectors(ShoulderCenter - SpineMid, RightShoulder - RightElbow));
                var ElbowtoArduinoRight = Convert.ToInt32(180-AngleBetweenTwoVectors(RightElbow - RightWrist,RightElbow - RightShoulder));
                var WristtoArduinoRight = Convert.ToInt32(AngleBetweenTwoVectors(RightWrist - RightElbow, RightWrist - RightHand));

                var ShoulderRottoArduinoRight = Convert.ToInt32(Math.Abs(((Math.Atan2(rightShoulder.Position.X - rightElbow.Position.X, rightShoulder.Position.Z - rightElbow.Position.Z) - Math.Atan2(rightHip.Position.X - rightShoulder.Position.X, rightHip.Position.Z - rightShoulder.Position.Z)) * 180 / Math.PI)));

                var ElbowtoArduinoLeft = Convert.ToInt32(AngleBetweenTwoVectors(LeftElbow - LeftShoulder, LeftElbow - LeftWrist));
                var ShouldertoArduinoLeft = Convert.ToInt32(AngleBetweenTwoVectors(ShoulderCenter - SpineMid, LeftShoulder - LeftElbow));
                var WristtoArduinoLeft = Convert.ToInt32(AngleBetweenTwoVectors(LeftWrist - LeftElbow, LeftWrist - LeftHand));

                var ShoulderRottoArduinoLeft = Convert.ToInt32(Math.Abs(((Math.Atan2(leftShoulder.Position.X - leftElbow.Position.X, leftShoulder.Position.Z - leftElbow.Position.Z) - Math.Atan2(leftHip.Position.X - leftShoulder.Position.X, leftHip.Position.Z - leftShoulder.Position.Z)) * 180 / Math.PI)));

                ShoulderValueRight.Text = ShouldertoArduinoRight.ToString();
                ElbowValueRight.Text = ElbowtoArduinoRight.ToString();
                WristValueRight.Text = WristtoArduinoRight.ToString();
                ShoulderRotValueRight.Text = ShoulderRottoArduinoRight.ToString();
                WristRotValueRight.Text = RightHandState.ToString();

                ShoulderValueLeft.Text = ShouldertoArduinoLeft.ToString();
                ElbowValueLeft.Text = ElbowtoArduinoLeft.ToString();
                WristValueLeft.Text = WristtoArduinoRight.ToString();
                ShoulderRotValueLeft.Text = ShoulderRottoArduinoRight.ToString();

                int[] arduino = new int[5] { ShouldertoArduinoRight, ShoulderRottoArduinoRight, ElbowtoArduinoRight, WristtoArduinoRight, RightHandState };

                for (int i = 0; i < 5; i++)
                {

                    if (arduino[i] > 180)
                        arduino[i] = 180;
                    if (arduino[i] == 0)
                        arduino[i] = 20;

                }
                byte[] data = new byte[] { Convert.ToByte(arduino[0]), Convert.ToByte(arduino[1]), Convert.ToByte(arduino[2]), Convert.ToByte(arduino[3]), Convert.ToByte(arduino[4]) };

                CmdSend(data);
                sending_socket.SendTo(data, ep);
                if (rightWrist.Position.Y - rightShoulder.Position.Y > 0.0)
                {
                    RightRaised.Text = "Raised";
                }
                else
                {
                    RightRaised.Text = "Lowered";
                }

                if (leftWrist.Position.Y - leftShoulder.Position.Y > 0.0)
                {
                    LeftRaised.Text = "Raised";
                }

                else
                {
                    LeftRaised.Text = "Lowered";
                }
            }

            if (dataReceived)
            {
                using (DrawingContext dc = this.drawingGroup.Open())
                {
                    // Draw a transparent background to set the render size
                    dc.DrawRectangle(Brushes.Black, null, new Rect(0.0, 0.0, this.displayWidth, this.displayHeight));

                    int penIndex = 0;
                    foreach (Body body in this.bodies)
                    {
                        Pen drawPen = this.bodyColors[penIndex++];

                        if (body.IsTracked)
                        {
                            this.DrawClippedEdges(body, dc);

                            IReadOnlyDictionary<JointType, Joint> joints = body.Joints;

                            // convert the joint points to depth (display) space
                            Dictionary<JointType, Point> jointPoints = new Dictionary<JointType, Point>();

                            foreach (JointType jointType in joints.Keys)
                            {
                                // sometimes the depth(Z) of an inferred joint may show as negative
                                // clamp down to 0.1f to prevent coordinatemapper from returning (-Infinity, -Infinity)
                                CameraSpacePoint position = joints[jointType].Position;
                                if (position.Z < 0)
                                {
                                    position.Z = InferredZPositionClamp;
                                }

                                DepthSpacePoint depthSpacePoint = this.coordinateMapper.MapCameraPointToDepthSpace(position);
                                jointPoints[jointType] = new Point(depthSpacePoint.X, depthSpacePoint.Y);
                            }

                            this.DrawBody(joints, jointPoints, dc, drawPen);

                            this.DrawHand(body.HandLeftState, jointPoints[JointType.HandLeft], dc);
                            this.DrawHand(body.HandRightState, jointPoints[JointType.HandRight], dc);
                        }
                    }

                    // prevent drawing outside of our render area
                    this.drawingGroup.ClipGeometry = new RectangleGeometry(new Rect(0.0, 0.0, this.displayWidth, this.displayHeight));
                }
            }
        }

        /// <summary>
        /// Draws a body
        /// </summary>
        /// <param name="joints">joints to draw</param>
        /// <param name="jointPoints">translated positions of joints to draw</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        /// <param name="drawingPen">specifies color to draw a specific body</param>
        private void DrawBody(IReadOnlyDictionary<JointType, Joint> joints, IDictionary<JointType, Point> jointPoints, DrawingContext drawingContext, Pen drawingPen)
        {
            // Draw the bones
            foreach (var bone in this.bones)
            {
                this.DrawBone(joints, jointPoints, bone.Item1, bone.Item2, drawingContext, drawingPen);
            }

            // Draw the joints
            foreach (JointType jointType in joints.Keys)
            {
                Brush drawBrush = null;

                TrackingState trackingState = joints[jointType].TrackingState;

                if (trackingState == TrackingState.Tracked)
                {
                    drawBrush = this.trackedJointBrush;
                }
                else if (trackingState == TrackingState.Inferred)
                {
                    drawBrush = this.inferredJointBrush;
                }

                if (drawBrush != null)
                {
                    drawingContext.DrawEllipse(drawBrush, null, jointPoints[jointType], JointThickness, JointThickness);
                }
            }

        }

        /// <summary>
        /// Draws one bone of a body (joint to joint)
        /// </summary>
        /// <param name="joints">joints to draw</param>
        /// <param name="jointPoints">translated positions of joints to draw</param>
        /// <param name="jointType0">first joint of bone to draw</param>
        /// <param name="jointType1">second joint of bone to draw</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        /// /// <param name="drawingPen">specifies color to draw a specific bone</param>
        private void DrawBone(IReadOnlyDictionary<JointType, Joint> joints, IDictionary<JointType, Point> jointPoints, JointType jointType0, JointType jointType1, DrawingContext drawingContext, Pen drawingPen)
        {
            Joint joint0 = joints[jointType0];
            Joint joint1 = joints[jointType1];

            // If we can't find either of these joints, exit
            if (joint0.TrackingState == TrackingState.NotTracked ||
                joint1.TrackingState == TrackingState.NotTracked)
            {
                return;
            }

            // We assume all drawn bones are inferred unless BOTH joints are tracked
            Pen drawPen = this.inferredBonePen;
            if ((joint0.TrackingState == TrackingState.Tracked) && (joint1.TrackingState == TrackingState.Tracked))
            {
                drawPen = drawingPen;
            }

            drawingContext.DrawLine(drawPen, jointPoints[jointType0], jointPoints[jointType1]);
        }

        /// <summary>
        /// Draws a hand symbol if the hand is tracked: red circle = closed, green circle = opened; blue circle = lasso
        /// </summary>
        /// <param name="handState">state of the hand</param>
        /// <param name="handPosition">position of the hand</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        private void DrawHand(HandState handState, Point handPosition, DrawingContext drawingContext)
        {
            switch (handState)
            {
                case HandState.Closed:
                    drawingContext.DrawEllipse(this.handClosedBrush, null, handPosition, HandSize, HandSize);

                    break;

                case HandState.Open:
                    drawingContext.DrawEllipse(this.handOpenBrush, null, handPosition, HandSize, HandSize);
                    break;

                case HandState.Lasso:
                    drawingContext.DrawEllipse(this.handLassoBrush, null, handPosition, HandSize, HandSize);
                    break;
            }
        }

        /// <summary>
        /// Draws indicators to show which edges are clipping body data
        /// </summary>
        /// <param name="body">body to draw clipping information for</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        private void DrawClippedEdges(Body body, DrawingContext drawingContext)
        {
            FrameEdges clippedEdges = body.ClippedEdges;

            if (clippedEdges.HasFlag(FrameEdges.Bottom))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, this.displayHeight - ClipBoundsThickness, this.displayWidth, ClipBoundsThickness));
            }

            if (clippedEdges.HasFlag(FrameEdges.Top))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, 0, this.displayWidth, ClipBoundsThickness));
            }

            if (clippedEdges.HasFlag(FrameEdges.Left))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, 0, ClipBoundsThickness, this.displayHeight));
            }

            if (clippedEdges.HasFlag(FrameEdges.Right))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(this.displayWidth - ClipBoundsThickness, 0, ClipBoundsThickness, this.displayHeight));
            }
        }

        /// <summary>
        /// Handles the event which the sensor becomes unavailable (E.g. paused, closed, unplugged).
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void Sensor_IsAvailableChanged(object sender, IsAvailableChangedEventArgs e)
        {
            // on failure, set the status text
            this.StatusText = this.kinectSensor.IsAvailable ? Properties.Resources.RunningStatusText
                                                            : Properties.Resources.SensorNotAvailableStatusText;
        }
        private void Connect_Click(object sender, RoutedEventArgs e)
        {
            if (Connect.Content == "Connect")
            {
                //Sets up serial port
                sp.PortName = Comm_Port_Names.Text;
                sp.BaudRate = Convert.ToInt32(Baud_Rates.Text);
                sp.Handshake = System.IO.Ports.Handshake.None;
                sp.Parity = Parity.None;
                sp.DataBits = 8;
                sp.StopBits = StopBits.One;
                sp.ReadTimeout = 200;
                sp.WriteTimeout = 50;
                sp.Open();

                //Sets button State and Creates function call on data recieved
                Connect.Content = "Disconnect";
                //sp.DataReceived += new System.IO.Ports.SerialDataReceivedEventHandler(Receive);

            }
            else
            {
                try // just in case serial port is not open could also be acheved using if(serial.IsOpen)
                {
                    sp.Close();
                    Connect.Content = "Connect";
                }
                catch
                {
                }
            }
        }


        #region Sending   
             
        public double AngleBetweenTwoVectors(Vector3D vectorA, Vector3D vectorB)
        {
            double dotProduct = 0.0;
            vectorA.Normalize();
            vectorB.Normalize();
            dotProduct = Vector3D.DotProduct(vectorA, vectorB);

            return (double)Math.Acos(dotProduct) / Math.PI * 180;
        }

        public void CmdSend(Byte[] data)
        {
            if (sp.IsOpen)
            {
                try
                {

                    // Send the binary data out the port
                    sending_socket.SendTo(data,ep);
                    sp.Write(data, 0, data.Length);
                    Thread.Sleep(20);
                }
                catch (Exception ex)
                {
                    //para.Inlines.Add("Failed to SEND" + data + "\n" + ex + "\n");
                    //mcFlowDoc.Blocks.Add(para);
                    //Commdata.Document = mcFlowDoc;
                }
            }
            else
            {
            }
        }
        #endregion

        private void Baud_Rates_SelectionChanged(object sender, SelectionChangedEventArgs e)
        {

        }

        private void Connectwifi_Click(object sender, RoutedEventArgs e)
        {
            if (Connectwifi.Content == "Connect WiFi")
            {
                send_to = IPAddress.Parse(IPaddress.Text);
                port = Convert.ToInt32(Port.Text);
                //Sets up UDP connection

                //Sets button State and Creates function call on data received
                Connectwifi.Content = "Disconnect WiFi";


            }
            else
            {
                try // just in case port is not open 
                {

                    Connectwifi.Content = "Connect WiFi";
                    IPaddress.Text = "Insert IP Address:";
                    Port.Text = "Insert Port:";
                    IPEndPoint ep = new IPEndPoint(IPAddress.Parse("127.0.0.1"), 8888);

                }
                catch
                {
                }
            }
        }
        public void TextBox_GotFocus(object sender, RoutedEventArgs e)
        {
            TextBox tb = (TextBox)sender;
            tb.Text = string.Empty;
            tb.GotFocus -= TextBox_GotFocus;
        }


    }
}
