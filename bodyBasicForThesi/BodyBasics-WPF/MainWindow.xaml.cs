//------------------------------------------------------------------------------
// <copyright file="MainWindow.xaml.cs" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

namespace Microsoft.Samples.Kinect.BodyBasics
{
    using System;
    using System.Collections.Generic;
    using System.ComponentModel;
    using System.Web.UI.DataVisualization.Charting;
    using System.Diagnostics;
    using System.Globalization;
    using System.IO;
    using System.Windows;
    using System.Windows.Media;
    using System.Windows.Media.Imaging;
    using Microsoft.Kinect;

    using Accord.Statistics.Models.Markov;
    using Accord.Statistics.Models.Markov.Learning;
    using Accord.Statistics.Models.Markov.Topology;

    /// <summary>
    /// Interaction logic for MainWindow
    /// </summary>
    public partial class MainWindow : Window, INotifyPropertyChanged
    {
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

        private int frameController = 0;
        private int featureVectorFrameCounter = 0;
        private double pi = 3.141592653;
        //private double[][] featureVector;
        List<int> featureVector = new List<int>();
        List<List<int>> TrainingSequence = new List<List<int>>();
        private int gestureCounter = 0;
        private int gestureStartFlag = 0;

        /// <summary>
        /// List of colors for each body tracked
        /// </summary>
        private List<Pen> bodyColors;

        /// <summary>
        /// Current status text to display
        /// </summary>
        private string statusText = null;

        private int bodyIndex;

        // flag to asses if a body is currently tracked
        private bool bodyTracked = false;

        /// <summary>
        /// Initializes a new instance of the MainWindow class.
        /// </summary>
        struct point3DDouble
        {
            public double X;
            public double Y;
            public double Z;
            public point3DDouble(double a, double b, double c)
            {
                X = a;
                Y = b;
                Z = c;
            }
        }
        public MainWindow()
        {
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
            /*
            // Right Leg
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HipRight, JointType.KneeRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.KneeRight, JointType.AnkleRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.AnkleRight, JointType.FootRight));

            // Left Leg
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HipLeft, JointType.KneeLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.KneeLeft, JointType.AnkleLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.AnkleLeft, JointType.FootLeft));
            */
            // populate body colors, one for each BodyIndex
            //Console.WriteLine(JointType.Head);

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

        //private 

        

        point3DDouble getVector(point3DDouble a, point3DDouble b)
        {
            point3DDouble returnValue = new point3DDouble();
            returnValue.X = b.X - a.X;
            returnValue.Y = b.Y - a.Y;
            returnValue.Z = b.Z - a.Z;
            return returnValue;
        }

        double getAngle(point3DDouble v1, point3DDouble v2)
        {
            double dotProduct = (v1.X * v2.X) + (v1.Y * v2.Y) + (v1.Z * v2.Z);
            double mag1 = (v1.X * v1.X) + (v1.Y * v1.Y) + (v1.Z * v1.Z);
            double mag2 = (v2.X * v2.X) + (v2.Y * v2.Y) + (v2.Z * v2.Z);
            mag1 = Math.Sqrt(mag1);
            mag2 = Math.Sqrt(mag2);
            double angle = Math.Acos(dotProduct / (mag1 * mag2));
            angle = angle * 180 / pi;
            return angle;
        }
        double returnAngleBetweenJoints(CameraSpacePoint a, CameraSpacePoint b, CameraSpacePoint c)
        {
            point3DDouble vector1 = new point3DDouble();
            point3DDouble vector2 = new point3DDouble();
            vector1 = getVector(new point3DDouble(a.X, a.Y, a.Z), new point3DDouble(b.X, b.Y, b.Z));
            vector2 = getVector(new point3DDouble(a.X, a.Y, a.Z), new point3DDouble(c.X, c.Y, c.Z));

            double returnValue = getAngle(vector1, vector2);

            return returnValue;
        }

        
        private void Reader_FrameArrived(object sender, BodyFrameArrivedEventArgs e)
        {
            frameController++;
            if (frameController % 3 == 0) return;
            
            //Vector4 vector1 = new Vector4();
            //Vector vector2 = new Vector(45, 70);
            //Double angleBetween = 120;

            //// angleBetween is approximately equal to 0.9548
            ////angleBetween = Vector.AngleBetween(vector1, vector2);

            //int flagValue = 0;

            //if(flagValue == 0)
            //{
            //    Console.WriteLine(angleBetween);
            //    flagValue = 1;
            //}



            bool dataReceived = false;

            //point3DDouble v1 = new point3DDouble(4.5, 4.1, -5.4);
            //point3DDouble v2 = new point3DDouble(3, -2, 0);

            //point3DDouble vector1 = getVector(v1, v2);
            //double angle = getAngle(new point3DDouble(-5,-4,-4), new point3DDouble(-1,4,-4));

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
            }

            if (dataReceived)
            {
                Body body = null;
                if (this.bodyTracked)
                {
                    if (this.bodies[this.bodyIndex].IsTracked)
                    {
                        body = this.bodies[this.bodyIndex];
                    }
                    else
                    {
                        bodyTracked = false;
                    }
                }
                if (!bodyTracked)
                {
                    for (int i = 0; i < this.bodies.Length; ++i)
                    {
                        if (this.bodies[i].IsTracked)
                        {
                            this.bodyIndex = i;
                            this.bodyTracked = true;
                            break;
                        }
                    }
                }

                using (DrawingContext dc = this.drawingGroup.Open())
                {
                    // Draw a transparent background to set the render size
                    dc.DrawRectangle(Brushes.Black, null, new Rect(0.0, 0.0, this.displayWidth, this.displayHeight));
                    //dc.DrawRectangle(Brushes.Red, null, new Rect(2.0, 2.0, 50, 20));

                    
                    //dc.DrawText("";
                    int penIndex = 0;
                    if (body != null && this.bodyTracked && body.IsTracked)
                    {
                        //Console.WriteLine(vector1.X + " " + );

                        Pen drawPen = this.bodyColors[penIndex++];

                        if (body.IsTracked)
                        {

                            IReadOnlyDictionary<JointType, Joint> joints = body.Joints;
                            CameraSpacePoint Head = joints[JointType.Head].Position;
                            CameraSpacePoint FootLeft = joints[JointType.FootLeft].Position;
                            CameraSpacePoint handRight = joints[JointType.HandRight].Position;
                            CameraSpacePoint shoulderRight = joints[JointType.ShoulderRight].Position;
                            CameraSpacePoint shoulderLeft = joints[JointType.ShoulderLeft].Position;
                            CameraSpacePoint elbowRight = joints[JointType.ElbowRight].Position;
                            CameraSpacePoint handLeft = joints[JointType.HandLeft].Position;
                            CameraSpacePoint elbowLeft = joints[JointType.ElbowLeft].Position;
                            CameraSpacePoint spineBase = joints[JointType.SpineBase].Position;
                            CameraSpacePoint spineShoulder = joints[JointType.SpineShoulder].Position;



                            this.DrawClippedEdges(body, dc);

                            //IReadOnlyDictionary<JointType, Joint> joints = body.Joints;

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
                                //jointPoints[jointType] = new Point(0, 0);
                                
                                //Console.WriteLine();
                            }

                            this.DrawBody(joints, jointPoints, dc, drawPen);

                            this.DrawHand(body.HandLeftState, jointPoints[JointType.HandLeft], dc);
                            this.DrawHand(body.HandRightState, jointPoints[JointType.HandRight], dc);



                            
                            float headToSpineDistance = Head.Y - spineBase.Y;
                            float SpineToFootDistance = spineBase.Y - FootLeft.Y;

                            float ratio = headToSpineDistance / SpineToFootDistance;

                            string ShowOnScreen = "";
                            //Console.WriteLine(ratio);
                            if (ratio < 1.3 && ratio >= 0.5) ShowOnScreen = "Standing";
                            else if (ratio < 0.5) ShowOnScreen = "Lying Down";
                            else if (ratio > 1.3) ShowOnScreen = "Sitting";

                            //FormattedText formattedText = new FormattedText(ShowOnScreen, CultureInfo.GetCultureInfo("en-us"), FlowDirection.LeftToRight, new Typeface("Verdana"), 32, Brushes.White);


                            if (body.HandLeftState == HandState.Closed) gestureStartFlag = 1;

                            if (gestureStartFlag == 1)
                            {
                                //System.Threading.Thread.Sleep(3000);

                                double angleBetweenRightHand = returnAngleBetweenJoints(elbowRight, shoulderRight, handRight);
                                double angleBetweenLeftHand = returnAngleBetweenJoints(elbowLeft, shoulderLeft, handLeft);

                                double angleBetweenLeftSpine = returnAngleBetweenJoints(shoulderRight, spineShoulder, elbowRight);
                                double angleBetweenRightSpine = returnAngleBetweenJoints(shoulderLeft, spineShoulder, elbowLeft);


                                featureVector.Add((int)angleBetweenRightHand);
                                featureVector.Add((int)angleBetweenLeftHand);
                                featureVector.Add((int)angleBetweenLeftSpine);
                                featureVector.Add((int)angleBetweenRightSpine);

                                //featureVector[gestureCounter][featureVectorFrameCounter * 4] = angleBetweenRightHand;
                                //featureVector[gestureCounter][(featureVectorFrameCounter * 4) + 1] = angleBetweenLeftHand;
                                //featureVector[gestureCounter][(featureVectorFrameCounter * 4) + 2] = angleBetweenLeftSpine;
                                //featureVector[gestureCounter][(featureVectorFrameCounter * 4) + 3] = angleBetweenRightSpine;
                                featureVectorFrameCounter++;
                                if (gestureCounter < 40)
                                {
                                    FormattedText formattedText = new FormattedText("Performing Gesture", CultureInfo.GetCultureInfo("en-us"), FlowDirection.LeftToRight, new Typeface("Verdana"), 32, Brushes.White);
                                    dc.DrawText(formattedText, new Point(3, 3));

                                }
                                if (featureVectorFrameCounter > 20 && gestureCounter < 40)
                                {

                                    for (int i = 0; i<featureVector.Count; i++)
                                        File.AppendAllText(@"Resources\TrainingSequences.txt", featureVector[i] + " ");
                                    File.AppendAllText(@"Resources\TrainingSequences.txt", Environment.NewLine);
                                    File.AppendAllText(@"Resources\TrainingLabels.txt", "2" + Environment.NewLine);

                                    featureVectorFrameCounter = 0;
                                    gestureCounter++;
                                    featureVector.Clear();
                                }
                                if (gestureCounter == 40)
                                {
                                    FormattedText formattedText2 = new FormattedText("Gesture Training Ended", CultureInfo.GetCultureInfo("en-us"), FlowDirection.LeftToRight, new Typeface("Verdana"), 32, Brushes.White);
                                    dc.DrawText(formattedText2, new Point(3, 3));
                                    //String sequenceFile = @"Resources\TrainingSequences.txt";
                                    //String labelFile = @"Resources\TrainingLabels.txt";


                                    ////using (StreamWriter sw = new StreamWriter(sequenceFile, true))
                                    ////{
                                    //    for (int i = 0; i < TrainingSequence.Count; i++)
                                    //    {
                                    //        //if(i > 1 && trainingLabels[i] != trainingLabels[i]) Console.WriteLine();
                                    //        for (int j = 0; j < TrainingSequence[i].Count; j++)
                                    //            File.AppendAllText(@"Resources\TrainingSequences.txt", TrainingSequence[i][j] + " ");

                                    //    //sw.Write(TrainingSequence[i][j].ToString() + " ");
                                    //        File.AppendAllText(@"Resources\TrainingSequences.txt", Environment.NewLine);
                                    //    }
                                    ////}
                                }
                                //make feature vector 20 frames eache frame has 4 angles start end calculation...
                            }
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
            //Point center = new Point(0,0);
            //drawingContext.DrawEllipse(this.handLassoBrush, null, center, HandSize, HandSize);
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
    }
}
