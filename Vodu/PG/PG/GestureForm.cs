﻿using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.IO;
using System.Linq;
using System.Reflection;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
using System.Diagnostics;
using Microsoft.Kinect;

using Accord.Statistics.Models.Markov;
using Accord.Statistics.Models.Markov.Learning;
using Accord.Statistics.Models.Markov.Topology;


namespace PG
{
    public partial class GestureForm:Form
    {
        private const float InferredZPositionClamp = 0.1f;
        private KinectSensor kinectSensor = null;
        private BodyFrameReader bodyFrameReader = null;
        private Body[] bodies = null;
        
        private bool isTraining;
        private bool isTracking;
        private bool isKinectOn;
        private bool goForward;
        private bool returnToUser;

        private Point current, previous;
        private Point reference, transformation;
        private Point start, end, robot, intersection;
        
        private StreamWriter simulator;
        private const double normalizer = 20.0;
        private const double speedChange = 2.0;
        private double A, B, D;
        private double speed;

        int fpscounter = 0;

        Dictionary <int, String> labelToName;
        List<List<int>> trainingSequences, ts;
        List<int> featureVector, trainingLabels, handX, handY, handZ;

        HiddenMarkovClassifier classifier;

        List<Tuple<int, HiddenMarkovModel>> HMM;
        BaumWelchLearning teacher;

        private int numOfGestures;

        int px, py, pz;
        // index for the currently tracked body
        private int bodyIndex;

        // flag to asses if a body is currently tracked
        private bool bodyTracked = false;
        public GestureForm()
        {
            InitializeComponent();                   
        }

        //private void comboPointing_SelectedIndexChanged(object sender,EventArgs e)
        //{
        //    if(comboPointing.SelectedIndex == 0)
        //    {
        //        labelA.Visible = true;
        //        labelB.Visible = true;
        //        labelD.Visible = true;
        //        textA.Visible = true;
        //        textB.Visible = true;
        //        textD.Visible = true;
        //    }
        //    else
        //    {
        //        labelA.Visible = false;
        //        labelB.Visible = false;
        //        labelD.Visible = false;
        //        textA.Visible = false;
        //        textB.Visible = false;
        //        textD.Visible = false;
        //    }
        //}

        //private void textA_TextChanged(object sender,EventArgs e)
        //{
        //    A = Convert.ToInt32(textA.Text);            
        //}

        //private void textB_TextChanged(object sender,EventArgs e)
        //{
        //    B = Convert.ToInt32(textB.Text);
        //}

        //private void textD_TextChanged(object sender,EventArgs e)
        //{
        //    D = Convert.ToInt32(textD.Text);
        //}

        private void GestureForm_Load(object sender,EventArgs e)
        {

            HMM = new List<Tuple<int, HiddenMarkovModel>>();
            numOfGestures = 5;
            ts = new List<List<int>>();

            trainingSequences = new List<List<int>>();
            featureVector = new List<int>();
            trainingLabels = new List<int>();
            handX = new List<int>();
            handY = new List<int>();
            handZ = new List<int>();

            isTraining = false;
            isTracking = false;
            isKinectOn = false;
            goForward = true;
            returnToUser = false;

            A = Convert.ToInt32(textA.Text);
            B = Convert.ToInt32(textB.Text);
            D = Convert.ToInt32(textD.Text);

            speed = 2.0;    
            comboTrain.SelectedItem = "Forward"; 
            comboPointing.SelectedItem = "Method 1";

            labelToName = new Dictionary<int, String>();
            labelToName.Add(0, "Forward");
            labelToName.Add(1, "Backward");
            labelToName.Add(2, "Speed Up");
            labelToName.Add(3, "Speed Down");
            labelToName.Add(4, "Return");

            System.IO.File.WriteAllText(@"Simulator/data.txt","r 0.0 -200.0");

            simulator = new StreamWriter(@"Simulator/data.txt", true);
            
             

            reference = new Point(210.0, 210.0, 210.0);
            robot = new Point(0.0, -200.0);
            intersection = new Point(0.0, 0.0);

            simulator.WriteLine("r " + robot.X + " " + robot.Z);

            this.kinectSensor = KinectSensor.GetDefault();

            if(kinectSensor != null)
            {
                this.kinectSensor.Open();
                //this.kinectSensor.Close();
            }

            GetFrames();    
            
        }

        

        public void GetFrames()
        {
            this.bodyFrameReader = this.kinectSensor.BodyFrameSource.OpenReader();
            this.bodyFrameReader.FrameArrived += this.Reader_FrameArrived;            
        }

        public double distance(Point a, Point b)
        {
            return Math.Sqrt((a.X-b.X)*(a.X-b.X)+(a.Z-b.Z)*(a.Z-b.Z));
        }
        
        public void forward()
        {
            goForward = true;
        }

        public void backward()
        {
            goForward = false;
        }

        public void speedUp()
        {
            speed += speedChange;
        }

        public void speedDown()
        {
            if(speed >=0.0) speed -= speedChange;
        }

        public void Return()
        {
            returnToUser = true;
        }

        private void buttonKinect_Click(object sender,EventArgs e)
        {
            if(isKinectOn == false)
            {
                isKinectOn = true;
                buttonKinect.Text = "Stop Kinect";
                if(kinectSensor != null)
                {
                    this.kinectSensor.Open();   
                    //this.kinectSensor.Close();
                }
            }
            else
            {
                isKinectOn = false;
                buttonKinect.Text = "Start Kinect";
                if(kinectSensor != null)
                {
                    //this.kinectSensor.Open();   
                    this.kinectSensor.Close();
                }
            }
        }

        private void buttonSimulator_Click(object sender,EventArgs e)
        {
            
            Process.Start(@"Simulator\Simulator.exe");           
        }

        private void Reader_FrameArrived(object sender,BodyFrameArrivedEventArgs e)
        {
            bool dataReceived = false;
            if(isTracking == true)
            {
                fpscounter++;
                if(fpscounter % 3 == 0) return;
            }
            using(BodyFrame bodyFrame = e.FrameReference.AcquireFrame())
            {
                if(bodyFrame != null)
                {
                    if(this.bodies == null)
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

            if(dataReceived)
            {
                if (dataReceived)
                {
                    Body body = null;
                    if(this.bodyTracked) {
                        if(this.bodies[this.bodyIndex].IsTracked) {
                            body = this.bodies[this.bodyIndex];
                        } else {
                            bodyTracked = false;
                        }
                    }
                    if(!bodyTracked) {
                        for (int i=0; i<this.bodies.Length; ++i)
                        {
                            if(this.bodies[i].IsTracked) {
                                this.bodyIndex = i;
                                this.bodyTracked = true;
                                break;
                            }
                        }
                    }

                    if (body != null && this.bodyTracked && body.IsTracked)
                    {
                        IReadOnlyDictionary<JointType,Joint> joints = body.Joints;
                        CameraSpacePoint handRight = joints[JointType.HandRight].Position;
                        CameraSpacePoint shoulderRight = joints[JointType.ShoulderRight].Position;
                        CameraSpacePoint elbowRight = joints[JointType.ElbowRight].Position;
                        CameraSpacePoint handLeft = joints[JointType.HandLeft].Position;
                        CameraSpacePoint elbowLeft = joints[JointType.ElbowLeft].Position;

                        Console.WriteLine(handRight.X);
                        Console.WriteLine(handRight.Y);
                        Console.WriteLine(handRight.Z);

                        if (returnToUser == true)
                        {
                            end = new Point((double)shoulderRight.X*100.0,(double)shoulderRight.Z*100.0);

                            double length = speed;

                            double alpha = Math.Atan2(end.Z-robot.Z, end.X-robot.X);
                            robot = new Point(robot.X+length*Math.Cos(alpha),robot.Z+length*Math.Sin(alpha));  // extending the line by length cm from start position
                            simulator.WriteLine("r " + robot.X + " " + robot.Z);

                            if(distance(robot, end) < 10.0) returnToUser = false;

                        } 

                        /// Pointing Gesture
                        //Debug.WriteLine(handRight.Y*100.0 + " " + elbowRight.Y*100 + " " + shoulderRight.Y*100);
                        double tempz = shoulderRight.Z*100.0 - handRight.Z*100.0;
                        double handRightY = handRight.Y*100.0 - tempz*0.41;
                        tempz = shoulderRight.Z*100.0 - elbowRight.Z*100.0;
                        double elbowRightY = elbowRight.Y*100.0 - tempz*0.42;
                        double shoulderRightY = shoulderRight.Y * 100.0;

                        if(isTracking == false && Math.Abs(shoulderRightY-elbowRightY) <= 3.0 && Math.Abs(shoulderRightY-handRightY) < 8.0)
                        {
                            labelRobot.Text = "Robot Status: Moving";
                            returnToUser = false;

                            if(comboPointing.SelectedIndex == 0)
                            {
                                start = new Point((double)shoulderRight.X*100.0,(double)shoulderRight.Z*100.0);
                                end = new Point((double)handRight.X*100.0,(double)handRight.Z*100.0);


                                //Console.WriteLine(start.X + " " + start.Z + " " + end.X + " " + end.Z);

                                // calculating slope(m) and intercept(c)
                                double dz = end.Z - start.Z;
                                double dx = end.X - start.X;

                                double m = dz/dx;
                                double c = start.Z-m*start.X;
                                double x, z;
                                if(dx == 0.0)
                                {
                                    x = end.X;
                                    z = -D;
                                }
                                else if(dx > 0.0)
                                {
                                    z = m*A+c;
                                    if(z < -D)
                                    {
                                        z = -D;
                                        x = (z-c)/m;
                                    }
                                    else x = A;
                                }
                                else
                                {
                                    z = (m*(-B))+c;
                                    if(z < -D)
                                    {
                                        z = -D;
                                        x = (z-c)/m;
                                    }
                                    else x = -B;
                                }
                                
                                simulator.WriteLine("s " + start.X + " " + start.Z);
                                simulator.WriteLine("p " + x + " " + z);
                                simulator.WriteLine("i " + x + " " + z);
                                double length = speed;

                                double alpha = Math.Atan2(z-robot.Z, x-robot.X);
                                robot = new Point(robot.X+length*Math.Cos(alpha),robot.Z+length*Math.Sin(alpha));  // extending the line by length cm from start position
                                simulator.WriteLine("r " + robot.X + " " + robot.Z);

                            }
                            else
                            {
                                start = new Point((double)shoulderRight.X*100.0,(double)shoulderRight.Z*100.0);
                                end = new Point((double)handRight.X*100.0,(double)handRight.Z*100.0);
                                                                
                                // calculating slope(m) and intercept(c)
                                double dz = end.Z - start.Z;
                                double dx = end.X - start.X;

                                double m1 = dz/dx;
                                double c1 = start.Z-m1*start.X;

                                double m2 = -(1/m1);
                                double c2 = robot.Z-(robot.X*m2);

                                intersection.X = (c2-c1)/(m1-m2);
                                intersection.Z = m1*intersection.X+c1;

                                double length = 800.0;
                                double alpha = Math.Atan2(end.Z-start.Z,end.X-start.X);
                                end = new Point(start.X+length*Math.Cos(alpha),start.Z+length*Math.Sin(alpha));  // extending the line by length cm from start position


                                simulator.WriteLine("s " + start.X + " " + start.Z);
                                simulator.WriteLine("p " + end.X + " " + end.Z);
                                simulator.WriteLine("i " + intersection.X + " " + intersection.Z);
                                
                                
                                if(distance(intersection, robot) > 10.0)
                                {
                                    length = speed;
                                    alpha = Math.Atan2(intersection.Z-robot.Z, intersection.X-robot.X);
                                    robot = new Point(robot.X+length*Math.Cos(alpha), robot.Z+length*Math.Sin(alpha));  // extending the line by length cm from start position
                                                   
                                }
                                else
                                {
                                    if(goForward)
                                    {
                                        length = speed;
                                        alpha = Math.Atan2(end.Z-robot.Z, end.X-robot.X);
                                        robot = new Point(robot.X+length*Math.Cos(alpha), robot.Z+length*Math.Sin(alpha));  // extending the line by length cm from start position                               
                                    }
                                    else
                                    {
                                        length = speed;
                                        alpha = Math.Atan2(start.Z-robot.Z, start.X-robot.X);
                                        robot = new Point(robot.X+length*Math.Cos(alpha), robot.Z+length*Math.Sin(alpha));  // extending the line by length cm from start position                               
                                    
                                    }
                                    
                                }

                                simulator.WriteLine("r " + robot.X + " " + robot.Z);                                
                            }
                        }
                        else
                        {
                            labelRobot.Text = "Robot Status: Idle";
                        }

                        if(isTracking)
                        {
                            current = new Point(handRight.X*100.0-transformation.X,handRight.Y*100.0-transformation.Y,handRight.Z*100.0-transformation.Z);
                            int tx = Convert.ToInt32(Math.Floor(current.X/normalizer));
                            int ty = Convert.ToInt32(Math.Floor(current.Y/normalizer));
                            int tz = Convert.ToInt32(Math.Floor(current.Z/normalizer));
                            //Debug.WriteLine(current.X + " " + current.Y + " " + current.Z);
                            //Debug.WriteLine(tx + " " + ty + " " + tz);
                            handX.Add(tx);
                            handY.Add(ty);
                            handZ.Add(tz);

                        }

                        if(isTracking == false && elbowLeft.Y*100.0+14.0 < handLeft.Y*100.0 &&  body.HandLeftState == HandState.Closed)
                        {
                            labelGesture.Text = "Performing Gesture";
                            fpscounter = 0;
                            handX.Clear();
                            handY.Clear();
                            handZ.Clear();

                            current = new Point(handRight.X*100.0,handRight.Y*100.0,handRight.Z*100.0);
                            transformation = new Point(current.X-reference.X,current.Y-reference.Y,current.Z-reference.Z);
                            current = reference;
                            handX.Add(Convert.ToInt32(current.X/normalizer));
                            handY.Add(Convert.ToInt32(current.Y/normalizer));
                            handZ.Add(Convert.ToInt32(current.Z/normalizer));
                            isTracking = true;
                            featureVector.Clear();
                        }

                        if(isTracking == true && body.HandLeftState == HandState.Open)
                        {
                            isTracking = false;
                            Console.WriteLine();
                            if(isTraining == true)
                            {
                                featureVector.Clear();
                                for(int i = 0;i<handX.Count;i++) featureVector.Add(handX[i]);
                                //featureVector.Add(29);
                                //featureVector.Add(29);
                                //featureVector.Add(29);
                                //featureVector.Add(29);
                                //featureVector.Add(29);
                                for(int i = 0;i<handY.Count;i++) featureVector.Add(handY[i]);
                                //featureVector.Add(29);
                                //featureVector.Add(29);
                                //featureVector.Add(29);
                                //featureVector.Add(29);
                                //featureVector.Add(29);
                                for(int i = 0;i<handZ.Count;i++) featureVector.Add(handZ[i]);
                                trainingSequences.Add(new List<int>(featureVector));
                                trainingLabels.Add(comboTrain.SelectedIndex);
                                
                                labelGesture.Text = "Gesture Recognition: Idle";
                            }
                            else
                            {
                                featureVector.Clear();
                                for(int i = 0;i<handX.Count;i++) featureVector.Add(handX[i]);
                                //featureVector.Add(29);
                                //featureVector.Add(29);
                                //featureVector.Add(29);
                                //featureVector.Add(29);
                                //featureVector.Add(29);
                                for(int i = 0;i<handY.Count;i++) featureVector.Add(handY[i]);
                                //featureVector.Add(29);
                                //featureVector.Add(29);
                                //featureVector.Add(29);
                                //featureVector.Add(29);
                                //featureVector.Add(29);
                                for(int i = 0;i<handZ.Count;i++) featureVector.Add(handZ[i]);


                                double mx = -999999.0;
                                int recognizedLabel = 0;
                                for(int i = 0;i<numOfGestures;i++)
                                {
                                    double likelihood = HMM[i].Item2.LogLikelihood(featureVector.ToArray());
                                    Console.WriteLine(likelihood);
                                    if(Double.IsInfinity(likelihood)) continue;
                                    if(likelihood >= -100.0 && likelihood >  mx)
                                    {
                                        mx = likelihood;
                                        recognizedLabel = i;
                                    }
                                }

                                if(mx != -999999.0)
                                {
                                    labelGesture.Text = "Gesture Performed: " + labelToName[recognizedLabel];

                                    if(recognizedLabel == 0) forward();
                                    else if(recognizedLabel == 1) backward();
                                    else if(recognizedLabel == 2) speedUp();
                                    else if(recognizedLabel == 3) speedDown();
                                    else Return();
                                }
                                else labelGesture.Text = "Gesture Performed: Unrecognized Gesture";

                                ////////////////////////////////////

                                //int recognizedLabel = classifier.Decide(featureVector.ToArray());
                                //labelGesture.Text = labelToName[recognizedLabel];

                                //if(recognizedLabel == 0) forward();
                                //else if(recognizedLabel == 1) backward();
                                //else if(recognizedLabel == 2) speedUp();
                                //else if(recognizedLabel == 3) speedDown();

                                ///////////////////////////////////////////////////

                                //String sequenceFile = @"Resources\S.txt";
                                //String labelFile =  @"Resources\L.txt";

                                ////System.IO.File.WriteAllText(sequenceFile, "");
                                ////System.IO.File.WriteAllText(labelFile, "");

                                //for(int i = 0;i<featureVector.Count;i++)
                                //{

                                //    System.IO.File.AppendAllText(sequenceFile,featureVector[i].ToString());
                                //}

                                //System.IO.File.AppendAllText(sequenceFile, "  " + recognizedLabel.ToString() + Environment.NewLine);
                            }
                        }
                    }
                }         
            }
        }

        private void buttonTrain_Click(object sender,EventArgs e)
        {
            labelMode.Text = "Mode: Training";
            comboTrain.Visible = true;
            isTraining = true;
        }

        private void buttonRecog_Click(object sender,EventArgs e)
        {
            labelMode.Text = "Mode: Recognition";
            comboTrain.Visible = false;
            isTraining = false;
        }

        private void buttonLearnHMM_Click(object sender,EventArgs e)
        {
            //ITopology forward = new Forward(states: 6);
            //classifier = new HiddenMarkovClassifier(classes: 5,topology: forward,symbols: 20);
            //var teacher = new HiddenMarkovClassifierLearning(classifier,
            //    modelIndex => new BaumWelchLearning(classifier.Models[modelIndex])
            //    {
            //        Tolerance = 0.0001, // iterate until log-likelihood changes less than 0.001
            //        Iterations = 0     // don't place an upper limit on the number of iterations
            //    });

            //int[][] inputSequences = trainingSequences.Select(a => a.ToArray()).ToArray();
            //int[] outputLabels = trainingLabels.ToArray();

            //double error = teacher.Run(inputSequences,outputLabels);

            //////////////////////////////////////////////////

            for(int i = 0;i< numOfGestures;i++)
            {
                ts.Clear();
                for(int j = 0;j<trainingSequences.Count;j++)
                {
                    if(trainingLabels[j] == i)
                    {
                        ts.Add(new List<int>(trainingSequences[j]));
                    }
                }

                int[][] inputSequences = ts.Select(a => a.ToArray()).ToArray();
                HiddenMarkovModel hmm = new HiddenMarkovModel(6,20);
                teacher = new BaumWelchLearning(hmm) { Tolerance = 0.0001,Iterations = 0 };
                teacher.Run(inputSequences);
                HMM.Add(new Tuple<int,HiddenMarkovModel>(i,hmm));
            }
        }

        private void buttonSaveToFile_Click(object sender,EventArgs e)
        {
            String sequenceFile = @"Resources\TrainingSequences.txt";
            String labelFile =  @"Resources\TrainingLabels.txt";

            System.IO.File.WriteAllText(sequenceFile,"");
            System.IO.File.WriteAllText(labelFile,"");

            using(StreamWriter sw = new StreamWriter(sequenceFile, true))
            {
                for(int i = 0;i<trainingSequences.Count;i++)
                {
                    //if(i > 1 && trainingLabels[i] != trainingLabels[i]) Console.WriteLine();
                    for(int j = 0;j<trainingSequences[i].Count;j++)
                        sw.Write(trainingSequences[i][j].ToString() + " ");
                    sw.WriteLine("");
                }                
            }

            using(StreamWriter sw = new StreamWriter(labelFile, true))
            {
                for(int i = 0;i<trainingLabels.Count;i++)
                    sw.WriteLine(trainingLabels[i].ToString());
            }


        }

        private void buttonLoadFromFile_Click(object sender,EventArgs e)
        {
            String sequenceFile = @"Resources\TrainingSequences.txt";
            String labelFile =  @"Resources\TrainingLabels.txt";

            using(StreamReader sr = File.OpenText(sequenceFile))
            {
                trainingSequences.Clear();

                String line;
                while((line = sr.ReadLine()) != null)
                {
                    featureVector.Clear();
                    String[] fv = line.Split(' ');
                    for(int i=0; i<fv.Length-1; i++) featureVector.Add(Convert.ToInt32(fv[i]));
                    trainingSequences.Add(new List<int>(featureVector));
                }
            }

            using(StreamReader sr = File.OpenText(labelFile))
            {      
                trainingLabels.Clear(); 
                         
                String line;
                while((line = sr.ReadLine()) != null)
                {                    
                    trainingLabels.Add(line[0]-48);
                }
            }
        }
    }

    public class Point
    {
        public double X, Y, Z;
        public Point(double xx, double yy, double zz)
        {
            this.X = xx;
            this.Y = yy;
            this.Z = zz;
        }
        public Point(double xx,double zz)
        {
            this.X = xx;
            this.Z = zz;
        }
    }
}
