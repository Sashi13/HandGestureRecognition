using Microsoft.Kinect;
using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Globalization;
using System.IO.Ports;
using System.Runtime.InteropServices;
using System.Threading;
using System.Windows;
using System.Windows.Automation.Peers;
using System.Windows.Automation.Provider;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;

namespace GestureWithBT_WPF
{
    public partial class MainWindow : Window
    {

        [DllImport("user32.dll")] private static extern bool SetCursorPos(int X, int Y);
        [DllImport("user32.dll")]
        static extern void mouse_event(int dwFlags, int dx, int dy,
                               int dwData, int dwExtraInfo);
        
        //Hex codes for mouse press
        public enum MouseEventFlags
        {
            LEFTDOWN = 0x00000002,
            LEFTUP = 0x00000004,
            MIDDLEDOWN = 0x00000020,
            MIDDLEUP = 0x00000040,
            MOVE = 0x00000001,
            ABSOLUTE = 0x00008000,
            RIGHTDOWN = 0x00000008,
            RIGHTUP = 0x00000010
        }
        private const double HandSize = 30;
        private const double JointThickness = 2;
        private const float InferredZPositionClamp = 0.1f;
        private readonly Brush handClosedBrush = new SolidColorBrush(Color.FromArgb(128, 255, 0, 0));
        private readonly Brush handOpenBrush = new SolidColorBrush(Color.FromArgb(128, 0, 255, 0));
        private readonly Brush handLassoBrush = new SolidColorBrush(Color.FromArgb(128, 0, 0, 255 ));
        private readonly Brush trackedJointBrush = new SolidColorBrush(Color.FromArgb(255, 68, 192, 68));
        private readonly Brush inferredJointBrush = Brushes.Yellow;
        private readonly Pen inferredBonePen = new Pen(Brushes.Gray, 1);
        private DrawingGroup drawingGroup;
        private DrawingImage imageSource;
        private KinectSensor kinectSensor = null;
        private CoordinateMapper coordinateMapper = null;
        private BodyFrameReader bodyFrameReader = null;
        private ColorFrameReader colorFrameReader = null;
        private WriteableBitmap colorBitmap = null;
        public IReadOnlyDictionary<JointType, Joint> joints = null;
        private List<Tuple<JointType, JointType>> bones;
        private int displayWidth;
        private int displayHeight;
        private SerialPort serialPort = new SerialPort();
        bool comp_btn_clicked, mech_btn_clicked, bt_conn_btn_clicked;
        private BackgroundWorker hardWorker;
        private Thread readThread = null;
        static string distanceWarning = "";
        string result = "";
        string message = "";
        string hand_state = "";
        bool KinectStatus = false;

        public MainWindow()
        {
            this.InitializeComponent();
            this.InitializeKinect();
            bt_conn_panel.Visibility = Visibility.Hidden;
            stop_btn.Visibility = Visibility.Hidden;
            conn_label.Content = "Arduino Disconnected";
            conn_label.Foreground = Brushes.Red;
            drawPanel.Visibility = Visibility.Hidden;
            bt_disc_btn.IsEnabled = false;
        }

        private void comp_btn_Click(object sender, RoutedEventArgs e)
        {
            Console.WriteLine("Clicked");
            if (KinectStatus)
            {
                comp_btn.IsEnabled = false;
                mech_btn.IsEnabled = true;
                drawPanel.Visibility = Visibility.Visible;
                controlPanel.Visibility = Visibility.Hidden;
                this.stop_btn.Visibility = Visibility.Visible;
                this.WindowState = WindowState.Maximized;
                this.comp_btn_clicked = true;
                this.mech_btn_clicked = false;
            }
            else 
            {
               MessageBox.Show("Kinect is not connected!", "Error", MessageBoxButton.OK, MessageBoxImage.Error);
            }
            if (bt_conn_btn_clicked) 
            {
                ButtonAutomationPeer peer = new ButtonAutomationPeer(bt_disc_btn);
                IInvokeProvider invokeProv = peer.GetPattern(PatternInterface.Invoke) as IInvokeProvider;
                invokeProv.Invoke();
            }

        }

        private void mech_btn_Click(object sender, RoutedEventArgs e)
        {
            if (KinectStatus)
            {
                comp_btn.IsEnabled = true;
                mech_btn.IsEnabled = false;
                this.bt_conn_panel.Visibility = Visibility.Visible;
                this.stop_btn.Visibility = Visibility.Visible;
                this.comp_btn_clicked = false;
            }
            else
            {
                MessageBox.Show("Kinect is not connected!", "Error", MessageBoxButton.OK, MessageBoxImage.Error);
            }
        }

        private void stop_btn_Click(object sender, RoutedEventArgs e)
        {
            comp_btn.IsEnabled = true;
            mech_btn.IsEnabled = true;
            drawPanel.Visibility = Visibility.Hidden;
            controlPanel.Visibility = Visibility.Visible;
            this.bt_conn_panel.Visibility = Visibility.Hidden;
            this.stop_btn.Visibility = Visibility.Hidden;
            this.WindowState = WindowState.Normal;
            this.comp_btn_clicked = false; 
            if (mech_btn_clicked)
            {
                ButtonAutomationPeer peer = new ButtonAutomationPeer(mech_btn);
                IInvokeProvider invokeProv = peer.GetPattern(PatternInterface.Invoke) as IInvokeProvider;
                invokeProv.Invoke();
            }
        }

        private void bt_conn_btn_Click(object sender, RoutedEventArgs e)
        {
            InitializeConnection();
            bt_conn_btn_clicked = true;
            this.mech_btn_clicked = true;
        }

        private void bt_disc_btn_Click(object sender, RoutedEventArgs e)
        {
            closePort();
            bt_conn_btn_clicked = false;
            this.mech_btn_clicked = false;
        }

        public void InitializeKinect()
        {
            this.kinectSensor = KinectSensor.GetDefault();
            this.coordinateMapper = this.kinectSensor.CoordinateMapper;

            this.displayWidth = 640;
            this.displayHeight = 480;

            // open the reader for the color frames
            this.colorFrameReader = this.kinectSensor.ColorFrameSource.OpenReader();

            // wire handler for frame arrival
            this.colorFrameReader.FrameArrived += this.Reader_ColorFrameArrived;

            // create the colorFrameDescription from the ColorFrameSource using Bgra format
            FrameDescription colorFrameDescription = this.kinectSensor.ColorFrameSource.CreateFrameDescription(ColorImageFormat.Bgra);

            // create the bitmap to display
            this.colorBitmap = new WriteableBitmap(colorFrameDescription.Width, colorFrameDescription.Height, 96.0, 96.0, PixelFormats.Bgr32, null);

            // open the reader for the body frames
            this.bodyFrameReader = this.kinectSensor.BodyFrameSource.OpenReader();

            // a bone defined as a line between two joints
            this.bones = new List<Tuple<JointType, JointType>>();

            this.kinectSensor.IsAvailableChanged += this.Sensor_IsAvailableChanged;

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

            // open the sensor
            this.kinectSensor.Open();

            // Create the drawing group we'll use for drawing
            this.drawingGroup = new DrawingGroup();

            // Create an image source that we can use in our image control
            this.imageSource = new DrawingImage(this.drawingGroup);

            // use the window objegvt as the view model in this simple example
            this.DataContext = this;

            this.hardWorker = new BackgroundWorker();

        }

        public ImageSource BodyImageSource
        {
            get
            {
                return this.imageSource;
            }
        }

        public ImageSource ColorImageSource
        {
            get
            {
                return this.colorBitmap;
            }
        }

        [Obsolete]
        private void MainWindow_Loaded(object sender, RoutedEventArgs e)
        {
            if (this.bodyFrameReader != null)
            {
                this.bodyFrameReader.FrameArrived += this.BodyReader_FrameArrived;
            }
        }

        private void MainWindow_Closed(object sender, CancelEventArgs e)
        {
            if (this.bodyFrameReader != null)
            {
                this.bodyFrameReader.Dispose();
                this.bodyFrameReader = null;
            }

            if (this.colorFrameReader != null)
            {
                this.colorFrameReader.Dispose();
                this.colorFrameReader = null;
            }

            if (this.kinectSensor != null)
            {
                this.kinectSensor.Close();
                this.kinectSensor = null;
            }
        }

        private static Body FindClosestBody(BodyFrame bodyFrame)
        {
            Body result = null;
            double closestBodyDistance = double.MaxValue;

            Body[] bodies = new Body[bodyFrame.BodyCount];
            bodyFrame.GetAndRefreshBodyData(bodies);

            foreach (var body in bodies)
            {
                if (body.IsTracked)
                {
                    var currentLocation = body.Joints[JointType.SpineBase].Position;

                    var RightHandPosition = body.Joints[JointType.HandRight].Position.Y;
                    var LeftHandPosition = body.Joints[JointType.HandLeft].Position.Y;

                    var RightHipPosition = body.Joints[JointType.HipRight].Position.Y;
                    var LeftHipPosition = body.Joints[JointType.HipLeft].Position.Y;

                    Joint spineMid = body.Joints[JointType.SpineMid];
                    Joint spineBase = body.Joints[JointType.SpineBase];

                    var currentDistance = VectorLength(currentLocation);
                    var stomach = ((spineMid.Position.Y) + (spineBase.Position.Y)) / 2;

                    bool DistanceFromCam = currentDistance > 1;
                    if (!DistanceFromCam)
                    {
                        distanceWarning = "User required to stay 1 meter from Kinect";
                    }
                    else 
                    {
                        distanceWarning = "";
                    }


                    var compareHandPos = ((LeftHandPosition > stomach) && (RightHandPosition > stomach));


                    if ((result == null || currentDistance < closestBodyDistance) && compareHandPos && DistanceFromCam)
                    {
                        result = body;
                        closestBodyDistance = currentDistance;
                    }
                }
            }

            return result;
        }

        private static double VectorLength(CameraSpacePoint point)
        {
            var result = Math.Pow(point.X, 2) + Math.Pow(point.Y, 2) + Math.Pow(point.Z, 2);

            result = Math.Sqrt(result);

            return result;
        }

        //Turn on rgb camera and capture image and display on bitmap
        private void Reader_ColorFrameArrived(object sender, ColorFrameArrivedEventArgs e)
        {
            // ColorFrame is IDisposable
            using (ColorFrame colorFrame = e.FrameReference.AcquireFrame())
            {
                if (colorFrame != null)
                {
                    FrameDescription colorFrameDescription = colorFrame.FrameDescription;

                    using (KinectBuffer colorBuffer = colorFrame.LockRawImageBuffer())
                    {
                        this.colorBitmap.Lock();

                        // verify data and write the new color frame data to the display bitmap
                        if ((colorFrameDescription.Width == this.colorBitmap.PixelWidth) && (colorFrameDescription.Height == this.colorBitmap.PixelHeight))
                        {
                            colorFrame.CopyConvertedFrameDataToIntPtr(
                                this.colorBitmap.BackBuffer,
                                (uint)(colorFrameDescription.Width * colorFrameDescription.Height * 4),
                                ColorImageFormat.Bgra);

                            Int32Rect rect = new Int32Rect(0, 0, this.colorBitmap.PixelWidth, this.colorBitmap.PixelHeight);
                            this.colorBitmap.AddDirtyRect(rect);
                        }
                        this.colorBitmap.Unlock();
                    }
                }
            }
        }

        [Obsolete]
        private void BodyReader_FrameArrived(object sender, BodyFrameArrivedEventArgs e)
        {
            Body selectedBody = null;
            var frameReference = e.FrameReference;
            using (var frame = frameReference.AcquireFrame())
            {
                

                if (frame == null)
                {
                    return;
                }
                Body[] bodies = new Body[frame.BodyCount];
                selectedBody = FindClosestBody(frame);
            }

            if (selectedBody == null)
            {
                using (DrawingContext dc = this.drawingGroup.Open()) 
                {

                    Console.WriteLine("No frame");

                    dc.DrawRectangle(Brushes.Black, null, new Rect(0.0, 0.0, this.displayWidth, this.displayHeight));
                    FormattedText ft  = null;
                    if (!distanceWarning.Equals(""))
                    {
                        ft = new FormattedText(distanceWarning,
                               CultureInfo.GetCultureInfo("en-us"),
                               FlowDirection.LeftToRight,
                               new Typeface("Verdana"),
                               20, Brushes.Red);

                    }
                    else
                    {
                        ft = new FormattedText("Raise your hand to control",
                                    CultureInfo.GetCultureInfo("en-us"),
                                    FlowDirection.LeftToRight,
                                    new Typeface("Verdana"),
                                    20, Brushes.Red);
                    }

                    ft.TextAlignment = TextAlignment.Center;

                    var centerpoint = new Point(330, 180);
                    dc.DrawText(ft, centerpoint);
                }
                    
            }
            else
            {
                using (DrawingContext dc = this.drawingGroup.Open())
                {
                    VisualBrush vBrush = new VisualBrush();

                    vBrush.Opacity = 0.50;

                    dc.DrawRectangle(Brushes.Black, null, new Rect(0.0, 0.0, this.displayWidth, this.displayHeight));
                    Color brush = Color.FromRgb(255, 0, 144);
                    Pen drawPen = new Pen(new SolidColorBrush(brush), 4);
                    joints = selectedBody.Joints;

                    // convert the joint points to depth (display) space
                    Dictionary<JointType, Point> jointPoints = new Dictionary<JointType, Point>();

                    foreach (JointType jointType in joints.Keys)
                    {
                        CameraSpacePoint position = joints[jointType].Position;
                        if (position.Z < 0)
                        {
                            position.Z = InferredZPositionClamp;
                        }

                        DepthSpacePoint depthSpacePoint = this.coordinateMapper.MapCameraPointToDepthSpace(position);
                        jointPoints[jointType] = new Point(depthSpacePoint.X, depthSpacePoint.Y);
                    }
                    this.DrawBody(jointPoints, dc, drawPen, selectedBody);
                    this.DrawText(joints[JointType.HandRight], dc);
                    this.DrawHand(selectedBody.HandLeftState, jointPoints[JointType.HandLeft], dc);
                    this.DrawHand(selectedBody.HandRightState, jointPoints[JointType.HandRight], dc);

                    if (comp_btn_clicked)
                    {
                        MouseCtrl(selectedBody.HandLeftState);
                    }
                     
                    if (mech_btn_clicked)
                    {
                        armCtrl(selectedBody.HandLeftState);
                    }
                }
            }
        }

        [Obsolete]
        private void DrawBody(IDictionary<JointType, Point> jointPoints, DrawingContext drawingContext, Pen drawingPen, Body body)
        {

            // Draw the bones
            foreach (var bone in this.bones)
            {
                this.DrawBone(jointPoints, bone.Item1, bone.Item2, drawingContext, drawingPen);
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

        [Obsolete]
        private void DrawText(Joint joint, DrawingContext drawingContext)
        { 
            float pos_x = joint.Position.X;
            float pos_y = joint.Position.Y;
            float pos_z = joint.Position.Z;

            string position_str = "X-Axis: " + pos_x.ToString() + "\n" + "Y-Axis: " + pos_y.ToString() + "\n" + "Z-Axis: " + pos_z.ToString() 
                + "\n" + "Hand state: " + hand_state;

            drawingContext.DrawText(new FormattedText(position_str, CultureInfo.InvariantCulture, FlowDirection.LeftToRight,
                new Typeface("Segoe UI"), 20, Brushes.White), new Point(0, 0));
        }

        private void DrawBone(IDictionary<JointType, Point> jointPoints, JointType jointType0, JointType jointType1, DrawingContext drawingContext, Pen drawingPen)
        {
            Joint joint0 = joints[jointType0];
            Joint joint1 = joints[jointType1];

            // If we can't find either of these joints, exit
            if (joint0.TrackingState == TrackingState.NotTracked ||
                joint1.TrackingState == TrackingState.NotTracked)
            {
                return;
            }

            Pen drawPen = this.inferredBonePen;
            if ((joint0.TrackingState == TrackingState.Tracked) && (joint1.TrackingState == TrackingState.Tracked))
            {
                drawPen = drawingPen;
            }

            drawingContext.DrawLine(drawPen, jointPoints[jointType0], jointPoints[jointType1]);
        }

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

        public void MouseCtrl(HandState handState)
        {
            Joint rightShoulder = joints[JointType.ShoulderRight];
            Joint head = joints[JointType.Head];
            Joint spineBase = joints[JointType.SpineBase];
            Joint rightHand = joints[JointType.HandRight];
            Joint spineShoulder = joints[JointType.SpineShoulder];
            Joint spineMid = joints[JointType.SpineMid];
            var stomach = ((spineMid.Position.Y) + (spineBase.Position.Y)) / 2;
            //Draw hitbox which centered around right shoulder
            //Reference: https://stackoverflow.com/questions/13313005/kinect-sdk-1-6-and-joint-scaleto-method
            double xScaled = (rightHand.Position.X - spineShoulder.Position.X) / ((rightShoulder.Position.X - spineShoulder.Position.X) * 2) * SystemParameters.PrimaryScreenWidth;
            double yScaled = (rightHand.Position.Y - head.Position.Y) / (stomach - head.Position.Y) * SystemParameters.PrimaryScreenHeight;

            int CursorX = (int)xScaled;
            int CursorY = (int)yScaled;

            SetCursorPos(CursorX, CursorY);

            switch (handState)
            {
                case HandState.Closed:
                    mouse_event((int)(MouseEventFlags.LEFTDOWN), (int)xScaled, (int)yScaled, 0, 0);
                    hand_state = "Closed";
                    break;

                case HandState.Lasso:
                    mouse_event((int)(MouseEventFlags.RIGHTUP), (int)xScaled, (int)yScaled, 0, 0);
                    hand_state = "Lasso";
                    break;

                case HandState.Open:
                    mouse_event((int)(MouseEventFlags.LEFTUP), (int)xScaled, (int)yScaled, 0, 0);
                    hand_state = "Open";
                    break;
                default:
                    break;
            }
        }

        //Robot arm is based on inverse kinemactic. This inverse kinematic is based Fabrik. 
        //The robot arm is based on 4DOF. Therefore it requires 3 points.
        //The working area of the arm is limited. The x, y, z values is therefore limited.
        public void armCtrl(HandState handState)
        {
            Joint rightHand = joints[JointType.HandRight];
            Joint head = joints[JointType.Head];
            Joint spineMid = joints[JointType.SpineMid];
            Joint spineBase = joints[JointType.SpineBase];
            var stomach = ((spineMid.Position.Y) - (spineBase.Position.Y)) / 2;

            double pos_x = rightHand.Position.X;
            double pos_y = rightHand.Position.Y;
            double pos_z = rightHand.Position.Z;

            double x = 0, y = 0, z = 0;

            if ((pos_x >= -1) && (pos_x <= 1))
            {
                x = pos_x;
            }

            else if (pos_x <= -1)
            {
                x = -1;
            }

            else if (pos_x >= 1)
            {
                x = 1;
            }

            if ((pos_y >= -0.7) && (pos_y <= 1.3)) 
            {
                y = pos_y;
            }

            else if (pos_y <= -0.7)
            {
                y = -0.7;
            }

            else if (pos_y >= 1.3)
            {
                y = 1.3;
            }

            if ((pos_z >= 0.6) && (pos_z <= 1.45))
            {
                z = pos_z;
            }

            else if (pos_z <= 0.6)
            {
                z = 0.6;
            }

            else if (pos_z >= 1.45)
            {
                z = 1.45;
            }

            switch (handState)
            {
                case HandState.Closed:
                    hand_state = "Closed";
                    break;

                case HandState.Lasso:
                    hand_state = "Lasso";
                    break;

                case HandState.Open:
                    hand_state = "Open";
                    break;

                default:
                    break;
            }

            x = Math.Round((map(x, -1, 1, -0.26, 0.26)), 2) * 1000;
            y = Math.Round((map(y, -0.7, 1.3, -0.45, 0.56)), 2) * 1000;
            z = Math.Round(z, 2) * 1000;

            result = (x).ToString() + "," + (y).ToString() + "," + (z).ToString() + "," + hand_state + "\n";
          
            Console.WriteLine(message);

            if (message.Equals("619")) 
            {
                serialPort.Write(result);
                message = "";
                Console.WriteLine(result);
            }
        }


        private static double map(double value, double fromSource, double toSource, double fromTarget, double toTarget)
        {
            return (value - fromSource) / (toSource - fromSource) * (toTarget - fromTarget) + fromTarget;
        }

        //Create a new thread to handle bluetooth connection
        public void InitializeConnection()
        {
            hardWorker = new BackgroundWorker();
            try
            {
                openPort(com_port_combo.Text, Int32.Parse(baud_rate_combo.Text));
            }
            catch(FormatException e) 
            {
                MessageBox.Show("Please select com port and baud rate in order to connect!");
            }
            readThread = new Thread(new ThreadStart(this.Read));
            readThread.Start();
            this.hardWorker.RunWorkerAsync();
            Console.WriteLine("Main");
        }

        //Open bluetooth connection
        private void openPort(string com, int baud)
        {
            Mouse.OverrideCursor = System.Windows.Input.Cursors.Wait;
            try
            {
                System.ComponentModel.IContainer components =
                new System.ComponentModel.Container();
                serialPort = new System.IO.Ports.SerialPort(components);
                serialPort.PortName = com;
                serialPort.BaudRate = baud;
                serialPort.DtrEnable = true;
                serialPort.ReadTimeout = 100000;
                serialPort.WriteTimeout = 500;
                serialPort.DataBits = 8;
                serialPort.StopBits = StopBits.One;
                serialPort.Parity = Parity.None;
                serialPort.Open();
                conn_label.Content = "Arduino Connected";
                conn_label.Foreground = Brushes.Green;
                Mouse.OverrideCursor = null;
                bt_disc_btn.IsEnabled = true;
                bt_conn_btn.IsEnabled = false;
                this.WindowState = WindowState.Maximized;
                drawPanel.Visibility = Visibility.Visible;
                controlPanel.Visibility = Visibility.Hidden;
            }
            catch (Exception e)
            {
                MessageBox.Show("Failed to connect!", "Error", MessageBoxButton.OK, MessageBoxImage.Error);
                Mouse.OverrideCursor = null;
                bt_conn_btn.IsEnabled = true;
                bt_disc_btn.IsEnabled = false;
            }
        }
        //Close bluetooth connection
        private void closePort()
        {
            Mouse.OverrideCursor = System.Windows.Input.Cursors.Wait;

            try
            {
                if (!(readThread == null))
                    readThread.Abort();
            }
            catch (NullReferenceException){ }
            try
            {
                serialPort.DiscardInBuffer();
                serialPort.DiscardOutBuffer();
                serialPort.Close();
                conn_label.Content = "Arduino Disconnected";
                conn_label.Foreground = Brushes.Red;
                Mouse.OverrideCursor = null;
                bt_disc_btn.IsEnabled = false;
                bt_conn_btn.IsEnabled = true;
            }
            catch (Exception er)
            {
                MessageBox.Show("Failed to disconnect!", "Error", MessageBoxButton.OK, MessageBoxImage.Error);
                Mouse.OverrideCursor = null;
                bt_disc_btn.IsEnabled = true;
                bt_conn_btn.IsEnabled = false;
            }
        }


        private void Read()
        {
            Console.WriteLine("Working");
            while (serialPort.IsOpen)
            {
                Console.WriteLine("serial port open");
                Thread.Sleep(50);
                if (serialPort.IsOpen)
                {
                    serialPort.DataReceived += new SerialDataReceivedEventHandler(port_OnReceiveDatazz);
                }
            }
        }

        //Receive data from Arduino
        private void port_OnReceiveDatazz(object sender, SerialDataReceivedEventArgs e)
        {
            if (serialPort.IsOpen)
            {
                SerialPort spL = (SerialPort)sender;
                byte[] buf = new byte[spL.BytesToRead];
                Console.WriteLine("SIGNAL FROM ARDUINO!");
                try
                {
                    message = spL.ReadTo("\n");
                }
                catch (Exception err)
                { }
            }   
        }

        //Check Kinect connected or not
        private void Sensor_IsAvailableChanged(object sender, IsAvailableChangedEventArgs e) =>
        
            // on failure, set the status text
           this.KinectStatus = this.kinectSensor.IsAvailable ? true : false;
        
    }
}
