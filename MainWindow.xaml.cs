//------------------------------------------------------------------------------
// <copyright file="MainWindow.xaml.cs" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

namespace Microsoft.Samples.Kinect.FaceBasics
{
    using System;
    using System.Collections.Generic;
    using System.ComponentModel;
    using System.Diagnostics;
    using System.Globalization;
    using System.IO;
    using System.Windows;
    using System.Windows.Media;
    using System.Windows.Media.Imaging;
    using System.Windows.Media.Media3D;
    using Microsoft.Kinect;
    using Microsoft.Kinect.Face;
    using System.Windows.Shapes;

    using System.Linq;
    using System.Windows.Controls;

    /// <summary>
    /// Interaction logic for MainWindow
    /// </summary>
    public partial class MainWindow : Window, INotifyPropertyChanged
    {
        private const double DrawFaceShapeThickness = 8;

        private const double DrawTextFontSize = 30;

        private const double FacePointRadius = 1.0;

        private const float TextLayoutOffsetX = -0.1f;
        
        private const float TextLayoutOffsetY = -0.15f;

        private const double FaceRotationIncrementInDegrees = 5.0;

        private FormattedText textFaceNotTracked = new FormattedText(
                        "No bodies or faces are tracked ...",
                        CultureInfo.GetCultureInfo("en-us"),
                        FlowDirection.LeftToRight,
                        new Typeface("Georgia"),
                        DrawTextFontSize,
                        Brushes.White);

        private Point textLayoutFaceNotTracked = new Point(10.0, 10.0);

        private DrawingGroup drawingGroup;

        private DrawingImage imageSource;

        private KinectSensor kinectSensor = null;

        private CoordinateMapper coordinateMapper = null;

        private BodyFrameReader bodyFrameReader = null;

        private Body[] bodies = null;

        private int bodyCount;

        private FaceFrameSource[] faceFrameSources = null;

        private FaceFrameReader[] faceFrameReaders = null;

        private FaceFrameResult[] faceFrameResults = null;

        private int displayWidth;

        private int displayHeight;

        private Rect displayRect;

        private List<Brush> faceBrush;

        private string statusText = null;

        public MainWindow()
        {
            // one sensor is currently supported
            this.kinectSensor = KinectSensor.GetDefault();

            // get the coordinate mapper
            this.coordinateMapper = this.kinectSensor.CoordinateMapper;

            // get the color frame details
            FrameDescription frameDescription = this.kinectSensor.ColorFrameSource.FrameDescription;

            // set the display specifics
            this.displayWidth = frameDescription.Width;
            this.displayHeight = frameDescription.Height;
            this.displayRect = new Rect(0.0, 0.0, this.displayWidth, this.displayHeight);

            // open the reader for the body frames
            this.bodyFrameReader = this.kinectSensor.BodyFrameSource.OpenReader();

            // wire handler for body frame arrival
            this.bodyFrameReader.FrameArrived += this.Reader_BodyFrameArrived;

            // set the maximum number of bodies that would be tracked by Kinect
            this.bodyCount = this.kinectSensor.BodyFrameSource.BodyCount;

            // allocate storage to store body objects
            this.bodies = new Body[this.bodyCount];

            // specify the required face frame results
            FaceFrameFeatures faceFrameFeatures =
                FaceFrameFeatures.BoundingBoxInColorSpace
                | FaceFrameFeatures.PointsInColorSpace
                | FaceFrameFeatures.RotationOrientation
                | FaceFrameFeatures.LeftEyeClosed
                | FaceFrameFeatures.RightEyeClosed;

            // create a face frame source + reader to track each face in the FOV
            this.faceFrameSources = new FaceFrameSource[this.bodyCount];
            this.faceFrameReaders = new FaceFrameReader[this.bodyCount];
            for (int i = 0; i < this.bodyCount; i++)
            {
                // create the face frame source with the required face frame features and an initial tracking Id of 0
                this.faceFrameSources[i] = new FaceFrameSource(this.kinectSensor, 0, faceFrameFeatures);

                // open the corresponding reader
                this.faceFrameReaders[i] = this.faceFrameSources[i].OpenReader();
            }

            // allocate storage to store face frame results for each face in the FOV
            this.faceFrameResults = new FaceFrameResult[this.bodyCount];

            // populate face result colors - one for each face index
            this.faceBrush = new List<Brush>()
            {
                Brushes.White, 
                Brushes.Orange,
                Brushes.Green,
                Brushes.Red,
                Brushes.LightBlue,
                Brushes.Yellow
            };

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

        public event PropertyChangedEventHandler PropertyChanged;

        public ImageSource ImageSource
        {
            get
            {
                return this.imageSource;
            }
        }

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

        private static void ExtractFaceRotationInDegrees(Vector4 rotQuaternion, out int pitch, out int yaw, out int roll)
        {
            double x = rotQuaternion.X;
            double y = rotQuaternion.Y;
            double z = rotQuaternion.Z;
            double w = rotQuaternion.W;

            // convert face rotation quaternion to Euler angles in degrees
            double yawD, pitchD, rollD;
            pitchD = Math.Atan2(2 * ((y * z) + (w * x)), (w * w) - (x * x) - (y * y) + (z * z)) / Math.PI * 180.0;
            yawD = Math.Asin(2 * ((w * y) - (x * z))) / Math.PI * 180.0;
            rollD = Math.Atan2(2 * ((x * y) + (w * z)), (w * w) + (x * x) - (y * y) - (z * z)) / Math.PI * 180.0;

            // clamp the values to a multiple of the specified increment to control the refresh rate
            double increment = FaceRotationIncrementInDegrees;
            pitch = (int)(Math.Floor((pitchD + ((increment / 2.0) * (pitchD > 0 ? 1.0 : -1.0))) / increment) * increment);
            yaw = (int)(Math.Floor((yawD + ((increment / 2.0) * (yawD > 0 ? 1.0 : -1.0))) / increment) * increment);
            roll = (int)(Math.Floor((rollD + ((increment / 2.0) * (rollD > 0 ? 1.0 : -1.0))) / increment) * increment);
        }

        private void MainWindow_Loaded(object sender, RoutedEventArgs e)
        {
            for (int i = 0; i < this.bodyCount; i++)
            {
                if (this.faceFrameReaders[i] != null)
                {
                    // wire handler for face frame arrival
                    this.faceFrameReaders[i].FrameArrived += this.Reader_FaceFrameArrived;
                }
            }

            if (this.bodyFrameReader != null)
            {
                // wire handler for body frame arrival
                this.bodyFrameReader.FrameArrived += this.Reader_BodyFrameArrived;
            }

            kinectSensor = KinectSensor.GetDefault();

            if (kinectSensor != null)
            {
                _bodySource = kinectSensor.BodyFrameSource;
                _bodyReader = _bodySource.OpenReader();
                _bodyReader.FrameArrived += BodyReader_FrameArrived;

                _faceSource = new HighDefinitionFaceFrameSource(kinectSensor);

                _faceReader = _faceSource.OpenReader();
                _faceReader.FrameArrived += FaceReader_FrameArrived;

                _faceModel = new FaceModel();
                _faceAlignment = new FaceAlignment();

                kinectSensor.Open();
            }
        }

        private void MainWindow_Closing(object sender, CancelEventArgs e)
        {
            for (int i = 0; i < this.bodyCount; i++)
            {
                if (this.faceFrameReaders[i] != null)
                {
                    // FaceFrameReader is IDisposable
                    this.faceFrameReaders[i].Dispose();
                    this.faceFrameReaders[i] = null;
                }

                if (this.faceFrameSources[i] != null)
                {
                    // FaceFrameSource is IDisposable
                    this.faceFrameSources[i].Dispose();
                    this.faceFrameSources[i] = null;
                }
            }
            
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

            if (_faceModel != null)
            {
                _faceModel.Dispose();
                _faceModel = null;
            }

            GC.SuppressFinalize(this);
        }

        private void Reader_FaceFrameArrived(object sender, FaceFrameArrivedEventArgs e)
        {
            using (FaceFrame faceFrame = e.FrameReference.AcquireFrame())
            {
                if (faceFrame != null)
                {
                    // get the index of the face source from the face source array
                    int index = this.GetFaceSourceIndex(faceFrame.FaceFrameSource);

                    // check if this face frame has valid face frame results
                    if (this.ValidateFaceBoxAndPoints(faceFrame.FaceFrameResult))
                    {
                        // store this face frame result to draw later
                        this.faceFrameResults[index] = faceFrame.FaceFrameResult;
                    }
                    else
                    {
                        // indicates that the latest face frame result from this reader is invalid
                        this.faceFrameResults[index] = null;
                    }
                }
            }
        }

        private int GetFaceSourceIndex(FaceFrameSource faceFrameSource)
        {
            int index = -1;

            for (int i = 0; i < this.bodyCount; i++)
            {
                if (this.faceFrameSources[i] == faceFrameSource)
                {
                    index = i;
                    break;
                }
            }

            return index;
        }

        private void Reader_BodyFrameArrived(object sender, BodyFrameArrivedEventArgs e)
        {
            using (var bodyFrame = e.FrameReference.AcquireFrame())
            {
                if (bodyFrame != null)
                {
                    // update body data
                    bodyFrame.GetAndRefreshBodyData(this.bodies);

                    using (DrawingContext dc = this.drawingGroup.Open())
                    {
                        // draw the dark background
                        dc.DrawRectangle(Brushes.Black, null, this.displayRect);

                        bool drawFaceResult = false;

                        // iterate through each face source
                        for (int i = 0; i < this.bodyCount; i++)
                        {
                            // check if a valid face is tracked in this face source
                            if (this.faceFrameSources[i].IsTrackingIdValid)
                            {
                                // check if we have valid face frame results
                                if (this.faceFrameResults[i] != null)
                                {
                                    // draw face frame results
                                    this.DrawFaceFrameResults(i, this.faceFrameResults[i], dc);

                                    if (!drawFaceResult)
                                    {
                                        drawFaceResult = true;
                                    }
                                }
                            }
                            else
                            {
                                // check if the corresponding body is tracked 
                                if (this.bodies[i].IsTracked)
                                {
                                    // update the face frame source to track this body
                                    this.faceFrameSources[i].TrackingId = this.bodies[i].TrackingId;
                                }
                            }
                        }

                        if (!drawFaceResult)
                        {
                            // if no faces were drawn then this indicates one of the following:
                            // a body was not tracked 
                            // a body was tracked but the corresponding face was not tracked
                            // a body and the corresponding face was tracked though the face box or the face points were not valid
                            dc.DrawText(
                                this.textFaceNotTracked, 
                                this.textLayoutFaceNotTracked);
                        }

                        this.drawingGroup.ClipGeometry = new RectangleGeometry(this.displayRect);
                    }
                }
            }
        }

        private void DrawFaceFrameResults(int faceIndex, FaceFrameResult faceResult, DrawingContext drawingContext)
        {
            // choose the brush based on the face index
            Brush drawingBrush = this.faceBrush[0];
            if (faceIndex < this.bodyCount)
            {
                drawingBrush = this.faceBrush[faceIndex];
            }

            Pen drawingPen = new Pen(drawingBrush, DrawFaceShapeThickness);

            // draw the face bounding box
            var faceBoxSource = faceResult.FaceBoundingBoxInColorSpace;
            Rect faceBox = new Rect(faceBoxSource.Left, faceBoxSource.Top, faceBoxSource.Right - faceBoxSource.Left, faceBoxSource.Bottom - faceBoxSource.Top);
            drawingContext.DrawRectangle(null, drawingPen, faceBox);

            if (faceResult.FacePointsInColorSpace != null)
            {
                // draw each face point
                foreach (PointF pointF in faceResult.FacePointsInColorSpace.Values)
                {
                    drawingContext.DrawEllipse(null, drawingPen, new Point(pointF.X, pointF.Y), FacePointRadius, FacePointRadius);
                }
            }

            string faceText = string.Empty;

            // extract each face property information and store it in faceText
            if (faceResult.FaceProperties != null)
            {
                foreach (var item in faceResult.FaceProperties)
                {
                    faceText += item.Key.ToString() + " : ";

                    // consider a "maybe" as a "no" to restrict 
                    // the detection result refresh rate
                    if (item.Value == DetectionResult.Maybe)
                    {
                        faceText += DetectionResult.No + "\n";
                    }
                    else
                    {
                        faceText += item.Value.ToString() + "\n";
                    }                    
                }
            }

            // extract face rotation in degrees as Euler angles
            if (faceResult.FaceRotationQuaternion != null)
            {
                int pitch, yaw, roll;
                ExtractFaceRotationInDegrees(faceResult.FaceRotationQuaternion, out pitch, out yaw, out roll);
                faceText += "FaceYaw : " + yaw + "\n" +
                            "FacePitch : " + pitch + "\n" +
                            "FacenRoll : " + roll + "\n";
            }

            // render the face property and face rotation information
            Point faceTextLayout;
            if (this.GetFaceTextPositionInColorSpace(faceIndex, out faceTextLayout))
            {
                drawingContext.DrawText(
                        new FormattedText(
                            faceText,
                            CultureInfo.GetCultureInfo("en-us"),
                            FlowDirection.LeftToRight,
                            new Typeface("Georgia"),
                            DrawTextFontSize,
                            drawingBrush),
                        faceTextLayout);
            }
        }

        private bool GetFaceTextPositionInColorSpace(int faceIndex, out Point faceTextLayout)
        {
            faceTextLayout = new Point();
            bool isLayoutValid = false;

            Body body = this.bodies[faceIndex];
            if (body.IsTracked)
            {
                var headJoint = body.Joints[JointType.Head].Position;

                CameraSpacePoint textPoint = new CameraSpacePoint()
                {
                    X = headJoint.X + TextLayoutOffsetX,
                    Y = headJoint.Y + TextLayoutOffsetY,
                    Z = headJoint.Z
                };

                ColorSpacePoint textPointInColor = this.coordinateMapper.MapCameraPointToColorSpace(textPoint);

                faceTextLayout.X = textPointInColor.X;
                faceTextLayout.Y = textPointInColor.Y;
                isLayoutValid = true;                
            }

            return isLayoutValid;
        }

        private bool ValidateFaceBoxAndPoints(FaceFrameResult faceResult)
        {
            bool isFaceValid = faceResult != null;

            if (isFaceValid)
            {
                var faceBox = faceResult.FaceBoundingBoxInColorSpace;
                if (faceBox != null)
                {
                    // check if we have a valid rectangle within the bounds of the screen space
                    isFaceValid = (faceBox.Right - faceBox.Left) > 0 &&
                                  (faceBox.Bottom - faceBox.Top) > 0 &&
                                  faceBox.Right <= this.displayWidth &&
                                  faceBox.Bottom <= this.displayHeight;

                    if (isFaceValid)
                    {
                        var facePoints = faceResult.FacePointsInColorSpace;
                        if (facePoints != null)
                        {
                            foreach (PointF pointF in facePoints.Values)
                            {
                                // check if we have a valid face point within the bounds of the screen space
                                bool isFacePointValid = pointF.X > 0.0f &&
                                                        pointF.Y > 0.0f &&
                                                        pointF.X < this.displayWidth &&
                                                        pointF.Y < this.displayHeight;

                                if (!isFacePointValid)
                                {
                                    isFaceValid = false;
                                    break;
                                }
                            }
                        }
                    }
                }
            }

            return isFaceValid;
        }

        private void Sensor_IsAvailableChanged(object sender, IsAvailableChangedEventArgs e)
        {
            if (this.kinectSensor != null)
            {
                // on failure, set the status text
                this.StatusText = this.kinectSensor.IsAvailable ? Properties.Resources.RunningStatusText
                                                                : Properties.Resources.SensorNotAvailableStatusText;
            }
        }




        private BodyFrameSource _bodySource = null;

        private BodyFrameReader _bodyReader = null;

        private HighDefinitionFaceFrameSource _faceSource = null;

        private HighDefinitionFaceFrameReader _faceReader = null;

        private FaceAlignment _faceAlignment = null;

        private FaceModel _faceModel = null;

        private List<Ellipse> _points = new List<Ellipse>();

        private void BodyReader_FrameArrived(object sender, BodyFrameArrivedEventArgs e)
        {
            using (var frame = e.FrameReference.AcquireFrame())
            {
                if (frame != null)
                {
                    Body[] bodies = new Body[frame.BodyCount];
                    frame.GetAndRefreshBodyData(bodies);

                    Body body = bodies.Where(b => b.IsTracked).FirstOrDefault();

                    if (!_faceSource.IsTrackingIdValid)
                    {
                        if (body != null)
                        {
                            _faceSource.TrackingId = body.TrackingId;
                        }
                    }
                }
            }
        }

        private void FaceReader_FrameArrived(object sender, HighDefinitionFaceFrameArrivedEventArgs e)
        {
            using (var frame = e.FrameReference.AcquireFrame())
            {
                if (frame != null && frame.IsFaceTracked)
                {
                    frame.GetAndRefreshFaceAlignmentResult(_faceAlignment);
                    UpdateFacePoints();
                }
            }
        }

        private void UpdateFacePoints()
        {
            if (_faceModel == null) return;

            var vertices = _faceModel.CalculateVerticesForAlignment(_faceAlignment);

            if (vertices.Count > 0)
            {
                

                if (_points.Count == 0)
                {
                    for (int index = 0; index < vertices.Count; index++)
                    {
                        Ellipse ellipse = new Ellipse
                        {
                            Width = 2.0,
                            Height = 2.0,
                            Fill = new SolidColorBrush(Colors.Blue)
                        };

                        _points.Add(ellipse);
                    }

                    foreach (Ellipse ellipse in _points)
                    {
                        canvas.Children.Add(ellipse);
                    }
                }

                for (int index = 0; index < vertices.Count; index++)
                {
                    CameraSpacePoint vertice = vertices[index];
                    DepthSpacePoint point = kinectSensor.CoordinateMapper.MapCameraPointToDepthSpace(vertice);

                    if (float.IsInfinity(point.X) || float.IsInfinity(point.Y)) return;

                    Ellipse ellipse = _points[index];

                    Canvas.SetLeft(ellipse, point.X);
                    Canvas.SetTop(ellipse, point.Y);
                }

                var _previousEyeSize = _leftEyeSize;

                if (_previousEyeSize == 0)
                {
                    _leftEyeSize = CalculateLeftEyeSize();
                    _previousEyeSize = _leftEyeSize;
                }
                else
                {
                    var EyeSizeDifference = (_leftEyeSize / _previousEyeSize) * 100;
                    _leftEyeSize = CalculateLeftEyeSize();
                }


                LeftEyeSizeTextBlock.Text = _leftEyeSize.ToString();



                _eyeBrowInBetweenDistance = CalculateEyeBrowInBetweenDistance();

                DistanceBetweenEyebrowTextBlock.Text = _eyeBrowInBetweenDistance.ToString();



                _eyeBrowAndEyeDistance = CalculateEyeBrowAndEyeDistance();

                DistanceBetweenEyeAndEyebrowTextBlock.Text = _eyeBrowAndEyeDistance.ToString();
            }
        }

        #region Calculate Left Eye Size
        private double CalculateLeftEyeSize()
        {
            var vertices = _faceModel.CalculateVerticesForAlignment(_faceAlignment);

            CameraSpacePoint LefteyeMidtopCameraPoint = vertices[(int)HighDetailFacePoints.LefteyeMidtop];
            float x1 = LefteyeMidtopCameraPoint.X;
            float y1 = LefteyeMidtopCameraPoint.Y;
            float z1 = LefteyeMidtopCameraPoint.Z;

            CameraSpacePoint LefteyeMidbottomCameraPoint = vertices[(int)HighDetailFacePoints.LefteyeMidbottom];
            float x2 = LefteyeMidbottomCameraPoint.X;
            float y2 = LefteyeMidbottomCameraPoint.Y;
            float z2 = LefteyeMidbottomCameraPoint.Z;

            var _eyeSize = (Math.Sqrt(Math.Pow(x2 - x1, 2) + Math.Pow(y2 - y1, 2) + Math.Pow(z2 - z1, 2)));

            return _eyeSize;
        }

        private double _leftEyeSize;

        public double LeftEyeSize
        {
            get
            {
                return _leftEyeSize;
            }

            set
            {
                _leftEyeSize = CalculateLeftEyeSize();
                PropertyChanged(this, new PropertyChangedEventArgs("LeftEyeSize"));
            }
        }
        #endregion

        #region Calculate Left Eye Size using Area
        private double CalculateEyeSizeUsingArea()
        {
            var vertices = _faceModel.CalculateVerticesForAlignment(_faceAlignment);

            CameraSpacePoint LefteyeMidtopCameraPoint = vertices[(int)HighDetailFacePoints.LefteyeMidtop];
            float x1 = LefteyeMidtopCameraPoint.X;
            float y1 = LefteyeMidtopCameraPoint.Y;
            float z1 = LefteyeMidtopCameraPoint.Z;

            CameraSpacePoint LefteyeMidbottomCameraPoint = vertices[(int)HighDetailFacePoints.LefteyeMidbottom];
            float x2 = LefteyeMidbottomCameraPoint.X;
            float y2 = LefteyeMidbottomCameraPoint.Y;
            float z2 = LefteyeMidbottomCameraPoint.Z;

            CameraSpacePoint LefteyeInnercornerCameraPoint = vertices[(int)HighDetailFacePoints.LefteyeInnercorner];
            float x3 = LefteyeMidbottomCameraPoint.X;
            float y3 = LefteyeMidbottomCameraPoint.Y;
            float z3 = LefteyeMidbottomCameraPoint.Z;

            CameraSpacePoint LefteyeOutercornerCameraPoint = vertices[(int)HighDetailFacePoints.LefteyeOutercorner];
            float x4 = LefteyeMidbottomCameraPoint.X;
            float y4 = LefteyeMidbottomCameraPoint.Y;
            float z4 = LefteyeMidbottomCameraPoint.Z;

            var _eyeSizeUsingArea = (Math.Sqrt(Math.Pow(x2 - x1, 2) + Math.Pow(y2 - y1, 2) + Math.Pow(z2 - z1, 2)));

            return _eyeSizeUsingArea;
        }

        private double _eyeSizeUsingArea;

        public double EyeSizeUsingArea
        {
            get
            {
                return _eyeSizeUsingArea;
            }

            set
            {
                _eyeSizeUsingArea = CalculateEyeSizeUsingArea();
                PropertyChanged(this, new PropertyChangedEventArgs("EyeSizeUsingArea"));
            }
        }
        #endregion

        #region Calculate Distance between Eyebrow
        private double CalculateEyeBrowInBetweenDistance()
        {
            var vertices = _faceModel.CalculateVerticesForAlignment(_faceAlignment);

            CameraSpacePoint LefteyeMidtopCameraPoint = vertices[(int)HighDetailFacePoints.LefteyebrowInner];
            float x1 = LefteyeMidtopCameraPoint.X;
            float y1 = LefteyeMidtopCameraPoint.Y;
            float z1 = LefteyeMidtopCameraPoint.Z;

            CameraSpacePoint LefteyeMidbottomCameraPoint = vertices[(int)HighDetailFacePoints.RighteyebrowInner];
            float x2 = LefteyeMidbottomCameraPoint.X;
            float y2 = LefteyeMidbottomCameraPoint.Y;
            float z2 = LefteyeMidbottomCameraPoint.Z;

            var _eyeBrowInBetweenDistance = (Math.Sqrt(Math.Pow(x2 - x1, 2) + Math.Pow(y2 - y1, 2) + Math.Pow(z2 - z1, 2)));

            return _eyeBrowInBetweenDistance;
        }

        private double _eyeBrowInBetweenDistance;

        public double EyeBrowInBetweenDistance
        {
            get
            {
                return _eyeBrowInBetweenDistance;
            }

            set
            {
                _eyeBrowInBetweenDistance = CalculateEyeBrowInBetweenDistance();
                PropertyChanged(this, new PropertyChangedEventArgs("EyeBrowInBetweenDistance"));
            }
        }
        #endregion

        #region Calculate Distance Between Eyebrow and Eye
        private double CalculateEyeBrowAndEyeDistance()
        {
            var vertices = _faceModel.CalculateVerticesForAlignment(_faceAlignment);

            CameraSpacePoint LefteyeMidtopCameraPoint = vertices[(int)HighDetailFacePoints.LefteyebrowCenter];
            float x1 = LefteyeMidtopCameraPoint.X;
            float y1 = LefteyeMidtopCameraPoint.Y;
            float z1 = LefteyeMidtopCameraPoint.Z;

            CameraSpacePoint LefteyeMidbottomCameraPoint = vertices[(int)HighDetailFacePoints.LefteyeMidtop];
            float x2 = LefteyeMidbottomCameraPoint.X;
            float y2 = LefteyeMidbottomCameraPoint.Y;
            float z2 = LefteyeMidbottomCameraPoint.Z;

            var _eyeBrowAndEyeDistance = (Math.Sqrt(Math.Pow(x2 - x1, 2) + Math.Pow(y2 - y1, 2) + Math.Pow(z2 - z1, 2)));

            return _eyeBrowAndEyeDistance;
        }

        private double _eyeBrowAndEyeDistance;

        public double EyeBrowAndEyeDistance
        {
            get
            {
                return _eyeBrowAndEyeDistance;
            }

            set
            {
                _eyeBrowAndEyeDistance = CalculateEyeBrowAndEyeDistance();
                PropertyChanged(this, new PropertyChangedEventArgs("EyeBrowAndEyeDistance"));
            }
        }
        #endregion
    }
}
