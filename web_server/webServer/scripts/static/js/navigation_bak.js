var app = new Vue({
    el: '#app',
    // storing the state of the page
    data: {
        connected: false,
        ros: null,
        logs: [],
        loading: false,
        rosbridge_address: 'ws://localhost:9090',
        port: '9090',
        mapViewer: null,
        mapGridClient: null,
        interval: null,

        isStartRobot: false,

        dataShow1: 0,
        dataShow2: 0,
    },

    // helper methods to connect to ROS
    methods: {
        connect: function () {
            this.loading = true
            this.ros = new ROSLIB.Ros({
                url: this.rosbridge_address
            })

            this.ros.on('connection', () => {
                this.logs.unshift((new Date()).toTimeString() + ' - Connected!')
                this.connected = true
                this.loading = false

                var mapViewer = new ROS2D.Viewer({
                    divID: 'map',
                    width: 1300,
                    height: 800
                })

                // this.center = new ROSLIB.Vector3()
                // this.startShift = new ROSLIB.Vector3()
                // this.startScale = new ROSLIB.Vector3()

                //////////////////////////////////////////////////////////////////////
                // Callback functions when there is mouse interaction with the polygon
                var clickedPolygon = false
                var selectedPointIndex = null
                var pointCallBack = function(type, event, index) {
                    if (type === 'mousedown') {
                        if (event.nativeEvent.shiftKey === true) {
                            polygon.remPoint(index);
                        }
                        else {
                            selectedPointIndex = index;
                        }
                    }
                    clickedPolygon = true;
                }
                var lineCallBack = function(type, event, index) {
                    if (type === 'mousedown') {
                        if (event.nativeEvent.ctrlKey === true) {
                            polygon.splitLine(index);
                        }
                    }
                    clickedPolygon = true;
                }
                // Create the polygon
                var polygon = new ROS2D.PolygonMarker({
                    lineColor : createjs.Graphics.getRGB(100, 100, 255, 1),
                    pointCallBack : pointCallBack,
                    lineCallBack : lineCallBack
                })
                // Event listeners for mouse interaction with the stage
                mapViewer.scene.mouseMoveOutside = false // doesn't seem to work
                
                /////////////////////////////////////////////////////////////////////////////
                // Add zoom to the viewer.
                var zoomView = new ROS2D.ZoomView({
                    rootObject: mapViewer.scene
                })
                // Add panning to the viewer.
                var panView = new ROS2D.PanView({
                    rootObject: mapViewer.scene
                })

                //////////////////////////////////////////////////////////////////////////////
                // Add planned path
                var plannedPath = new ROS2D.NavPath({
                    ros: this.ros,
                    rootObject: mapViewer.scene,
                    pathTopic: '/plan'
                })
                // Add robot pose and trace
                var robotTrace = new ROS2D.PoseAndTrace({
                    ros: this.ros,
                    rootObject: mapViewer.scene,
                    poseTopic: '/robot_pose',
                    withTrace: true,
                    maxTraceLength : 200
                })
                // Add navigRation goal
                var navGoal = new ROS2D.NavGoal({
                    ros: this.ros,
                    rootObject : mapViewer.scene,
                    actionMsgType: 'move_base_msgs/MoveBaseAction',
                    actionTopic : '/move_base'
                })
                // Add Pose Estimate
                let topicPoseEstimate = new ROSLIB.Topic({
                    ros: this.ros,
                    rootObject : mapViewer.scene,
                    name: '/initialpose',
                    messageType: 'geometry_msgs/PoseWithCovarianceStamped'
                })

                ///////////////////////////////////////////////////////////
                // Setup the map client.
                this.mapGridClient = new ROS2D.OccupancyGridClient({
                    ros: this.ros,
                    rootObject: mapViewer.scene,
                    continuous: true
                })
                // Scale the canvas to fit to the map
                this.mapGridClient.on('change', () => {
                    gridWidth = this.mapGridClient.currentGrid.width/13;
                    gridHeigh = this.mapGridClient.currentGrid.height/13;
                    this.dataShow1 = gridWidth;
                    this.dataShow2 = gridHeigh;
                    mapViewer.scaleToDimensions(gridWidth, gridHeigh);

                    grid_pose_x = this.mapGridClient.currentGrid.pose.position.x/16;
                    grid_pose_y = this.mapGridClient.currentGrid.pose.position.y/8;
                    mapViewer.shift(grid_pose_x, grid_pose_y);

                    plannedPath.initScale();
                    robotTrace.initScale();
                    navGoal.initScale();
                })

                ////////////////////////////////////////////////////////////////////////////////////////
                // Setup mouse event handlers
                this.mouseDown = false
                var zoomKey = false
                var panKey = false
                var isPoseEstimated = false
                var startPos = new ROSLIB.Vector3()
                // EVENT: Mouse DOWN
                mapViewer.scene.addEventListener('stagemousedown', function(event) {
                    if (event.nativeEvent.ctrlKey === true) {
                        console.log("Ctrl + Mouse");
                        zoomKey = true;
                        zoomView.startZoom(event.stageX, event.stageY);
                    }
                    else if (event.nativeEvent.shiftKey === true) {
                        console.log("Shift + Mouse");
                        panKey = true;
                        panView.startPan(event.stageX, event.stageY);
                    }
                    else {
                        var pos = mapViewer.scene.globalToRos(event.stageX, event.stageY);
                        console.log("x: " + pos.x);
                        console.log("y: " + pos.y);
                        if (event.nativeEvent.altKey === true){
                            console.log("Alt + Mouse");
                            navGoal.startGoalSelection(pos);
                            isPoseEstimated = true;
                        }
                        else{
                            console.log("Mouse");
                            navGoal.startGoalSelection(pos);
                        }
                    }

                    startPos.x = event.stageX;
                    startPos.y = event.stageY;
                    console.log("startPos.x: " + startPos.x);
                    console.log("startPos.y: " + startPos.y);
                    
                    this.mouseDown = true;
                })
                // EVENT: Mouse MOVE
                mapViewer.scene.addEventListener('stagemousemove', function(event) {
                    if (this.mouseDown === true) {
                        if (zoomKey === true) {
                            var dy = event.stageY - startPos.y;
                            var zoom = 1 + 10*Math.abs(dy) / mapViewer.scene.canvas.clientHeight;
                            if (dy < 0)
                                zoom = 1 / zoom;
                            zoomView.zoom(zoom);
                        }
                        else if (panKey === true) {
                            panView.pan(event.stageX, event.stageY);
                        }
                        else {
                            var pos = mapViewer.scene.globalToRos(event.stageX, event.stageY);
                            navGoal.orientGoalSelection(pos);

                            // Move point when it's dragged
                            if (selectedPointIndex !== null) {
                                var pos = mapViewer.scene.globalToRos(event.stageX, event.stageY);
                                polygon.movePoint(selectedPointIndex, pos);
                            }
                        }
                    }
                })
                // EVENT: Mouse UP
                mapViewer.scene.addEventListener('stagemouseup', function(event) {
                    if (this.mouseDown === true) {
                        if (zoomKey === true) {
                            zoomKey = false;
                        }
                        else if (panKey === true) {
                            panKey = false;
                        }
                        else {
                            var pos = mapViewer.scene.globalToRos(event.stageX, event.stageY);

                            // Add point when not clicked on the polygon
                            if (selectedPointIndex !== null) {
                                selectedPointIndex = null;
                            }
                            else if (mapViewer.scene.mouseInBounds === true && clickedPolygon === false) {
                                var goalPose = navGoal.endGoalSelection(pos);
                                var goalPolygon = navGoal.goalOrientationMarker;
                                
                                startPos.x = goalPose.position.x;
                                startPos.y = goalPose.position.y;
                                console.log("startPos.x: " + startPos.x);
                                console.log("startPos.y: " + startPos.y);
                                console.log("goalPose.orientation.z: " + goalPose.orientation.z);
                                console.log("goalPose.orientation.w: " + goalPose.orientation.w);

                                if(isPoseEstimated === true){
                                    console.log("map.js-228-Pose Estimate!");
                                    var covariance;
                                    let msgPoseEstimate = new ROSLIB.Message({
                                        header: { seq: 1, stamp: 0, frame_id: 'map', },
                                        pose: { 
                                            pose: {
                                                position: {x: startPos.x, y: startPos.y, z: 0,},
                                                orientation: {x:0, y:0, z: goalPose.orientation.z, w: goalPose.orientation.w,},
                                            },
                                            covariance: covariance,
                                         },
                                    });
                                    topicPoseEstimate.publish(msgPoseEstimate);

                                    mapViewer.scene.addChild(goalPolygon);
                                    isPoseEstimated = false;
                                }else{
                                    console.log("map.js-243-Nav Goal!");
                                    navGoal.sendGoal(goalPose);

                                    polygon.addPointPath(startPos);
                                    // Add the polygon to the viewer
                                    mapViewer.scene.addChild(goalPolygon);
                                    mapViewer.scene.addChild(polygon);			
                                }
                            }
                            clickedPolygon = false;
                        }
                        this.mouseDown = false;
                    }
                })

            })
            this.ros.on('error', (error) => {
                this.logs.unshift((new Date()).toTimeString() + ` - Error: ${error}`)
            })
            this.ros.on('close', () => {
                console.log("map.js-246-close")
                this.logs.unshift((new Date()).toTimeString() + ' - Disconnected!')
                this.connected = false
                this.loading = false
                document.getElementById('map').innerHTML = ''
            })
        },

        disconnect: function () {
            this.ros.close()
        },

        RobotStart() {
            this.isStartRobot = false;
            console.log("isStartRobot: " + this.isStartRobot);
        },
        RobotStop(){
            this.isStartRobot = true;
            console.log("isStartRobot: " + this.isStartRobot);
        },
    },

    mounted() {
        this.interval = setInterval(() => {
            if (this.ros != null && this.ros.isConnected) {
                this.ros.getNodes((data) => { }, (error) => { })
            }
        }, 10000)
    },
})
  