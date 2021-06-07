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

        robotPos: null,
        topicSendMode: null,
        mode: 0,

        dataShow1: 0,
        dataShow2: 0,
        mapName: "mapName",
        isSaved: false,
    },

    // helper methods to connect to ROS
    methods: {
        connect: function () {
            console.log("mapping.js-26-connect");
            this.loading = true
            var ros = new ROSLIB.Ros({
                url: this.rosbridge_address
            })
            this.ros = ros

            var mapViewer = new ROS2D.Viewer({
                divID: 'map',
                width: 1300,
                height: 800
            })

            this.ros.on('connection', () => {
                console.log("mapping.js-56-connection");
                this.logs.unshift((new Date()).toTimeString() + ' - Connected!')
                this.connected = true
                this.loading = false

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

                //////////////////////////////////////////////////////////////////////////////
                // Add robot pose and trace
                var robotPos = new ROSLIB.Pose()
                var robotTrace = new ROS2D.PoseAndTrace({
                    ros: this.ros,
                    rootObject: mapViewer.scene,
                    withTrace: true,
                    maxTraceLength : 200    
                })                
                let poseListener = new ROSLIB.Topic({
                    ros: this.ros,
                    name: '/slam_out_pose',
                    messageType: 'geometry_msgs/PoseStamped',
                    throttle_rate : 100
                })
                poseListener.subscribe((msgPose) => {
                    robotPos.position.x = msgPose.pose.position.x
                    robotPos.position.y = msgPose.pose.position.y
                    robotPos.orientation.z = msgPose.pose.orientation.z
                    robotPos.orientation.w = msgPose.pose.orientation.w

                    robotTrace.updatePose(robotPos)
                    var trace = robotTrace.trace
                    var robotMarker = robotTrace.robotMarker
                    mapViewer.scene.addChild(trace);
                    mapViewer.scene.addChild(robotMarker);		
                })
                ///////////////////////////////////////////////////////////////////////////////

                ///////////////////////////////////////////////////////////
                var zoomValue = 20;
                var zoom = 0;
                var panX = -6;
                var panY = -12;
                // Setup the map client.
                this.mapGridClient = new ROS2D.OccupancyGridClient({
                    ros: this.ros,
                    rootObject: mapViewer.scene,
                    continuous: true
                })
                // Scale the canvas to fit to the map
                this.mapGridClient.on('change', () => {
                    mapViewer.scaleToDimensions(zoomValue, zoomValue);
                    mapViewer.shift(panX, panY);

                    robotTrace.initScale();
                })        
                //////////////////////////////////////////////////////////////////////////////

                //////////////////////////////////////////////////////////////////////////////
                // Setup mouse event handlers
                this.mouseDown = false
                var zoomKey = false
                var panKey = false
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
                            zoom = 1 + 10*Math.abs(dy) / mapViewer.scene.canvas.clientHeight;
                            if (dy < 0)
                                zoom = 1 / zoom;
                                console.log("zoom: " + zoom);
                            zoomView.zoom(zoom);
                        }
                        else if (panKey === true) {
                            panView.pan(event.stageX, event.stageY);
                        }
                    }
                })
                // EVENT: Mouse UP
                mapViewer.scene.addEventListener('stagemouseup', function(event) {
                    if (this.mouseDown === true) {
                        if (zoomKey === true) {
                            zoomKey = false;
                            console.log("scaleX: " + mapViewer.scene.scaleX);
                            console.log("width: " + mapViewer.width);
                            zoomValue = mapViewer.width/mapViewer.scene.scaleX;
                            console.log("zoomValue: " + zoomValue);

                            // console.log("mapViewer.scene.x_prev_shift: " + mapViewer.scene.x_prev_shift);
                            // console.log("mapViewer.scene.y_prev_shift: " + mapViewer.scene.y_prev_shift);
                            // console.log("mapViewer.scene.x: " + mapViewer.scene.x);
                            // console.log("mapViewer.scene.y: " + mapViewer.scene.y);
                            panX = (mapViewer.scene.x_prev_shift-mapViewer.scene.x)/mapViewer.scene.scaleX;
                            panY = (mapViewer.scene.y-mapViewer.scene.y_prev_shift)/mapViewer.scene.scaleY;
                            // console.log("panX: " + panX);
                            // console.log("panY: " + panY);
                        }
                        else if (panKey === true) {
                            panKey = false;  

                            gloablPose = {x: event.stageX - startPos.x, y: event.stageY - startPos.y};
                            // console.log("gloablPose.x: " + gloablPose.x);
                            // console.log("gloablPose.y: " + gloablPose.y);
                            rosPose = {x: gloablPose.x/mapViewer.scene.scaleX, y: gloablPose.y/mapViewer.scene.scaleY};
                            // console.log("rosPose.x: " + rosPose.x);
                            // console.log("rosPose.y: " + rosPose.y);
                            panX = panX - rosPose.x; 
                            panY = panY + rosPose.y;    
                        }
                        this.mouseDown = false;
                    }
                })
                //////////////////////////////////////////////////////////////////////////////

                //////////////////////////////////////////////////////////////////////////////
                let topicSendMode = new ROSLIB.Topic({
                    ros: this.ros,
                    rootObject : mapViewer.scene,
                    name: '/client_count',
                    messageType: 'std_msgs/Int32'
                })
                this.topicSendMode = topicSendMode
                ///////////////////////////////////////////////////////////////////////////////
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

        SaveMap() {
            this.isSaved = true;
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
  