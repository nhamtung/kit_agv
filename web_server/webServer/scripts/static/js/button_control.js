let vueApp = new Vue({
    el: "#vueApp",
    data: {
        // ros connection
        ros: null,
        rosbridge_address: 'ws://localhost:9090',
        connected: false,
        // page content
        menu_title: 'Connection',
        main_title: 'Main title, from Vue!!',

        // subscriber data
        position: { x: 0, y: 0, z: 0, },
        posx: 0,
        posy: 0,
        posz: 0,

        x: 0,
        z: 0,
        delta_x: 0.1,
        delta_z: 0.3,
    },
    methods: {
        connect: function () {
            // define ROSBridge connection object
            this.ros = new ROSLIB.Ros({
                url: this.rosbridge_address
            })

            // define callbacks
            this.ros.on('connection', () => {
                this.connected = true
                console.log('Connection to ROSBridge established!')
                let topic = new ROSLIB.Topic({
                    ros: this.ros,
                    // name: '/odom',
                    // messageType: 'nav_msgs/Odometry'
                    name: '/cmd_vel',
                    messageType: 'geometry_msgs/Twist'
                })
                topic.subscribe((message) => {
                    // position = message.pose.pose.position
                    this.posx = message.linear.x
                    this.posy = message.linear.y
                    this.posz = message.angular.z
                    console.log(message)
                })
            })
            this.ros.on('error', (error) => {
                console.log('Something went wrong when trying to connect')
                console.log(error)
            })
            this.ros.on('close', () => {
                this.connected = false
                console.log('Connection to ROSBridge was closed!')
            })
        },
        disconnect: function () {
            this.ros.close()
        },
        forward: function () {
            let topic = new ROSLIB.Topic({
                ros: this.ros,
                name: '/cmd_vel',
                messageType: 'geometry_msgs/Twist'
            })
            this.x = this.x + this.delta_x
            let message = new ROSLIB.Message({
                linear: { x: this.x, y: 0, z: 0, },
                angular: { x: 0, y: 0, z: this.z, },
            })
            topic.publish(message)
        },
        backward: function () {
            let topic = new ROSLIB.Topic({
                ros: this.ros,
                name: '/cmd_vel',
                messageType: 'geometry_msgs/Twist'
            })
            this.x = this.x - this.delta_x
            let message = new ROSLIB.Message({
                linear: { x: this.x, y: 0, z: 0, },
                angular: { x: 0, y: 0, z: this.z, },
            })
            topic.publish(message)
        },
        turnLeft: function () {
            let topic = new ROSLIB.Topic({
                ros: this.ros,
                name: '/cmd_vel',
                messageType: 'geometry_msgs/Twist'
            })
            this.z = this.z + this.delta_z
            let message = new ROSLIB.Message({
                linear: { x: this.x, y: 0, z: 0, },
                angular: { x: 0, y: 0, z: this.z, },
            })
            topic.publish(message)
        },
        turnRight: function () {
            let topic = new ROSLIB.Topic({
                ros: this.ros,
                name: '/cmd_vel',
                messageType: 'geometry_msgs/Twist'
            })
            this.z = this.z - this.delta_z
            let message = new ROSLIB.Message({
                linear: { x: this.x, y: 0, z: 0, },
                angular: { x: 0, y: 0, z: this.z, },
            })
            topic.publish(message)
        },
        stop: function () {
            let topic = new ROSLIB.Topic({
                ros: this.ros,
                name: '/cmd_vel',
                messageType: 'geometry_msgs/Twist'
            })
            this.x = 0
            this.z = 0
            let message = new ROSLIB.Message({
                linear: { x: 0, y: 0, z: 0, },
                angular: { x: 0, y: 0, z: 0, },
            })
            topic.publish(message)
        },
    },
    mounted() {
        // page is ready
        console.log('page is ready!')
    },
})
