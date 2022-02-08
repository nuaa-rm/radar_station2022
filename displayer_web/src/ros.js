import ROSLIB from 'roslib';
import { Modal, message } from 'antd'
import { getDvaApp } from "umi";

let dispatch = () => {}
const initTimer = setInterval(()=>{
  const app = getDvaApp()
  if (app) {
    dispatch = app._store.dispatch
    clearInterval(initTimer)
    initRos()
    console.log('Ros init success')
  }
}, 100)

let ros;
let rosError = false

function initRos() {
  ros = new ROSLIB.Ros({
    url: 'ws://localhost:42859'
  });

  ros.on('error', (e) => {
    rosError = true
    dispatch({
      type: 'rosModel/rosConnectChange',
      payload: { connected: false },
    })
    Modal.error({
      title: 'Error happen when connecting to ros',
      content: (
        <div>
          <p>{e.msg?e.msg:'Unknown Error'}</p>
        </div>
      ),
      okText: 'Retry',
      onOk: () => {
        rosError = false
        initRos()
      },
    });
  })

  ros.on('connection', () => {
    console.log('Connected to websocket server.');
    dispatch({
      type: 'rosModel/rosConnectChange',
      payload: true
    })
    message.success('Connected to ros server.')
  })

  ros.on('close', () => {
    console.log('Connection to websocket server closed.');
    if (!rosError) {
      dispatch({
        type: 'rosModel/rosConnectChange',
        payload: { connected: false },
      })
      Modal.error({
        title: 'Connection to ros closed',
        content: (
          <div>
            <p>Server stopped</p>
          </div>
        ),
        okText: 'Retry',
        onOk: () => {
          initRos()
        },
      });
    }
  })


  const cameraOneTopic = new ROSLIB.Topic({
    ros: ros, name: '/usb_cam/image_raw',
    messageType: 'sensor_msgs/Image',
    throttle_rate: 40
  });

  const cameraTwoTopic = new ROSLIB.Topic({
    ros: ros, name: '/cameraTwo/image/compressed',
    messageType: 'sensor_msgs/CompressedImage',
    throttle_rate: 40
  });

  cameraOneTopic.subscribe(function (message) {
    dispatch({
      type: 'rosModel/rosCameraChange',
      payload: {
        camera: 'cameraOne',
        image: "data:image/jpg;base64," + message.data
      }
    })
  });

  cameraTwoTopic.subscribe(function (message) {
    dispatch({
      type: 'rosModel/rosCameraChange',
      payload: {
        camera: 'cameraTwo',
        image: "data:image/jpg;base64," + message
      }
    })
  });
}

