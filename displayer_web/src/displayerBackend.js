import axois from 'axios';
import {io} from 'socket.io-client';
import {getDvaApp} from "umi";
import {message, Modal} from "antd";

class DisplayerBackend {
  constructor() {
    const that = this;
    this.timer = setInterval(() => {
      that.init().then();
    }, 100);
  }

  async init() {
    if (getDvaApp()) {
      clearInterval(this.timer);
      this.dispatch = getDvaApp()._store.dispatch;
      const config = await axois.get('/api/getConfig');
      console.log(config)
      this.dispatch({
        type: 'configProvider/init',
        payload: config.data
      })
      this.io = io('/api/ws');
      this.io.on('connect', () => {
        message.success('已成功连接至后端服务器')
      });
      this.io.on('disconnect', (reason) => {
        if (reason === 'io server disconnect' || reason === 'io client disconnect') {
          Modal.error({
            title: '连接已断开',
            content: '后端服务器已关闭，请重新启动服务器后刷新页面'
          })
        }
      });
      this.io.on('connect_error', () => {
        message.error('连接后端服务器出错，稍后将自动重试')
      });
      this.io.on('setPath', (msg) => {
        this.dispatch({
          type: 'configProvider/setPath',
          payload: msg
        })
      })
    }
  }

  saveCalibrate(camera, path) {
    this.io.emit('cameraCalibrate', { camera, path })
  }
}

export const displayerBackend = new DisplayerBackend();
